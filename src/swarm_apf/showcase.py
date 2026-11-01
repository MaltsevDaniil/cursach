"""Large, explicitly illustrative formation demos; separate from E2 benchmarks."""
import argparse
from dataclasses import asdict
from pathlib import Path
import numpy as np
from .formation import FormationLayout, FormationPlan, slot_position
from .controller_team import ControllerTeam
from .geometry import Obstacle
from .models import SwarmConfig
from .swarm import SwarmSimulator
from .experiment_runner import json_dump
from .swarm_visualization import save_swarm_figure

SCENES = ("assembly", "islands", "wide_passage")


def make_scene(name, n=20, seed=7):
    if name not in SCENES or n not in (10,20,30):
        raise ValueError("known scene and 10/20/30 robots required")
    layout = FormationLayout(tuple(range(n)), "V", 1.5)
    plan = FormationPlan((-8,0), (16,0), .45)
    start = np.array([slot_position(layout,plan.at(0),i) for i in layout.robot_ids])
    # Independent perturbations, same seed across controllers.
    start += np.random.default_rng(seed).uniform(-.3,.3,start.shape)
    goals = np.array([slot_position(layout,plan.at(plan.arrival_time),i) for i in layout.robot_ids])
    obstacles = {
        "assembly": (),
        "islands": (Obstacle((0,0),1.2),Obstacle((7,3),1.),Obstacle((8,-4),1.2)),
        "wide_passage": tuple(Obstacle((x,y),.65) for x in np.arange(-1,9,1.1) for y in (-4,4))
    }[name]
    return layout,plan,start,goals,obstacles


def animate(result, layout, plan, destination, frames=140):
    """Pillow rasterization of accepted states; no interpolated collision repairs."""
    from PIL import Image, ImageDraw
    if frames < 2:
        raise ValueError("at least two frames")
    allpoints = np.concatenate([result.positions.reshape(-1,2),result.goals])
    xmin,ymin=allpoints.min(axis=0)-2
    xmax,ymax=allpoints.max(axis=0)+2
    for o in result.obstacles:
        xmin,ymin=min(xmin,o.center[0]-o.radius-1),min(ymin,o.center[1]-o.radius-1)
        xmax,ymax=max(xmax,o.center[0]+o.radius+1),max(ymax,o.center[1]+o.radius+1)
    width,height=1000,700
    scale=min((width-80)/(xmax-xmin),(height-120)/(ymax-ymin))
    def xy(p):
        return (40+(p[0]-xmin)*scale,height-45-(p[1]-ymin)*scale)
    def disk(draw,p,r,fill=None,outline=None):
        x,y=xy(p);r=r*scale
        draw.ellipse((x-r,y-r,x+r,y+r),fill=fill,outline=outline,width=1)
    import colorsys
    colors=[tuple(int(v*220) for v in colorsys.hsv_to_rgb(i/len(result.robot_ids),.8,.9))
            for i in range(len(result.robot_ids))]
    indices=np.unique(np.linspace(0,len(result.times)-1,min(frames,len(result.times))).astype(int))
    images=[]
    for k in indices:
        im=Image.new("RGB",(width,height),"#f5f7fb");draw=ImageDraw.Draw(im)
        draw.text((24,16),f"SWARM / {len(result.robot_ids)} robots / t={result.times[k]:.2f}s",fill="#15233d")
        draw.text((24,34),f"Final outcome: {result.status} | accepted states | dashed targets: planned slots",fill="#15233d")
        draw.text((24,52),"Colored disks: robots   gray: obstacles   rings: final goals   crosses: moving slots",fill="#42516b")
        for o in result.obstacles:
            disk(draw,o.center,o.radius,fill="#8f99aa")
        for i,rid in enumerate(result.robot_ids):
            color=colors[i]
            trail=result.positions[max(0,k-180):k+1,i]
            if len(trail)>1:
                draw.line([xy(p) for p in trail],fill=color,width=2)
            disk(draw,result.goals[i],result.config.robot_radius,outline=color)
            x,y=xy(slot_position(layout,plan.at(float(result.times[k])),rid))
            draw.line((x-3,y,x+3,y),fill="#aeb7c5")
            draw.line((x,y-3,x,y+3),fill="#aeb7c5")
            disk(draw,result.positions[k,i],result.config.robot_radius,fill=color,outline="#ffffff")
        if k==len(result.times)-1 and result.rejected_step is not None:
            for a,b in zip(result.rejected_step.start,result.rejected_step.proposed):
                draw.line([xy(a),xy(b)],fill="red",width=3)
        images.append(im)
    destination=Path(destination)
    destination.parent.mkdir(parents=True,exist_ok=True)
    images[0].save(destination,save_all=True,append_images=images[1:],duration=70,loop=0,optimize=False)


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--scene",choices=SCENES,default="islands")
    p.add_argument("--robots",type=int,choices=[10,20,30],default=20)
    p.add_argument("--controller",choices=["C","D"],default="C")
    p.add_argument("--seed",type=int,default=7)
    p.add_argument("--max-steps",type=int,default=2200)
    p.add_argument("--frames",type=int,default=140)
    p.add_argument("--out",type=Path,required=True)
    p.add_argument("--no-animation",action="store_true")
    args=p.parse_args(argv)
    if args.out.exists():
        raise FileExistsError(args.out)
    if args.frames<2 or args.max_steps<0 or args.seed<0:
        p.error("frames>=2, steps>=0, seed>=0 required")
    layout,plan,start,goals,obstacles=make_scene(args.scene,args.robots,args.seed)
    team=ControllerTeam(args.controller,layout,plan)
    sim=SwarmSimulator(SwarmConfig(max_speed=.8),team)
    sim.reset(start,goals,obstacles)
    result=sim.run(args.max_steps)
    sim.save_history(args.out)
    team.save_records(args.out/"modes.csv",result)
    json_dump(args.out/"showcase.json",dict(scene=args.scene,seed=args.seed,layout=asdict(layout),
        plan=asdict(plan),max_steps=args.max_steps,frames=args.frames,
        note="Illustrative wide formation; not E2; local fields, no global path planner"))
    save_swarm_figure(result,args.out/"trajectory.png")
    if not args.no_animation:
        animate(result,layout,plan,args.out/"motion.gif",args.frames)
    print(args.scene,args.robots,result.status,result.accepted_steps,flush=True)
    return 0


if __name__=="__main__":
    raise SystemExit(main())
