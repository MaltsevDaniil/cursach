"""Reactive funnel experiment: fixed final slots, no timed motion stages."""
import argparse
from pathlib import Path
import numpy as np
from .geometry import Obstacle
from .formation import FormationLayout, FormationPlan, slot_position
from .controller_team import ControllerTeam
from .models import SwarmConfig
from .swarm import SwarmSimulator
from .showcase import animate
from .swarm_visualization import save_swarm_figure
from .experiment_runner import json_dump


def scene(n=5):
    if n not in (5,10):
        raise ValueError("N must be 5 or 10")
    layout=FormationLayout(tuple(range(n)),"line",1.4)
    # Constant final reference: time does not move the targets.
    plan=FormationPlan((12,0),(12,0))
    goals=np.array([slot_position(layout,plan.at(0),i) for i in layout.robot_ids])
    initial=np.array([[-10-(i//5)*1.5,(i%5-2)*1.4] for i in range(n)])
    circles=[]
    for sign in (-1,1):
        for x in np.linspace(-7,0,29):
            circles.append(Obstacle((float(x),sign*(.65-x*.65)),.3))
        for x in np.linspace(.25,3,12):
            circles.append(Obstacle((float(x),sign*.65),.3))
    return layout,plan,initial,goals,tuple(circles)


def audit(result):
    crossed=[]
    for i in range(len(result.robot_ids)):
        p=result.positions[:,i]
        # Crossing the exit plane through the central opening, not around walls.
        valid=False
        for a,b in zip(p[:-1],p[1:]):
            if a[0] < 3 <= b[0]:
                y=a[1]+(b[1]-a[1])*(3-a[0])/(b[0]-a[0])
                valid |= abs(y) < .15+1e-9
        crossed.append(bool(valid))
    return dict(per_robot_crossed_throat=crossed,all_crossed_throat=all(crossed),
                arrived=int(np.sum(np.linalg.norm(result.positions[-1]-result.goals,axis=1)<=result.config.goal_tolerance)),
                status=result.status,throat_surface_width=.7,robot_diameter=.4,
                note="finite walls permit external bypass; goal success alone is not funnel success")


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--out",type=Path,required=True)
    p.add_argument("--robots",type=int,choices=[5,10],default=5)
    p.add_argument("--controller",choices=["C","D"],default="D")
    p.add_argument("--steps",type=int,default=1800)
    p.add_argument("--no-animation",action="store_true")
    args=p.parse_args(argv)
    if args.out.exists():
        raise FileExistsError(args.out)
    if args.steps<0:
        p.error("steps must be nonnegative")
    layout,plan,start,goals,obstacles=scene(args.robots)
    sim=SwarmSimulator(SwarmConfig(dt=.05,max_speed=.6),ControllerTeam(args.controller,layout,plan))
    sim.reset(start,goals,obstacles)
    result=sim.run(args.steps)
    sim.save_history(args.out)
    sim.controller.save_records(args.out/"modes.csv",result)
    json_dump(args.out/"funnel_audit.json",audit(result))
    save_swarm_figure(result,args.out/"trajectory.png")
    if not args.no_animation:
        animate(result,layout,plan,args.out/"motion.gif",100)
    print(args.controller,audit(result),flush=True)
    return 0


if __name__=="__main__":
    raise SystemExit(main())
