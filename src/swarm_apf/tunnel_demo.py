"""Scheduled line -> file -> tunnel -> line demo. Not autonomous D recovery."""
import argparse
from dataclasses import dataclass
from pathlib import Path
import numpy as np
from .models import SwarmConfig
from .geometry import Obstacle
from .formation import FormationLayout, FormationPlan
from .swarm import SwarmSimulator
from .showcase import animate
from .swarm_visualization import save_swarm_figure
from .experiment_runner import json_dump


@dataclass
class TunnelSchedule:
    n: int = 10
    speed: float = .45

    def __post_init__(self):
        if self.n not in (5,10,15) or not np.isfinite(self.speed) or self.speed <= 0:
            raise ValueError("N=5/10/15 and positive finite speed required")
        i=np.arange(self.n)
        y=(i-(self.n-1)/2)*1.4
        line=np.column_stack([np.full(self.n,-8.),y])
        stagger=np.column_stack([-8.-i*1.1,y])
        file=stagger.copy();file[:,1]=0
        passed=file+[(8+4+(self.n-1)*1.1)-(-8),0]
        spread=passed.copy();spread[:,1]=y
        final=spread.copy();final[:,0]=passed[0,0]
        self.shapes=[line,stagger,file,passed,spread,final]
        self.ends=[0.]
        for a,b in zip(self.shapes,self.shapes[1:]):
            self.ends.append(self.ends[-1]+float(np.linalg.norm(b-a,axis=1).max())/self.speed+1.)
        self.labels=["stagger","compress","cross tunnel","restore width","align formation"]

    def slot(self,t,ident):
        if ident not in range(self.n) or not np.isfinite(t) or t<0:
            raise ValueError("invalid time/ID")
        for k,end in enumerate(self.ends[1:]):
            if t<end:
                fraction=(t-self.ends[k])/(end-self.ends[k])
                return self.shapes[k][ident]*(1-fraction)+self.shapes[k+1][ident]*fraction
        return self.shapes[-1][ident].copy()

    def metadata(self):
        return dict(version="TUNNEL-SCHEDULE-1",n=self.n,speed=self.speed,
                    stage_ends=self.ends,stages=self.labels,
                    scope="pre-agreed global timetable, no autonomous passage detection")


class ScheduledController:
    def __init__(self,schedule):
        self.schedule=schedule

    def command(self,obs):
        delta=self.schedule.slot(obs.time,obs.robot_id)-obs.self_position
        distance=float(np.linalg.norm(delta))
        return delta/distance*min(obs.max_speed,distance/obs.dt) if distance>1e-12 else np.zeros(2)

    def metadata(self):
        return self.schedule.metadata()


def run(n=10):
    schedule=TunnelSchedule(n)
    # Overlapping wall disks seal the sides, one shared 0.70-wide passage.
    obstacles=tuple(Obstacle((float(x),y),.45) for x in np.linspace(0,8,17) for y in (-.8,.8))
    cfg=SwarmConfig(dt=.1,robot_radius=.2,max_speed=.7,goal_tolerance=.12)
    sim=SwarmSimulator(cfg,ScheduledController(schedule))
    sim.reset(schedule.shapes[0],schedule.shapes[-1],obstacles)
    result=sim.run(int(np.ceil((schedule.ends[-1]+5)/cfg.dt)))
    return schedule,sim,result


def audit(result,schedule):
    passage=[]
    for i in range(schedule.n):
        track=result.positions[:,i]
        inside=(track[:,0]>=0)&(track[:,0]<=8)
        passage.append(bool(inside.any() and np.all(np.abs(track[inside,1]) <= .15)))
    offsets=result.positions[-1]-result.positions[-1].mean(axis=0)
    desired=schedule.shapes[0]-schedule.shapes[0].mean(axis=0)
    return dict(all_robots_used_same_tunnel=all(passage),per_robot_passage=passage,
                final_shape_rms=float(np.sqrt(np.mean(np.sum((offsets-desired)**2,axis=1)))),
                tunnel_surface_width=.7,robot_diameter=.4,
                controller="scheduled slot tracking; not C/D",stage_ends=schedule.ends)


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--robots",type=int,choices=[5,10,15],default=10)
    p.add_argument("--out",type=Path,required=True)
    p.add_argument("--frames",type=int,default=120)
    args=p.parse_args(argv)
    if args.out.exists():
        raise FileExistsError(args.out)
    if args.frames<2:
        p.error("frames>=2")
    schedule,sim,result=run(args.robots)
    sim.save_history(args.out)
    json_dump(args.out/"tunnel_audit.json",audit(result,schedule))
    save_swarm_figure(result,args.out/"trajectory.png")
    animate(result,FormationLayout(tuple(range(args.robots))),FormationPlan((0,0),(1,0)),
            args.out/"motion.gif",args.frames,slot_at=schedule.slot)
    print(result.status,result.accepted_steps,audit(result,schedule),flush=True)
    return 0


if __name__=="__main__":
    raise SystemExit(main())
