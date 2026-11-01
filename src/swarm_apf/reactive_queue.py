"""Experimental local queue E. Shared heading/anchor, no map or timed stages."""
import numpy as np


class ReactiveQueueController:
    version="E-LOCAL-QUEUE-EXPERIMENT-1"
    def __init__(self, robot_id, group_goal=(12.,0.), heading=(1.,0.)):
        self.robot_id=robot_id
        self.goal=np.array(group_goal,dtype=float)
        self.e=np.array(heading,dtype=float)
        if self.goal.shape!=(2,) or self.e.shape!=(2,) or not np.isfinite(self.goal).all() or not np.isfinite(self.e).all() or np.linalg.norm(self.e)<=1e-12:
            raise ValueError("finite goal and nonzero heading required")
        self.e/=np.linalg.norm(self.e)
        self.n=np.array([-self.e[1],self.e[0]])
        self.reset()

    def reset(self):
        self.encountered=False
        self.cleared=False

    def metadata(self):
        return dict(version=self.version,group_goal=self.goal.tolist(),heading=self.e.tolist(),
                    policy="local wall/neighbor velocity constraints; shared goal axis",
                    parameters=dict(goal_gain=.8,lateral_speed=.5,wall_gain=1.5,wall_margin=.04,
                                    neighbor_gain=.6,neighbor_margin=.08,queue_half_width=.8,
                                    queue_lookahead=1.1,queue_gap=.55,priority_epsilon=.05,
                                    projection_sweeps=24),
                    limitations="heuristic projection, not certified collision avoidance")

    def command(self,obs):
        if obs.robot_id != self.robot_id:
            raise ValueError("wrong robot ID")
        if obs.obstacles:
            self.encountered=True
        elif self.encountered:
            self.cleared=True
        position=np.array(obs.self_position)
        target=np.array(obs.goal) if self.cleared or not self.encountered else self.goal
        delta=target-position
        # Smooth velocity rather than normalized constant-speed steering.
        desired=.8*delta
        if self.encountered and not self.cleared:
            desired=self.e*obs.max_speed+self.n*np.clip(np.dot(delta,self.n),-.5,.5)
        norm=np.linalg.norm(desired)
        if norm>obs.max_speed:
            desired*=obs.max_speed/norm
        constraints=[]
        for wall in obs.obstacles:
            z=np.array(wall.relative_center);distance=np.linalg.norm(z)
            if distance<=1e-12:
                return np.zeros(2)
            outward=-z/distance
            gap=distance-wall.radius-obs.robot_radius
            constraints.append((outward,-1.5*max(0.,gap-.04)))
        for neighbor in obs.neighbors:
            z=np.array(neighbor.relative_position);distance=np.linalg.norm(z)
            if distance<=1e-12:
                return np.zeros(2)
            outward=-z/distance
            gap=distance-2*obs.robot_radius
            # Conservative local barrier ignores future neighbor command.
            constraints.append((outward,-.6*max(0.,gap-.08)))
            forward=np.dot(z,self.e);side=abs(np.dot(z,self.n))
            priority=(forward>.05 or (abs(forward)<=.05 and neighbor.robot_id<self.robot_id))
            if priority and side<.8 and -.05<forward<1.1:
                desired-=self.e*max(0.,np.dot(desired,self.e)-max(0.,forward-.55)*.6)
        velocity=desired.copy()
        # Alternating projection onto local half-spaces and speed ball.
        for _ in range(24):
            for normal,lower in constraints:
                violation=lower-np.dot(normal,velocity)
                if violation>0:
                    velocity+=violation*normal
            speed=np.linalg.norm(velocity)
            if speed>obs.max_speed:
                velocity*=obs.max_speed/speed
        return velocity


class ReactiveQueueTeam:
    def __init__(self,ids,group_goal):
        self.agents={i:ReactiveQueueController(i,group_goal) for i in ids}
    def command(self,obs):
        return self.agents[obs.robot_id].command(obs)
    def reset(self):
        for agent in self.agents.values():
            agent.reset()
    def metadata(self):
        return {str(i):a.metadata() for i,a in self.agents.items()}
    def save_records(self,*args):
        pass
