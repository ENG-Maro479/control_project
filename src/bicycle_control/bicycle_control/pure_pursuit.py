"""
High-Level Lateral Steering Controller: Geometric Pure Pursuit.
Calculates steering curvature from lookahead arc geometry.
"""

import math  # noqa: F401
import numpy as np  # noqa: F401


class PurePursuitController:
    """Adaptive Pure Pursuit lateral controller."""

    def __init__(self, wheelbase=1.25, kv=0.25, l_min=0.8, l_max=2.5,
                 max_steer_rad=math.radians(35.0)):
        self.L = wheelbase
        self.kv = kv
        self.l_min = l_min
        self.l_max = l_max
        self.max_steer_rad = max_steer_rad

    def compute_lookahead(self, v):
        """Adaptive lookahead distance: Ld = clip(kv * v + l_min, l_min, l_max)."""
        return float(np.clip(self.kv*v+self.l_min,self.l_min,self.l_max))
    

    def find_target_waypoint(self, x, y, path_points, lookahead):
        """Searches along path for the target waypoint at lookahead distance."""
        n=len(path_points)
        nearest=0
        best=float('inf')
        for i,p in enumerate(path_points):
            d=(p[0]-x)**2+(p[1]-y)**2
            if d<best :
                best=d
                nearest=i
        idx=nearest
        for _ in range(n) :
            p=path_points[idx]
            if math.hypot(p[0]-x,p[1]-y)>=lookahead:
                return idx,p
            idx=(idx+1)%n
        return idx,path_points[idx]


    def compute_steering(self, x, y, yaw, target_pt, lookahead):
        """Computes steering angle in radians using Pure Pursuit geometry."""
        dx=target_pt[0]-x
        dy=target_pt[1]-y
        x_local=math.cos(yaw)*dx+math.sin(yaw)*dy
        y_local=-math.sin(yaw)*dx+math.cos(yaw)*dy
        alpha=math.atan2(y_local,x_local)
        ld=max(lookahead,1e-3)
        delta=math.atan2(2.0*self.L*math.sin(alpha),ld)
        return float(np.clip(delta,-self.max_steer_rad,self.max_steer_rad))
    
