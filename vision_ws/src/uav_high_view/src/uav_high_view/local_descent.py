"""Bounded local descent proposals ranked on sensed occupancy, never truth.

This only proposes motion endpoints. The existing 3-D flight planner still
checks and executes both the high translation and the descent trajectory.
"""
from itertools import permutations
import math


def propose_column(grid,current_xy,now,radius=1.5,max_candidates=25,max_age=2.):
    """Descent endpoint independent of the later tour; 3-D planner still approves."""
    if not 0<radius<=2. or not 1<=max_candidates<=25:raise ValueError('descent search bounds')
    if grid.stamp is None or not 0<=now-grid.stamp<=max_age:return None
    start=grid.cell(current_xy)
    if start is None:return None
    if not grid.blocked[start]:
        return dict(xy=tuple(current_xy),classes=(),cost_m=0.,kind='CURRENT_COLUMN',candidates=1)
    candidates=[];steps=math.ceil(radius/grid.resolution)
    for dx in range(-steps,steps+1,2):
        for dy in range(-steps,steps+1,2):
            cell=(start[0]+dx,start[1]+dy)
            if not 0<=cell[0]<grid.nx or not 0<=cell[1]<grid.ny or grid.blocked[cell]:continue
            xy=(grid.bounds[0]+(cell[0]+.5)*grid.resolution,grid.bounds[2]+(cell[1]+.5)*grid.resolution)
            distance=math.dist(current_xy,xy)
            if distance<=radius:candidates.append((distance,xy))
    candidates=sorted(candidates)[:max_candidates]
    if not candidates:return None
    distance,xy=candidates[0]
    return dict(xy=xy,classes=(),cost_m=distance,kind='NEARBY_COLUMN',candidates=len(candidates))


def propose(grid, current_xy, targets, exit_xy, now, radius=1.5, max_candidates=25,
            max_age=2.):
    if not 0<radius<=2. or not 1<=max_candidates<=25:raise ValueError('descent search bounds')
    if grid.stamp is None or not 0<=now-grid.stamp<=max_age or not 1<=len(targets)<=3:return None
    start=grid.cell(current_xy)
    if start is None:return None
    # If the sensed vertical occupancy column is clear, optimize directly
    # from the interruption point; no compulsory excursion to the home column.
    if not grid.blocked[start]:
        order=grid.order(current_xy,targets,exit_xy,now,max_age)
        if order is not None:
            return dict(xy=tuple(current_xy),classes=order[1],cost_m=order[0],kind='CURRENT_COLUMN',candidates=1)
    names=sorted(targets)
    # Reuse three reverse Dijkstra tables for every nearby proposal.
    tables={n:grid.distances(targets[n]) for n in names}
    tails={}
    for order in permutations(names):
        tails[order]=sum(tables[a].get(grid.cell(targets[b]),math.inf) for a,b in zip(order,order[1:]))+tables[order[-1]].get(grid.cell(exit_xy),math.inf)
    candidates=[]
    steps=math.ceil(radius/grid.resolution)
    for dx in range(-steps,steps+1,2):
        for dy in range(-steps,steps+1,2):
            cell=(start[0]+dx,start[1]+dy)
            if not 0<=cell[0]<grid.nx or not 0<=cell[1]<grid.ny or grid.blocked[cell]:continue
            xy=(grid.bounds[0]+(cell[0]+.5)*grid.resolution,grid.bounds[2]+(cell[1]+.5)*grid.resolution)
            distance=math.dist(current_xy,xy)
            if distance<=radius:candidates.append((distance,cell,xy))
    candidates=sorted(candidates)[:max_candidates]
    ranked=[]
    for distance,cell,xy in candidates:
        for order,tail in tails.items():
            cost=distance+tables[order[0]].get(cell,math.inf)+tail
            if math.isfinite(cost):ranked.append((cost,order,xy))
    if not ranked:return None
    cost,order,xy=min(ranked)
    return dict(xy=xy,classes=order,cost_m=cost,kind='NEARBY_COLUMN',candidates=len(candidates))
