"""Coarse sensed-occupancy route costs; not flight collision authorization."""
import heapq
from itertools import permutations
import math
import numpy as np


class GridCost:
    def __init__(self,bounds=(-4.8,4.8,-.5,7.4),resolution=.15,inflation=.35):
        if len(bounds)!=4 or not all(math.isfinite(v) for v in tuple(bounds)+(resolution,inflation)) or resolution<=0 or inflation<0 or bounds[1]<=bounds[0] or bounds[3]<=bounds[2]:raise ValueError('grid bounds')
        self.bounds=tuple(bounds);self.resolution=resolution;self.inflation=inflation
        self.nx=math.ceil((bounds[1]-bounds[0])/resolution)
        self.ny=math.ceil((bounds[3]-bounds[2])/resolution)
        if self.nx*self.ny>10000 or resolution<=0 or inflation<0:raise ValueError('grid bounds')
        self.blocked=np.zeros((self.nx,self.ny),dtype=bool)
        self.stamp=None

    def cell(self,xy):
        x=int(math.floor((xy[0]-self.bounds[0])/self.resolution));y=int(math.floor((xy[1]-self.bounds[2])/self.resolution))
        if not 0<=x<self.nx or not 0<=y<self.ny:return None
        return x,y

    def update(self,points,stamp,min_z,max_z):
        points=np.asarray(points)
        if len(points)>250000 or points.ndim!=2 or points.shape[1]!=3:raise ValueError('point cap/layout')
        if len(points)==0:self.stamp=None;return
        valid=np.isfinite(points).all(axis=1)&(points[:,2]>=min_z)&(points[:,2]<=max_z)
        points=points[valid]
        ij=np.floor((points[:,:2]-[self.bounds[0],self.bounds[2]])/self.resolution).astype(int)
        ij=ij[(ij[:,0]>=0)&(ij[:,0]<self.nx)&(ij[:,1]>=0)&(ij[:,1]<self.ny)]
        base=np.zeros_like(self.blocked);base[ij[:,0],ij[:,1]]=True
        inflated=base.copy();cells=math.ceil(self.inflation/self.resolution)
        for dx in range(-cells,cells+1):
            for dy in range(-cells,cells+1):
                if math.hypot(dx,dy)*self.resolution>self.inflation:continue
                x0,x1=max(0,dx),min(self.nx,self.nx+dx);y0,y1=max(0,dy),min(self.ny,self.ny+dy)
                inflated[x0:x1,y0:y1]|=base[x0-dx:x1-dx,y0-dy:y1-dy]
        self.blocked=inflated;self.stamp=stamp

    def distances(self,source):
        start=self.cell(source)
        if start is None or self.blocked[start]:return {}
        distances={start:0.};heap=[(0.,start)]
        while heap:
            cost,p=heapq.heappop(heap)
            if cost!=distances[p]:continue
            for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                q=p[0]+dx,p[1]+dy
                if not 0<=q[0]<self.nx or not 0<=q[1]<self.ny or self.blocked[q]:continue
                if dx and dy and (self.blocked[p[0]+dx,p[1]] or self.blocked[p[0],p[1]+dy]):continue
                nc=cost+self.resolution*math.hypot(dx,dy)
                if nc<distances.get(q,float('inf')):
                    distances[q]=nc;heapq.heappush(heap,(nc,q))
        return distances

    def order(self,start,targets,exit_xy,now,max_age=2.):
        if self.stamp is None or not 0<=now-self.stamp<=max_age or not 1<=len(targets)<=3:return None
        names=['@start']+sorted(targets)+['@exit'];points=[start]+[targets[n] for n in sorted(targets)]+[exit_xy]
        costs={}
        for name,point in zip(names,points):
            distance=self.distances(point)
            for other,q in zip(names,points):costs[name,other]=distance.get(self.cell(q),float('inf'))
        candidates=[]
        for order in permutations(sorted(targets)):
            nodes=('@start',)+order+('@exit',)
            cost=sum(costs[a,b] for a,b in zip(nodes,nodes[1:]))
            if math.isfinite(cost):candidates.append((cost,order))
        return min(candidates) if candidates else None
