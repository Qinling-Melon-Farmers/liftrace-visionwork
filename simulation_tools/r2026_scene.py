"""Competition scene geometry; never supplies unknown door positions to flight control."""
import math
import random

NOMINAL_TREES = [(-1.8, 2.0), (1.8, 2.7), (-1.5, 5.2), (1.7, 5.7)]
PATTERNS = ('LL', 'LR', 'RL', 'RR')


def rectangle_vertices(x,y,sx,sy,yaw=0.0):
    c,s=math.cos(yaw),math.sin(yaw)
    return [(x+dx*c-dy*s,y+dx*s+dy*c)
            for dx,dy in [(-sx/2,-sy/2),(sx/2,-sy/2),(sx/2,sy/2),(-sx/2,sy/2)]]


def polygons_overlap(a,b):
    """Separating-axis test for convex footprints, with touching not penetration."""
    for poly in (a,b):
        for p,q in zip(poly,poly[1:]+poly[:1]):
            axis=(-(q[1]-p[1]),q[0]-p[0])
            aa=[x*axis[0]+y*axis[1] for x,y in a];bb=[x*axis[0]+y*axis[1] for x,y in b]
            if max(aa)<=min(bb)+1e-10 or max(bb)<=min(aa)+1e-10:return False
    return True


def door_segments(door):
    """Complement of the exact 0.80m opening; zero-length edge slabs omitted."""
    return [dict(side=side,wall_y=(lo+hi)/2,wall_length=hi-lo)
            for side,lo,hi in [('south',7.6,door['gap_min_y']),
                               ('north',door['gap_max_y'],9.1)] if hi-lo>1e-9]


def scene_layout(seed, door_seed=None, obstacle_seed=None, pattern=None, *,
                 door_mode='left_right', door_centers=None, outer_wall_height=None):
    if seed < 0:
        raise ValueError('scene seed must be nonnegative; zero preserves the fixture')
    door_rng = random.Random(seed if door_seed is None else door_seed)
    tree_rng = random.Random(seed if obstacle_seed is None else obstacle_seed)
    if door_mode not in ('left_right','continuous'):
        raise ValueError('unknown door mode')
    if door_mode=='continuous' and pattern is not None:
        raise ValueError('continuous opening cannot also specify an L/R pattern')
    if door_centers is not None and (door_mode!='continuous' or len(door_centers)!=2
            or any(not math.isfinite(c) or not 8.0<=c<=8.7 for c in door_centers)):
        raise ValueError('two door centers must be within [8.0,8.7]')
    if outer_wall_height is not None and (not math.isfinite(outer_wall_height) or not 1.5<=outer_wall_height<=4.):
        raise ValueError('outer wall height must be in [1.5,4.0]')
    pattern = pattern or ('LR' if seed == 0 else door_rng.choice(PATTERNS)) if door_mode=='left_right' else 'CONTINUOUS'
    if door_mode=='left_right' and pattern not in PATTERNS:
        raise ValueError('two doors have only left/right openings: LL, LR, RL, RR')
    doors = []
    for index, (x, side) in enumerate(zip((-1.6, 1.6), pattern if door_mode=='left_right' else ('continuous','continuous'))):
        # Along travel direction +X, left is +Y. Corridor interior [7.6,9.1].
        if door_mode=='continuous':
            center=door_rng.uniform(8.0,8.7) if door_centers is None else door_centers[index]
            lo,hi=center-.4,center+.4
        else:lo, hi = (8.3, 9.1) if side == 'L' else (7.6, 8.4)
        door=dict(name=('Wall_20','Wall_22')[index], x=x, side=side,
                          gap_min_y=lo, gap_max_y=hi, clear_width=.8,
                          wall_y=7.95 if side == 'L' else 8.75, wall_length=.7)
        if door_mode=='continuous':
            door.pop('wall_y');door.pop('wall_length')
            door.update(center_y=(lo+hi)/2,segments=door_segments(door))
        doors.append(door)
    trees = []
    if seed == 0:
        trees = [dict(x=x, y=y, yaw=0.0) for x,y in NOMINAL_TREES]
    else:
        for _ in range(10000):
            x,y=tree_rng.uniform(-4.05,4.05),tree_rng.uniform(.25,6.65)
            if math.hypot(x,y)<1.15 or (x < -3.1 and y > 5.65):
                continue  # keep the official starting pad and corridor entrance clear
            if any(math.hypot(x-t['x'],y-t['y'])<1.2 for t in trees):
                continue
            trees.append(dict(x=x,y=y,yaw=tree_rng.uniform(-math.pi,math.pi)))
            if len(trees)==4:
                break
        if len(trees)!=4:
            raise RuntimeError('Could not place four separated tree/box groups')
    result = dict(schema_version=1,scene_seed=seed,door_seed=door_seed,
                obstacle_seed=obstacle_seed,door_pattern=pattern,doors=doors,trees=trees,
                frame='world / nominal competition frame',corridor_width=1.5,
                target_seed='independent field_seed at spawner',
                qualification='Geometry fixture only; randomized-door SITL not yet validated')
    if door_mode!='left_right':result.update(schema_version=2,door_mode=door_mode)
    if outer_wall_height is not None:result['outer_wall_height_m']=outer_wall_height
    return result
