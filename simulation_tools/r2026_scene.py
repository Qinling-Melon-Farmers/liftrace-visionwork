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


def scene_layout(seed, door_seed=None, obstacle_seed=None, pattern=None):
    if seed < 0:
        raise ValueError('scene seed must be nonnegative; zero preserves the fixture')
    door_rng = random.Random(seed if door_seed is None else door_seed)
    tree_rng = random.Random(seed if obstacle_seed is None else obstacle_seed)
    pattern = pattern or ('LR' if seed == 0 else door_rng.choice(PATTERNS))
    if pattern not in PATTERNS:
        raise ValueError('two doors have only left/right openings: LL, LR, RL, RR')
    doors = []
    for index, (x, side) in enumerate(zip((-1.6, 1.6), pattern)):
        # Along travel direction +X, left is +Y. Corridor interior [7.6,9.1].
        lo, hi = (8.3, 9.1) if side == 'L' else (7.6, 8.4)
        doors.append(dict(name=('Wall_20','Wall_22')[index], x=x, side=side,
                          gap_min_y=lo, gap_max_y=hi, clear_width=.8,
                          wall_y=7.95 if side == 'L' else 8.75, wall_length=.7))
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
    return dict(schema_version=1,scene_seed=seed,door_seed=door_seed,
                obstacle_seed=obstacle_seed,door_pattern=pattern,doors=doors,trees=trees,
                frame='world / nominal competition frame',corridor_width=1.5,
                target_seed='independent field_seed at spawner',
                qualification='Geometry fixture only; randomized-door SITL not yet validated')
