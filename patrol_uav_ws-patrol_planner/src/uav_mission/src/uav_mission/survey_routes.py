"""Configurable fixed-heading survey shapes; no obstacle or detection bypass."""
import math

def survey_route(pattern, x_lanes, y_min, y_max, start_high=False):
    expected={'rectangle':2,'snake2':2,'snake3':3}
    if pattern not in expected or len(x_lanes)!=expected[pattern]:raise ValueError('route pattern/lane count mismatch')
    vals=[*x_lanes,y_min,y_max]
    if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in vals):raise ValueError('invalid survey geometry')
    if y_min>=y_max or any(b<=a for a,b in zip(x_lanes,x_lanes[1:])):raise ValueError('survey lanes must increase')
    if type(start_high) is not bool:raise ValueError('start_high must be bool')
    a,b=(y_max,y_min) if start_high else (y_min,y_max)
    if pattern=='rectangle':return [[x_lanes[0],a],[x_lanes[1],a],[x_lanes[1],b],[x_lanes[0],b],[x_lanes[0],a]]
    out=[]
    for i,x in enumerate(x_lanes):out.extend([[x,a if i%2==0 else b],[x,b if i%2==0 else a]])
    return out
