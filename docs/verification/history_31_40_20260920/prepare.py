from pathlib import Path
import importlib.util,json,copy,math,shutil,xml.etree.ElementTree as ET
import yaml,numpy as np
r=Path('/home/xhj/liftrace-worktrees/r2026-high-view-search');d=r/'docs/verification/history_31_40_20260920';d.mkdir(parents=True,exist_ok=True)
spec=importlib.util.spec_from_file_location('rotate',Path('/home/xhj/liftrace-sim/tools/rotate_start_frame.py'));rot=importlib.util.module_from_spec(spec);spec.loader.exec_module(rot)
base=r/'docs/verification/fov_landing_inner_20260919/seed_2672'
old31=r/'docs/verification/high_fast_five_20260915';old36=r/'docs/verification/high_fast_new_seeds_20260915'
hist=json.loads((old31/'runs.json').read_text())
for v in json.loads((old36/'matrix.json').read_text())['results']:hist.append(dict(seed=v['seed'],label='fast_high_new',run=v['run'],world=str(old36/f'seed_{v["seed"]}/field.world'),source='92bd3eed'))
cases=[];validation=[]
for seed in range(31,41):
    source=(old31 if seed<36 else old36)/f'seed_{seed}';out=d/f'seed_{seed}';out.mkdir(exist_ok=True)
    historical=next(v for v in hist if v['seed']==seed and v['label'].startswith('fast_high'))
    truth=yaml.safe_load((Path(historical['run'])/'random_field_truth.yaml').read_text())
    rot.rotate_world(source/'field.world',out/'field.world')
    tree=ET.parse(out/'field.world');field=next(v for v in tree.iter('model') if v.get('name')=='toudi2')
    # Preserve every horizontal fixture; only outer wall height follows the latest 4m observer scene.
    for link in field.findall('link'):
        if link.get('name') not in ('Wall_1','Wall_9','Wall_11','Wall_12'):continue
        pose=list(map(float,link.findtext('pose').split()));pose[2]=2.;link.find('pose').text=' '.join(map(str,pose))
        for size in link.findall('.//box/size'):
            dims=list(map(float,size.text.split()));dims[2]=4.;size.text=' '.join(map(str,dims))
    tree.write(out/'field.world',encoding='utf-8',xml_declaration=True)
    cfg=yaml.safe_load((source/'field_config.yaml').read_text())
    for key in ('field','search_region'):rot.bound_dict(cfg[key])
    cfg['static_exclusion_boxes']=[rot.bounds(v) for v in cfg['static_exclusion_boxes']]
    for obj in cfg['static_exclusions']:obj['world_x'],obj['world_y']=obj['world_y'],-obj['world_x']
    cfg['ignore_model_names']+=',presentation_overview_camera,presentation_follow_camera'
    cfg['spawn']['frozen_layout']=[dict(**{'class':t['class']},x=t['y'],y=-t['x'],yaw=t['yaw']-math.pi/2) for t in truth['targets']]
    (out/'field_config.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False))
    gate=rot.rotate_gate(yaml.safe_load((source/'gate_geometry.yaml').read_text()))
    g=gate['post_delivery_gate'];g['low_height_region']['max_height']=1.2
    for door in g['doors']:door['z_max']=1.2
    g['landing_observation_region']=yaml.safe_load((base/'fast_gate.yaml').read_text())['post_delivery_gate']['landing_observation_region']
    (out/'fast_gate.yaml').write_text(yaml.safe_dump(gate,sort_keys=False))
    for name in ('fast_runtime.yaml','frame_overrides.yaml','repair_overrides.yaml','presentation.yaml'):shutil.copyfile(base/name,out/name)
    # Verify full paired layout geometry, rather than assuming equality from seed numbers.
    oldfield=next(v for v in ET.parse(source/'field.world').iter('model') if v.get('name')=='toudi2')
    checks=[]
    for oldlink in oldfield.findall('link'):
        newlink=next(v for v in field.findall('link') if v.get('name')==oldlink.get('name'))
        oldp=list(map(float,oldlink.findtext('pose').split()));newp=list(map(float,newlink.findtext('pose').split()))
        np.testing.assert_allclose(newp[:2],rot.xy(oldp)[:2],atol=1e-9)
        oldsize=list(map(float,oldlink.findtext('collision/geometry/box/size').split()));newsize=list(map(float,newlink.findtext('collision/geometry/box/size').split()))
        np.testing.assert_allclose(newsize[:2],oldsize[1::-1],atol=1e-9);checks.append(oldlink.get('name'))
    for oldobj,newobj in zip(oldfield.findall('model'),field.findall('model')):
        op=list(map(float,oldobj.findtext('pose').split()));np_=list(map(float,newobj.findtext('pose').split()));np.testing.assert_allclose(np_[:2],rot.xy(op)[:2],atol=1e-9);np.testing.assert_allclose(np_[5],op[5]-math.pi/2,atol=1e-9)
    assert len(cfg['spawn']['frozen_layout'])==5
    for t,new in zip(truth['targets'],cfg['spawn']['frozen_layout']):np.testing.assert_allclose([new['y']*-1,new['x']],[t['x'],t['y']],atol=1e-9)
    validation.append(dict(seed=seed,paired_horizontal_layout=True,walls_checked=checks,trees_checked=len(field.findall('model')),targets_checked=5,source_run=historical['run'],changes_beyond_rotation=['outer walls raised to 4m','latest 2.6m/fast corridor/FOV/landing repair profile']))
    cases.append(dict(seed=seed,scene_dir=str(out),historical_run=historical['run']))
(d/'cases.json').write_text(json.dumps(cases,indent=2));(d/'historical_runs.json').write_text(json.dumps(hist,indent=2));(d/'layout_validation.json').write_text(json.dumps(validation,indent=2))
shutil.copyfile('/home/xhj/prepare_history_ten.py',d/'prepare.py')
print('Prepared and geometrically verified ten rotated frozen layouts')
