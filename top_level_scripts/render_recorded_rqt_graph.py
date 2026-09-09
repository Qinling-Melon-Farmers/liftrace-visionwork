#!/usr/bin/env python3
"""Render a recorded ROS Master graph with the installed rqt_graph backend."""
import argparse, json, subprocess, sys
from pathlib import Path
from types import SimpleNamespace
from rosgraph.impl.graph import Edge
from rqt_graph.dotcode import NODE_NODE_GRAPH, RosGraphDotcodeGenerator
from qt_dotgraph.pydotfactory import PydotFactory

def render(snapshot, out, project):
    sys.path.insert(0,str(project/'vision_ws/src/uav_vision_eval/scripts'))
    from ros_topology_snapshot import CORE_TOPICS
    data=json.loads(snapshot.read_text());pub=data['publishers'];sub=data['subscribers']
    nodes=set(n for mapping in [pub,sub,data.get('services',{})] for peers in mapping.values() for n in peers)
    edges=[Edge(a,b,t) for t, peers in pub.items() for a in peers for b in sub.get(t,[])]
    # These are actual simulator, evaluation, logging or simulation-start nodes.
    # Hardware sensor/servo providers were not registered and are not invented.
    excluded={'/gazebo','/gazebo_contact_monitor','/planner_anchor_spawner',
              '/random_field_spawner','/navigation_vcl06_assertion',
              '/competition_key_recorder','/mock_raw_servo_server','/rosout',
              '/uav_arming_node','/navigation_mission_start_gate','/visual_delivery_audit',
              '/camera_video_recorder','/overview_video_recorder'}
    core_topics=set(CORE_TOPICS)|{'/Odometry','/livox/lidar','/livox/imu',
                                 '/cloud_registered_body','/mavros/vision_pose/pose'}
    out.mkdir(parents=True,exist_ok=True)
    try:
        source_snapshot=str(snapshot.resolve().relative_to(project.resolve()))
    except ValueError:
        source_snapshot=str(snapshot)
    # Keep Gazebo as the actual sensor publisher in the core view. The flight
    # view is a runtime-node subset of SITL, not an invented hardware graph.
    core_excluded=excluded-{'/gazebo'}
    summary={'source_snapshot':source_snapshot,'backend':'rqt_graph.RosGraphDotcodeGenerator',
             'mode':NODE_NODE_GRAPH,'recorded_state_only':True,
             'services_are_not_topic_edges':True,
             'core_excluded_nodes':sorted(nodes&core_excluded),
             'flight_excluded_nodes':sorted(nodes&excluded),'views':{}}
    for name in ('full','core','flight'):
        chosen=nodes-excluded if name=='flight' else nodes-core_excluded if name=='core' else nodes
        selected=[e for e in edges if e.start in chosen and e.end in chosen
                  and (name!='core' or e.label in core_topics)
                  and (name!='flight' or e.label not in ('/clock','/rosout','/rosout_agg'))]
        if name=='core':chosen=set(n for e in selected for n in (e.start,e.end))
        selected=sorted(selected,key=lambda e:(e.start,e.end,e.label))
        graph=SimpleNamespace(nn_nodes=sorted(chosen),nn_edges=selected,bad_nodes={})
        generator=RosGraphDotcodeGenerator.__new__(RosGraphDotcodeGenerator)
        dot=generator.generate_dotcode(rosgraphinst=graph,ns_filter='/',topic_filter='/',
            graph_mode=NODE_NODE_GRAPH,dotcode_factory=PydotFactory(),
            hide_single_connection_topics=False,hide_dead_end_topics=False,
            cluster_namespaces_level=0,accumulate_actions=False,orientation='TB',
            rank='same',ranksep=.5,rankdir='TB',simplify=False,quiet=False,
            unreachable=False,hide_tf_nodes=False,group_tf_nodes=False,
            group_image_nodes=False,hide_dynamic_reconfigure=False)
        path=out/('rqt_graph_nodes_only_'+name+'.dot');path.write_text(dot)
        subprocess.run(['dot','-Tsvg',str(path),'-o',str(path.with_suffix('.svg'))],check=True)
        for raster_format in ('png','jpg'):
            subprocess.run(['dot','-T'+raster_format,'-Gdpi=144','-Gbgcolor=white',
                            str(path),'-o',str(path.with_suffix('.'+raster_format))],check=True)
        summary['views'][name]={'nodes':len(chosen),'edges':len(selected),
                              'topics':len(set(e.label for e in selected)),
                              'node_names':sorted(chosen)}
    (out/'graph_manifest.json').write_text(json.dumps(summary,indent=2))
    (out/'index.html').write_text('''<!doctype html><meta charset="utf-8"><title>ROS Nodes only</title>
<style>body{font:16px sans-serif;margin:24px}iframe{width:100%;height:82vh;border:1px solid #bbb}button{padding:10px;margin:4px}</style>
<h1>ROS 计算图 · Nodes only</h1><p>椭圆：实际注册节点；连线文字：ROS 话题；箭头：发布→订阅。
完整图保留评测和录像；核心图仅保留主要传感器、任务与控制话题；仅飞行版为本轮实际节点的运行链筛选。
未启动实机雷达、相机SDK或机械舵机节点，不将它们虚构入图。服务不画成话题。
这是归档ROS注册快照的离线重绘，不代表实时流量或板端验收。<a href="README.md">来源与范围</a></p>
<button onclick="show('full')">完整</button><button onclick="show('core')">核心</button><button onclick="show('flight')">仅飞行运行链</button>
<a id="download" href="rqt_graph_nodes_only_core.svg">打开 SVG</a>
<a id="png" href="rqt_graph_nodes_only_core.png">PNG</a>
<a id="jpg" href="rqt_graph_nodes_only_core.jpg">JPEG</a>
<iframe id="graph" src="rqt_graph_nodes_only_core.svg"></iframe>
<script>function show(n){let p='rqt_graph_nodes_only_'+n;document.getElementById('graph').src=p+'.svg';document.getElementById('download').href=p+'.svg';document.getElementById('png').href=p+'.png';document.getElementById('jpg').href=p+'.jpg'}</script>''')
    print(json.dumps({n:{k:v for k,v in d.items() if k!='node_names'} for n,d in summary['views'].items()},indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('snapshot',type=Path);p.add_argument('output',type=Path);p.add_argument('--project-root',required=True,type=Path);a=p.parse_args()
    render(a.snapshot,a.output,a.project_root)
