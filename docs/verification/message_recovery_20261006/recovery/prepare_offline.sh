set -e
source /home/xhj/miniconda3/etc/profile.d/conda.sh
conda activate rl_drone
export PYTHONDONTWRITEBYTECODE=1
python - <<'PY'
from pathlib import Path
import subprocess, re, json
out=Path('/tmp/recovery_review_20261006')
repo=Path('/home/xhj/liftrace-worktrees/r2026-high-view-search')
head='966f6adf80050e2033e067cad1ab2b6dd41cbe66'
ev='b2632da2'
base='patrol_uav_ws-patrol_planner/src/Fast-Planner/fast_planner/'
snap=out/'snapshot'; stub=out/'stubs'
for d in (snap,stub/'ros',stub/'plan_env'):d.mkdir(parents=True,exist_ok=True)
def read_git(path,ref=head):
    text=subprocess.check_output(['git','show',ref+':'+path],cwd=repo).decode('utf-8')
    dest=snap/path;dest.parent.mkdir(parents=True,exist_ok=True);dest.write_text(text)
    return text
files=['plan_env/src/sdf_map.cpp','plan_env/include/plan_env/sdf_map.h',
       'plan_env/include/plan_env/vertical_obstacle_support.h','plan_env/include/plan_env/search_region.h',
       'plan_env/include/plan_env/reference_height.h','path_searching/src/kinodynamic_astar.cpp',
       'path_searching/include/path_searching/kinodynamic_astar.h','path_searching/include/path_searching/line_preference.h',
       'bspline/src/non_uniform_bspline.cpp','bspline/include/bspline/non_uniform_bspline.h']
src={p:read_git(base+p) for p in files}
for p in ['plan_manage/src/kino_replan_fsm.cpp','plan_manage/src/planner_manager.cpp','plan_manage/src/traj_server.cpp']:
    read_git(base+p)
read_git('patrol_uav_ws-patrol_planner/src/patrol_control/src/patrol_control.cpp')
read_git('patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/high_view_probe.py')
boundary=read_git('patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/task_frame_continuity.py',ev)
(out/'task_frame_continuity_candidate.py').write_text(boundary)
for p in ['patrol_uav_ws-patrol_planner/src/uav_mission/scripts/ev_task_boundary.py',
          'patrol_uav_ws-patrol_planner/src/uav_mission/config/ev_task_boundary.example.yaml']:
    read_git(p,ev)
param_path='docs/verification/seed38_resume_20261005/generated/resume_on_38/resume_on_seed38/effective_parameters.json'
params=json.loads(read_git(param_path))
selected={k:v for k,v in params.items() if (k.startswith('/fast_planner_node/sdf_map/') and
          any(s in k for s in ('inflation','ground_height','horizontal_avoidance','map_size','resolution'))) or
          'navigation_height_constraint' in k or k=='/external_planner_max_command_z'}
(out/'profile_parameters.json').write_text(json.dumps(selected,indent=2))
provenance=[]
def extract(text,signature,path):
    start=text.index(signature)
    i=text.index('{',start);depth=0;state='code';j=i
    while j<len(text):
        c=text[j];n=text[j:j+2]
        if state=='line':
            if c=='\n':state='code'
        elif state=='block':
            if n=='*/':state='code';j+=1
        elif state in ('"',"'"):
            if c=='\\':j+=1
            elif c==state:state='code'
        else:
            if n=='//':state='line';j+=1
            elif n=='/*':state='block';j+=1
            elif c in ('"',"'"):state=c
            elif c=='{':depth+=1
            elif c=='}':
                depth-=1
                if depth==0:break
        j+=1
    assert depth==0
    line=text.count('\n',0,start)+1
    provenance.append({'path':path,'signature':signature,'start_line':line,'end_line':text.count('\n',0,j)+1})
    return '\n#line '+str(line)+' "'+str(snap/base/path)+'"\n'+text[start:j+1]+'\n'
ros=r'''#pragma once
#include <any>
#include <map>
#include <string>
namespace ros {
struct Time {};
namespace param {
inline std::map<std::string,std::any> values;
template<typename T> bool getCached(const std::string& k,T& v) {
 auto i=values.find(k);if(i==values.end()) return false;
 auto p=std::any_cast<T>(&i->second);if(!p)return false;v=*p;return true;
}
template<typename T> void param(const std::string& k,T& v,const T& d) {if(!getCached(k,v))v=d;}
}
struct NodeHandle {
 template<typename T> bool getParamCached(const std::string& k,T& v){return param::getCached(k,v);}
 template<typename T> void param(const std::string& k,T& v,const T& d){ros::param::param(k,v,d);}
 std::string resolveName(const std::string& k){return k;}
};
}
#define ROS_WARN_THROTTLE(...)
#define ROS_WARN_STREAM_THROTTLE(...)
#define ROS_ERROR(...)
#define ROS_ERROR_COND(...)
'''
(stub/'ros/ros.h').write_text(ros);(stub/'ros/console.h').write_text('#pragma once\n#include "ros.h"\n')
struct=src['plan_env/include/plan_env/sdf_map.h'].split('struct MappingParameters {',1)[1].split('\n};',1)[0]
header=r'''#pragma once
#include <Eigen/Eigen>
#include <algorithm>
#include <memory>
#include <vector>
#include <string>
#include <limits>
#include <cmath>
#include <ros/ros.h>
#include <plan_env/search_region.h>
#include <plan_env/vertical_obstacle_support.h>
using namespace std;
namespace pcl {
struct PointXYZ {float x,y,z;};
template<typename T> struct PointCloud {vector<T> points;};
}
namespace sensor_msgs {
struct PointCloud2 {vector<pcl::PointXYZ> points;};
using PointCloud2ConstPtr=shared_ptr<const PointCloud2>;
}
namespace pcl {
inline void fromROSMsg(const sensor_msgs::PointCloud2& m,PointCloud<PointXYZ>& c){c.points=m.points;}
}
struct MappingParameters {'''+struct+r'''
};
struct MappingData {
vector<double> occupancy_buffer_,distance_buffer_,distance_buffer_neg_,distance_buffer_all_,tmp_buffer1_,tmp_buffer2_;
vector<char> occupancy_buffer_inflate_,occupancy_buffer_neg;
Eigen::Vector3d camera_pos_;
Eigen::Vector3i local_bound_min_,local_bound_max_;
bool has_odom_=true,has_cloud_=false,esdf_need_update_=false;
};
class SDFMap {
public:
MappingParameters mp_{};
MappingData md_{};
ros::NodeHandle node_;
using Ptr=shared_ptr<SDFMap>;
inline int toAddress(const Eigen::Vector3i& id);
inline int toAddress(int& x,int& y,int& z);
inline void boundIndex(Eigen::Vector3i& id);
inline double getDistance(const Eigen::Vector3d& pos);
inline double getDistance(const Eigen::Vector3i& id);
inline int getOccupancy(Eigen::Vector3d pos);
inline int getInflateOccupancy(Eigen::Vector3d pos);
inline bool isInMap(const Eigen::Vector3d& pos);
inline bool isInMap(const Eigen::Vector3i& idx);
inline void posToIndex(const Eigen::Vector3d& pos,Eigen::Vector3i& id);
inline void indexToPos(const Eigen::Vector3i& id,Eigen::Vector3d& pos);
void resetBuffer();
void resetBuffer(Eigen::Vector3d min_pos,Eigen::Vector3d max_pos);
void applyFlightCeiling();
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& img);
void updateESDF3d();
void getRegion(Eigen::Vector3d& ori,Eigen::Vector3d& size);
template<typename F_get_val,typename F_set_val> void fillESDF(F_get_val,F_set_val,int,int,int);
};
'''
hpath='plan_env/include/plan_env/sdf_map.h'
for signature in ['inline int SDFMap::toAddress(const Eigen::Vector3i&','inline int SDFMap::toAddress(int&',
 'inline void SDFMap::boundIndex','inline double SDFMap::getDistance(const Eigen::Vector3d&',
 'inline double SDFMap::getDistance(const Eigen::Vector3i&','inline int SDFMap::getOccupancy(Eigen::Vector3d',
 'inline int SDFMap::getInflateOccupancy','inline bool SDFMap::isInMap(const Eigen::Vector3d&',
 'inline bool SDFMap::isInMap(const Eigen::Vector3i&','inline void SDFMap::posToIndex','inline void SDFMap::indexToPos']:
    header+=extract(src[hpath],signature,hpath)
(stub/'plan_env/sdf_map.h').write_text(header)
cpath='plan_env/src/sdf_map.cpp'
code='#include <plan_env/sdf_map.h>\n'
for signature in ['void SDFMap::resetBuffer()', 'void SDFMap::resetBuffer(Eigen::Vector3d',
 'void SDFMap::applyFlightCeiling','void SDFMap::cloudCallback','void SDFMap::updateESDF3d','void SDFMap::getRegion']:
    code+=extract(src[cpath],signature,cpath)
code+='template<typename F_get_val,typename F_set_val>\n'+extract(src[cpath],'void SDFMap::fillESDF',cpath)
# Template definitions must precede concrete ESDF instantiations.
pos=code.index('template<typename F_get_val')
tpl=code[pos:];code=code[:pos]
pos=code.index('\n#line ',code.index('#include'))
code=code[:pos]+tpl+code[pos:]
(out/'production_map.cpp').write_text(code)
(stub/'plan_env/edt_environment.h').write_text('''#pragma once
#include <plan_env/sdf_map.h>
namespace fast_planner { struct EDTEnvironment { using Ptr=std::shared_ptr<EDTEnvironment>; SDFMap::Ptr sdf_map_; }; }
''')
(out/'source_methods.json').write_text(json.dumps(provenance,indent=2))
(out/'metadata.json').write_text(json.dumps({'root':str(repo),'source_head':head,
 'continuity_source':subprocess.check_output(['git','rev-parse',ev],cwd=repo).decode().strip(),
 'main':subprocess.check_output(['git','rev-parse','main'],cwd=repo).decode().strip(),
 'method':'unchanged source function extraction for SDFMap; full unchanged kino and bspline translation units; inert ROS/PCL message transport stubs; no ROS linkage',
 'map_extent_note':'compact 4x4m fixture only; actual resolution/inflation/support/middle policy from archived seed38 profile; synthetic points, not a flight replay'},
 indent=2))
print('Frozen source and inert transport stubs created:',out)
PY

