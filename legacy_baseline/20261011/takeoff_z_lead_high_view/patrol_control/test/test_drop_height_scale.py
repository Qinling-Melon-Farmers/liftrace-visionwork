"""Run the production pixel projection with offline CameraInfo/TF inputs."""
from pathlib import Path
import subprocess
import tempfile
import unittest

PACKAGE = Path(__file__).resolve().parents[1]


def production_method(source, signature):
    start = source.index(signature)
    opening = source.index('{', start)
    depth = 0
    for end in range(opening, len(source)):
        if source[end] == '{':
            depth += 1
        elif source[end] == '}':
            depth -= 1
            if depth == 0:
                return source[start:end + 1]
    raise AssertionError('Unclosed production method')


class DropHeightScaleTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source = (PACKAGE / 'src/patrol_control.cpp').read_text()
        methods = '\n'.join(production_method(source, signature) for signature in (
            'void LLController::dropCameraInfoCallback(',
            'bool LLController::dropPixelScales(',
            'void LLController::projectDropOffsetToTarget('))
        program = r'''
#include <array>
#include <cassert>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <patrol_control/drop_action.h>
#define ROS_WARN_THROTTLE(...) ((void)0)
#define ROS_INFO_THROTTLE(...) ((void)0)
namespace ros {
struct Duration { double value; explicit Duration(double v):value(v){} double toSec() const{return value;} };
struct Time { double value=0; explicit Time(double v=0):value(v){} bool isZero()const{return value==0;} };
Duration operator-(Time a,Time b){return Duration(a.value-b.value);}
}
struct Header { ros::Time stamp; std::string frame_id; };
struct Position { double x=0,y=0,z=0; };
namespace geometry_msgs {
struct TransformStamped { Header header; struct { Position translation; } transform; };
struct PoseStamped { Header header; struct { Position position; double orientation=0; } pose; };
}
namespace sensor_msgs {
struct CameraInfo {
  using ConstPtr=std::shared_ptr<const CameraInfo>;
  Header header; unsigned int width=1280,height=720; std::array<double,9>K{{725,0,640,0,800,360,0,0,1}};
};
}
namespace uav_vision { struct DropOffset { Header header; double dx_px=40,dy_px=10,radius_px=100; }; }
namespace tf { double getYaw(double yaw){return yaw;} }
namespace tf2 { struct TransformException:std::runtime_error { using std::runtime_error::runtime_error; }; }
struct Buffer {
  bool available=true; double z=1.02,stamp_delta=0,last_query=-1;
  geometry_msgs::TransformStamped lookupTransform(const std::string& map,const std::string& camera,ros::Time stamp,ros::Duration){
    assert(map=="camera_init" && camera=="downward_camera_optical_frame");
    last_query=stamp.value;
    if(!available)throw tf2::TransformException("missing TF");
    geometry_msgs::TransformStamped result;result.header.stamp=ros::Time(stamp.value+stamp_delta);
    result.transform.translation.z=z;return result;
  }
};
class LLController {
public:
  bool drop_metric_scale_enabled_=true,drop_camera_info_valid_=false;
  bool external_mission_mode_=true,external_landing_active_=false;
  bool have_waypoint_mark=false,have_cross_mark=false,have_land_mark=false;
  double drop_fx_=0,drop_fy_=0,drop_ground_z_=-.22,drop_tf_max_age_sec_=.20;
  double pixel_to_meter_ratio_=.0015,drop_circle_radius_m_=.5,drop_cross_radius_m_=.175,landing_pad_radius_m_=.30;
  double max_alignment_move_distance_=.15,current_pixel_error=0,align_height=.38;
  std::string drop_camera_frame_="downward_camera_optical_frame",drop_map_frame_="camera_init",current_align_mode_="drop_circle";
  std::array<double,4>pixel_to_body_matrix_{{-1,0,0,1}};
  geometry_msgs::PoseStamped uav_pose,waypoint_mark_point,cross_mark_point,land_mark_point;
  Buffer drop_tf_buffer_;
  void dropCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr&);
  bool dropPixelScales(const ros::Time&,double*,double*);
  void projectDropOffsetToTarget(const uav_vision::DropOffset&);
};
using patrol_control::projectPixelOffsetToBody;
PRODUCTION_METHODS
void near(double a,double b){assert(std::abs(a-b)<1e-10);}
sensor_msgs::CameraInfo::ConstPtr info(){
  auto result=std::make_shared<sensor_msgs::CameraInfo>();result->header.frame_id="downward_camera_optical_frame";return result;
}
LLController controller(){
  LLController c;c.dropCameraInfoCallback(info());c.uav_pose.pose.position.x=1;c.uav_pose.pose.position.y=2;
  c.uav_pose.header.frame_id="camera_init";return c;
}
uav_vision::DropOffset observation(){uav_vision::DropOffset msg;msg.header.stamp=ros::Time(10);return msg;}
void rejected(LLController& c,uav_vision::DropOffset msg){
  c.have_waypoint_mark=true;c.have_cross_mark=true;
  c.waypoint_mark_point.pose.position.x=9;c.cross_mark_point.pose.position.x=8;
  c.projectDropOffsetToTarget(msg);
  near(c.waypoint_mark_point.pose.position.x,9);near(c.cross_mark_point.pose.position.x,8);
  if(c.current_align_mode_=="drop_cross")assert(!c.have_cross_mark);
  else assert(!c.have_waypoint_mark);
}
int main(int argc,char**argv){
  assert(argc==2);std::string test=argv[1];auto c=controller();auto msg=observation();
  if(test=="radii"){
    for(double radius:{0.,30.,100.,250.,std::numeric_limits<double>::quiet_NaN()}){
      msg.radius_px=radius;c.projectDropOffsetToTarget(msg);assert(c.have_waypoint_mark);
      near(c.waypoint_mark_point.pose.position.x,1-40*1.24/725);
      near(c.waypoint_mark_point.pose.position.y,2+10*1.24/800);
      near(c.waypoint_mark_point.pose.position.z,c.align_height);
    }
    near(c.drop_tf_buffer_.last_query,10); // exposure timestamp, never latest-TF lookup
    c.current_align_mode_="drop_cross";c.projectDropOffsetToTarget(msg);assert(c.have_cross_mark);
    near(c.cross_mark_point.pose.position.x,c.waypoint_mark_point.pose.position.x);
  }else if(test=="height"){
    c.drop_tf_buffer_.z=.40; // camera AGL .62: exactly half of the first case
    c.projectDropOffsetToTarget(msg);near(c.waypoint_mark_point.pose.position.x,1-40*.62/725);
    c.drop_ground_z_=.10;c.drop_tf_buffer_.z=.72;c.projectDropOffsetToTarget(msg);
    near(c.waypoint_mark_point.pose.position.x,1-40*.62/725); // changed FC ground origin
  }else if(test=="axes"){
    c.drop_fx_=600;c.drop_fy_=400;c.drop_tf_buffer_.z=.98;c.pixel_to_body_matrix_={{0,-1,-1,0}};
    c.uav_pose.pose.orientation=std::acos(-1.)/2;msg.dx_px=20;msg.dy_px=20;c.projectDropOffsetToTarget(msg);
    near(c.waypoint_mark_point.pose.position.x,1+.04);near(c.waypoint_mark_point.pose.position.y,2-.06);
  }else if(test=="limit"){
    msg.dx_px=400;msg.dy_px=800;c.projectDropOffsetToTarget(msg);
    auto p=c.waypoint_mark_point.pose.position;
    near(std::hypot(p.x-1,p.y-2),.15);near((p.x-1)/(p.y-2),-(400./725)/(800./800));
  }else if(test=="tf"){
    c.drop_tf_buffer_.available=false;rejected(c,msg);
    c=controller();msg.header.stamp=ros::Time(0);rejected(c,msg);msg=observation();
    c=controller();c.drop_tf_buffer_.stamp_delta=-.21;rejected(c,msg);
    for(double height:{-.1,0.,.05,std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()}){
      c=controller();c.drop_ground_z_=0;c.drop_tf_buffer_.z=height;rejected(c,msg);
      c.current_align_mode_="drop_cross";rejected(c,msg);
    }
  }else if(test=="calibration"){
    c.drop_camera_info_valid_=false;rejected(c,msg);
    for(int i=0;i<5;++i){
      c=controller();auto bad=std::make_shared<sensor_msgs::CameraInfo>(*info());
      if(i==0)bad->header.frame_id="wrong_camera";
      if(i==1)bad->width=0;
      if(i==2)bad->K[0]=0;
      if(i==3)bad->K[4]=-1;
      if(i==4)bad->K[0]=std::numeric_limits<double>::quiet_NaN();
      c.dropCameraInfoCallback(bad);assert(!c.drop_camera_info_valid_);rejected(c,msg);
    }
    c.dropCameraInfoCallback(info());c.projectDropOffsetToTarget(msg);assert(c.have_waypoint_mark);
  }else if(test=="legacy"){
    c.drop_metric_scale_enabled_=false;msg.dx_px=10;msg.dy_px=0;msg.radius_px=50;
    c.projectDropOffsetToTarget(msg);near(c.waypoint_mark_point.pose.position.x,.9);
    msg.radius_px=5;c.projectDropOffsetToTarget(msg);near(c.waypoint_mark_point.pose.position.x,.985);
    c.drop_metric_scale_enabled_=true;c.external_mission_mode_=false;c.current_align_mode_="landing";msg.radius_px=50;
    c.projectDropOffsetToTarget(msg);near(c.land_mark_point.pose.position.x,.94);
    c.external_mission_mode_=true;c.external_landing_active_=true;msg.dx_px=30;
    c.projectDropOffsetToTarget(msg);near(c.land_mark_point.pose.position.x,.94);
    c=controller();c.current_align_mode_="disabled";c.projectDropOffsetToTarget(msg);assert(!c.have_waypoint_mark);
  }else assert(false);
}
'''.replace('PRODUCTION_METHODS', methods)
        cls.directory = tempfile.TemporaryDirectory(prefix='drop-height-regression-')
        cls.addClassCleanup(cls.directory.cleanup)
        folder = Path(cls.directory.name)
        (folder / 'test.cpp').write_text(program)
        cls.binary = folder / 'test'
        subprocess.run(['g++', '-std=c++14', '-O0', '-fsanitize=undefined',
                        '-fno-sanitize-recover=all', '-I', str(PACKAGE / 'include'),
                        str(folder / 'test.cpp'), '-o', str(cls.binary)], check=True)

    def run_case(self, case):
        subprocess.run([str(self.binary), case], check=True)

    def test_inner_outer_radius_and_cross_do_not_change_scale(self):
        self.run_case('radii')

    def test_height_and_measured_ground_origin(self):
        self.run_case('height')

    def test_unequal_focal_lengths_axis_mapping_and_yaw(self):
        self.run_case('axes')

    def test_original_movement_limit_is_preserved(self):
        self.run_case('limit')

    def test_missing_stale_or_invalid_height_never_uses_ring_size(self):
        self.run_case('tf')

    def test_invalid_camera_info_and_recovery(self):
        self.run_case('calibration')

    def test_legacy_and_external_landing_paths_are_preserved(self):
        self.run_case('legacy')


if __name__ == '__main__':
    unittest.main()
