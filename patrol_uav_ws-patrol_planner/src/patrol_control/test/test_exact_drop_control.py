"""Compile and execute production callbacks with real generated ROS messages, no nodes."""
from pathlib import Path
import shlex
import subprocess
import tempfile
import unittest
from test_drop_height_scale import production_method

PACKAGE=Path(__file__).resolve().parents[1]
ROOT=PACKAGE.parents[2]

PROGRAM=r'''
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <uav_vision/DropOffset.h>
#include <uav_vision/DropReady.h>
#include <uav_vision/AlignmentTargetContext.h>
#include <patrol_control/drop_geometry.h>
#include <array>
#include <cassert>
#include <limits>

class LLController {
public:
  bool drop_exact_projection_enabled_=true, have_drop_offset_=false, uav_drop_ready_=false;
  bool have_waypoint_mark=false,have_cross_mark=false,drop_condition_met=false;
  bool have_servo_alignment_context_=true;
  double drop_offset_timeout_=.5,drop_ground_z_=0,align_height=.38,max_alignment_move_distance_=5;
  double drop_exact_alignment_max_error_m_=0;
  std::string drop_map_frame_="camera_init",drop_camera_frame_="downward_camera_optical_frame";
  std::string current_align_mode_="drop_circle",latest_drop_ready_reason_;
  unsigned servo_alignment_decision_seq_=900,servo_alignment_target_id_=7;
  std::string servo_alignment_target_class_="panzer";
  int detect_point_counter=0,legacy_calls=0;
  ros::Time last_drop_projection_stamp_,drop_projection_cutoff_=ros::Time(99.1);
  ros::Time latest_drop_offset_time_,latest_drop_ready_time_;
  uav_vision::DropOffset latest_drop_offset_;
  uav_vision::AlignmentTargetContext servo_alignment_context_;
  geometry_msgs::PoseStamped uav_pose,waypoint_mark_point,cross_mark_point,waypoint_temp;
  std::array<double,4> adjust_target_position{{0,0,0,0}};
  std::array<std::array<double,2>,3>drop_slot_offsets_{{{{-.12,0}},{{0,-.12}},{{0,.12}}}};
  std::array<std::array<double,2>,3>dynamic_drop_slot_offsets_=drop_slot_offsets_;
  void projectDropOffsetToTarget(const uav_vision::DropOffset&){++legacy_calls;}
  bool hasFreshDropOffset()const;
  void dropOffsetCallback(const uav_vision::DropOffset::ConstPtr&);
  bool projectExactDropOffsetToTarget(const uav_vision::DropOffset&);
  bool exactDropReleaseReady()const;
  void dropReadyCallback(const uav_vision::DropReady::ConstPtr&);
  void applyDropSlotOffset(int,bool);
};
PRODUCTION_METHODS

LLController controller(){
  LLController c;c.uav_pose.header.frame_id="camera_init";c.uav_pose.header.stamp=ros::Time(100);
  c.uav_pose.pose.orientation.w=1;c.uav_pose.pose.position.x=.22;c.uav_pose.pose.position.y=.0075;
  auto &ctx=c.servo_alignment_context_;ctx.active=true;ctx.has_target=true;ctx.header.stamp=ros::Time(100);
  ctx.align_mode="drop_circle";ctx.deadline=ros::Time(200);ctx.decision_seq=900;
  ctx.semantic_target_id=7;ctx.semantic_target_class="panzer";ctx.payload_slot=1;
  ctx.semantic_target_first_seen=ros::Time(1);
  return c;
}
uav_vision::DropOffset::Ptr observation(double stamp=99.9){
  auto m=boost::make_shared<uav_vision::DropOffset>();
  m->header.stamp=ros::Time(stamp);m->header.frame_id="downward_camera_optical_frame";
  m->map_valid=true;m->map_frame="camera_init";m->map_point.x=.22;m->map_point.y=.0075;
  m->alignment_error_m=0;m->alignment_tolerance_m=.006;m->quality=.9;
  m->target_id=99;m->target_first_seen=ros::Time(98);return m;
}
void ready(LLController &c){auto r=boost::make_shared<uav_vision::DropReady>();r->header.stamp=ros::Time::now();r->ready=true;c.dropReadyCallback(r);}
int main(int argc,char **argv){
  assert(argc==2);ros::Time::init();ros::Time::setNow(ros::Time(100));
  std::string test=argv[1];auto c=controller();auto m=observation();
  if(test=="projection"){
    m->dx_px=1000;c.dropOffsetCallback(m);ready(c);
    assert(c.have_drop_offset_ && c.have_waypoint_mark && c.exactDropReleaseReady());
    assert(c.waypoint_mark_point.pose.position.x==m->map_point.x && c.legacy_calls==0);
    c.adjust_target_position[0]=m->map_point.x;c.adjust_target_position[1]=m->map_point.y;
    c.applyDropSlotOffset(1,false);assert(c.adjust_target_position[0]==m->map_point.x);
    // Latest yaw/position never re-projects an already absolute target.
    c.uav_pose.pose.orientation=tf::createQuaternionMsgFromYaw(1.1);
    c.uav_pose.pose.position.x=-.5;m->header.stamp=ros::Time(99.95);c.dropOffsetCallback(m);
    assert(c.waypoint_mark_point.pose.position.x==m->map_point.x);assert(!c.exactDropReleaseReady());
  }else if(test=="reorder"){
    c.dropOffsetCallback(m);ready(c);const auto receipt=c.latest_drop_offset_time_;
    ros::Time::setNow(ros::Time(100.05));c.dropOffsetCallback(m);
    auto older=observation(99.8);older->map_valid=false;c.dropOffsetCallback(older);
    assert(c.have_drop_offset_ && c.uav_drop_ready_ && c.latest_drop_offset_time_==receipt);
    assert(c.latest_drop_offset_.header.stamp==m->header.stamp);
    ros::Time::setNow(ros::Time(100.41));assert(!c.hasFreshDropOffset());
  }else if(test=="invalid_current"){
    c.dropOffsetCallback(m);ready(c);auto invalid=observation(99.95);invalid->map_valid=false;
    c.dropOffsetCallback(invalid);assert(!c.have_drop_offset_ && !c.uav_drop_ready_ && c.legacy_calls==0);
    c.dropOffsetCallback(m);assert(!c.have_drop_offset_); // cannot resurrect older valid geometry
  }else if(test=="geometry_id"){
    c.dropOffsetCallback(m);m=observation(99.95);m->target_id=100;m->target_first_seen=ros::Time(99.5);
    c.dropOffsetCallback(m);assert(c.have_drop_offset_ && c.latest_drop_offset_.target_id==100);
  }else if(test=="time"){
    for(double stamp:{0.,99.,100.01}){c=controller();c.dropOffsetCallback(observation(stamp));assert(!c.have_drop_offset_);}
    c=controller();c.drop_projection_cutoff_=ros::Time(99.95);c.dropOffsetCallback(m);assert(!c.have_drop_offset_);
  }else if(test=="invalid_geometry"){
    for(int i=0;i<7;++i){c=controller();m=observation();
      if(i==0)m->header.frame_id="wrong";if(i==1)m->map_frame="wrong";
      if(i==2)m->map_point.z=.1;if(i==3)m->map_point.x=std::numeric_limits<double>::quiet_NaN();
      if(i==4)m->alignment_error_m=-1;if(i==5)m->alignment_tolerance_m=0;if(i==6)m->quality=2;
      c.dropOffsetCallback(m);assert(!c.have_drop_offset_ && c.legacy_calls==0);
    }
  }else if(test=="context"){
    for(int i=0;i<5;++i){c=controller();auto &ctx=c.servo_alignment_context_;
      if(i==0)ctx.semantic_target_id=8;if(i==1)ctx.decision_seq=901;if(i==2)ctx.deadline=ros::Time(100);
      if(i==3)ctx.payload_slot=2;if(i==4)ctx.active=false;
      c.dropOffsetCallback(m);assert(!c.have_drop_offset_);
    }
    c=controller();c.current_align_mode_="drop_cross";auto &ctx=c.servo_alignment_context_;
    ctx.align_mode="drop_cross";ctx.semantic_target_class="red_cross";c.servo_alignment_target_class_="red_cross";
    m->target_id=7;m->target_first_seen=ctx.semantic_target_first_seen;
    c.dropOffsetCallback(m);assert(c.have_cross_mark);
    m=observation(99.95);c.dropOffsetCallback(m);assert(!c.have_drop_offset_);
  }else if(test=="release"){
    c.dropOffsetCallback(m);ready(c);assert(c.exactDropReleaseReady());
    c.uav_pose.pose.position.x+=.02;assert(!c.exactDropReleaseReady());
    c.uav_pose.pose.position.x=m->map_point.x;c.uav_pose.header.stamp=ros::Time(99);assert(!c.exactDropReleaseReady());
    c.uav_pose.header.stamp=ros::Time(100);c.drop_exact_alignment_max_error_m_=.001;
    c.uav_pose.pose.position.x+=.003;assert(!c.exactDropReleaseReady());
  }else if(test=="legacy"){
    c.drop_exact_projection_enabled_=false;c.dropOffsetCallback(m);assert(c.legacy_calls==1);
    c.adjust_target_position[0]=1;c.applyDropSlotOffset(1,false);assert(c.adjust_target_position[0]==.88);
  }else assert(false);
}
'''


class ExactDropControlTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source=(PACKAGE/'src/patrol_control.cpp').read_text()
        signatures=('bool LLController::hasFreshDropOffset()',
                    'void LLController::dropOffsetCallback(',
                    'bool LLController::projectExactDropOffsetToTarget(',
                    'bool LLController::exactDropReleaseReady()',
                    'void LLController::dropReadyCallback(',
                    'void LLController::applyDropSlotOffset(')
        methods='\n'.join(production_method(source,s) for s in signatures)
        cls.temp=tempfile.TemporaryDirectory(prefix='drop-exact-control-')
        cls.addClassCleanup(cls.temp.cleanup)
        folder=Path(cls.temp.name);program=folder/'test.cpp';program.write_text(PROGRAM.replace('PRODUCTION_METHODS',methods))
        cls.binary=folder/'test'
        flags=shlex.split(subprocess.check_output(['pkg-config','--cflags','--libs','roscpp','tf'],text=True))
        subprocess.run(['g++','-std=c++14','-O0','-fsanitize=undefined','-fno-sanitize-recover=all',
                        '-I',str(PACKAGE/'include'),'-I',str(ROOT/'vision_ws/devel/include'),
                        str(program),'-o',str(cls.binary),*flags],check=True)

    def run_case(self,case):subprocess.run([str(self.binary),case],check=True)
    def test_absolute_projection_and_single_slot_compensation(self):self.run_case('projection')
    def test_duplicate_and_reorder_preserve_original_lease(self):self.run_case('reorder')
    def test_invalid_current_observation_blocks_without_pixel_fallback(self):self.run_case('invalid_current')
    def test_geometry_tracking_id_can_change_within_semantic_transaction(self):self.run_case('geometry_id')
    def test_source_time_future_stale_and_cutoff(self):self.run_case('time')
    def test_invalid_frames_points_and_tolerance(self):self.run_case('invalid_geometry')
    def test_semantic_context_identity_and_cross_identity(self):self.run_case('context')
    def test_current_pose_movement_and_freshness_recheck(self):self.run_case('release')
    def test_default_legacy_path_and_offsets_preserved(self):self.run_case('legacy')


if __name__=='__main__':unittest.main()
