"""Check actual expanded Gazebo names, measured geometry and motor-model defaults."""
from pathlib import Path
import math,os,subprocess,unittest,xml.etree.ElementTree as ET
import yaml

class VehicleContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root=Path(__file__).resolve().parents[4]
        cls.models=cls.root/'vision_ws/src/uav_vision_eval/models'
        cls.outer=ET.parse(cls.models/'iris_mid360_ks2a543_installed/model.sdf')
        cls.body=ET.parse(cls.models/'r2026_iris_dynamics/model.sdf')
        cls.lidar=ET.parse(cls.models/'r2026_mid360_mounted/model.sdf')
        cls.launch=ET.parse(cls.root/'patrol_uav_ws-patrol_planner/src/uav_mission/launch/navigation_horizontal_search_vcl06.launch')
    def test_plugin_links_resolve_in_real_sdf_expansion(self):
        env=os.environ.copy();env['GAZEBO_MODEL_PATH']=str(self.models)+':'+str(self.root/'simulation_assets/models')
        result=subprocess.run(['gz','sdf','-p',str(self.models/'iris_mid360_ks2a543_installed/model.sdf')],env=env,capture_output=True,text=True,check=True)
        model=ET.fromstring(result.stdout).find('model');links={l.get('name'):l for l in model.findall('link')}
        plugin=model.find("plugin[@name='uav_contact_envelope']")
        for key,collision in [('link_name','collision_name'),('ray_transparent_link','ray_transparent_collision')]:
            self.assertIn(plugin.findtext(key),links)
            self.assertIsNotNone(links[plugin.findtext(key)].find("collision[@name='%s']"%plugin.findtext(collision)))
    def test_sitl_lidar_type_and_measured_sensor_positions(self):
        self.assertEqual(self.lidar.findtext('.//publish_pointcloud_type'),'1')
        camera=float(self.outer.findtext("model/link[@name='downward_camera_link']/pose").split()[2])
        mount=next(n for n in self.outer.findall('model/include') if n.findtext('name')=='mid360')
        imu=float(mount.findtext('pose').split()[2])+float(self.lidar.findtext("model/link/sensor[@type='imu']/pose").split()[2])
        self.assertAlmostEqual(camera,-.16);self.assertAlmostEqual(imu-camera,.21)
        self.assertEqual(self.body.findtext("model/plugin[@name='rotors_gazebo_imu_plugin']/linkName"),'base_link')
        self.assertEqual(self.body.findtext("model/link[@name='base_link']/pose"),'0 0 0 0 0 0')
    def test_envelope_support_and_contact_settings(self):
        collision=self.body.find("model/link[@name='base_link']/collision[@name='competition_guard_collision']")
        size=[float(v) for v in collision.findtext('geometry/box/size').split()]
        self.assertEqual(size,[.55,.55,.4]);z=float(collision.findtext('pose').split()[2])
        self.assertAlmostEqual(z-size[2]/2,-.22)
        spawn=float(self.launch.find("arg[@name='spawn_z']").get('default'))
        self.assertAlmostEqual(spawn+z-size[2]/2,.005)
        self.assertEqual(collision.findtext('surface/contact/ode/min_depth'),'0.001')
        self.assertEqual(collision.findtext('surface/contact/ode/max_vel'),'0')
    def test_hover_initial_value_matches_assembled_mass_and_motor_mapping(self):
        trees=[self.outer,self.body,self.lidar,ET.parse(self.root/'simulation_assets/models/gps/gps.sdf')]
        mass=sum(float(n.text) for tree in trees for n in tree.iter('mass'))
        k=float(self.body.findtext('.//motorConstant'));scale=float(self.body.findtext('.//input_scaling'));offset=float(self.body.findtext('.//zero_position_armed'))
        expected=(math.sqrt(mass*9.8/(4*k))-offset)/scale
        params=yaml.safe_load(self.launch.find("rosparam[@param='/simulation/px4_parameters']").text)
        self.assertLess(abs(params['MPC_THR_HOVER']-expected),.001)
if __name__=='__main__':unittest.main()
