import unittest
from uav_mission.random_field_policy import frozen_footprint_layout,Footprint

class FrozenFieldTest(unittest.TestCase):
    def call(self,rows,occupied=(),boxes=()):
        return frozen_footprint_layout(rows,[('a',.5),('b',.5)],occupied,(-4,4,-4,4),(-4,4,-4,4),.02,.05,occupied_boxes=boxes)
    def test_keeps_pose_class_order_and_yaw(self):
        rows=[dict(class_='b',x=2,y=1,yaw=.3),dict(class_='a',x=-2,y=-1,yaw=-2)]
        for r in rows:r['class']=r.pop('class_')
        layout,yaws=self.call(rows)
        self.assertEqual(layout,[('a',-2.,-1.),('b',2.,1.)]);self.assertEqual(yaws['a'],-2)
    def test_rejects_bound_wall_collision_and_duplicate(self):
        rows=[{'class':'a','x':0.,'y':0.,'yaw':0.},{'class':'b','x':2.,'y':0.,'yaw':0.}]
        for occupied,boxes in [([Footprint('tree',0,0,.4)],()),((),[(-.1,.1,-3,3)])]:
            with self.assertRaises(ValueError):self.call(rows,occupied,boxes)
        for update in [{'x':3.7},{'x':float('nan')},{'class':'b'},{'x':2.}]:
            modified=[dict(rows[0],**update),rows[1]]
            with self.assertRaises(ValueError):self.call(modified)

if __name__=='__main__':unittest.main()
