import random,unittest
from uav_mission.random_field_policy import (
    Footprint,plan_footprint_layout,footprint_clear_boxes,
    STANDARD_FOOTPRINT_RADIUS,RED_CROSS_FOOTPRINT_RADIUS)

class WallLayoutTest(unittest.TestCase):
    boxes=((-3.3,4.8,7.4,7.6),)
    occupied=[Footprint('H0',0,0,.5),Footprint('H1',4.2,8.5,.5)]+[
        Footprint('tree',x,y,.43) for x,y in [(-1.8,2),(1.8,2.7),(-1.5,5.2),(1.7,5.7)]]
    specs=[(x,STANDARD_FOOTPRINT_RADIUS) for x in ['tent','pillbox','bridge','panzer']]+[('red_cross',RED_CROSS_FOOTPRINT_RADIUS)]
    def layout(self,seed,boxes):
        return plan_footprint_layout(random.Random(seed),self.specs,self.occupied,
            (-4.8,4.8,-.5,7.6),(-4.8,4.8,-.5,9.1),.02,.05,occupied_boxes=boxes)
    def test_retains_seed11_baseline(self):
        self.assertEqual(self.layout(11,()),self.layout(11,self.boxes))
        recorded=[(-.4572,4.0342),(4.0724,3.2718),(-3.0273,3.6465),(-3.8964,1.9576),(-3.9296,6.0581)]
        for (_,x,y),(rx,ry) in zip(self.layout(11,self.boxes),recorded):
            self.assertLess(abs(x-rx),.000051)
            self.assertLess(abs(y-ry),.000051)
    def test_previous_wall_intersections_are_rejected(self):
        for x,y in [(-2.0227,7.288),(1.2234,7.1764)]:
            self.assertFalse(footprint_clear_boxes(x,y,RED_CROSS_FOOTPRINT_RADIUS,self.boxes,.02))
    def test_thousand_layouts_have_no_wall_overlap(self):
        for seed in range(1,1001):
            layout=self.layout(seed,self.boxes);self.assertIsNotNone(layout)
            for (_,x,y),(_,radius) in zip(layout,self.specs):
                self.assertTrue(footprint_clear_boxes(x,y,radius,self.boxes,.02))
if __name__=='__main__':unittest.main()
