import unittest
from uav_mission.survey_routes import survey_route
class SurveyRoutesTest(unittest.TestCase):
 def test_closed_rectangle(self):
  r=survey_route('rectangle',[1,5],-3,3);self.assertEqual(r[0],r[-1]);self.assertEqual(len(r),5)
 def test_two_and_three_scan_lanes(self):
  self.assertEqual(survey_route('snake2',[1,5],-3,3),[[1,-3],[1,3],[5,3],[5,-3]])
  self.assertEqual(survey_route('snake3',[1,3,5],-3,3)[-2:],[[5,-3],[5,3]])
 def test_validation(self):
  for args in [('snake3',[1,5],-3,3),('rectangle',[5,1],-3,3),('snake2',[1,float('nan')],-3,3),('snake2',[1,5],3,-3)]:
   with self.assertRaises(ValueError):survey_route(*args)
 def test_reverse_start(self):self.assertEqual(survey_route('snake2',[1,5],-3,3,True)[0],[1,3])
