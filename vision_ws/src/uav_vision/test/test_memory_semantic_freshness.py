"""Production CandidateRecord regression; no ROS master or flight process."""
import ast
import unittest
from pathlib import Path
from types import SimpleNamespace as N

class Stamp(float):
    def to_sec(self): return float(self)
    def __sub__(self, other): return Stamp(float(self)-float(other))

def load_record(path=None):
    path=path or Path(__file__).resolve().parents[1]/'scripts/target_memory.py'
    tree=ast.parse(Path(path).read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='CandidateRecord')
    env=dict(Point=lambda x=0,y=0,z=0:N(x=x,y=y,z=z),
             TargetCandidate=lambda:N(header=N()),
             ST_DETECTED=0,ST_OBSERVING=1,ST_CONFIRMED=2,ST_REJECTED=3,ST_EXPIRED=4,
             STANDARD_CLASSES={'tent','pillbox','bridge','panzer','tank'})
    exec(compile(ast.Module(body=[cls],type_ignores=[]),str(path),'exec'),env)
    return env['CandidateRecord']

Record=load_record()
def detection(name='panzer',confidence=.95,geometry=.84):
    return N(class_name=name,class_confidence=confidence,geometry_confidence=geometry,
             roi=N(),center_px=N(x=10.,y=10.,z=1.),center_refined=True,
             center_source='circle_geometry',association_valid=True,reject_reason='',
             map_valid=True,map_point=N(x=1.,y=2.,z=0.),map_frame='camera_init',
             map_quality=geometry,transform_age_sec=0.)
def advance(r,name,t,confidence=.95):
    r.update(detection(name,confidence),Stamp(t),3,2,.70,1.0,1.0)
def long_history():
    r=Record(1,detection(),Stamp(1.))
    for i in range(1,100):advance(r,'panzer',1.+i*.1)
    return r

class SemanticFreshnessTest(unittest.TestCase):
    def test_long_high_history_does_not_confirm_old_class_with_low_new_label(self):
        r=long_history();advance(r,'pillbox',11.)
        self.assertEqual((r.class_name,r.consecutive_observe_count),('panzer',0))
        self.assertFalse(r.to_msg(Stamp(11.)).map_valid)
        self.assertNotEqual(r.state,2)
        advance(r,'pillbox',11.1);advance(r,'pillbox',11.2)
        self.assertEqual((r.id,r.class_name,r.state,r.consecutive_observe_count),(1,'pillbox',2,3))
    def test_duplicate_images_do_not_finish_class_switch(self):
        r=long_history();advance(r,'pillbox',11.)
        for _ in range(8):advance(r,'pillbox',11.)
        self.assertEqual(r.pending_class_count,1);self.assertEqual(r.consecutive_observe_count,0)
    def test_one_wrong_frame_requires_three_returning_same_label_frames(self):
        r=long_history();advance(r,'pillbox',11.)
        for i in range(2):
            advance(r,'panzer',11.1+i*.1);self.assertLess(r.consecutive_observe_count,3)
        advance(r,'panzer',11.3);self.assertEqual(r.consecutive_observe_count,3)
    def test_low_confidence_challenger_cannot_relabel(self):
        r=long_history()
        for i in range(12):advance(r,'pillbox',11.+i*.1,.65)
        self.assertEqual(r.class_name,'panzer');self.assertEqual(r.consecutive_observe_count,0)
    def test_gap_and_mode_reset_clear_new_label_streak(self):
        r=long_history();advance(r,'pillbox',11.);advance(r,'pillbox',13.)
        self.assertEqual(r.pending_class_count,1)
        r.reset_confirmation();advance(r,'pillbox',13.1)
        self.assertEqual(r.observed_class_count,1)
    def test_alternating_labels_never_build_confirmation(self):
        r=long_history()
        for i in range(12):
            advance(r,'pillbox' if i%2==0 else 'panzer',11.+i*.1)
            self.assertLess(r.consecutive_observe_count,3)
    def test_same_class_duplicate_merge_keeps_one_streak(self):
        r=long_history();other=Record(2,detection(),Stamp(10.9))
        r.merge_from(other);self.assertEqual(r.consecutive_observe_count,100)
    def test_old_conflicting_duplicate_cannot_restore_old_confirmed_label(self):
        old=long_history();new=Record(9,detection('pillbox'),Stamp(11.))
        advance(new,'pillbox',11.1);advance(new,'pillbox',11.2)
        new.merge_from(old)
        self.assertEqual(new.class_name,'pillbox');self.assertFalse(new.current_map_valid)
        self.assertEqual(new.consecutive_observe_count,0)
    def test_newer_conflict_cannot_rejuvenate_old_label(self):
        old=long_history();new=Record(9,detection('pillbox'),Stamp(11.))
        advance(new,'pillbox',11.1);advance(new,'pillbox',11.2)
        old.merge_from(new)
        self.assertEqual(old.consecutive_observe_count,0);self.assertFalse(old.current_map_valid)
        for i in range(3):advance(old,'pillbox',11.3+i*.1)
        self.assertEqual(old.class_name,'pillbox');self.assertEqual(old.state,2)
    def test_auxiliary_geometry_and_cross_keep_normal_confirmation(self):
        for name in ('circle','landing_pad','red_cross'):
            r=Record(0,detection(name),Stamp(1.))
            advance(r,name,1.1);advance(r,name,1.2)
            self.assertEqual((r.state,r.consecutive_observe_count),(2,3))

if __name__=='__main__': unittest.main()
