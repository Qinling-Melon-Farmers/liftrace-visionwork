import unittest
import json
import tempfile
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from bag_replay import Timeline,Frames,TransformTree,TOPICS,render,verify

class ReplayContracts(unittest.TestCase):
    def test_state_never_reads_future(self):
        t=Timeline([dict(t=1,m='one'),dict(t=2,m='two')])
        self.assertIsNone(t.msg(.9));self.assertEqual(t.msg(1.9),'one');self.assertIsNone(t.msg(3,.5))
    def test_pixels_require_near_image_stamp(self):
        t=Frames([dict(stamp=1,m='one'),dict(stamp=2,m='two')])
        self.assertEqual(t.match(1.02)['m'],'one');self.assertIsNone(t.match(1.1))
    def tree(self,static=True):
        tr=dict(header=dict(frame_id='map',stamp=dict(secs=10,nsecs=0)),child_frame_id='camera_init',transform=dict(translation=dict(x=1,y=2,z=0),rotation=dict(x=0,y=0,z=np.sqrt(.5),w=np.sqrt(.5))))
        return TransformTree(dict(tf_static=[dict(m=dict(transforms=[tr]))] if static else [],tf=[] if static else [dict(m=dict(transforms=[tr]))]),10)
    def test_transform_rotation_translation_inverse(self):
        tree=self.tree();mat=tree.matrix('camera_init','map',9);np.testing.assert_allclose(mat@np.array([1,0,0,1]),[1,3,0,1],atol=1e-9)
        np.testing.assert_allclose(tree.matrix('map','camera_init',9)@mat,np.eye(4),atol=1e-9)
    def test_unknown_frame_is_not_identity(self):self.assertIsNone(self.tree().matrix('unknown','map',0))
    def test_dynamic_tf_age_and_future(self):
        t=self.tree(False);self.assertIsNotNone(t.matrix('camera_init','map',.2));self.assertIsNone(t.matrix('camera_init','map',.6));self.assertIsNone(t.matrix('camera_init','map',-.1))
    def test_missing_optional_topics_render_without_fabricated_camera(self):
        with tempfile.TemporaryDirectory() as tmp:
            out=Path(tmp);(out/'data.json').write_text(json.dumps(dict(bag='synthetic-empty',start=0,duration=.2,frames=[],rows={k:[] for k in TOPICS},missing=list(TOPICS))))
            args=SimpleNamespace(out=tmp,fps=5,frame='map',keep_frames=False)
            render(args);verify(args)
            self.assertFalse((out/'camera_raw.mp4').exists())
            self.assertTrue((out/'dashboard.mp4').exists())
            self.assertEqual(json.loads((out/'summary.json').read_text())['classes'],{})

if __name__=='__main__':unittest.main()
