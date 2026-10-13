"""Local offline contracts; do not connect to devices."""
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import wb_replay
import wb_offline_support as native


class OfflineWorkbenchTests(unittest.TestCase):
    def test_windows_device_names_and_alias_paths_rejected(self):
        for name in ('CON.bag', 'nul.bag', 'COM1.bag', 'bad?.bag', 'file.bag.', 'file.bag '):
            with self.subTest(name=name), self.assertRaises(ValueError):
                wb_replay.relative(name)

    def test_malformed_persisted_locations_are_ignored(self):
        with tempfile.TemporaryDirectory() as tmp:
            profile = Path(tmp)
            (profile/'offline_locations.json').write_text('{"sources": [], "outputs": 5}')
            library = wb_replay.ReplayLibrary(profile)
            self.assertEqual(library.output_locations, {})

    def test_native_bag_picker_rejects_non_bag_and_symlink(self):
        with tempfile.TemporaryDirectory() as tmp:
            library = wb_replay.ReplayLibrary(Path(tmp)/'profile')
            invalid = Path(tmp)/'wrong.txt';invalid.write_bytes(b'#ROSBAG V2.0\n')
            with patch('wb_offline_support.pick', return_value=str(invalid)):
                with self.assertRaises(ValueError):
                    library.select_local({'kind':'bag'})
            if sys.platform != 'win32':
                link=Path(tmp)/'linked.bag';link.symlink_to(invalid)
                with patch('wb_offline_support.pick', return_value=str(link)):
                    with self.assertRaises(ValueError):
                        library.select_local({'kind':'bag'})

    def test_native_lock_excludes_second_worker(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'.lock'
            first = native.acquire(path)
            try:
                with self.assertRaises(ValueError):
                    native.acquire(path)
            finally:
                first.close()
            native.acquire(path).close()

    def test_upload_stream_does_not_accept_truncated_bag(self):
        with tempfile.TemporaryDirectory() as tmp:
            library = wb_replay.ReplayLibrary(tmp)
            with self.assertRaises(ValueError):
                library.ingest(io.BytesIO(b'#ROSBAG V2.0\n'), 20, 'sample.bag')
            self.assertEqual(list((library.output/'imports').rglob('*.part')), [])
            response = library.ingest(io.BytesIO(b'#ROSBAG V2.0\n'), 13, 'sample.bag')
            self.assertEqual(library.root('imports').joinpath(response['path']).read_bytes(), b'#ROSBAG V2.0\n')

    def test_traversal_and_arbitrary_web_output_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            for value in ('../private', '/etc/passwd', 'C:/private', 'a\\b', 'a//b'):
                with self.assertRaises(ValueError):
                    wb_replay.inside(Path(tmp), value)
            library = wb_replay.ReplayLibrary(tmp)
            with self.assertRaises(ValueError):
                library.select_local({'kind': 'shell'})

    def test_native_output_selection_is_persisted(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'chosen'; path.mkdir()
            library = wb_replay.ReplayLibrary(Path(tmp)/'profile')
            with patch('wb_offline_support.pick', return_value=str(path)):
                result = library.select_local({'kind': 'folder'})
            restored = wb_replay.ReplayLibrary(Path(tmp)/'profile')
            self.assertEqual(restored.output_locations[result['root']], str(path))
            self.assertEqual(restored.root(result['root']), path)

    def test_analysis_result_requires_all_named_images(self):
        with tempfile.TemporaryDirectory() as tmp:
            library = wb_replay.ReplayLibrary(tmp)
            output = Path(tmp)/'analysis';output.mkdir()
            (output/'analysis_report.html').write_text('<html>report</html>')
            (output/'analysis_summary.json').write_text(json.dumps({'images':['delivery_curves.png']}))
            self.assertFalse(library.validated_analysis(output))
            (output/'delivery_curves.png').write_bytes(b'png-fixture')
            self.assertTrue(library.validated_analysis(output))

    def test_pending_generated_report_cannot_be_served(self):
        with tempfile.TemporaryDirectory() as tmp:
            library = wb_replay.ReplayLibrary(tmp)
            identifier = 'a'*32
            jobdir = library.output/identifier;result = jobdir/'result';result.mkdir(parents=True)
            (result/'analysis_report.html').write_text('report')
            (result/'analysis_summary.json').write_text(json.dumps({'images':['delivery_curves.png']}))
            (result/'delivery_curves.png').write_bytes(b'fixture')
            job = {'id':identifier, 'state':'running', 'created_at':0, 'result_root':'generated','result_path':identifier+'/result'}
            library._save(jobdir,job)
            with self.assertRaises(ValueError):
                library.artifact('generated',identifier+'/result/analysis_report.html')
            job['state']='ready';library._save(jobdir,job)
            self.assertEqual(library.artifact('generated',identifier+'/result/analysis_report.html'),result/'analysis_report.html')


if __name__ == '__main__':
    unittest.main()
