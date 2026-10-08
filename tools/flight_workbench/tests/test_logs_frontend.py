"""Run the small frontend contract fixture with an already installed Node."""
from pathlib import Path
import shutil
import subprocess
import unittest


class FrontendTests(unittest.TestCase):
    def test_log_page_actions(self):
        node = shutil.which('node')
        if not node:
            for root in ('/mnt/c', '/mnt/d'):
                candidate = Path(root) / 'Program Files/nodejs/node.exe'
                if candidate.is_file():
                    node = str(candidate); break
        if not node: self.skipTest('No installed Node; no packages installed for this check')
        tool = Path(__file__).resolve().parents[1]
        source = (tool / 'web/logs.js').read_text(encoding='utf-8') + '\n' + (
            tool / 'tests/test_logs_frontend.js').read_text(encoding='utf-8')
        result = subprocess.run([node, '-'], input=source, text=True, encoding='utf-8',
                                capture_output=True, timeout=15)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('PASS:', result.stdout)

    def test_probe_link_ui_rejects_failed_stopped_and_stale_cache(self):
        node = shutil.which('node')
        if not node:
            for root in ('/mnt/c', '/mnt/d'):
                candidate = Path(root) / 'Program Files/nodejs/node.exe'
                if candidate.is_file(): node = str(candidate); break
        if not node: self.skipTest('No installed Node')
        tool = Path(__file__).resolve().parents[1]
        import json
        app = (tool / 'web/app.js').read_text(encoding='utf-8')
        observe = (tool / 'web/observe.js').read_text(encoding='utf-8')
        source = "const assert=require('assert'),vm=require('vm');const c={document:{readyState:'loading',addEventListener(){}},window:{}};vm.createContext(c);\n"
        source += 'vm.runInContext(' + json.dumps(app) + ',c);\n'
        source += r'''
c.state.telemetry={at:Date.now()/1000,master:true,probe_link:{status:'live',usable:true}};
c.state.sessions.probe={state:'running'};assert(c.probeView().usable);
c.state.sessions.probe={state:'failed',exit_code:255};assert(!c.probeView().usable);
c.state.sessions.probe={state:'failed',exit_code:130};assert(!c.probeView().usable);
c.state.sessions.probe={state:'running'};c.state.telemetry.at-=10;assert(!c.probeView().usable);
c.state.sessions.servo={state:'running'};assert(c.servoInitBlocked());
c.state.snapshot={};assert(!c.supportsCapability('recording'));assert(!c.supportsCapability('probe_reconnect'));
c.state.snapshot={capabilities:{recording:true,probe_reconnect:true}};assert(c.supportsCapability('recording'));assert(c.supportsCapability('probe_reconnect'));
assert(c.api.probeReconnect.toString().includes('/api/action/probe_reconnect'));
const o={document:{readyState:'loading',addEventListener(){}},window:{}};vm.createContext(o);
'''
        source += 'vm.runInContext(' + json.dumps(observe) + ',o);\n'
        source += "assert(!o.obsFresh({at:Date.now()/1000,probe_link:{usable:false}},Date.now()/1000));console.log('PASS: probe cache invalidation');"
        result = subprocess.run([node, '-'], input=source, text=True, encoding='utf-8', capture_output=True, timeout=15)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == '__main__': unittest.main(verbosity=2)
