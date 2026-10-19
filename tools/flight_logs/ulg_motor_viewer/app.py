"""User viewer with retained Tk UI and offline --output batch extension."""
from __future__ import annotations
import sys
import os
from pathlib import Path


def batch(argv, default_output=False):
    from analysis import main as batch_main
    argv = [arg for arg in argv if arg not in ('--batch', '--headless', '--gui')]
    if default_output and '--output' not in argv:
        from datetime import datetime
        output = Path(__file__).parent / 'outputs' / ('batch_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
        argv += ['--output', str(output)]
        print(f'Headless output: {output}')
    return batch_main(argv)


def main():
    argv = sys.argv[1:]
    if '--help' in argv or '--output' in argv or '--batch' in argv or '--headless' in argv:
        return batch(argv, default_output='--headless' in argv)
    force_gui = '--gui' in argv
    sys.argv = [sys.argv[0]] + [arg for arg in argv if arg != '--gui']
    if not force_gui and sys.platform != 'win32' and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        if len(sys.argv) == 1:
            print('No graphical display. Supply ULog files/directories with --headless or --output PATH.', file=sys.stderr)
            return 2
        return batch(sys.argv[1:], default_output=True)
    # Tk imports are intentionally deferred: headless export never initializes Tk.
    import importlib.util
    legacy_path = Path(__file__).parent / 'original' / 'ulg_motor_viewer' / 'app.py'
    spec = importlib.util.spec_from_file_location('user_motor_viewer', legacy_path)
    legacy = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = legacy
    try:
        spec.loader.exec_module(legacy)
    except ImportError as exc:
        if len(sys.argv) > 1 and not force_gui:
            print(f'GUI unavailable ({exc}); switching to headless.')
            return batch(sys.argv[1:], default_output=True)
        print(f'GUI dependency unavailable: {exc}. Use --output PATH for offline export.', file=sys.stderr)
        return 2
    from plotting import configure_fonts
    configure_fonts()
    import tkinter as tk
    from tkinter import filedialog, ttk
    import threading
    import numpy as np
    from analysis import read_log, export

    original_stats = legacy.MotorLog.stats

    def safe_stats(record, motor):
        if np.any(np.isfinite(record.motors[motor])):
            return original_stats(record, motor)
        return {'motor': motor, 'valid_samples': 0, 'minimum': np.nan, 'mean': np.nan,
                'maximum': np.nan, 'peak_time_s': np.nan, 'samples_ge_095_percent': np.nan,
                'samples_ge_0999_percent': np.nan, 'estimated_seconds_ge_0999': 0.0}

    legacy.MotorLog.stats = safe_stats

    def read_for_ui(path):
        analysis = read_log(path)
        results = []
        status = analysis.get('vehicle_status')
        for stream in analysis.motor_streams():
            motors = {int(key[8:-1])+1: np.asarray(stream.data[key], float).copy()
                      for key in analysis.motor_fields(stream)}
            if not motors:
                continue
            times = status.time if status and 'nav_state' in status.data else np.array([])
            states = status.data['nav_state'] if len(times) else np.array([], dtype=int)
            record = legacy.MotorLog(path, stream.instance, analysis.start_us, stream.time,
                                     motors, times, states)
            record.analysis = analysis
            record.raw_motors = {k: v.copy() for k, v in motors.items()}
            _, record.windows = analysis.context(stream.time)
            results.append(record)
        if not results:
            raise ValueError('No normalized actuator_motors fields; use --output for source-aware fallback export')
        return results

    legacy.read_log = read_for_ui

    class Viewer(legacy.Viewer):
        def __init__(self, root):
            self.airborne_only = tk.BooleanVar(value=True)
            super().__init__(root)
            root.title('ULog motor viewer + offline low-altitude timeline')
            bar = ttk.Frame(root, padding=4)
            bar.pack(fill='x', before=root.winfo_children()[0])
            ttk.Checkbutton(bar, text='Compare eligible airborne command windows only',
                            variable=self.airborne_only, command=self.redraw).pack(side='left')
            button = ttk.Button(bar, text='Export full timeline / CSV / summary', command=self.export_full)
            button.pack(side='left', padx=8)
            self.operation_buttons.append(button)

        def redraw(self):
            for record in self.records.values():
                for key, raw in record.raw_motors.items():
                    record.motors[key] = (np.where(record.windows == 'motor_airborne', raw, np.nan)
                                         if self.airborne_only.get() else raw.copy())
            for iid, record in self.records.items():
                finite = [v[np.isfinite(v)] for v in record.motors.values() if np.any(np.isfinite(v))]
                self.tree.set(iid, 'peak', f'{max(v.max() for v in finite):.3f}' if finite else 'UNKNOWN')
                self.tree.set(iid, 'result', 'airborne' if self.airborne_only.get() and finite
                              else 'unknown' if not finite else 'all samples')
            # Original renderer assumes at least one finite sample per motor.
            hidden = {}
            for iid, record in list(self.records.items()):
                if not any(np.any(np.isfinite(v)) for v in record.motors.values()):
                    hidden[iid] = self.records.pop(iid)
            try:
                super().redraw()
                if self.airborne_only.get():
                    self.ax.set_title('Eligible airborne motor COMMAND windows (ground/stops/guards excluded)')
                    self.canvas.draw_idle()
            finally:
                self.records.update(hidden)
            if hidden:
                self.status.set('Some logs have no eligible airborne samples; use full export for UNKNOWN and state context.')

        def export_full(self):
            if self.busy or not self.records:
                return
            folder = filedialog.askdirectory(title='Full timeline export destination')
            if not folder:
                return
            analyses = {r.path: r.analysis for r in self.records.values()}
            self.set_busy(True)

            def worker():
                from datetime import datetime
                try:
                    base = Path(folder) / ('ulog_timeline_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
                    base.mkdir()
                    for index, analysis in enumerate(analyses.values(), 1):
                        export(analysis, base / f'{index:03d}_{analysis.path.stem}')
                    self.events.put(('done', f'Full export complete: {base}'))
                except Exception as exc:
                    self.events.put(('done', f'Full export failed: {exc}'))
            threading.Thread(target=worker, daemon=True).start()

    try:
        root = tk.Tk()
    except tk.TclError as exc:
        if len(sys.argv) > 1 and not force_gui:
            print(f'GUI unavailable ({exc}); switching to headless.')
            return batch(sys.argv[1:], default_output=True)
        print(f'GUI unavailable: {exc}. Supply files with --headless or --output PATH.', file=sys.stderr)
        return 2
    viewer = Viewer(root)
    if len(sys.argv) > 1:
        root.after(200, lambda: viewer.load([Path(arg) for arg in sys.argv[1:]]))
    root.mainloop()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
