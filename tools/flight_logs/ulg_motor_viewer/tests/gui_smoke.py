"""Withdrawn-window smoke: actual Tk load, toggle, legacy CSV/PNG, full export."""
import sys
import time
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import app
import tkinter as tk
from tkinter import filedialog, ttk


def descendants(widget):
    for child in widget.winfo_children():
        yield child
        yield from descendants(child)


def smoke(root):
    root.withdraw()
    errors = []
    root.report_callback_exception = lambda *args: errors.append(str(args))
    tree = next(w for w in descendants(root) if isinstance(w, ttk.Treeview))
    output = Path(__file__).resolve().parents[1] / ('.validation_gui_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    output.mkdir(exist_ok=False)
    filedialog.askdirectory = lambda **kwargs: str(output)
    filedialog.asksaveasfilename = lambda **kwargs: str(output/'interactive.png')

    def until(predicate):
        deadline = time.monotonic()+45
        while not predicate():
            root.update()
            if errors:
                raise AssertionError(errors)
            if time.monotonic() > deadline:
                raise TimeoutError('GUI smoke timed out')
            time.sleep(.05)
        root.update()

    buttons = {w.cget('text'): w for w in descendants(root) if isinstance(w, ttk.Button)}
    until(lambda: len(tree.get_children()) == len(sys.argv)-1
          and str(buttons['导出全部 CSV'].cget('state')) == 'normal')
    assert not any('error' in tree.item(i)['tags'] for i in tree.get_children()), 'ULog load failure'
    toggle = next(w for w in descendants(root) if isinstance(w, ttk.Checkbutton)
                  and w.cget('text') == 'Compare eligible airborne command windows only')
    toggle.invoke()
    root.update()
    toggle.invoke()
    root.update()
    buttons['选择全部有效日志'].invoke()
    root.update()
    buttons['保存当前曲线 PNG'].invoke()
    assert (output/'interactive.png').stat().st_size > 1000
    buttons['导出全部 CSV'].invoke()
    until(lambda: bool(list(output.glob('ulg_motors_*/motor_statistics.csv')))
          and str(buttons['导出全部 CSV'].cget('state')) == 'normal')
    buttons['Export full timeline / CSV / summary'].invoke()
    until(lambda: len(list(output.glob('ulog_timeline_*/*/overview.png'))) == len(sys.argv)-1
          and str(buttons['Export full timeline / CSV / summary'].cget('state')) == 'normal')
    assert not errors, errors
    print('GUI smoke PASS: batch load, compare toggle, multi-select, original CSV/PNG and full export')
    root.destroy()


if __name__ == '__main__':
    tk.Tk.mainloop = smoke
    raise SystemExit(app.main())
