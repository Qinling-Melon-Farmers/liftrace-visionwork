"""Batch ULog motor-command viewer. Run: python app.py [file_or_directory ...]."""
from __future__ import annotations

import csv
import hashlib
import queue
import sys
import threading
from dataclasses import dataclass
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

try:
    import numpy as np
    import matplotlib
    matplotlib.use('TkAgg')
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
    from matplotlib.figure import Figure
    from matplotlib.ticker import AutoMinorLocator, MaxNLocator, MultipleLocator
    from pyulog import ULog
except ImportError as exc:
    raise SystemExit(
        f'缺少依赖：{exc}\n请运行：python -m pip install -r "{Path(__file__).with_name("requirements.txt")}"'
    ) from exc

matplotlib.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False

# PX4 vehicle_status.nav_state. Unknown/custom values retain their numeric ID.
# MANUAL on a multicopter is the manual stabilized mode, not altitude hold.
MODE_NAMES = {
    0: '手动（自稳）', 1: '定高', 2: '定点', 3: '自动任务',
    4: '自动悬停', 5: '自动返航', 10: '特技', 12: '自动下降',
    13: '飞行终止', 14: '板外（Offboard）', 15: '自稳',
    16: '混合自稳', 17: '自动起飞', 18: '自动降落',
    19: '跟随', 20: '精准降落', 21: '环绕', 22: 'VTOL 起飞',
}
MODE_COLORS = {
    0: '#d6a25f', 1: '#e4cc60', 2: '#7db685', 3: '#8b9bd3',
    4: '#72bac0', 5: '#b39acb', 10: '#dd8d73', 12: '#d390a0',
    13: '#cf5959', 14: '#5a9ed0', 15: '#d6a25f', 16: '#c4ae79',
    17: '#85b89a', 18: '#b79778', 19: '#8fbec3', 20: '#c098b4',
    21: '#c1bd83', 22: '#89aab3',
}


def mode_name(state: int) -> str:
    return MODE_NAMES.get(state, '模式未知' if state == -1 else f'模式 {state}')


@dataclass
class MotorLog:
    path: Path
    instance: int
    start_us: int
    time: np.ndarray
    motors: dict[int, np.ndarray]
    mode_time: np.ndarray
    mode_state: np.ndarray

    def modes_at(self, times: np.ndarray) -> np.ndarray:
        states = np.full(len(times), -1, dtype=int)
        if len(self.mode_time):
            indices = np.searchsorted(self.mode_time, times, side='right') - 1
            known = indices >= 0
            states[known] = self.mode_state[indices[known]]
        return states

    def mode_regions(self) -> list[tuple[float, float, int]]:
        begin, end = float(self.time[0]), float(self.time[-1])
        changes = self.mode_time[(self.mode_time > begin) & (self.mode_time < end)]
        bounds = np.r_[begin, changes, end]
        states = self.modes_at(bounds[:-1])
        return [(float(a), float(b), int(state)) for a, b, state in
                zip(bounds[:-1], bounds[1:], states) if b > a]

    def stats(self, motor: int) -> dict:
        values = self.motors[motor]
        finite = np.isfinite(values)
        valid = values[finite]
        # Durations are estimates using the preceding command held until the next
        # sample. Intervals adjoining missing values are excluded.
        dt = np.diff(self.time)
        intervals = finite[:-1] & finite[1:] & (dt > 0)
        full_seconds = float(dt[intervals & (values[:-1] >= .999)].sum())
        return {
            'motor': motor,
            'valid_samples': len(valid),
            'minimum': float(valid.min()),
            'mean': float(valid.mean()),
            'maximum': float(valid.max()),
            'peak_time_s': float(self.time[np.flatnonzero(finite)[np.argmax(valid)]]),
            'samples_ge_095_percent': float(100 * np.mean(valid >= .95)),
            'samples_ge_0999_percent': float(100 * np.mean(valid >= .999)),
            'estimated_seconds_ge_0999': full_seconds,
        }


def read_log(path: Path) -> list[MotorLog]:
    with path.open('rb') as stream:
        header = stream.read(16)
    if not header.startswith(b'ULog\x01\x12\x35'):
        raise ValueError('无效 ULog 文件头；可能是未完整下载、全零或损坏文件')
    # Retain actual navigation mode as well as motor commands, not RC intention.
    ulog = ULog(str(path), message_name_filter_list=['actuator_motors', 'vehicle_status'])
    mode_time = np.array([], dtype=float)
    mode_state = np.array([], dtype=int)
    status = next((d.data for d in ulog.data_list
                   if d.name == 'vehicle_status' and d.multi_id == 0), None)
    if status is not None and 'timestamp' in status and 'nav_state' in status:
        timestamps = np.asarray(status['timestamp'], dtype=float)
        states = np.asarray(status['nav_state'], dtype=float)
        valid = np.isfinite(timestamps) & np.isfinite(states)
        timestamps, states = timestamps[valid], states[valid].astype(int)
        order = np.argsort(timestamps, kind='stable')
        timestamps, states = timestamps[order], states[order]
        if len(timestamps):
            # The last publication at a repeated timestamp determines the mode.
            keep = np.r_[timestamps[1:] != timestamps[:-1], True]
            timestamps, states = timestamps[keep], states[keep]
            changed = np.r_[True, states[1:] != states[:-1]]
            mode_time = (timestamps[changed] - ulog.start_timestamp) / 1e6
            mode_state = states[changed]
    results = []
    for dataset in ulog.data_list:
        if dataset.name != 'actuator_motors':
            continue
        data = dataset.data
        if 'timestamp' not in data or not len(data['timestamp']):
            continue
        raw_time = np.asarray(data['timestamp'], dtype=np.float64)
        # Keep chronological samples; do not interpret stale pre-log timestamps
        # as calendar time. Duplicate samples are harmless for CSV/plotting.
        order = np.argsort(raw_time, kind='stable')
        times = (raw_time[order] - ulog.start_timestamp) / 1e6
        motors = {}
        for index in range(4):
            key = f'control[{index}]'
            if key not in data:
                continue
            values = np.asarray(data[key], dtype=np.float64)[order]
            if np.any(np.isfinite(values)):
                motors[index + 1] = values
        if motors:
            results.append(MotorLog(path, dataset.multi_id, ulog.start_timestamp, times, motors,
                                    mode_time, mode_state))
    if not results:
        raise ValueError('未找到有效 actuator_motors.control[] 数据；旧版日志需单独适配')
    return results


def plot_samples(t: np.ndarray, y: np.ndarray, budget: int = 20000):
    """Reduce display points while preserving sampled extrema and NaN gaps."""
    if len(t) <= budget:
        return t, y
    indices = {0, len(t) - 1}
    for block in np.array_split(np.arange(len(t)), budget // 4):
        indices.update((int(block[0]), int(block[-1])))
        valid = block[np.isfinite(y[block])]
        if len(valid):
            indices.add(int(valid[np.argmin(y[valid])]))
            indices.add(int(valid[np.argmax(y[valid])]))
    edges = np.flatnonzero(np.diff(np.isfinite(y).astype(int)))
    indices.update(edges.tolist())
    indices.update((edges + 1).tolist())
    selected = np.array(sorted(indices))
    return t[selected], y[selected]


def discover(paths: list[Path], recursive: bool) -> list[Path]:
    files = []
    for path in paths:
        if path.is_dir():
            candidates = path.rglob('*') if recursive else path.glob('*')
            files.extend(p.resolve() for p in candidates if p.is_file() and p.suffix.lower() == '.ulg')
        else:
            files.append(path.resolve())
    return sorted(set(files), key=lambda p: str(p).lower())


class Viewer:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title('PX4 ULog 电机控制量 · 批量分析')
        self.root.geometry('1280x850')
        self.root.minsize(950, 650)
        self.records: dict[str, MotorLog] = {}
        self.errors: dict[str, str] = {}
        self.loaded: set[Path] = set()
        self.events = queue.Queue()
        self.busy = False
        self.serial = 0
        self.status = tk.StringVar(value='请选择 ULog 文件或文件夹。支持 Ctrl / Shift 多选日志进行叠加。')
        self.recursive = tk.BooleanVar(value=False)
        self.channels = [tk.BooleanVar(value=True) for _ in range(4)]
        self.show_modes = tk.BooleanVar(value=True)
        self.x_step = tk.StringVar(value='自动')
        self.operation_buttons = []
        self.build_ui()
        self.root.after(100, self.poll)

    def build_ui(self):
        bar = ttk.Frame(self.root, padding=8)
        bar.pack(fill='x')
        for title, callback in [
            ('添加文件', self.choose_files), ('添加文件夹', self.choose_folder),
            ('清空列表', self.clear), ('导出全部 CSV', self.export_csv),
            ('保存当前曲线 PNG', self.save_png),
        ]:
            button = ttk.Button(bar, text=title, command=callback)
            button.pack(side='left', padx=3)
            self.operation_buttons.append(button)
        ttk.Checkbutton(bar, text='包含子文件夹', variable=self.recursive).pack(side='left', padx=10)
        ttk.Label(self.root, text='控制量为飞控分配指令，不代表实际转速或推力，也不能单独证明电机故障。',
                  foreground='#9a4e16', padding=(10, 0, 10, 6)).pack(anchor='w')
        pane = ttk.Panedwindow(self.root, orient='horizontal')
        pane.pack(fill='both', expand=True, padx=8)
        left = ttk.Frame(pane)
        right = ttk.Frame(pane)
        pane.add(left, weight=1)
        pane.add(right, weight=3)
        ttk.Label(left, text='已导入日志（多选可对比）').pack(anchor='w', pady=4)
        table = ttk.Frame(left)
        table.pack(fill='both', expand=True)
        self.tree = ttk.Treeview(table, columns=('span', 'motors', 'peak', 'result'), selectmode='extended')
        self.tree.heading('#0', text='文件 / 实例')
        self.tree.column('#0', width=230, minwidth=150)
        for key, label, width in [('span', '跨度(s)', 75), ('motors', '通道数', 60),
                                  ('peak', '最大控制量', 85), ('result', '状态', 90)]:
            self.tree.heading(key, text=label)
            self.tree.column(key, width=width, minwidth=55)
        vs = ttk.Scrollbar(table, orient='vertical', command=self.tree.yview)
        hs = ttk.Scrollbar(table, orient='horizontal', command=self.tree.xview)
        self.tree.configure(yscrollcommand=vs.set, xscrollcommand=hs.set)
        self.tree.grid(row=0, column=0, sticky='nsew')
        vs.grid(row=0, column=1, sticky='ns')
        hs.grid(row=1, column=0, sticky='ew')
        table.rowconfigure(0, weight=1)
        table.columnconfigure(0, weight=1)
        self.tree.tag_configure('error', foreground='#b03030')
        self.tree.bind('<<TreeviewSelect>>', lambda event: self.redraw())
        ttk.Button(left, text='选择全部有效日志', command=self.select_all).pack(anchor='w', pady=6)
        ttk.Label(left, text='选中日志统计 / 读取错误').pack(anchor='w')
        text_frame = ttk.Frame(left)
        text_frame.pack(fill='x')
        self.details = tk.Text(text_frame, height=12, wrap='word', state='disabled')
        scroll = ttk.Scrollbar(text_frame, command=self.details.yview)
        self.details.configure(yscrollcommand=scroll.set)
        self.details.pack(side='left', fill='both', expand=True)
        scroll.pack(side='right', fill='y')
        selector = ttk.Frame(right)
        selector.pack(fill='x', pady=4)
        for index, variable in enumerate(self.channels):
            ttk.Checkbutton(selector, text=f'M{index+1}', variable=variable, command=self.redraw).grid(
                row=0, column=index, padx=5, sticky='w')
        ttk.Checkbutton(selector, text='显示飞行模式分区', variable=self.show_modes,
                        command=self.redraw).grid(row=0, column=4, padx=8, sticky='w')
        ttk.Label(selector, text='时间主刻度(s)：').grid(row=1, column=0, columnspan=2, sticky='w', pady=5)
        steps = ttk.Combobox(selector, textvariable=self.x_step, state='readonly', width=7,
                             values=('自动', '0.2', '0.5', '1', '2', '5', '10', '30'))
        steps.grid(row=1, column=2, sticky='w')
        steps.bind('<<ComboboxSelected>>', lambda event: self.redraw())
        ttk.Label(selector, text='控制量：主刻度 0.05 / 次刻度 0.01').grid(
            row=1, column=3, columnspan=2, padx=8, sticky='w')
        self.figure = Figure(figsize=(9, 6), dpi=100, constrained_layout=True)
        self.ax = self.figure.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.figure, master=right)
        self.canvas.get_tk_widget().pack(fill='both', expand=True)
        NavigationToolbar2Tk(self.canvas, right).update()
        ttk.Label(self.root, textvariable=self.status, relief='sunken', anchor='w', padding=6).pack(fill='x')
        self.redraw()

    def set_busy(self, value: bool):
        self.busy = value
        for button in self.operation_buttons:
            button.configure(state='disabled' if value else 'normal')

    def choose_files(self):
        if self.busy:
            return
        names = filedialog.askopenfilenames(title='选择 ULog 文件', filetypes=[('PX4 ULog', '*.ulg'), ('所有文件', '*.*')])
        if names:
            self.load([Path(name) for name in names])

    def choose_folder(self):
        if self.busy:
            return
        folder = filedialog.askdirectory(title='选择日志文件夹')
        if folder:
            self.load([Path(folder)])

    def load(self, paths: list[Path]):
        if self.busy:
            return
        recursive = self.recursive.get()
        known = self.loaded.copy()
        self.set_busy(True)
        self.status.set('正在扫描并读取日志…')

        def worker():
            successes = failures = skipped = 0
            try:
                files = discover(paths, recursive)
                for index, path in enumerate(files, 1):
                    if path in known:
                        skipped += 1
                        continue
                    self.events.put(('status', f'读取 {index}/{len(files)}：{path.name}'))
                    try:
                        records = read_log(path)
                        self.events.put(('loaded', records))
                        successes += 1
                    except Exception as exc:
                        self.events.put(('error', (path, f'{type(exc).__name__}: {exc}')))
                        failures += 1
                self.events.put(('done', f'导入完成：成功 {successes}，失败 {failures}，跳过已导入 {skipped}。'))
            except Exception as exc:
                self.events.put(('done', f'扫描失败：{exc}'))
        threading.Thread(target=worker, daemon=True).start()

    def poll(self):
        try:
            while True:
                kind, payload = self.events.get_nowait()
                if kind == 'status':
                    self.status.set(payload)
                elif kind == 'loaded':
                    for record in payload:
                        self.serial += 1
                        iid = str(self.serial)
                        self.records[iid] = record
                        self.loaded.add(record.path)
                        peak = max(np.nanmax(v) for v in record.motors.values())
                        self.tree.insert('', 'end', iid=iid,
                            text=f'{record.path.name} [实例 {record.instance}]',
                            values=(f'{record.time[-1]-record.time[0]:.2f}', len(record.motors), f'{peak:.3f}', '成功'))
                        if not self.tree.selection():
                            self.tree.selection_set(iid)
                elif kind == 'error':
                    path, error = payload
                    iid = self.tree.insert('', 'end', text=path.name, values=('', '', '', '失败'), tags=('error',))
                    self.errors[iid] = f'{path}\n{error}'
                elif kind == 'done':
                    self.set_busy(False)
                    self.status.set(payload)
                    self.redraw()
        except queue.Empty:
            pass
        self.root.after(100, self.poll)

    def select_all(self):
        self.tree.selection_set(list(self.records))
        self.redraw()

    def clear(self):
        if self.busy:
            return
        self.tree.delete(*self.tree.get_children())
        self.records.clear()
        self.loaded.clear()
        self.errors.clear()
        self.redraw()
        self.status.set('已清空，可以重新导入更新后的日志。')

    def redraw(self):
        self.ax.clear()
        lines = []
        selected = [(iid, self.records[iid]) for iid in self.tree.selection() if iid in self.records]
        for iid in self.tree.selection():
            if iid in self.errors:
                lines.append(self.errors[iid])
        styles = ['-', '--', '-.', ':']
        colors = matplotlib.colormaps['tab20']
        line_count = 0
        for log_index, (iid, record) in enumerate(selected):
            lines.append(f'[{iid}] {record.path} / 实例 {record.instance}')
            regions = record.mode_regions()
            if self.show_modes.get():
                # A single log gets full-height background regions. Each compared
                # log gets its own labelled band, preventing contradictory overlays.
                count = len(selected)
                band_height = min(.09, .32 / max(1, count))
                bottom = log_index * band_height if count > 1 else 0
                top = bottom + band_height if count > 1 else 1
                span = max(float(record.time[-1] - record.time[0]), .001)
                for begin, end, state in regions:
                    color = MODE_COLORS.get(state, '#aaaaaa')
                    self.ax.axvspan(begin, end, ymin=bottom, ymax=top, facecolor=color,
                                    alpha=.30 if count > 1 else .15, zorder=0)
                    self.ax.axvline(begin, color=color, alpha=.4, linewidth=.6, zorder=0)
                    if end - begin >= span * .035:
                        label = mode_name(state)
                        if count > 1:
                            label = f'[{iid}] {label}'
                        self.ax.text((begin+end)/2, (bottom+top)/2 if count > 1 else .98,
                                     label, transform=self.ax.get_xaxis_transform(), ha='center',
                                     va='center' if count > 1 else 'top', fontsize=8, clip_on=True,
                                     color='#444444', zorder=4)
            lines.append('实际飞行模式（vehicle_status.nav_state）：')
            for begin, end, state in regions:
                lines.append(f'  {begin:.2f}～{end:.2f}s：{mode_name(state)}')
            if not len(record.mode_time):
                lines.append('  缺少模式数据，不能从遥控开关或文件名推断模式。')
            for motor, values in record.motors.items():
                if not self.channels[motor-1].get():
                    continue
                x, y = plot_samples(record.time, values)
                self.ax.plot(x, y, color=colors((motor-1) % 20), linestyle=styles[log_index % len(styles)],
                             label=f'[{iid}] M{motor}', linewidth=1.2)
                line_count += 1
                stat = record.stats(motor)
                lines.append(
                    f'M{motor}: 均值 {stat["mean"]:.3f}，峰值 {stat["maximum"]:.3f}'
                    f' @ {stat["peak_time_s"]:.2f}s\n'
                    f'  ≥0.95 样本占比 {stat["samples_ge_095_percent"]:.1f}%，'
                    f'≥0.999 累计估算 {stat["estimated_seconds_ge_0999"]:.2f}s')
            lines.append('')
        self.ax.set(title='电机控制指令（不是实际转速）', xlabel='相对各自日志开始时间 / s', ylabel='控制量')
        # Reversible motors may legitimately have negative commands. Do not clip.
        minimum = min((float(np.nanmin(v)) for _, r in selected for v in r.motors.values()), default=0)
        maximum = max((float(np.nanmax(v)) for _, r in selected for v in r.motors.values()), default=1)
        self.ax.set_ylim(min(-.05, minimum-.05), max(1.05, maximum+.05))
        self.ax.axhline(.95, color='#999999', linestyle=':', linewidth=.8)
        self.ax.axhline(1, color='#bb5555', linestyle=':', linewidth=.8)
        if self.x_step.get() == '自动':
            self.ax.xaxis.set_major_locator(MaxNLocator(nbins=16, min_n_ticks=5))
        else:
            # Avoid thousands of ticks when a tiny fixed interval is chosen for
            # a long log. Zooming in automatically restores the chosen interval.
            self.ax.xaxis.set_major_locator(AdaptiveTimeLocator(float(self.x_step.get())))
        self.ax.xaxis.set_minor_locator(AutoMinorLocator(5))
        self.ax.yaxis.set_major_locator(MultipleLocator(.05))
        self.ax.yaxis.set_minor_locator(AutoMinorLocator(5))
        self.ax.tick_params(axis='both', which='major', labelsize=8)
        self.ax.tick_params(axis='both', which='minor', length=2)
        self.ax.grid(which='major', alpha=.3, linewidth=.6)
        self.ax.grid(which='minor', alpha=.15, linewidth=.4, linestyle=':')
        if line_count:
            self.ax.legend(fontsize=8, ncol=min(4, max(1, (line_count+7)//8)), loc='best')
        else:
            self.ax.text(.5, .5, '选择有效日志和电机通道以显示曲线', ha='center', transform=self.ax.transAxes)
        self.details.configure(state='normal')
        self.details.delete('1.0', 'end')
        self.details.insert('1.0', '\n'.join(lines) or '支持多选日志；颜色区分电机，线型区分日志。')
        self.details.configure(state='disabled')
        self.canvas.draw_idle()

    def export_csv(self):
        if self.busy:
            return
        if not self.records:
            messagebox.showinfo('没有数据', '请先导入有效日志。')
            return
        folder = filedialog.askdirectory(title='选择导出目录（将新建一个结果子文件夹）')
        if not folder:
            return
        records = list(self.records.values())
        self.set_busy(True)
        self.status.set('正在导出全部原始采样数据…')

        def worker():
            from datetime import datetime
            try:
                base = Path(folder) / ('ulg_motors_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
                base.mkdir()
                stat_rows = []
                for index, record in enumerate(records, 1):
                    self.events.put(('status', f'导出 {index}/{len(records)}：{record.path.name}'))
                    digest = hashlib.sha256(str(record.path).encode('utf-8')).hexdigest()[:8]
                    name = f'{record.path.stem}_{digest}_instance{record.instance}.csv'
                    motors = sorted(record.motors)
                    with (base / name).open('w', encoding='utf-8-sig', newline='') as stream:
                        writer = csv.writer(stream)
                        writer.writerow(['relative_log_time_s', 'timestamp_boot_us', 'nav_state', 'flight_mode'] +
                                        [f'motor_{m}_command' for m in motors])
                        modes = record.modes_at(record.time)
                        for i, time in enumerate(record.time):
                            writer.writerow([f'{time:.6f}', round(time*1e6+record.start_us),
                                             int(modes[i]), mode_name(int(modes[i]))] +
                                [record.motors[m][i] if np.isfinite(record.motors[m][i]) else '' for m in motors])
                    for motor in motors:
                        stat_rows.append({'source_file': str(record.path), 'instance': record.instance,
                                          'motor_sample_span_s': float(record.time[-1]-record.time[0]),
                                          **record.stats(motor)})
                with (base / 'motor_statistics.csv').open('w', encoding='utf-8-sig', newline='') as stream:
                    writer = csv.DictWriter(stream, fieldnames=list(stat_rows[0]))
                    writer.writeheader()
                    writer.writerows(stat_rows)
                self.events.put(('done', f'导出完成：{base}'))
            except Exception as exc:
                self.events.put(('done', f'导出失败，可能已写入部分文件：{exc}'))
        threading.Thread(target=worker, daemon=True).start()

    def save_png(self):
        if self.busy:
            return
        if not any(iid in self.records for iid in self.tree.selection()):
            messagebox.showinfo('没有曲线', '请先选择有效日志。')
            return
        name = filedialog.asksaveasfilename(title='保存当前曲线', defaultextension='.png', filetypes=[('PNG 图片', '*.png')])
        if name:
            try:
                self.figure.savefig(name, dpi=180)
                self.status.set(f'图片已保存：{name}')
            except Exception as exc:
                messagebox.showerror('保存失败', str(exc))

class AdaptiveTimeLocator(MultipleLocator):
    """Keep a requested interval unless the current view would exceed 100 ticks."""
    def __init__(self, requested: float):
        super().__init__(requested)
        self.requested = requested

    def tick_values(self, vmin, vmax):
        multiplier = max(1, int(np.ceil(abs(vmax-vmin) / (100 * self.requested))))
        self.set_params(base=self.requested * multiplier)
        return super().tick_values(vmin, vmax)


def main():
    root = tk.Tk()
    viewer = Viewer(root)
    if len(sys.argv) > 1:
        root.after(200, lambda: viewer.load([Path(arg) for arg in sys.argv[1:]]))
    root.mainloop()


if __name__ == '__main__':
    main()
