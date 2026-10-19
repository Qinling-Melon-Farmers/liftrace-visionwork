"""Use one receipt-time target history for video panels and aligned CSV rows."""
import bisect
import csv
from pathlib import Path


def iter_target_states(rows, fps, count):
    times = [row['t'] for row in rows]
    history = {}
    last = -1
    for index in range(count):
        seconds = index / fps
        position = bisect.bisect_right(times, seconds) - 1
        row = rows[position] if position >= 0 else None
        if row and row['t'] != last:
            last = row['t']
            for target in row['m']['targets']:
                if target['map_valid']:
                    ident = f"{target['class_name']}#{target['id']}"
                    history[ident] = dict(t=row['t'], m=target)
        current = {
            f"{target['class_name']}#{target['id']}"
            for target in (row['m']['targets'] if row and seconds - row['t'] <= .5 else [])
            if target['map_valid']
        }
        yield index, seconds, history.copy(), current


def video_time(seconds):
    milliseconds = round(seconds * 1000)
    hours, rest = divmod(milliseconds, 3600000)
    minutes, rest = divmod(rest, 60000)
    secs, millis = divmod(rest, 1000)
    return f'{hours:02d}:{minutes:02d}:{secs:02d}.{millis:03d}'


def write_target_csvs(out, rows, fps, count, aligned_name='target_coordinates.csv'):
    out = Path(out)
    original_fields = ['bag_seconds', 'class', 'id', 'frame', 'map_valid', 'x', 'y', 'z', 'state']
    with (out / 'target_coordinates_records.csv').open('w', encoding='utf-8-sig', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(original_fields)
        for row in rows:
            for target in row['m']['targets']:
                point = target['map_point']
                writer.writerow([row['t'], target['class_name'], target['id'], target['map_frame'], target['map_valid'],
                                 point.get('x', 0.), point.get('y', 0.), point.get('z', 0.), target['state']])
    fields = original_fields + ['video_frame', 'video_seconds', 'video_time', 'record_bag_seconds',
                                'age_seconds', 'display_status', 'in_video_panel']
    with (out / aligned_name).open('w', encoding='utf-8-sig', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for index, seconds, history, current in iter_target_states(rows, fps, count):
            base = dict(bag_seconds=seconds, video_frame=index, video_seconds=seconds, video_time=video_time(seconds))
            if not history:
                writer.writerow(dict(base, display_status='NO_TARGET', in_video_panel=False))
            for order, (ident, row) in enumerate(history.items()):
                target = row['m']
                point = target['map_point']
                writer.writerow(dict(base, **{
                    'class': target['class_name'], 'id': target['id'], 'frame': target['map_frame'],
                    'map_valid': target['map_valid'], 'x': point.get('x', 0.), 'y': point.get('y', 0.),
                    'z': point.get('z', 0.), 'state': target['state'], 'record_bag_seconds': row['t'],
                    'age_seconds': seconds - row['t'], 'display_status': 'CURRENT' if ident in current else 'HISTORY',
                    'in_video_panel': order < 3,
                }))
