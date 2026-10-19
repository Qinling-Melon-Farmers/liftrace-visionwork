#!/usr/bin/env python3
"""固定旧验证/测试集及困难图像，记录模型对照；不将训练视频当泛化验收。"""
import argparse
import json
from pathlib import Path

from ultralytics import YOLO


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', required=True)
    parser.add_argument('--data', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--images', nargs='*', default=[])
    parser.add_argument('--device', default='0')
    args = parser.parse_args()
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    model = YOLO(args.model)
    summary = {'model': str(Path(args.model).resolve()), 'data': args.data,
               'imgsz': 640, 'prediction_conf': .5, 'prediction_iou': .45,
               'prediction_rect': False, 'names': model.names, 'splits': {}, 'images': []}
    for split in ('val', 'test'):
        result = model.val(data=args.data, split=split, device=args.device,
                           imgsz=640, batch=16, workers=2, plots=True,
                           project=str(output), name=split, verbose=False)
        summary['splits'][split] = {
            'metrics': {k: float(v) for k, v in result.results_dict.items()},
            'map_by_class': result.box.maps.tolist(),
            'measured_classes': [
                {'class_id': int(cid), 'class_name': model.names[int(cid)],
                 'precision': float(result.box.p[i]), 'recall': float(result.box.r[i]),
                 'ap50': float(result.box.ap50[i]), 'ap50_95': float(result.box.ap[i])}
                for i, cid in enumerate(result.box.ap_class_index)],
            'confusion_matrix': result.confusion_matrix.matrix.tolist(),
        }
    for folder in args.images:
        for path in sorted(Path(folder).glob('*.jpg')):
            if not (path.name.startswith('frame_') or path.name.startswith('source_')):
                continue
            result = model.predict(str(path), imgsz=640, device=args.device,
                                   conf=.5, iou=.45, rect=False, verbose=False)[0]
            summary['images'].append({'path': str(path), 'detections': [
                {'class': model.names[int(box.cls.item())],
                 'confidence': float(box.conf.item()),
                 'xyxy': box.xyxy.cpu().numpy()[0].tolist()}
                for box in result.boxes]})
    (output / 'summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    print(json.dumps(summary['splits'], indent=2))


if __name__ == '__main__':
    main()
