#!/usr/bin/env python3
"""Create final training handoff only after training and evaluation have finished."""
import argparse
import csv
import json
import shutil
import zipfile
from datetime import datetime, timezone
from pathlib import Path
import yaml

NAMES=['bridge','panzer','pillbox','tent','red_cross']


def dump(path,obj):
    path.write_text(json.dumps(obj,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,required=True);p.add_argument('--evaluation',type=Path,required=True)
    p.add_argument('--recipe',type=Path,required=True);p.add_argument('--sources',type=Path,required=True)
    p.add_argument('--decisions',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    summary=json.loads((a.evaluation/'summary.json').read_text())
    stats=json.loads(a.sources.read_text());roi=json.loads((a.evaluation/'bag_roi_comparison.json').read_text())
    actual=sum(g['frames'] for g in summary['bag_counts'].values())
    if actual!=805:raise ValueError('Expected all 805 bag frames')
    if list(summary['names'].values())!=NAMES:raise ValueError('Incorrect model class order')
    dataset=Path(stats['dataset']);manifest=json.loads((dataset/'manifest.json').read_text())
    manifest['review_decisions']=json.loads(a.decisions.read_text());dump(dataset/'manifest.json',manifest)
    for metadata in [a.sources.parent/'review_1fps/review_manifest.json',a.sources.parent/'bbox_proposals/bbox_review.json']:
        raw=json.loads(metadata.read_text());raw['label_scope']=manifest['label_scope'];dump(metadata,raw)
    with (a.run/'results.csv').open() as f:rows=list(csv.DictReader(f))
    best_row=max(rows,key=lambda row:float(row['metrics/mAP50-95(B)']))
    history={'completed_epochs':len(rows),'best_epoch_by_standard_val_fitness':int(best_row['epoch']),
             'elapsed_training_s':float(rows[-1]['time']),'best_standard_val_map50_95':float(best_row['metrics/mAP50-95(B)']),
             'selection_policy':'Use standard training val-selected best.pt exactly once on 805 bag frames; never use bag results to select an epoch.'}
    pt=a.run/'weights/best.pt'
    if not pt.is_file():raise ValueError('Missing final best.pt')
    shutil.copy2(pt,a.output/'best.pt');shutil.copy2(a.recipe,a.output/'recipe.yaml')
    shutil.copy2(a.sources,a.output/'source_statistics.json')
    shutil.copy2(a.run/'results.csv',a.output/'results.csv')
    shutil.copy2(a.decisions,a.output/'annotation_review_decisions.json')
    dump(a.output/'training_history.json',history);dump(a.output/'test_summary.json',summary)
    shutil.copy2(a.evaluation/'bag_roi_comparison.json',a.output/'bag_roi_comparison.json')
    shutil.copy2(a.evaluation/'H_negative_comparison.json',a.output/'H_negative_comparison.json')
    for child in a.evaluation.iterdir():
        if child.is_dir():
            for image in child.glob('confusion_matrix*.png'):
                shutil.copy2(image,a.output/(child.name+'_'+image.name))
    all_models={'old_0928':summary['baseline_splits']['old_0928'],
                'candidate_1011':summary['baseline_splits']['candidate_1011'],'new_v8':summary['splits']}
    with (a.output/'class_metrics.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=['model','split','class_name','precision','recall','ap50','ap50_95']);writer.writeheader()
        for tag,splits in all_models.items():
            for split,metrics in splits.items():
                measured={row['class_name']:row for row in metrics['measured_classes']}
                for cls in NAMES:
                    row=measured.get(cls,{})
                    writer.writerow({'model':tag,'split':split,'class_name':cls,**{key:row.get(key,'NO_GROUND_TRUTH') for key in ('precision','recall','ap50','ap50_95')}})
    report=[
      '新视频整合与五分类候选训练报告（2026-10-11）',
      '高置信错类与未解决ROI须优先查看下方逐项记录；不得以局部类别匹配数量宣称零误报或五类全面改善。',
      f'训练完成 {history["completed_epochs"]} 轮，标准训练 val 最佳第 {history["best_epoch_by_standard_val_fitness"]} 轮；训练耗时 {history["elapsed_training_s"]:.1f} 秒。',
      '只取训练标准 val 选出的 best.pt 评测一次805帧；没有用bag反复选epoch。',
      '当前为候选交付，不宣称五类全面改善，不自动替换现场模型。RKNN转换/打包由主代理转交导出负责人；本报告为笔记本PT结果，无板端性能或实飞结论。',
      '',
      '数据量与标注：',
      'train 2313，历史val 203，历史test 67，总计2583张；另805帧bag全部不训练。',
      '继承v7训练1890=1774旧图+58地堡源帧+58缩小变体；只继承一次，不叠加历史各版本。',
      '新视频81个1fps源帧，保留76、排除19/39/54/63/76序号5帧；76个尺度变体+76个温和曝光/对比度/白平衡变体。所有新视频相关图像仅train。',
      'bridge额外83、red_cross额外112光度变体，共195，人工增强平衡；最终实例bridge306/panzer612/pillbox519/tent421/red_cross306，最大/最小2倍。',
      '新源图实例bridge18/panzer21/pillbox22/tent22/red_cross10，共93次观察；反复拍摄同一靶，不作为93个独立实体或场景。',
      '旧1774中有629个显式光度变体，1145个未标该变换的清单项仍含历史旋转生成图和相关视频帧，不能冒称1145个独立采集源。新76唯一源与新增347离线增强分开统计。',
      '全部train空标签236；其中H仅80=5源+75光度增强，test H为6，全部模拟H/纹理，无真实H负例。',
      '中心黑圈与内部图案为五类统一ROI，红十字也包中心黑圈，不改成仅红笔画；多目标和可辨识裁切实例完整标注。013_2/078_3假框删除，041右边缘漏车补框；不确定帧整帧排除待审。',
      '主代理看完81 full contact、9 crop、7 overlay；类别纠正来自主代理审图，并非用户逐框手标。',
      '',
      '划分与结论限制：',
      '旧bridge_20260624视频train/val混合；IMG_20250319同日序列亦混合。63对跨集pHash相似标记未当作已证明重复。旧val/test只作历史回归，父模型也已见过历史数据。',
      'test缺少pillbox/tent真值；不得用不存在的两类样本推算分数。805帧已多次用于分析和选型，是留出回归/困难集，不是未触碰盲测。',
      'AP为标准验证器全置信度曲线；P/R为其工作点。三模型统一batch1方形640、NMS IoU0.45复算历史标注集。805帧则严格固定conf0.5、IoU0.45、batch1、FP32、按类NMS。',
      '0.7/0.75统计从同一0.5输出筛选，不重复推理、不提高阈值遮蔽错类；整帧top1不是P/R。',
      '',
      '每类P/R/AP50/AP50-95（历史回归）：',
      'split model class P R AP50 AP50-95'
    ]
    for tag,splits in all_models.items():
        for split,metrics in splits.items():
            measured={row['class_name']:row for row in metrics['measured_classes']}
            for cls in NAMES:
                row=measured.get(cls)
                if row:report.append(f'{split} {tag} {cls} '+ ' '.join(f'{row[k]:.4f}' for k in ('precision','recall','ap50','ap50_95')))
                else:report.append(f'{split} {tag} {cls} 无真值样本，不能评估')
    report+=['','混淆矩阵：test_summary.json记录完整矩阵（行=预测、列=真值，末行/列背景），本目录另附图。',
             'H test 6张固定conf0.5误报帧：'+json.dumps(summary['H_negative_false_positive_frames'],ensure_ascii=False),
             '',f'805帧PT回归与人工ROI：{len(roi["reviews"])}处人工审查ROI，非全片穷尽标注。']
    for tag,c in summary['reviewed_roi_counts'].items():
        report.append(f'{tag}: 已审ROI最高置信匹配框类别正确 {c["correct_top1"]}/{c["reviewed_roi_observations"]}；高置信错类框>=0.7 {c["high_conf_wrong_boxes"]}，>=0.75 {c["wrong_boxes_at_conf_075"]}。不是实例泛化召回率或全bag准确率。')
    for r in roi['reviews']:
        parts=[]
        for tag,compared in r['models'].items():
            box=compared['top1'];parts.append(tag+'='+(f'{box["semantic_name"]}@{box["confidence"]:.3f}' if box else '无匹配框'))
        report.append(f'{r["bag"]} {r["frame"]} t={r["image_time_s"]:.3f} 真类={r["truth"]}; '+'; '.join(parts))
    report+=['','仍需真实独立五类完整真值、真实H/普通黑圈负例与板端RKNN一致性/性能验证。未运行ROS仿真、实机或部署；未提交、不修改控制代码或共享日志。']
    (a.output/'TRAINING_REPORT.txt').write_text('\n'.join(report)+'\n',encoding='utf-8')
    h=json.loads((a.evaluation/'H_negative_comparison.json').read_text())
    images=[s['image'] for s in manifest['added_samples'] if not s['augmentation'] and any(b['class_name']=='pillbox' for b in s['objects'])][:1]
    images+=[h['rows'][0]['image']]
    handoff={'agent_name':'Feynman','agent_id':'01a129b2-938c-78b0-973b-170523fa50d1',
      'final':True,'best_pt':str(a.output/'best.pt'),'names':NAMES,'training_run':str(a.run),
      'recipe':str(a.output/'recipe.yaml'),'source_statistics':str(a.output/'source_statistics.json'),
      'test_summary':str(a.output/'TRAINING_REPORT.txt'),'test_summary_json':str(a.output/'test_summary.json'),
      'validation_images':images,'model_id':'flight_5cls_new_video_v8_20261011',
      'precision_handoff':'FP16 nonquantized allowed by current primary-agent instruction; strict FP32 unsupported by installed toolkit. Transparently report precision.',
      'model_status':'candidate; no automatic replacement/deployment','completed_utc':datetime.now(timezone.utc).isoformat()}
    dump(a.output/'handoff.json',handoff)
    archive=a.output.parent/'new_video_v8_training_handoff_20261011.zip'
    if archive.exists():raise ValueError('Archive already exists')
    with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as z:
        for path in sorted(a.output.iterdir()):
            if path.is_file():z.write(path,path.name)
    print(json.dumps({'handoff':str(a.output/'handoff.json'),'archive':str(archive),'training':history},indent=2))


if __name__=='__main__':main()
