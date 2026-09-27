"""Export five-class YOLO11 weights and an RK3588 FP16 candidate; no hardware access."""
import argparse,json,shutil
from pathlib import Path
import numpy as np
import onnx,onnxruntime as ort
import torch
from ultralytics import YOLO
NAMES=['bridge','panzer','pillbox','tent','red_cross']
def main():
 p=argparse.ArgumentParser();p.add_argument('--weights',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--skip-rknn',action='store_true');a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
 torch.set_num_threads(4);model=YOLO(str(a.weights));assert list(model.names.values())==NAMES,model.names
 pt=a.output/'flight_5cls_20260928.pt'
 if a.weights.resolve()!=pt.resolve():shutil.copy2(a.weights,pt)
 model=YOLO(str(pt));path=model.export(format='onnx',imgsz=640,batch=1,dynamic=False,opset=12,simplify=False,nms=False,device='cpu')
 session=ort.InferenceSession(str(path),providers=['CPUExecutionProvider']);shape=session.get_outputs()[0].shape
 assert shape==[1,9,8400],shape
 info=dict(names=NAMES,onnx_output=shape,source_weights=str(a.weights),rknn_status='not_attempted',hardware_validated=False)
 (a.output/'export.json').write_text(json.dumps(info,indent=2))
 if a.skip_rknn:return
 from rknn.api import RKNN
 r=RKNN(verbose=False)
 try:
  assert r.config(target_platform='rk3588',mean_values=[[0,0,0]],std_values=[[1,1,1]])==0
  assert r.load_onnx(model=str(path))==0
  assert r.build(do_quantization=False)==0
  out=a.output/'flight_5cls_20260928_fp16.rknn';assert r.export_rknn(str(out))==0
  info.update(rknn_status='exported',rknn=str(out),toolkit_version='2.3.2',input='RGB float32 normalized 0..1 NHWC')
 except Exception as e:
  info.update(rknn_status='failed',error=repr(e));raise
 finally:
  r.release();(a.output/'export.json').write_text(json.dumps(info,indent=2))
if __name__=='__main__':main()
