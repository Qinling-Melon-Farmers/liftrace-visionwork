"""Toolkit CPU simulator check, NOT RK3588 NPU flight acceptance."""
import argparse,ast,json
from pathlib import Path
import cv2,numpy as np,onnxruntime as ort,yaml
from rknn.api import RKNN

def main():
 p=argparse.ArgumentParser();p.add_argument('--export',type=Path,required=True);p.add_argument('--image',type=Path,required=True);p.add_argument('--decoder',type=Path,required=True);p.add_argument('--metadata',type=Path,required=True);a=p.parse_args()
 tree=ast.parse(a.decoder.read_text());ns=dict(np=np,cv2=cv2);exec(compile(ast.Module(body=[n for n in tree.body if isinstance(n,ast.FunctionDef)],type_ignores=[]),str(a.decoder),'exec'),ns)
 meta=yaml.safe_load(a.metadata.read_text());im=cv2.imread(str(a.image));x,scale,pad,_=ns['_to_model_input'](im,640)
 path=a.export/'flight_5cls_20260928.onnx';r=RKNN(verbose=False);result=dict(hardware_tested=False)
 try:
  assert r.config(target_platform='rk3588',mean_values=[[0,0,0]],std_values=[[1,1,1]])==0
  assert r.load_onnx(model=str(path))==0
  assert r.build(do_quantization=False)==0
  assert r.init_runtime()==0
  outs=r.inference(inputs=[x],data_format=['nhwc']);ns['_validate_output_contract'](outs,meta)
  dets=ns['_decode_outputs'](outs,5,.5,640,im.shape,scale,pad,box_format='xywh')
  session=ort.InferenceSession(str(path),providers=['CPUExecutionProvider']);ref=session.run(None,{session.get_inputs()[0].name:x.transpose(0,3,1,2)})
  reference=ns['_decode_outputs'](ref,5,.5,640,im.shape,scale,pad,box_format='xywh')
  result.update(status='PASS',output_shape=list(outs[0].shape),max_abs=float(np.abs(outs[0]-ref[0]).max()),detections=[dict(class_name=meta['names'][d['class_id']],score=d['score'],box=d['bbox'].tolist()) for d in dets])
  assert [d['class_id'] for d in dets]==[d['class_id'] for d in reference],result
 except Exception as e:result.update(status='FAIL',error=repr(e));raise
 finally:r.release();(a.export/'rknn_simulator.json').write_text(json.dumps(result,indent=2))
 print(json.dumps(result))
if __name__=='__main__':main()
