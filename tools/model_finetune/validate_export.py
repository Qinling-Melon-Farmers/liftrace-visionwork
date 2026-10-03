"""Numerical ONNX check and board decoder contract regression; no ROS/hardware."""
import argparse,ast,json
from pathlib import Path
import cv2,numpy as np,onnxruntime as ort,torch,yaml
from ultralytics import YOLO

def main():
 p=argparse.ArgumentParser();p.add_argument('--export',type=Path,required=True);p.add_argument('--image',type=Path,required=True);p.add_argument('--decoder',type=Path,required=True);p.add_argument('--metadata',type=Path,required=True);a=p.parse_args();torch.set_num_threads(2)
 tree=ast.parse(a.decoder.read_text());defs=ast.Module(body=[n for n in tree.body if isinstance(n,ast.FunctionDef)],type_ignores=[]);ns=dict(cv2=cv2,np=np);exec(compile(defs,str(a.decoder),'exec'),ns)
 meta=yaml.safe_load(a.metadata.read_text());im=cv2.imread(str(a.image));tensor,scale,pad,_=ns['_to_model_input'](im,640,layout='NCHW');session=ort.InferenceSession(str(a.export/'flight_5cls_20260928.onnx'),providers=['CPUExecutionProvider']);out=session.run(None,{session.get_inputs()[0].name:tensor})
 ns['_validate_output_contract'](out,meta)
 try:ns['_validate_output_contract']([np.zeros((1,10,8400),np.float32)],meta)
 except ValueError:wrong_rejected=True
 else:raise AssertionError('Six-output weights silently accepted five-class metadata')
 synthetic=np.zeros((1,9,8400),np.float32);synthetic[0,:4,0]=[320,320,100,100];synthetic[0,8,0]=.95
 det=ns['_decode_outputs']([synthetic],5,.5,640,(640,640,3),1.,(0.,0.),box_format=meta['box_format']);assert len(det)==1 and meta['names'][det[0]['class_id']]=='red_cross'
 model=YOLO(str(a.export/'flight_5cls_20260928.pt')).model.eval().float();model.fuse()
 with torch.no_grad():pt=model(torch.from_numpy(tensor));pt=pt[0] if isinstance(pt,tuple) else pt;pt=pt.numpy()
 delta=np.abs(pt-out[0]);assert delta.max()<.02,float(delta.max())
 result=dict(onnx_shape=list(out[0].shape),pt_onnx_max_abs=float(delta.max()),wrong_six_class_rejected=wrong_rejected,red_cross_id=4,board_decoder_detections=[dict(class_name=meta['names'][d['class_id']],score=d['score'],box=d['bbox'].tolist()) for d in ns['_decode_outputs'](out,5,.5,640,im.shape,scale,pad,box_format=meta['box_format'])],hardware_tested=False)
 (a.export/'validation.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
if __name__=='__main__':main()
