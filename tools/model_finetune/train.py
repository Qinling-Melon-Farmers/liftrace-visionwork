"""Transfer existing six-class YOLO11n to five classes, then fine-tune offline."""
import argparse,copy,json,os
from pathlib import Path
os.environ.setdefault('WANDB_MODE','disabled')
import torch
from ultralytics import YOLO
from ultralytics.nn.tasks import DetectionModel
from prepare import NAMES

def main():
    p=argparse.ArgumentParser();p.add_argument('--baseline',type=Path,required=True);p.add_argument('--data',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--epochs',type=int,default=30);p.add_argument('--batch',type=int,default=16);a=p.parse_args()
    torch.set_num_threads(4);a.output.mkdir(parents=True,exist_ok=True)
    old=YOLO(str(a.baseline));assert list(old.names.values())==NAMES[:4]+['tank','red_cross']
    new=DetectionModel(copy.deepcopy(old.model.yaml),ch=3,nc=5,verbose=False);new.load(old.model,verbose=False)
    source=old.model.float().state_dict();state=new.state_dict();changed=[]
    for k,v in state.items():
        if k in source and source[k].shape!=v.shape:
            if '.cv3.' not in k or source[k].shape[0]!=6 or v.shape[0]!=5 or source[k].shape[1:]!=v.shape[1:]:raise RuntimeError('Unexpected head mismatch '+k)
            state[k]=source[k][[0,1,2,3,5]].clone();changed.append(k)
    assert len(changed)==6,changed
    new.load_state_dict(state);new.names=dict(enumerate(NAMES))
    for k in changed:assert torch.equal(new.state_dict()[k],source[k][[0,1,2,3,5]])
    ckpt=copy.deepcopy(old.ckpt);ckpt.update(model=new.half(),ema=None,optimizer=None,epoch=-1,best_fitness=None)
    initial=a.output/'initial_5cls.pt';torch.save(ckpt,initial)
    (a.output/'transfer.json').write_text(json.dumps(dict(baseline=str(a.baseline),class_rows=[0,1,2,3,5],remapped_tensors=changed,names=NAMES),indent=2))
    detector=YOLO(str(initial))
    detector.train(data=str(a.data),project=str(a.output),name='candidate',exist_ok=False,epochs=a.epochs,patience=10,batch=a.batch,imgsz=640,device=0,workers=4,cache=False,optimizer='AdamW',lr0=.0005,lrf=.1,cos_lr=True,warmup_epochs=2,weight_decay=.0005,seed=928,deterministic=True,amp=True,degrees=180,translate=.08,scale=.25,flipud=.5,fliplr=.5,hsv_h=.01,hsv_s=.3,hsv_v=.25,mosaic=.3,close_mosaic=5,mixup=0,plots=True,save_period=-1,verbose=False)
    print('TRAINING_COMPLETE',a.output/'candidate/weights/best.pt',flush=True)
if __name__=='__main__':main()
