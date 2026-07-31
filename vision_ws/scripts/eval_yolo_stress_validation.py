#!/usr/bin/env python3
import argparse,csv,json
from pathlib import Path
from ultralytics import YOLO

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--model",type=Path,required=True)
    p.add_argument("--stress-root",type=Path,required=True)
    p.add_argument("--output",type=Path,required=True)
    p.add_argument("--device",default="0")
    p.add_argument("--imgsz",type=int,default=640)
    p.add_argument("--batch",type=int,default=16)
    p.add_argument("--workers",type=int,default=4)
    a=p.parse_args()
    root=a.stress_root.resolve(); out=a.output.resolve(); out.mkdir(parents=True,exist_ok=True)
    model=YOLO(str(a.model))
    rows=[]
    for yaml_path in sorted(root.glob("condition_*.yaml")):
        condition=yaml_path.stem.replace("condition_","",1)
        print(f"VALIDATING {condition}",flush=True)
        metrics=model.val(data=str(yaml_path),split="val",imgsz=a.imgsz,batch=a.batch,device=a.device,workers=a.workers,plots=False,verbose=False,project=str(out/"runs"),name=condition,exist_ok=True)
        values=metrics.results_dict
        row={"condition":condition,"model":str(a.model)}
        for key,value in values.items():
            try: row[key]=float(value)
            except (TypeError,ValueError): row[key]=str(value)
        rows.append(row)
        print(json.dumps(row,ensure_ascii=False),flush=True)
    fields=sorted({key for row in rows for key in row})
    with (out/"summary.csv").open("w",newline="",encoding="utf-8") as f:
        w=csv.DictWriter(f,fieldnames=fields); w.writeheader(); w.writerows(rows)
    (out/"summary.json").write_text(json.dumps({"model":str(a.model),"stress_root":str(root),"conditions":rows},ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    print(json.dumps({"model":str(a.model),"conditions":len(rows),"output":str(out)},ensure_ascii=False,indent=2))
if __name__=="__main__":
    main()
