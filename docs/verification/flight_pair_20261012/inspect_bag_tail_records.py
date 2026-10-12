import struct,json
from pathlib import Path
out=Path('/home/xhj/liftrace-deliverables/flight_review_20261012');p=Path(json.load(open(out/'original_sources.json'))['full']);size=p.stat().st_size
records=[]
with p.open('rb') as f:
 magic=f.readline()
 while f.tell()<size:
  pos=f.tell();buf=f.read(4)
  if len(buf)<4:records.append(dict(pos=pos,error='missing header size'));break
  n=struct.unpack('<I',buf)[0]
  if n>1000000:records.append(dict(pos=pos,error='invalid header length',length=n));break
  h=f.read(n); fields={};i=0
  while i+4<=len(h):
   l=struct.unpack('<I',h[i:i+4])[0];i+=4;k,_,v=h[i:i+l].partition(b'=');i+=l;fields[k.decode(errors='replace')]=v
  buf=f.read(4)
  if len(buf)<4:records.append(dict(pos=pos,error='missing data size'));break
  d=struct.unpack('<I',buf)[0];end=f.tell()+d
  rec=dict(pos=pos,op=fields.get('op',b'').hex(),data_bytes=d,expected_end=end,available=size-f.tell())
  if end>size:rec['error']='truncated data';records.append(rec);break
  records.append(rec);f.seek(end)
result=dict(source=str(p),bytes=size,record_count=len(records),last_records=records[-4:]);json.dump(result,open(out/'full/tail_integrity.json','w'),indent=2,ensure_ascii=False);print(json.dumps(result,indent=2))