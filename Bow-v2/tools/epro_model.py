import json,collections,re,math
from pathlib import Path
import zipfile
import json,collections
from pathlib import Path

def parse_text(text):
 ds=[];d=None
 for line in text.splitlines():
  if '||' not in line:continue
  a,b=line.split('||',1)
  if not b.rstrip('|'):continue
  a=json.loads(a);b=json.loads(b.rstrip('|'))
  if a['type']=='DOCHEAD':
   d={'head':b,'rows':[]};ds.append(d)
  d['rows'].append((a,b))
 return ds

def attrs(d):
 a=collections.defaultdict(dict)
 for h,b in d['rows']:
  if h['type']=='ATTR':a[b['parentId']][b['key']]=b['value']
 return a


def meta(d):return next((b for h,b in d['rows'] if h['type']=='META'),{})
def key(x):return [int(t) if t.isdigit() else t for t in re.split(r'(\d+)',x)]
def model(version,board):
 z=zipfile.ZipFile(Path(__file__).resolve().parents[1]/'design'/f'{version}.epro2')
 ds=parse_text(z.read(next(n for n in z.namelist() if n.endswith('.epru'))).decode());lookup={d['head']['uuid']:d for d in ds}
 pcb=next(d for d in ds if d['head']['docType']=='PCB' and meta(d)['title']==board)
 sch=next(d for d in ds if d['head']['docType']=='SCH' and meta(d)['board']==meta(pcb)['board'])
 page=next(d for d in ds if d['head']['docType']=='SCH_PAGE' and meta(d)['schematic']==sch['head']['uuid'])
 pa=attrs(pcb);sa=attrs(page);parts={};pn={};nets=collections.defaultdict(list)
 for h,b in pcb['rows']:
  if h['type']=='COMPONENT':
   at=pa[h['id']];ref=at.get('Designator');dev=meta(lookup[at['Device']]);aa={**dev.get('attributes',{}),**at}
   parts[ref]={'id':h['id'],'value':aa.get('Value',''),'model':aa.get('Manufacturer Part',dev.get('title','')),'supplier':aa.get('Supplier Part',''),'footprint':meta(lookup[at['Footprint']]).get('title',''),'pins':{},'xy':[b['x'],b['y']]}
 for h,b in pcb['rows']:
  if h['type']=='PAD_NET':
   _,c,p,*_=json.loads(h['id']);ref=pa[c]['Designator'];pn[ref+'.'+p]=b['padNet'];parts[ref]['pins'][p]={'net':b['padNet'],'name':p}
   if b['padNet']:nets[b['padNet']].append(ref+'.'+p)
 for h,b in page['rows']:
  if h['type']!='COMPONENT':continue
  at=sa[h['id']];ref=at.get('Designator');sym=lookup.get(at.get('Symbol'))
  if not sym or ref not in parts:continue
  aa=attrs(sym)
  for hh,p in sym['rows']:
   if hh['type']=='PIN' and p.get('partId')==b['partId']:
    n=aa[hh['id']].get('Pin Number');name=aa[hh['id']].get('Pin Name','')
    if n in parts[ref]['pins']:parts[ref]['pins'][n]['name']=name
 # Check all schematic wire groups against PCB pad assignments.
 groups=collections.defaultdict(set);lines=[b for h,b in page['rows'] if h['type']=='LINE']
 for h,b in page['rows']:
  if h['type']!='COMPONENT':continue
  at=sa[h['id']];ref=at.get('Designator');sym=lookup.get(at.get('Symbol'))
  if not sym:continue
  aa=attrs(sym)
  for hh,p in sym['rows']:
   if hh['type']!='PIN' or p.get('partId')!=b['partId']:continue
   n=aa[hh['id']].get('Pin Number');x=p['x']*(-1 if b['isMirror'] else 1);y=p['y'];ang=-math.radians(b['rotation'])
   X=b['x']+x*math.cos(ang)-y*math.sin(ang);Y=b['y']+x*math.sin(ang)+y*math.cos(ang)
   for l in lines:
    x1,y1,x2,y2=[l[z] for z in ['startX','startY','endX','endY']]
    if min(x1,x2)-.001<=X<=max(x1,x2)+.001 and min(y1,y2)-.001<=Y<=max(y1,y2)+.001 and abs((X-x1)*(y2-y1)-(Y-y1)*(x2-x1))<.001:groups[l['lineGroup']].add(ref+'.'+n)
 mismatch=[]
 for g,ps in groups.items():
  ns={pn[p] for p in ps if p in pn}
  if len(ns)>1:mismatch.append([sorted(ps),sorted(ns)])
 return {'version':version,'board':board,'parts':parts,'nets':dict(nets),'mismatch':mismatch},pcb,page
