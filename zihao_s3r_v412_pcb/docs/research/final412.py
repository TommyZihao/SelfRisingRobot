# 作者：同济子豪兄 2026-10-26
from pathlib import Path
import json,time,subprocess,sys,concurrent.futures
P=Path(__file__).resolve().parent
right=[f'right_{kind}_{seed}' for kind in ['mirror','warm','scratch'] for seed in [2101,2102]]
forward=[f'forward_{kind}_{seed}' for kind in ['warm','scratch'] for seed in [2201,2202]]+[f'forward_{kind}_{seed}' for kind in ['sin','knots'] for seed in [2301,2302]]+[f'forward_sin_grip_{seed}' for seed in [2401,2402]]
while not all((P/'validation'/n/'summary.json').exists() for n in right+forward):time.sleep(10)
def rank(names):
 d=[json.loads((P/'validation'/n/'summary.json').read_text()) for n in names]
 return sorted(d,key=lambda x:(x['success'],-x['stood'],x.get('mean_forward_m',x.get('mean_turns',0))),reverse=True)
r=rank(right);f=rank(forward);selection=dict(right=r[0]['name'],forward=f[0]['name'],right_ranking=r,forward_ranking=f,selection='validation only',final_seedbase=86000000)
(P/'selection.json').write_text(json.dumps(selection,indent=2));print(selection['right'],selection['forward'],flush=True)
def run(name):
 with open(P/f'final_{name}.log','w') as log:subprocess.run([sys.executable,str(P/'eval412.py'),'--name',name,'--n','100','--seedbase','86000000','--split','final'],stdout=log,stderr=subprocess.STDOUT,check=True)
 print(name,'FINAL DONE',flush=True)
with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:list(pool.map(run,[selection['right'],selection['forward'],'right_41','forward_41','right_mirror_initial']))
