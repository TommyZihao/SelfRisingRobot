# 作者：同济子豪兄 2026-10-26
from pathlib import Path
import subprocess,sys,time,concurrent.futures
P=Path(__file__).resolve().parent
names=[f'right_{kind}_{seed}' for kind in ['mirror','warm','scratch'] for seed in [2101,2102]]+[f'forward_{kind}_{seed}' for kind in ['warm','scratch'] for seed in [2201,2202]]+[f'forward_{kind}_{seed}' for kind in ['sin','knots'] for seed in [2301,2302]]+['right_mirror_initial','right_41','forward_41']
def run(name):
 with open(P/f'eval_{name}.log','w') as log:subprocess.run([sys.executable,str(P/'eval412.py'),'--name',name],stdout=log,stderr=subprocess.STDOUT,check=True)
 print(name,'EVALUATED',flush=True)
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
 pending=set(names);futures=[]
 while pending:
  ready=[n for n in pending if n.endswith('_41') or n=='right_mirror_initial' and (P/'runs'/n/'final.zip').exists() or (P/'runs'/n/'complete.json').exists() or (P/(n+'.json')).exists()]
  for n in ready:futures.append(pool.submit(run,n));pending.remove(n)
  if pending:time.sleep(10)
 for f in futures:f.result()
