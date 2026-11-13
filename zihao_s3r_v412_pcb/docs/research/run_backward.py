# 作者：同济子豪兄 2026-10-26
from pathlib import Path
import concurrent.futures,subprocess,sys,json
P=Path(__file__).resolve().parent
names=[f'backward_{kind}_{seed}' for kind in ['warm','scratch'] for seed in [2601,2602]]+[f'backward_heading_{seed}' for seed in [2701,2702]]
def job(name):
 with open(P/(name+'.log'),'w') as log:
  parts=name.split('_');command=[sys.executable,str(P/'search_backward.py'),'--seed',parts[-1]] if parts[1]=='heading' else [sys.executable,str(P/'train412.py'),'--task','backward','--init',parts[1],'--seed',parts[-1]]
  subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True)
 with open(P/f'eval_{name}.log','w') as log:subprocess.run([sys.executable,str(P/'eval412.py'),'--name',name,'--n','30','--seedbase','93000000','--split','backward_validation'],stdout=log,stderr=subprocess.STDOUT,check=True)
 print(name,'READY',flush=True)
with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:list(pool.map(job,names))
with open(P/'eval_backward_41.log','w') as log:subprocess.run([sys.executable,str(P/'eval412.py'),'--name','backward_41','--n','30','--seedbase','93000000','--split','backward_validation'],stdout=log,stderr=subprocess.STDOUT,check=True)
rows=[json.loads((P/'backward_validation'/n/'summary.json').read_text()) for n in names];rows.sort(key=lambda x:(x['success'],-x['stood'],x['mean_forward_m']),reverse=True)
sel=json.loads((P/'selection.json').read_text());sel['backward']=rows[0]['name'];sel['backward_ranking']=rows;(P/'selection.json').write_text(json.dumps(sel,indent=2))
for name in [sel['backward'],'backward_41']:
 with open(P/f'backward_final_{name}.log','w') as log:subprocess.run([sys.executable,str(P/'eval412.py'),'--name',name,'--n','100','--seedbase','94000000','--split','backward_confirmation'],stdout=log,stderr=subprocess.STDOUT,check=True)
print('BACKWARD COMPLETE',sel['backward'],flush=True)
