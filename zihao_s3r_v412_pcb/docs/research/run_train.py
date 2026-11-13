# 作者：同济子豪兄 2026-10-26
from pathlib import Path
import concurrent.futures,subprocess,sys
P=Path(__file__).resolve().parent
cases=[('right',kind,seed) for kind in ['mirror','warm','scratch'] for seed in [2101,2102]]+[('forward',kind,seed) for kind in ['warm','scratch'] for seed in [2201,2202]]
def run(c):
 task,kind,seed=c;name=f'{task}_{kind}_{seed}'
 with open(P/(name+'.log'),'w') as log:subprocess.run([sys.executable,str(P/'train412.py'),'--task',task,'--init',kind,'--seed',str(seed)],stdout=log,stderr=subprocess.STDOUT,check=True)
 print(name,'COMPLETE',flush=True)
with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:list(pool.map(run,cases))
