# 作者：同济子豪兄 2026-10-26
from pathlib import Path
import subprocess,sys,time
P=Path(__file__).resolve().parent
while not (P/'forward_knots_2302.json').exists():time.sleep(10)
for seed in [2401,2402]:
 name=f'forward_sin_grip_{seed}'
 with open(P/(name+'.log'),'w') as log:subprocess.run([sys.executable,str(P/'search_forward.py'),'--kind','sin','--seed',str(seed),'--grip'],stdout=log,stderr=subprocess.STDOUT,check=True)
 with open(P/f'eval_{name}.log','w') as log:subprocess.run([sys.executable,str(P/'eval412.py'),'--name',name],stdout=log,stderr=subprocess.STDOUT,check=True)
 print(name,'DONE',flush=True)
