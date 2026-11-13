# 作者：同济子豪兄 2026-10-26
from pathlib import Path
import subprocess,sys
P=Path(__file__).resolve().parent
for kind in ['sin','knots']:
 for seed in [2301,2302]:
  with open(P/f'forward_{kind}_{seed}.log','w') as log:subprocess.run([sys.executable,str(P/'search_forward.py'),'--kind',kind,'--seed',str(seed)],stdout=log,stderr=subprocess.STDOUT,check=True)
  print(kind,seed,'DONE',flush=True)
