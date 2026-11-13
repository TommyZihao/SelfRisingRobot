# 作者：同济子豪兄 2026-10-26
from pathlib import Path
import subprocess,sys,json
P=Path(__file__).resolve().parent
for seed in [2501,2502]:
 name=f'forward_heading_{seed}'
 with open(P/(name+'.log'),'w') as log:subprocess.run([sys.executable,str(P/'search_heading.py'),'--seed',str(seed)],stdout=log,stderr=subprocess.STDOUT,check=True)
for name in ['forward_heading_2501','forward_heading_2502','forward_sin_grip_2402','forward_41']:
 with open(P/f'heading_val_{name}.log','w') as log:subprocess.run([sys.executable,str(P/'eval412.py'),'--name',name,'--n','30','--seedbase','89000000','--split','heading_validation'],stdout=log,stderr=subprocess.STDOUT,check=True)
rows=[json.loads((P/'heading_validation'/n/'summary.json').read_text()) for n in ['forward_heading_2501','forward_heading_2502','forward_sin_grip_2402']];rows.sort(key=lambda x:(x['success'],-x['stood'],x['mean_forward_m']),reverse=True)
sel=json.loads((P/'selection.json').read_text());sel['forward_stage1']=sel['forward'];sel['forward']=rows[0]['name'];sel['heading_ranking']=rows;sel['confirmation_seedbase']=90000000;(P/'selection.json').write_text(json.dumps(sel,indent=2))
for name in [sel['forward'],'forward_41']:
 with open(P/f'confirm_{name}.log','w') as log:subprocess.run([sys.executable,str(P/'eval412.py'),'--name',name,'--n','100','--seedbase','90000000','--split','confirmation'],stdout=log,stderr=subprocess.STDOUT,check=True)
print('HEADING COMPLETE',sel['forward'],flush=True)
