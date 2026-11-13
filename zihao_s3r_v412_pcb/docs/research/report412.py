# 作者：同济子豪兄 2026-10-26
from pathlib import Path
import json,csv
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
P=Path(__file__).resolve().parent;sel=json.loads((P/'selection.json').read_text())
fig,ax=plt.subplots(3,2,figsize=(12,11))
for f in (P/'runs').glob('*/progress.csv'):
 rows=list(csv.DictReader(f.open()));side=0 if f.parent.name.startswith('right') else 2 if f.parent.name.startswith('backward') else 1
 for col,key in enumerate(['rollout/ep_rew_mean','train/value_loss']):
  pairs=[(float(r['time/total_timesteps']),float(r[key])) for r in rows if r.get(key) and r.get('time/total_timesteps')]
  if pairs:ax[side,col].plot(*np.array(pairs).T,label=f.parent.name)
  ax[side,col].set_title((['Right roll: ','Forward: ','Backward: '][side])+key);ax[side,col].grid(alpha=.2);ax[side,col].set_xlabel('Environment steps')
for a in ax.ravel():a.legend(fontsize=6)
fig.tight_layout();fig.savefig(P/'training.png',dpi=150);plt.close(fig)
fig,ax=plt.subplots(figsize=(10,5))
for f in list(P.glob('forward*_progress.json'))+list(P.glob('backward*_progress.json')):
 rows=json.loads(f.read_text());ax.plot([r['generation'] for r in rows],[r['best']['score'] for r in rows],label=f.stem)
ax.set_xlabel('CEM generation');ax.set_ylabel('Best search score (objectives differ)');ax.legend(fontsize=7);ax.grid(alpha=.2);fig.tight_layout();fig.savefig(P/'search.png',dpi=150);plt.close(fig)
metrics=[]
for split in ['validation','heading_validation','backward_validation','final','confirmation','backward_confirmation']:
 for f in sorted((P/split).glob('*/summary.json')):metrics.append(dict(split=split,**json.loads(f.read_text())))
(P/'metrics.json').write_text(json.dumps(metrics,indent=2))
completed=[json.loads(f.read_text()) for f in (P/'runs').glob('*/complete.json')];steps=sum(r['steps'] for r in completed)
body=f'''<!doctype html><meta charset="utf-8"><title>RoBoBo 4.1.2 右翻与大幅前进/后退</title><style>body{{max-width:1200px;margin:30px auto;padding:0 20px;background:#102332;color:#e7f2ff;font:16px/1.7 system-ui}}table{{border-collapse:collapse}}th,td{{padding:8px;border-bottom:1px solid #53718b}}img,video{{max-width:100%}}a{{color:#8dddff}}</style><h1>4.1.2：第一次右翻与大幅前进/后退/后退</h1><p>作者：同济子豪兄 2026-10-26；实验2026-11-13。{len(completed)}组PPO，共{steps:,}步；另10组CEM周期/方向修正搜索。没有烧录。</p><p>5V、橡胶圆帽、地毯近似；未实测标定。所有评估实际pitch_neg起步。右翻成功定义是首次净翻转≥86.4°持续0.2秒，最终仍净≥72°，无站起；不是连续翻多圈成功。前进/后退32秒沿指令方向净≥4cm且满足后半程持续推进、末端朝下/接触、航向和横向漂移限制。不能只根据动作幅度或一个好视频判断可靠性。</p><p>选择右翻：{sel['right']}；前进：{sel['forward']}；后退：{sel.get('backward','待选择')}。右翻使用镜像初始化后PPO补训，前进使用最终验证选择的控制器；保留4.1其它动作及通信/回零修复。</p><img src="training.png"><img src="search.png">'''
for split in ['final','confirmation','backward_confirmation','validation','heading_validation','backward_validation']:
 body+=f'<h2>{split}</h2><table><tr><th>模型</th><th>N</th><th>通过率</th><th>站起率</th><th>平均沿指令方向净进度</th><th>下/上目标摆幅</th></tr>'
 for r in metrics:
  if r['split']!=split:continue
  prog=f"{r['mean_turns']*360:.1f}°" if 'mean_turns' in r else f"{r['mean_forward_m']*100:.1f} cm"
  span='—' if 'mean_lower_span_deg' not in r else f"{r['mean_lower_span_deg']:.1f}° / {r['mean_upper_span_deg']:.1f}°"
  body+=f"<tr><td>{r['name']}</td><td>{r['n']}</td><td>{r['success']:.1%}</td><td>{r['stood']:.1%}</td><td>{prog}</td><td>{span}</td></tr>"
 body+='</table>'
body+='<h2>完整视频：固定首个测试与挑选案例均保留</h2>'
for f in sorted((P/'videos').glob('*.mp4')):body+=f'<h3>{f.stem}</h3><video controls preload="metadata" src="videos/{f.name}"></video>'
body+='<p><a href="PROTOCOL.txt">实验方案</a> · <a href="selection.json">选择过程</a> · <a href="metrics.json">完整指标</a></p>'
(P/'report.html').write_text(body);print('REPORT',steps)
