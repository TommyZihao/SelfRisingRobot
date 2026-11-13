# 作者：同济子豪兄 2026-10-26
import argparse,json,subprocess
from pathlib import Path
import numpy as np,mujoco
from PIL import Image,ImageDraw,ImageFont
from eval412 import episode,P
p=argparse.ArgumentParser();p.add_argument('--name',required=True);p.add_argument('--seed',type=int,required=True);p.add_argument('--label',default='first');a=p.parse_args();out=P/'videos';out.mkdir(exist_ok=True);stem=f'{a.name}_{a.label}';font=ImageFont.load_default()
proc=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24','-s','800x600','-r','25','-i','-','-an','-c:v','libx264','-pix_fmt','yuv420p','-movflags','+faststart',str(out/(stem+'.mp4'))],stdin=subprocess.PIPE)
renderer=None;camera=None;trace=[]
def frame(w,i):
 global renderer,camera
 e=w.e;trace.append(dict(time=i['time_s'],targets=e.target.tolist(),actual=e.data.qpos[e.qids].tolist(),up=e.R()[2].tolist(),progress=i.get('forward_m',i.get('net_turns'))))
 if renderer is None:renderer=mujoco.Renderer(e.model,600,800);camera=mujoco.MjvCamera();camera.distance=.48;camera.azimuth=130;camera.elevation=-32
 if w.count%2:return
 camera.lookat[:]=e.data.subtree_com[e.body];camera.lookat[2]=.04;renderer.update_scene(e.data,camera=camera);im=Image.fromarray(renderer.render());d=ImageDraw.Draw(im);d.rectangle((0,0,800,110),fill='#102332')
 progress=f"net {'backward' if a.name.startswith('backward') else 'forward'} {i['forward_m']*100:.1f} cm | yaw {np.rad2deg(i['yaw_rad']):.1f} deg" if 'forward_m' in i else f"net roll {i['net_turns']*360:.1f} deg | first reached {i['first_roll']}"
 for k,line in enumerate([f'4.1.2 CANDIDATE | {a.name} | {i["time_s"]:.2f}s | 1x',progress,f'pitch_neg | {a.label} seed {a.seed} | stood {i["stood"]}','5V carpet/rubber-cap approximation | no physical reset']):d.text((10,5+26*k),line,font=font,fill='white')
 proc.stdin.write(np.asarray(im).tobytes())
 if w.count==400:im.save(out/(stem+'.png'))
r=episode(a.name,a.seed,frame);proc.stdin.close();assert proc.wait()==0
if renderer:renderer.close()
(out/(stem+'.json')).write_text(json.dumps(dict(result=r,trace=trace),indent=2));print(stem,r['success'],flush=True)
