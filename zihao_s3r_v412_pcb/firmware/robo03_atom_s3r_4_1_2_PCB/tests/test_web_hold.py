# 作者：同济子豪兄 2026-10-26
from pathlib import Path
import subprocess, tempfile
s=(Path(__file__).resolve().parents[1]/'web_ui.h').read_text()
clear=s[s.index('function clearInput()'):s.index('\n',s.index('function clearInput()'))]
start=s.index('async function send()');end=s.index("document.querySelectorAll('[data-action]')",start)
js=r'''
const assert=require('node:assert/strict');
let mode='walk',enabled=true,token=1,seq=0,busy=false,sending=false,cmd='idle',pendingAction=null,held=null;
const buttons=['forward','backward','roll_left','roll_right','yaw_left','yaw_right'].map(cmd=>({dataset:{cmd},classList:{add(){},remove(){}},setPointerCapture(){}}));
const document={querySelectorAll(){return buttons;}};
const sent=[];async function request(url){sent.push(new URL(url,'http://robot').searchParams.get('cmd'));}
function connected(){}
''' + clear + '\n' + s[start:end] + r'''
(async()=>{
 for(const b of buttons){
  const e={pointerId:1,preventDefault(){}};
  b.onpointerdown(e);await new Promise(setImmediate);
  for(let i=0;i<20;i++)await send();
  assert.equal(sent.at(-1),b.dataset.cmd);
  b.onpointerup(e);await new Promise(setImmediate);assert.equal(sent.at(-1),'idle');
  b.onpointerdown(e);await new Promise(setImmediate);b.onlostpointercapture(e);
  await new Promise(setImmediate);assert.equal(sent.at(-1),'idle');
  b.onpointerdown(e);await new Promise(setImmediate);b.onpointercancel(e);
  await new Promise(setImmediate);assert.equal(sent.at(-1),'idle');
  b.onkeydown({key:' ',repeat:false,preventDefault(){}});await new Promise(setImmediate);
  assert.equal(sent.at(-1),b.dataset.cmd);
  b.onkeyup({key:' ',preventDefault(){}});await new Promise(setImmediate);
  assert.equal(sent.at(-1),'idle');
  // Release while the command HTTP request is outstanding.
  b.onpointerdown(e);b.onpointerup(e);await new Promise(setImmediate);
  assert.equal(sent.at(-1),'idle');
 }
 console.log('PASS: all six buttons hold/release/cancel/capture-loss/keyboard/in-flight release');
})().catch(e=>{console.error(e);process.exitCode=1;});
'''
with tempfile.TemporaryDirectory() as d:
 p=Path(d)/'test.cjs';p.write_text(js);subprocess.run(['node',str(p)],check=True)
