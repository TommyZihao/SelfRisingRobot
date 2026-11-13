# 作者：同济子豪兄 2026-10-26
from pathlib import Path
import subprocess,tempfile
s=(Path(__file__).resolve().parents[1]/'web_ui.h').read_text()
def line(start):
 a=s.index(start);return s[a:s.index('\n',a)]
js='''const assert=require('node:assert/strict');
let homePending=null,mode='walk',enabled=true,epoch=10,token=123,seq=0,version=0,busy=false,sending=false,polling=false,cmd='yaw_left',pendingAction=null,held={pointer:1},screenPaused=false;
const document={hidden:false,querySelectorAll(){return []}};
const status={textContent:''};const $=()=>status;
const requests=[];let nextState;
function connected(){}
function render(s){epoch=s.epoch;mode=s.mode;enabled=s.enabled;screenPaused=s.screenPaused;if(!enabled){token=0;clearInput();}resumeAfterHome(s);}
async function request(url){requests.push(url);let state=nextState;if(url.startsWith('/mode'))state={epoch:epoch+1,mode:'walk',enabled:true,screenPaused:false,state:'walk_idle',token:999};return {json:async()=>state};}
'''+ '\n'.join(line(x) for x in ['function clearInput()','function resumeAfterHome(s)','async function choose(next)','async function halt(home=false)','async function poll()'])+'''
const returning=()=>({epoch:epoch+1,mode:'walk',enabled:false,screenPaused:false,state:'returning_home',stopReason:'manual'});
const completed=()=>({epoch,mode:'walk',enabled:false,screenPaused:false,state:'stopped',stopReason:'manual'});
(async()=>{
 nextState=returning();await halt(true);assert.ok(homePending);assert.equal(cmd,'idle');assert.equal(token,0);
 assert.equal(requests.filter(x=>x.startsWith('/mode')).length,0);
 render(completed());render(completed());await new Promise(setImmediate);
 assert.equal(requests.filter(x=>x.startsWith('/mode')).length,1);
 assert.equal(mode,'walk');assert.equal(enabled,true);assert.equal(token,999);assert.equal(cmd,'idle');
 nextState=returning();await halt(true);render(completed());nextState={...completed(),epoch:epoch+1};await halt(false);await new Promise(setImmediate);
 assert.equal(requests.filter(x=>x.startsWith('/mode')).length,1);assert.equal(homePending,null);
 nextState=returning();await halt(true);render({...completed(),screenPaused:true});await new Promise(setImmediate);assert.equal(homePending,null);
 screenPaused=false;nextState=returning();await halt(true);render({...completed(),epoch:epoch+1});await new Promise(setImmediate);assert.equal(homePending,null);
 nextState=returning();await halt(true);document.hidden=true;render(completed());await new Promise(setImmediate);assert.equal(homePending,null);
 document.hidden=false;enabled=true;token=999;cmd='yaw_right';let n=requests.length;await poll();assert.equal(requests.length,n);
 console.log('PASS: home waits for completion, resumes idle Walk once, cancels on stop/pause/epoch change/hidden; no telemetry poll contention while held');
})().catch(e=>{console.error(e);process.exitCode=1;});
'''
with tempfile.TemporaryDirectory() as d:
 f=Path(d)/'test.cjs';f.write_text(js);subprocess.run(['node',str(f)],check=True)
