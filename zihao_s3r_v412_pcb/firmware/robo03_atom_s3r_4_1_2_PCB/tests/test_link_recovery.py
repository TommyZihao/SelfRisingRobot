# 作者：同济子豪兄 2026-10-26
from pathlib import Path
import subprocess,tempfile,re
p=Path(__file__).resolve().parents[1];ui=(p/'web_ui.h').read_text();ino=(p/'robo03_atom_s3r_4_1_2_PCB.ino').read_text()
clear=ui[ui.index('function clearInput()'):ui.index('\n',ui.index('function clearInput()'))]
send=ui[ui.index('async function send()'):ui.index("document.querySelectorAll('[data-cmd]')",ui.index('async function send()'))]
js='''const assert=require('node:assert/strict');
let enabled=true,token=123,busy=false,sending=false,mode='walk',cmd='roll_left',pendingAction=null,held={pointer:1},seq=0;
let failure=null,requests=[];const document={querySelectorAll(){return []}};function connected(){}
async function request(url){requests.push(url);if(failure)throw failure;}
'''+clear+'\n'+send+'''
(async()=>{
 failure=Object.assign(Error('temporary timeout'),{name:'AbortError'});await send();
 assert.equal(token,123);assert.equal(cmd,'roll_left');assert.ok(held);
 failure=null;await send();assert.ok(requests.at(-1).endsWith('cmd=roll_left'));
 clearInput();await send();assert.ok(requests.at(-1).endsWith('cmd=idle'));
 cmd='roll_right';held={pointer:2};failure=Object.assign(Error('inactive'),{status:409});await send();
 assert.equal(token,0);assert.equal(cmd,'idle');assert.equal(held,null);
 failure=null;let n=requests.length;await send();assert.equal(requests.length,n);
 console.log('PASS: transient timeout retry, release, invalid-session stop, no automatic restart');
})().catch(e=>{console.error(e);process.exitCode=1;});
'''
walk=re.search(r'if\(crawler.command!=walk41::Idle&&uint32_t\(millis\(\)-lastCommandMs\)>COMMAND_TIMEOUT_MS\)\{[^}]*\}',ino).group()
action=re.search(r'if\(actionId>=0&&uint32_t\(millis\(\)-lastCommandMs\)>COMMAND_TIMEOUT_MS\)\{[^}]*\}',ino).group()
cpp='''#include <cassert>
#include <cstdint>
uint32_t now=0,lastCommandMs=0,commandTimeouts=0;constexpr uint32_t COMMAND_TIMEOUT_MS=400;
namespace walk41 {enum {Idle,RollLeft};}struct {int command=walk41::Idle;}crawler;
int actionId=-1;bool stopped=false;uint32_t millis(){return now;}void emergencyStop(const char*){stopped=true;}
void walk(){'''+walk+'''}
void action(){'''+action+'''}
int main(){now=10000;walk();action();assert(!stopped);
crawler.command=walk41::RollLeft;lastCommandMs=now;now+=400;walk();assert(!stopped);now++;walk();assert(stopped);
stopped=false;actionId=0;lastCommandMs=now;action();assert(!stopped);now+=401;action();assert(stopped);
stopped=false;crawler.command=walk41::RollLeft;lastCommandMs=0xffffff00U;now=lastCommandMs+401;walk();assert(stopped);
}
'''
with tempfile.TemporaryDirectory() as d:
 f=Path(d)/'test.cjs';f.write_text(js);subprocess.run(['node',str(f)],check=True)
 f=Path(d)/'test.cpp';f.write_text(cpp);out=Path(d)/'test';subprocess.run(['c++','-std=c++17',str(f),'-o',str(out)],check=True);subprocess.run([str(out)],check=True)
print('PASS: idle does not time out, active motion still stops after 400ms, timer wraparound')
