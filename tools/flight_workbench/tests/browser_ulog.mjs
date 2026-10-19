// Read-only browser check of an already imported log on an offline test backend.
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import {spawn} from 'node:child_process';
import assert from 'node:assert/strict';
const [base, browser] = process.argv.slice(2);
assert(base && browser, 'Pass offline backend URL and native Chromium executable');
const profile = fs.mkdtempSync(path.join(os.tmpdir(), 'wb-ulog-browser-'));
const child = spawn(browser, ['--headless=new', '--disable-gpu', '--no-first-run',
  '--no-default-browser-check', '--remote-debugging-port=0', '--user-data-dir=' + profile,
  'about:blank'], {windowsHide:true, stdio:'ignore'});
const sleep = ms => new Promise(resolve => setTimeout(resolve, ms));
const pending = new Map(), requests = [];
let socket, sequence = 0;
try {
  let port;
  for (let i=0; i<120; i++) {
    try { port = Number(fs.readFileSync(path.join(profile, 'DevToolsActivePort'), 'utf8').split('\n')[0]); break; } catch {}
    assert.equal(child.exitCode, null, 'Browser must remain running'); await sleep(100);
  }
  assert(port, 'CDP ready');
  const pages = await (await fetch(`http://127.0.0.1:${port}/json/list`)).json();
  socket = new WebSocket(pages.find(page => page.type === 'page').webSocketDebuggerUrl);
  await new Promise((resolve, reject) => { socket.onopen=resolve; socket.onerror=reject; });
  socket.onmessage = event => {
    const message = JSON.parse(event.data);
    if (message.method === 'Network.requestWillBeSent') requests.push(message.params.request);
    if (!message.id || !pending.has(message.id)) return;
    const promise=pending.get(message.id); pending.delete(message.id); clearTimeout(promise.timer);
    message.error ? promise.reject(new Error(JSON.stringify(message.error))) : promise.resolve(message.result);
  };
  const call = (method, params={}) => new Promise((resolve,reject) => {
    const id=++sequence;
    pending.set(id,{resolve,reject,timer:setTimeout(()=>reject(new Error(method+' timeout')),10000)});
    socket.send(JSON.stringify({id,method,params}));
  });
  const run = async expression => {
    const value = await call('Runtime.evaluate', {expression,returnByValue:true,awaitPromise:true});
    assert(!value.exceptionDetails, JSON.stringify(value.exceptionDetails)); return value.result.value;
  };
  await call('Network.enable'); await call('Page.navigate',{url:base+'/logs'});
  let ready=false;
  for (let i=0; i<100; i++) {
    ready=await run(`typeof ulogState !== 'undefined' && ulogState.records.some(r => r.state === 'ready')`);
    if (ready) break; await sleep(100);
  }
  assert(ready, 'Imported real ULog history loads without connection');
  assert(await run(`!document.querySelector('#ulog-result').hidden && document.querySelector('#ulog-title').textContent.endsWith('.ulg')`));
  assert(await run(`document.querySelector('#ulog-library').options.length >= 1 &&
    document.querySelector('#ulog-summary').textContent.includes('vehicle_local_position')`));
  await run(`document.querySelectorAll('#ulog-images img').forEach(image => image.loading='eager')`);
  for (let i=0; i<100; i++) {
    ready=await run(`Array.from(document.querySelectorAll('#ulog-images img')).length===2 &&
      Array.from(document.querySelectorAll('#ulog-images img')).every(image => image.complete && image.naturalWidth>0)`);
    if (ready) break; await sleep(100);
  }
  assert(ready, 'Trajectory and motor diagnostic PNGs decode in native browser');
  assert(await run(`Array.from(document.querySelectorAll('#ulog-exports a')).some(link => link.download==='analysis.zip') &&
    Array.from(document.querySelectorAll('#ulog-exports a')).some(link => link.download==='trajectory.csv')`));
  await run(`document.querySelector('#ulog-library-refresh').click()`); await sleep(500);
  assert(await run(`ulogState.records.some(r => r.id===ulogState.selected && r.state==='ready') && !document.querySelector('#ulog-result').hidden`));
  assert(!requests.some(request => request.method==='POST'), 'Read-only history creates no connection/operation');
  const snapshot = await (await fetch(base+'/api/snapshot')).json();
  assert.equal(snapshot.connection.state,'unknown'); assert.deepEqual(snapshot.sessions,{});
  console.log('PASS: native ULog history, summary, trajectory/diagnostic images, exports, refresh; no device requests');
} finally {
  if (socket && socket.readyState===WebSocket.OPEN) {
    socket.send(JSON.stringify({id:++sequence,method:'Browser.close'}));
    await sleep(500); socket.close();
  }
  if (child.exitCode===null) {
    if (process.platform==='win32') {
      const {spawnSync}=await import('node:child_process');
      spawnSync('taskkill',['/PID',String(child.pid),'/T','/F'],{windowsHide:true});
    } else child.kill();
  }
  for (const promise of pending.values()) clearTimeout(promise.timer);
  // Only this test profile, under the native temporary directory.
  assert.equal(path.dirname(path.resolve(profile)), path.resolve(os.tmpdir()));
  for (let i=0; i<30; i++) {
    try { fs.rmSync(profile,{recursive:true,force:true}); break; } catch { await sleep(100); }
  }
}
