// Run the actual browser script with a minimal DOM and deterministic clock.
// This checks control messages and gates, not rendering on a physical phone.
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import vm from 'node:vm';

const page = readFileSync(new URL('../ros2_ws/src/robot_web_teleop/web/index.html', import.meta.url), 'utf8');
const script = page.match(/<script>([\s\S]*?)<\/script>/)[1];
const elements = new Map();
for (const [, id] of page.matchAll(/\bid="([^"]+)"/g)) {
  assert(!elements.has(id), `Duplicate HTML id: ${id}`);
  elements.set(id, {value: id === 'speed' ? '35' : '', textContent: '',
    hidden: false, disabled: false, style: {setProperty() {}},
    classList: {toggle() {}}, addEventListener() {}, blur() {}, removeAttribute() {}});
}
const clock = {ms: 1000};
const context = vm.createContext({assert, clock, sent: [],
  performance: {now: () => clock.ms},
  document: {hidden: false, addEventListener() {}, getElementById(id) {
    assert(elements.has(id), `Missing HTML element: ${id}`);
    return elements.get(id);
  }},
  window: {addEventListener() {}},
  WebSocket: class { static OPEN = 1; },
  location: {protocol: 'http:', host: '192.168.0.28:8080'},
  setTimeout() {return 1;}, clearTimeout() {}, setInterval() {return 1;},
  AbortController, URL, fetch: () => new Promise(() => {})});
vm.runInContext(script, context, {filename: 'phone-index.html'});
vm.runInContext(`
  socket = {readyState: WebSocket.OPEN, bufferedAmount: 0,
    send: value => sent.push(JSON.parse(value)), close() {}};
  state = {camera_required:false, drive_ready:true, camera_fresh:false,
    drive_source:'idle', owner:true, armed:true};
  stateAt = now();
  assert.equal(ready(), true, 'Direct sight must allow missing video');
  state.camera_required = true;
  assert.equal(ready(), false, 'Video mode must reject missing video');
  state.camera_fresh = true; frameToken = 'one-frame'; frameAt = now();
  assert.equal(ready(), true);
  clock.ms += 501; stateAt = now();
  assert.equal(ready(), false, 'Old phone image cannot remain fresh');
  state.camera_required = false;
  authorized = true;
  halt('Bluetooth takeover');
  assert.equal(sent.at(-1).type, 'release', 'Automatic halt must never send E');
  assert.equal(authorized, false);
  $('stop').onclick();
  assert.equal(sent.at(-1).type, 'stop', 'Explicit STOP must reach both sources');
  assert.equal(sent.filter(item => item.type === 'stop').length, 1);
`, context);
console.log('PASS: phone gates, ordinary release, explicit global STOP, DOM references');
