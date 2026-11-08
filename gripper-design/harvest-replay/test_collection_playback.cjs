// Actual playback callback with lightweight DOM/timing stand-ins; no renderer.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const source = fs.readFileSync(path.join(__dirname, 'viewer.js'), 'utf8');
const html = fs.readFileSync(path.join(__dirname, 'index.html'), 'utf8');
assert(/<input\b[^>]*id="collectionNext"[^>]*\bchecked\b/.test(html));
assert(/let ready = false, playing = false/.test(source));
const start = source.indexOf('function animate('), end = source.indexOf('\nfunction play(', start);
assert(start >= 0 && end > start);
const branchStart = source.indexOf('function independentBranchBoundary(');
const branchEnd = source.indexOf('\nfunction collectionContinuityText(', branchStart);
assert(branchStart >= 0 && branchEnd > branchStart);
function endFrame({checked = true, group = {runs: ['first', 'second']}, run = 'first', playing = true, partial = false, nextPartial = false, stopped = false} = {}) {
  const calls = [], notice = {hidden: true, textContent: ''}, context = {
    playing, ready: true, currentTime: 1, times: [0, 1], lastWallTime: null,
    selectedRun: {id: run}, speed: 1,
    catalog: {runs: [{id: 'second', label: '독립 재시험', record_kind: nextPartial ? 'partial_diagnostic' : 'execution'}]},
    currentCollection: () => group,
    isPartialDiagnostic: () => partial,
    isStoppedExecution: () => stopped,
    $: id => id === 'branchNotice' ? notice : {checked},
    renderNow: () => {},
    pause: () => {calls.push('pause');context.playing = false;},
    play: () => {calls.push('play');context.playing = true;},
    selectRun: id => {calls.push('select:' + id);context.selectedRun = {id};return Promise.resolve({run: id});},
  };
  vm.runInNewContext(source.slice(branchStart, branchEnd) + '\n' + source.slice(start, end) + '\nanimate(0)', context);
  return {calls, context, notice};
}
(async () => {
  let result = endFrame();await Promise.resolve();
  assert.deepEqual(result.calls, ['pause', 'select:second', 'play']);
  result = endFrame({checked: false});await Promise.resolve();
  assert.deepEqual(result.calls, ['pause']);
  result = endFrame({group: null});await Promise.resolve();
  assert.deepEqual(result.calls, ['pause']);
  result = endFrame({run: 'second'});await Promise.resolve();
  assert.deepEqual(result.calls, ['pause']);
  result = endFrame({playing: false});await Promise.resolve();
  assert.deepEqual(result.calls, []);
  result = endFrame({partial: true});await Promise.resolve();
  assert.deepEqual(result.calls, ['pause'], 'A partial software-error record cannot automatically advance');
  result = endFrame({stopped: true});await Promise.resolve();
  assert.deepEqual(result.calls, ['pause'], 'A stopped native execution holds its actual terminal frame even if another run follows');
  assert.equal(result.notice.hidden, false);
  assert(result.notice.textContent.includes('중단'));
  result = endFrame({nextPartial: true});await Promise.resolve();
  assert.deepEqual(result.calls, ['pause'], 'A partial diagnostic requires manual selection even when it is next');
  assert.equal(result.notice.hidden, false);
  const group = {runs: ['first', 'second'], provenance: {history_graph: {edges: [{source: 'normal-parent', target: 'second', kind: 'software_fault_independent_branch'}]}}};
  result = endFrame({group});await Promise.resolve();
  assert.deepEqual(result.calls, ['pause'], 'An explicit independent fork requires manual selection');
  assert.equal(result.notice.hidden, false);
  assert(result.notice.textContent.includes('독립 재시험'));
  assert(result.notice.textContent.includes('직접 선택'));
  result = endFrame({group: {...group, provenance: {history_graph: {edges: [{target: 'unrelated', kind: 'software_fault_independent_branch'}]}}}});await Promise.resolve();
  assert.deepEqual(result.calls, ['pause', 'select:second', 'play'], 'An unrelated fork cannot block the next recorded segment');
  result = endFrame({group: {...group, provenance: {history_graph: {edges: [{target: 'second', kind: 'measured_state_handoff'}]}}}});await Promise.resolve();
  assert.deepEqual(result.calls, ['pause', 'select:second', 'play'], 'A measured handoff still automatically advances');
  assert(/id="branchNotice"[^>]*role="status"[^>]*hidden/.test(html));
  const selection = source.slice(source.indexOf('async function selectRun('), source.indexOf('const sequence = ++loadSequence;', source.indexOf('async function selectRun(')));
  assert(selection.includes("$('branchNotice').hidden = true"), 'Manual run selection clears the paused-boundary notice and remains available');
  const stoppedStart = source.indexOf('function isStoppedExecution('), stoppedEnd = source.indexOf('\nfunction renderNow(', stoppedStart);
  const stoppedContext = {};vm.createContext(stoppedContext);vm.runInContext(source.slice(stoppedStart, stoppedEnd), stoppedContext);
  assert.equal(stoppedContext.isStoppedExecution({}, {result_summary: {completed: false, stop_reason: 'penetration_over_contact_policy'}}), true);
  assert.equal(stoppedContext.isStoppedExecution({}, {result_summary: {completed: true, stop_reason: null}}), false);
  assert.equal(stoppedContext.isStoppedExecution({}, {result_summary: {completed: false, stop_reason: null}}), false, 'Missing evidence does not fabricate a physical stop');
  console.log('25 actual collection playback checks PASS: native stopped terminal-frame guard, default-on, partial and independent boundaries, measured handoff; no browser/GPU');
})().catch(error => {console.error(error);process.exitCode = 1;});
