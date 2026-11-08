// Render the actual collection continuity description without browser/GPU.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const source = fs.readFileSync(path.join(__dirname, 'viewer.js'), 'utf8');
const start = source.indexOf('function collectionContinuityText(');
const end = source.indexOf('\nfunction updateRunPicker(', start);
assert(start >= 0 && end > start);
const describe = vm.runInNewContext(source.slice(start, end) + '\ncollectionContinuityText');
assert.equal(describe(null), '');
assert.equal(describe({}), '기록 구간 전환 · 연속성은 시험 설명을 참조하세요.');
assert.equal(describe({continuity: 'measured_state_handoff'}), '측정 상태를 이어받은 실행 구간입니다.');
assert.equal(describe({continuity: 'independent_trials'}), '개별 시험이며, 구간마다 기록된 초기 상태로 전환됩니다.');
const branch = {continuity: 'unknown', provenance: {continuity_note: '  소프트웨어 오류 재시험은 마지막 정상 측정 상태에서 시작한 별도 분기입니다.  '}};
assert.equal(describe(branch), '기록 구간 전환 · 연속성은 시험 설명을 참조하세요. 소프트웨어 오류 재시험은 마지막 정상 측정 상태에서 시작한 별도 분기입니다.');
assert.equal(describe({provenance: {continuity_note: {unexpected: true}}}), describe({}));
assert.equal(describe({provenance: {continuity_note: '   '}}), describe({}));
console.log('7 actual collection continuity checks PASS: legacy defaults, measured/independent labels, explicit software-fault branch note and invalid/blank notes; no browser/GPU');
