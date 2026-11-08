const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const source = fs.readFileSync(__dirname + '/viewer.js', 'utf8');
const start = source.indexOf('function collectionSegments(');
const end = source.indexOf('\nfunction updateRunPicker(', start);
const elements = Object.fromEntries(['collectionTimelineRow','collectionTimeline','collectionTime','branchNotice'].map(id => [id, {setAttribute() {}}]));
const group = {id: 'g', runs: ['a','b','c']};
const context = {catalog: {runs: [{id:'a',duration_s:10},{id:'b',duration_s:20},{id:'c',duration_s:5}]},
  selectedRun: {id:'a'}, currentTime:104, times:[100,110], ready:true,
  currentCollection: () => group, $: id => elements[id],
  pause() {}, independentBranchBoundary: (_,id) => id==='b', snapshot: () => ({}),
  selectRun: async id => {context.selectedRun={id};context.times=[50,70];},
  seek: value => {context.currentTime=value;},
};
vm.createContext(context);vm.runInContext(source.slice(start,end),context);
const layout = context.collectionSegments(group,context.catalog.runs);
assert.equal(layout.total,35);
for (const [time,id,local] of [[-1,'a',0],[0,'a',0],[9.9,'a',9.9],[10,'b',0],[29,'b',19],[30,'c',0],[35,'c',5],[100,'c',5]]) {
  const p=context.collectionPosition(layout,time);assert.equal(p.id,id);assert.equal(p.localTime,local);
}
assert.throws(() => context.collectionPosition(layout,NaN));
assert.throws(() => context.collectionSegments({runs:['missing']},context.catalog.runs));
assert.throws(() => context.collectionSegments({runs:['a']},[{id:'a',duration_s:-1}]));
assert.equal(context.durationLabel(499.5),'8:19.5');
context.updateCollectionTimeline();assert.equal(elements.collectionTimeline.value,'4');
assert.equal(elements.collectionTimeline.max,'35');
(async () => {
  await context.seekCollection(15);
  assert.equal(context.selectedRun.id,'b');assert.equal(context.currentTime,55);
  assert.equal(elements.branchNotice.hidden,false);
  assert(elements.branchNotice.textContent.includes('독립 재시험'));
  // An unrelated newer selection during asynchronous loading must not seek it.
  context.selectRun=async () => {context.selectedRun={id:'newer'};};
  const before=context.currentTime;await context.seekCollection(32);
  assert.equal(context.currentTime,before);
  console.log('Collection timeline PASS: boundaries, nonzero source clock, real run loading, independent-trial notice and async selection race');
})().catch(error => {console.error(error);process.exitCode=1;});
