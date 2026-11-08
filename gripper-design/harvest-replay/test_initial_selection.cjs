// Exercise the actual viewer selection function without creating a renderer.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const source = fs.readFileSync(path.join(__dirname, 'viewer.js'), 'utf8');
const start = source.indexOf('function initialReplaySelection(');
const end = source.indexOf('\nfunction currentCollection(', start);
assert(start >= 0 && end > start);
const select = vm.runInNewContext(source.slice(start, end) + '\ninitialReplaySelection');
assert(source.includes('initialReplaySelection(catalog, requested, requestedCollection)'));
const catalog = {
  runs: [{id: 'legacy'}, {id: 'first22'}, {id: 'owned28'}, {id: 'failed20'}],
  collections: [
    {id: 'diagnostics', runs: ['first22', 'failed20']},
    {id: 'route', runs: ['first22', 'owned28']},
  ],
  default_collection: 'route', default_run: 'first22',
};
let passed = 0;
function check(value, run, collection, expected) {
  assert.deepEqual(JSON.parse(JSON.stringify(select(value, run, collection))), expected);
  passed++;
}
check(catalog, null, null, {collectionId: 'route', runId: 'first22'});
check(catalog, 'legacy', null, {collectionId: '', runId: 'legacy'});
check(catalog, 'failed20', null, {collectionId: 'diagnostics', runId: 'failed20'});
check(catalog, 'first22', null, {collectionId: 'route', runId: 'first22'});
check(catalog, 'owned28', 'diagnostics', {collectionId: 'diagnostics', runId: 'first22'});
check(catalog, 'failed20', 'diagnostics', {collectionId: 'diagnostics', runId: 'failed20'});
check(catalog, 'missing', 'missing', {collectionId: 'route', runId: 'first22'});
check({...catalog, default_run: 'failed20'}, null, null, {collectionId: 'route', runId: 'first22'});
check({...catalog, default_collection: 'missing', default_run: 'owned28'}, null, null,
  {collectionId: 'route', runId: 'owned28'});
check({runs: catalog.runs}, null, null, {collectionId: '', runId: 'legacy'});
check({...catalog, default_collection: null, default_run: null}, null, null,
  {collectionId: '', runId: 'legacy'});
check(catalog, 'owned28', 'route', {collectionId: 'route', runId: 'owned28'});
console.log(`${passed} actual viewer initial-selection checks PASS; no browser/GPU`);
