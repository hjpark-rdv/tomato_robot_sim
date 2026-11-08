// Real prefetch/prepared-transition functions with controlled IO and renderer.
// Exercise cancellation, memory bounds, async selection and camera preservation.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const source = fs.readFileSync(path.join(__dirname, 'viewer.js'), 'utf8');
const section = (start, end) => source.slice(source.indexOf(start), source.indexOf(end, source.indexOf(start)));
const preparation = section('function cancelRunPreparation(', 'async function selectRun(');
const selection = section('async function selectRun(', 'function snapshot(');
const trimming = section('function trimModelCache(', 'async function loadModel(');
const metadata = id => ({motion: {shape: [2, 1, 7], url: `${id}/poses`}, time: {url: `${id}/times`}, scene_url: `${id}/scene.glb.gz`});
const nodes = new Map();
const node = id => {
  if (!nodes.has(id)) nodes.set(id, {hidden: true, checked: true, dataset: {}, removeAttribute() {}});
  return nodes.get(id);
};
const images = [];
const context = {
  URL, AbortController, DOMException, Float32Array, Float64Array, console,
  resolveURL: (value, base) => new URL(value, base).href,
  location: {href: 'https://farm.test/replay/'},
  $: node, ready: true, selectedRun: {id: 'a'}, catalog: {runs: ['a', 'b', 'c'].map(id => ({id, replay_url: `${id}.json`}))},
  group: {runs: ['a', 'b', 'c']}, currentCollection() {return this.group;},
  // Bound functions are installed below because VM globals do not bind this.
  isPartialDiagnostic: () => false, isStoppedExecution: () => false, independentBranchBoundary: () => null,
  activeRunLoad: null, prefetchedRun: null, model: {}, camera: {}, scene: {},
  progressCalls: 0, reportLoadProgress: () => {context.progressCalls++;},
  modelLoads: [], binaryLoads: [],
  fetchJSON: async url => metadata(new URL(url).pathname.split('/').at(-1).split('.')[0]),
  fetchBinary: async (url, compression, label, bytes, options) => {
    context.binaryLoads.push({url, signal: options.signal});
    options.onProgress('test', 1, 1);
    return url.endsWith('/times') ? new Float64Array([0, 1]).buffer : new Float32Array(14).buffer;
  },
  loadModel: async (data, base, token) => {
    context.modelLoads.push(token);
    return {model: {getObjectByName: () => ({})}, info: {body_nodes: []}, instanceBatches: [], cacheKey: data.scene_url};
  },
  loadSensorCamera: async () => ({framesURL: 'https://farm.test/sensor/frames.json', frames: Array.from({length: 300}, (_, i) => ({timestamp_s: i, rgb: `${i}.png`, recognition: `${i}_gt.png`, depth_overlay: `${i}_depth.png`}))}),
  renderer: {compileAsync: async () => {}},
  trimModelCache: () => {},
  Image: class {
    constructor() {images.push(this);}
    removeAttribute(name) {delete this[name];}
  },
};
context.currentCollection = () => context.group;
vm.createContext(context);
vm.runInContext(preparation, context);
const flush = async () => {for (let i = 0; i < 12; i++) await Promise.resolve();};
(async () => {
  // Actual preparation concurrently fetches all arrays and model before any
  // selectRun call and does not mutate progress UI or active execution state.
  context.refreshRunPrefetch();
  const first = context.prefetchedRun;
  assert.equal(first.run.id, 'b');
  await first.promise;
  assert.equal(first.status, 'ready');
  assert.equal(context.binaryLoads.length, 2);
  assert.equal(context.modelLoads.length, 1);
  assert.equal(context.progressCalls, 0);
  assert.equal(context.selectedRun.id, 'a');
  assert.equal(first.sensorImages.length, 6, '300 frames warm only two exposures in three modes');
  const firstImage = first.sensorImages[0];firstImage.image.onload();
  assert.equal(firstImage.loaded, true);
  context.refreshRunPrefetch();assert.equal(context.prefetchedRun, first, 'Repeated playback cannot duplicate downloads');

  // Moving elsewhere drops the previous slot and aborts its downloads/images.
  context.selectedRun = {id: 'b'};context.refreshRunPrefetch();
  const second = context.prefetchedRun;
  assert.equal(second.run.id, 'c');
  assert.equal(first.controller.signal.aborted, true);
  assert.equal(first.sensorImages.length, 0);
  assert.equal(firstImage.image.src, undefined);
  await second.promise;
  node('collectionNext').checked = false;context.refreshRunPrefetch();
  assert.equal(context.prefetchedRun, null);
  assert.equal(second.controller.signal.aborted, true);
  assert.equal(second.sensorImages.length, 0);

  // Independent branches, partial records and the final segment never warm.
  node('collectionNext').checked = true;context.selectedRun = {id: 'a'};
  context.independentBranchBoundary = () => ({target: 'b'});context.refreshRunPrefetch();assert.equal(context.prefetchedRun, null);
  context.independentBranchBoundary = () => null;context.isPartialDiagnostic = () => true;
  context.refreshRunPrefetch();assert.equal(context.prefetchedRun, null);
  context.isPartialDiagnostic = () => false;context.catalog.runs[1].record_kind = 'partial_diagnostic';
  context.refreshRunPrefetch();assert.equal(context.prefetchedRun, null);
  delete context.catalog.runs[1].record_kind;context.isStoppedExecution = () => true;
  context.refreshRunPrefetch();assert.equal(context.prefetchedRun, null, 'A stopped native recording never prepares an automatic continuation');
  context.isStoppedExecution = () => false;context.selectedRun = {id: 'c'};context.refreshRunPrefetch();assert.equal(context.prefetchedRun, null);

  // Aborting metadata before it resolves cannot start a late model request.
  let releaseMetadata;
  context.fetchJSON = () => new Promise(resolve => {releaseMetadata = resolve;});
  const cancelled = context.createRunPreparation(context.catalog.runs[1], true);
  const cancelledResult = cancelled.promise.catch(error => error);
  const previousLoads = context.modelLoads.length;
  context.cancelRunPreparation(cancelled);releaseMetadata(metadata('b'));
  assert.equal((await cancelledResult).name, 'AbortError');
  assert.equal(context.modelLoads.length, previousLoads);

  // A real data error cancels remaining requests; a background failure stays
  // local and does not poison the currently displayed valid execution.
  context.fetchJSON = async () => metadata('bad');
  context.fetchBinary = async url => url.endsWith('/times') ? new Float64Array([1, 1]).buffer : new Float32Array(14).buffer;
  const bad = context.createRunPreparation(context.catalog.runs[1], true);
  await assert.rejects(bad.promise, /순서/);
  assert.equal(bad.status, 'error');assert.equal(bad.controller.signal.aborted, true);
  assert.equal(context.ready, true);

  // Model cache retains exactly active + one prepared/foreground owner and
  // disposes old models/aborts old fetches, including after cancelled parsing.
  const disposed = [], aborted = [];
  const cacheContext = {activeModelKey: 'active', activeRunLoad: null, prefetchedRun: {token: 'next'},
    modelCache: new Map([
      ['active', {value: {model: 'active'}}],
      ['prepared', {requestedSequence: 'next', value: {model: 'next'}}],
      ['obsolete', {value: {model: 'old'}}],
      ['fetching', {controller: {abort: () => aborted.push('fetching')}}],
    ]), disposeModelResources: value => disposed.push(value)};
  vm.createContext(cacheContext);vm.runInContext(trimming, cacheContext);cacheContext.trimModelCache();
  assert.deepEqual([...cacheContext.modelCache.keys()], ['active', 'prepared']);
  assert.deepEqual(disposed, ['old']);assert.deepEqual(aborted, ['fetching']);
  cacheContext.prefetchedRun = null;cacheContext.trimModelCache();
  assert.deepEqual([...cacheContext.modelCache.keys()], ['active']);

  // Actual selectRun adopts a fully prepared buffer synchronously, never
  // exposes the loading overlay and keeps camera view through origin changes.
  nodes.clear();
  const writes = [];
  const loading = new Proxy({hidden: true}, {set(target, key, value) {writes.push([key, value]);target[key] = value;return true;}});
  nodes.set('loading', loading);
  class Vector {constructor(...coords) {this.coords = coords;}add(v) {this.coords = this.coords.map((n, i) => n + v.coords[i]);return this;}}
  const model = {getObjectByName: () => ({})};
  const data = {metadata: metadata('b'), loaded: {model, info: {origin_world_m: [2, 4, 6], body_nodes: []}, instanceBatches: [], cacheKey: 'new'},
    poses: new Float32Array(14), times: new Float64Array([2, 3]), transmission: null, sensorResult: {value: null}};
  const prepared = {run: context.catalog.runs[1], value: data, status: 'ready', sensorImages: [], token: 'prepared'};
  Object.assign(context, {
    prefetchedRun: prepared, activeRunLoad: null, loadSequence: 0, selectedCollectionId: 'group', xyzOrigin: [1, 2, 3],
    model, staticInstanceBatches: [], retainedStaticInstances: 0, bodyBindings: [], transmissionBindings: [],
    sensorCamera: null, sensorFrameIndex: -1, sensorImageKey: '', sensorImageState: '', sensorImageRequest: 0,
    activeSensorImages: [], poses: null, times: null, transmission: null,
    camera: {position: new Vector(5, 6, 7)}, controls: {target: new Vector(1, 2, 3), update() {}},
    THREE: {Vector3: Vector}, scene: {add() {}, remove() {}},
    pause: () => {}, enableControls: () => {}, emptySensorCamera: () => {},
    seek: value => {context.currentTime = value;}, defineBounds: () => {}, details: () => {},
    view: () => {throw new Error('Automatic transition must not reset view');}, updateInstanceVisibility: () => {},
    renderNow: () => {}, refreshRunPrefetch: () => {}, history: {replaceState() {}},
    window: {}, snapshot: () => ({ready: context.ready, run: context.selectedRun.id}),
    failure: error => {throw error;},
    createRunPreparation: () => {throw new Error('Prepared selection must not issue new IO');},
  });
  context.window.parent = context.window;
  vm.runInContext(selection, context);
  const result = await context.selectRun('b', {automatic: true});
  assert.equal(result.run, 'b');assert.equal(result.ready, true);
  assert.equal(context.poses, data.poses);assert.equal(context.times, data.times);
  assert.equal(context.currentTime, 2);
  assert(!writes.some(([key, value]) => key === 'hidden' && value === false));
  assert.deepEqual(context.camera.position.coords, [4, 3, 9]);
  assert.deepEqual(context.controls.target.coords, [0, -1, 5]);
  assert.equal(context.activeRunLoad, null);
  await flush();
  console.log('Run prefetch PASS: real background model/motion/time/sensor preparation, bounded six images, deduplication, slot cancellation, branch/partial guards, delayed metadata cancellation, invalid data cancellation, active+next model lifetime, prepared buffer adoption, overlay-free automatic transition and preserved world viewpoint');
})().catch(error => {console.error(error);process.exitCode = 1;});
