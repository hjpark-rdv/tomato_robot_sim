// Exercise the real PIP selection/loading functions without browser or GPU.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const source = fs.readFileSync(path.join(__dirname, 'viewer.js'), 'utf8');
const start = source.indexOf('function resolveURL(');
const end = source.indexOf('\nasync function fetchJSON(', start);
assert(start >= 0 && end > start);
const nodes = new Map(), images = [];
const context = {
  URL, activeSensorImages: [], sensorCamera: null, sensorFrameIndex: -1, sensorImageKey: '', sensorImageRequest: 0,
  sensorMode: 'rgb', sensorStatus: 'empty', sensorImageState: 'empty', currentTime: 0,
  selectedRun: {id: 'actual'}, phaseLabels: {},
  $: id => {
    if (!nodes.has(id)) nodes.set(id, {dataset: {}, hidden: false, removeAttribute(key) {delete this[key];}});
    return nodes.get(id);
  },
  Image: class {constructor() {images.push(this);}},
  fetchJSON: async () => ({frames: []}),
};
vm.runInNewContext(source.slice(start, end), context);
const frames = [1, 2, 3].map(timestamp_s => ({timestamp_s, rgb: `frame${timestamp_s}/rgb.png`,
  recognition: `frame${timestamp_s}/harvest_region.png`, depth_overlay: `frame${timestamp_s}/depth.png`}));
context.sensorCamera = {frames, framesURL: 'https://farm.test/camera/frames.json', metadata: {}, maxAge: .6};
const select = time => {context.currentTime = time;context.updateSensorCamera();};
const image = () => nodes.get('cameraImage');
(async () => {
  select(.99);
  assert.equal(context.sensorFrameIndex, -1);
  assert.equal(image().hidden, true);
  assert.equal(images.length, 0);
  select(1);
  assert.equal(context.sensorFrameIndex, 0);
  const firstLoad = images.find(item => item.onload);
  select(1.999);
  assert.equal(context.sensorFrameIndex, 0, 'A future exposure must not be selected');
  select(2);
  assert.equal(context.sensorFrameIndex, 1);
  const secondLoad = images.filter(item => item.onload).at(-1);
  firstLoad.onload();
  assert.equal(image().hidden, true, 'Superseded image request cannot reveal an older frame');
  secondLoad.onload();
  assert.equal(image().src, 'https://farm.test/camera/frame2/rgb.png');
  select(2.7);
  assert.equal(context.sensorStatus, 'stale');
  assert.match(nodes.get('cameraState').textContent, /촬영 누락/);
  assert.equal(context.sensorFrameIndex, 1);
  context.sensorMode = 'recognition';select(2.7);
  images.filter(item => item.onload).at(-1).onload();
  assert.equal(image().src, 'https://farm.test/camera/frame2/harvest_region.png');
  context.sensorMode = 'depth_overlay';select(3);
  const futureLoad = images.filter(item => item.onload).at(-1);
  select(.5);futureLoad.onload();
  assert.equal(context.sensorFrameIndex, -1);
  assert.equal(image().hidden, true);
  assert.equal(image().src, undefined, 'Rewinding before first exposure cannot display a future frame');
  context.sensorMode = 'unrecorded';select(2);
  assert.equal(context.sensorStatus, 'missing');
  assert.equal(image().hidden, true);
  context.sensorMode = 'rgb';select(3);
  images.filter(item => item.onerror).at(-1).onerror();select(3.1);
  assert.equal(context.sensorStatus, 'error', 'Same failed exposure remains explicitly marked');
  assert.equal(await context.loadSensorCamera({}, 'https://farm.test/replay.json'), null);
  const metadata = {camera: {schema: 'mounted-rgbd-replay-v1', frames: 'camera/frames.json'}};
  context.fetchJSON = async () => ({frames: [{timestamp_s: 1}, {timestamp_s: 1}]});
  await assert.rejects(context.loadSensorCamera(metadata, 'https://farm.test/replay.json'), /순서/);
  context.fetchJSON = async () => ({frames});
  const loaded = await context.loadSensorCamera(metadata, 'https://farm.test/replay.json');
  assert.equal(loaded.framesURL, 'https://farm.test/camera/frames.json');
  assert.equal(loaded.maxAge, 1.6);
  // Execute the actual selectRun camera-reset block, then load a camera run
  // after a partial record without sensors. This is distinct from a slow real
  // browser image load; a timeout alone is not evidence of a reset defect.
  context.sensorCamera = null;
  context.emptySensorCamera('카메라 기록 없음');
  const resetStart = source.indexOf('sensorCamera = null;', source.indexOf('async function selectRun('));
  const resetEnd = source.indexOf('selectedRun = run;', resetStart);
  assert(resetStart >= 0 && resetEnd > resetStart);
  vm.runInNewContext(source.slice(resetStart, resetEnd), context);
  const controlFrames = [0, .5, 1, 1.5, 2, 2.5, 3].map((timestamp_s, index) => ({timestamp_s, rgb: `control${index}/rgb.png`}));
  context.fetchJSON = async () => ({frames: controlFrames});
  context.sensorCamera = await context.loadSensorCamera(metadata, 'https://farm.test/replay.json');
  context.sensorMode = 'rgb';
  context.selectedRun = {id: 'independent-control'};
  select(0);select(1.7);
  images.filter(item => item.onload).at(-1).onload();
  assert.equal(context.sensorFrameIndex, 3);
  assert.equal(image().src, 'https://farm.test/camera/control3/rgb.png');
  assert.equal(image().hidden, false);
  assert.equal(nodes.get('cameraEmpty').hidden, true);
  console.log('13 actual PIP checks PASS: causal time, exact exposure, asynchronous cancellation, stale, GT ROI mode, rewind, missing, error, legacy, timestamp validation, relative assets and actual partial-to-camera run reset; no browser/GPU');
})().catch(error => {console.error(error);process.exitCode = 1;});
