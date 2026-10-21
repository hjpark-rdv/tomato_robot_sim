import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';

const $ = id => document.getElementById(id);
const viewport = $('viewport');
const scene = new THREE.Scene();
scene.background = new THREE.Color('#edf1f3');
const modelCache = new Map();
const instanceCullingCache = new WeakMap();
let staticInstanceBatches = [];
let retainedStaticInstances = 0, visibleStaticInstances = 0;
const bodyBindings = [];
const transmissionBindings = [];
let renderer, camera, controls, model, sceneInfo, replay, selectedRun;
let catalog = { runs: [] }, poses, times, transmission;
let ready = false, playing = false, currentTime = 0, speed = 1;
let animationFrame = null, renderQueued = false, lastWallTime = null, loadSequence = 0;
let currentView = 'robot', frameIndex = 0, frameAlpha = 0;
let fullBounds = new THREE.Box3(), robotBounds = new THREE.Box3(), targetBounds = new THREE.Box3();
let recordedRobotBounds = new THREE.Box3();
let farmContentBounds = new THREE.Box3();
const scratchP = new THREE.Vector3(), scratchQ = new THREE.Quaternion(), scratchScale = new THREE.Vector3();
const scratchP2 = new THREE.Vector3(), scratchQ2 = new THREE.Quaternion(), scratchScale2 = new THREE.Vector3();
const scratchM = new THREE.Matrix4();
const xyzOrigin = [0, 0, 0];
const cameraFrustum = new THREE.Frustum();
const cameraMatrix = new THREE.Matrix4();
const cullingSphere = new THREE.Sphere();
const phaseLabels = {
  initial: '초기 상태', settle: '안정화', rail_move: '레일 이동', ready_hold: '준비 자세',
  prepare_open: '그리퍼 미리 열기', prepare_open_hold: '열린 상태 유지',
  prepare_lift_branch: '리프트 조절', prepare_joint_branch: '팔 준비', approach: '진입',
  belt_start_open: '열린 상태에서 벨트 작동', close: '벨트 작동 · 닫기',
  cycle_open: '벨트 작동 · 열기', cycle_open_hold: '열린 상태 유지', cycle_reclose: '벨트 작동 · 다시 닫기',
  belt_finish: '벨트 작동 마무리', cycle_open_final: '최종 열기', cycle_open_final_hold: '열린 상태 유지',
  withdraw: '후퇴', return_joint_branch: '팔 복귀', return_lift_branch: '리프트 복귀', ready_close: '그리퍼 정리', hold: '대기',
};
function engineLabel(value) {
  if (typeof value === 'string') return value;
  if (!value || typeof value !== 'object') return '기록 참조';
  const values = [value.physics_engine || value.name, value.backend, value.solver].filter(item => typeof item === 'string' && item);
  return [...new Set(values)].join(' · ') || '기록 참조';
}

function renderNow() {
  if (!renderer || !camera) return;
  renderer.render(scene, camera);
}
function updateInstanceVisibility() {
  if (!camera || !staticInstanceBatches.length) return;
  camera.updateMatrixWorld();
  cameraMatrix.multiplyMatrices(camera.projectionMatrix, camera.matrixWorldInverse);
  cameraFrustum.setFromProjectionMatrix(cameraMatrix);
  visibleStaticInstances = 0;
  for (const batch of staticInstanceBatches) {
    const { node, originalMatrices, originalColors, spheres, count } = batch;
    let visible = 0;
    for (let i = 0; i < count; i++) {
      const j = i * 4;
      cullingSphere.center.set(spheres[j], spheres[j + 1], spheres[j + 2]);
      cullingSphere.radius = spheres[j + 3];
      if (!cameraFrustum.intersectsSphere(cullingSphere)) continue;
      node.instanceMatrix.array.set(originalMatrices.subarray(i * 16, i * 16 + 16), visible * 16);
      if (originalColors) node.instanceColor.array.set(originalColors.subarray(i * 3, i * 3 + 3), visible * 3);
      visible++;
    }
    node.count = visible;
    node.instanceMatrix.clearUpdateRanges();
    if (visible) node.instanceMatrix.addUpdateRange(0, visible * 16);
    node.instanceMatrix.needsUpdate = true;
    if (originalColors) {
      node.instanceColor.clearUpdateRanges();
      if (visible) node.instanceColor.addUpdateRange(0, visible * 3);
      node.instanceColor.needsUpdate = true;
    }
    visibleStaticInstances += visible;
  }
}
function render() {
  if (renderQueued || playing || !renderer) return;
  renderQueued = true;
  requestAnimationFrame(() => { renderQueued = false; renderNow(); });
}
function enableControls(enabled) {
  for (const id of ['timeline', 'playPause', 'restart', 'speed']) $(id).disabled = !enabled;
  document.querySelectorAll('[data-view]').forEach(button => { button.disabled = !enabled; });
}
function pause() {
  playing = false;
  if (animationFrame !== null) cancelAnimationFrame(animationFrame);
  animationFrame = null;
  lastWallTime = null;
  $('playPause').textContent = '▶ 재생';
  $('playPause').setAttribute('aria-label', '재생');
  render();
}
function animate(now) {
  if (!playing) return;
  if (lastWallTime !== null) seek(currentTime + Math.max(0, now - lastWallTime) / 1000 * speed);
  lastWallTime = now;
  renderNow();
  if (currentTime >= times[times.length - 1]) { pause(); return; }
  animationFrame = requestAnimationFrame(animate);
}
function play() {
  if (!ready || playing) return;
  if (currentTime >= times[times.length - 1]) seek(times[0]);
  playing = true;
  lastWallTime = null;
  $('playPause').textContent = 'Ⅱ 일시정지';
  $('playPause').setAttribute('aria-label', '일시정지');
  animationFrame = requestAnimationFrame(animate);
}

function locateFrame(time) {
  if (time <= times[0]) return [0, 0];
  if (time >= times[times.length - 1]) return [times.length - 1, 0];
  let lo = 0, hi = times.length - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >>> 1;
    if (times[mid] <= time) lo = mid;
    else hi = mid;
  }
  const gap = times[hi] - times[lo];
  return [lo, gap > 0 ? (time - times[lo]) / gap : 0];
}
function readPose(offset, position, quaternion) {
  // The stored execution remains in USD Z-up world coordinates. Convert to
  // the exported glTF basis C(x,y,z)=(x,z,-y), including quaternion vector part.
  position.set(poses[offset] - xyzOrigin[0], poses[offset + 2] - xyzOrigin[2], -(poses[offset + 1] - xyzOrigin[1]));
  quaternion.set(poses[offset + 3], poses[offset + 5], -poses[offset + 4], poses[offset + 6]).normalize();
}
function updateReadout() {
  $('timeline').value = String(currentTime);
  $('time').textContent = `${currentTime.toFixed(3)} / ${times[times.length - 1].toFixed(3)} s`;
  const phase = replay.phase?.[frameIndex] ?? '';
  const gap = replay.jaw_gap_m?.[frameIndex];
  const released = (replay.releases || []).filter(event => event.time_s <= currentTime).length;
  $('phase').textContent = [phaseLabels[phase] || phase, Number.isFinite(gap) ? `그리퍼 ${(gap * 1000).toFixed(1)} mm` : '', replay.releases?.length ? `분리 ${released}개` : ''].filter(Boolean).join(' · ');
  $('timeline').setAttribute('aria-valuetext', `${currentTime.toFixed(3)}초 ${phase}`);
}
function seek(time) {
  if (!ready || !Number.isFinite(Number(time))) return currentTime;
  currentTime = THREE.MathUtils.clamp(Number(time), times[0], times[times.length - 1]);
  [frameIndex, frameAlpha] = locateFrame(currentTime);
  const next = Math.min(frameIndex + 1, times.length - 1);
  const bodyCount = replay.motion.shape[1];
  for (const { node, bodyIndex } of bodyBindings) {
    readPose((frameIndex * bodyCount + bodyIndex) * 7, scratchP, scratchQ);
    if (frameAlpha > 0) {
      readPose((next * bodyCount + bodyIndex) * 7, scratchP2, scratchQ2);
      scratchP.lerp(scratchP2, frameAlpha);
      scratchQ.slerp(scratchQ2, frameAlpha);
    }
    node.position.copy(scratchP);
    node.quaternion.copy(scratchQ);
    node.updateMatrix();
  }
  if (transmission) {
    const count = replay.transmission.shape[1];
    for (const { node, matrixIndex } of transmissionBindings) {
      scratchM.fromArray(transmission, (frameIndex * count + matrixIndex) * 16).decompose(scratchP, scratchQ, scratchScale);
      if (frameAlpha > 0) {
        scratchM.fromArray(transmission, (next * count + matrixIndex) * 16).decompose(scratchP2, scratchQ2, scratchScale2);
        scratchP.lerp(scratchP2, frameAlpha); scratchQ.slerp(scratchQ2, frameAlpha); scratchScale.lerp(scratchScale2, frameAlpha);
      }
      node.position.copy(scratchP); node.quaternion.copy(scratchQ); node.scale.copy(scratchScale); node.updateMatrix();
    }
  }
  model.updateMatrixWorld(true);
  updateReadout();
  render();
  return currentTime;
}

function fit(box, direction, padding = 1.15) {
  if (box.isEmpty()) return;
  const center = box.getCenter(new THREE.Vector3());
  const radius = Math.max(.1, box.getSize(new THREE.Vector3()).length() / 2);
  const halfFov = THREE.MathUtils.degToRad(camera.fov / 2);
  const angle = Math.min(halfFov, Math.atan(Math.tan(halfFov) * camera.aspect));
  const distance = radius / Math.sin(angle) * padding;
  camera.position.copy(center).add(new THREE.Vector3(...direction).normalize().multiplyScalar(distance));
  camera.near = .005;
  camera.far = Math.max(200, distance * 8);
  camera.updateProjectionMatrix();
  controls.target.copy(center);
  controls.minDistance = .03;
  controls.maxDistance = Math.max(100, distance * 4);
  controls.update();
  updateInstanceVisibility();
  render();
}
function liveRobotBounds() {
  const box = new THREE.Box3();
  for (const binding of bodyBindings) {
    if (/HarvestRobot|ActuatedTool/.test(binding.bodyPath)) box.expandByPoint(binding.node.getWorldPosition(new THREE.Vector3()));
  }
  if (!box.isEmpty()) box.expandByScalar(.25);
  return box.isEmpty() ? robotBounds : box;
}
function view(name) {
  if (!ready) return;
  currentView = name;
  camera.up.set(0, 1, 0);
  camera.fov = name === 'robot' || name === 'target' ? 60 : 42;
  camera.updateProjectionMatrix();
  let box, direction;
  if (name === 'whole') { box = farmContentBounds; direction = [-1.8, 1.5, 2.5]; }
  else if (name === 'top') { box = recordedRobotBounds.clone(); box.union(targetBounds); direction = [0, 1, .001]; }
  else {
    const robotCenter = recordedRobotBounds.getCenter(new THREE.Vector3());
    const targetCenter = targetBounds.getCenter(new THREE.Vector3());
    if (name === 'target') {
      controls.target.copy(targetCenter);
      camera.position.set(robotCenter.x + .20, targetCenter.y + .12, targetCenter.z + 1.05);
    } else {
      box = recordedRobotBounds.clone().union(targetBounds);
      const center = box.getCenter(new THREE.Vector3()), size = box.getSize(new THREE.Vector3());
      const halfFov = THREE.MathUtils.degToRad(camera.fov / 2);
      const fitDistance = Math.max(size.y / (2 * Math.tan(halfFov)), size.x / (2 * Math.tan(halfFov) * camera.aspect));
      controls.target.copy(center);
      // Stay inside the aisle: an oblique fit from outside either crop row
      // puts the camera behind the actual greenhouse wall.
      camera.position.set(robotCenter.x + .15, Math.max(1.35, Math.min(1.65, center.y + .4)), box.max.z + Math.max(1.6, fitDistance * .8));
    }
    camera.near = .005; camera.far = 200; camera.updateProjectionMatrix();
    controls.minDistance = .03; controls.maxDistance = 100;
    controls.update(); updateInstanceVisibility(); render();
  }
  if (name === 'whole' || name === 'top') fit(box.clone(), direction);
  document.querySelectorAll('[data-view]').forEach(button => button.setAttribute('aria-pressed', String(button.dataset.view === name)));
}

function resolveURL(path, base) { return new URL(path, base).href; }
async function fetchJSON(url) {
  const response = await fetch(url, { cache: 'no-cache' });
  if (!response.ok) throw new Error(`기록 HTTP ${response.status}: ${new URL(url).pathname}`);
  return response.json();
}
async function fetchBinary(url, compression, label, expectedBytes) {
  const response = await fetch(url);
  if (!response.ok) throw new Error(`데이터 HTTP ${response.status}: ${new URL(url).pathname}`);
  const total = Number(response.headers.get('content-length')) || expectedBytes;
  const reader = response.body.getReader(), chunks = [];
  let loaded = 0;
  while (true) {
    const { value, done } = await reader.read();
    if (done) break;
    chunks.push(value); loaded += value.byteLength;
    $('loadMessage').textContent = `${label} · ${(loaded / 1048576).toFixed(1)}${total ? ` / ${(total / 1048576).toFixed(1)}` : ''} MB`;
    if (total) $('progress').value = Math.min(100, loaded / total * 100);
    else $('progress').removeAttribute('value');
  }
  const blob = new Blob(chunks);
  const header = new Uint8Array(await blob.slice(0, 2).arrayBuffer());
  if (header[0] === 0x1f && header[1] === 0x8b) {
    if (typeof DecompressionStream === 'undefined') throw new Error('압축된 3D 기록을 읽으려면 최신 Chrome, Edge 또는 Safari가 필요합니다.');
    return new Response(blob.stream().pipeThrough(new DecompressionStream('gzip'))).arrayBuffer();
  }
  return blob.arrayBuffer();
}
function hasAncestor(node, predicate) {
  for (let p = node; p; p = p.parent) if (predicate(p)) return true;
  return false;
}
function batchStaticInstances(root, dynamicNames) {
  // Native EXT_mesh_gpu_instancing remains intact. Batch only repeated static
  // geometry/material identities, never bones, skin or measured rigid bodies.
  root.updateMatrixWorld(true);
  const groups = new Map();
  root.traverse(node => {
    if (!node.isMesh || node.isSkinnedMesh || node.isInstancedMesh || !node.visible || Array.isArray(node.material) || node.morphTargetInfluences?.length) return;
    if (hasAncestor(node, p => dynamicNames.has(p.name) || p.isBone || p.isSkinnedMesh || !p.visible)) return;
    const key = `${node.geometry.uuid}:${node.material.uuid}`;
    if (!groups.has(key)) groups.set(key, []);
    groups.get(key).push(node);
  });
  const rootInverse = new THREE.Matrix4().copy(root.matrixWorld).invert();
  for (const nodes of groups.values()) {
    if (nodes.length < 4) continue;
    const instance = new THREE.InstancedMesh(nodes[0].geometry, nodes[0].material, nodes.length);
    instance.name = `static_batch_${nodes[0].name}`;
    instance.castShadow = false; instance.receiveShadow = false;
    const matrix = new THREE.Matrix4();
    nodes.forEach((node, i) => { matrix.multiplyMatrices(rootInverse, node.matrixWorld); instance.setMatrixAt(i, matrix); });
    instance.instanceMatrix.needsUpdate = true;
    instance.computeBoundingBox(); instance.computeBoundingSphere();
    root.add(instance);
    for (const node of nodes) node.removeFromParent();
  }
}
function prepareInstanceVisibility(root, dynamicNames) {
  if (instanceCullingCache.has(root)) return instanceCullingCache.get(root);
  root.updateMatrixWorld(true);
  const batches = [];
  const localMatrix = new THREE.Matrix4(), worldMatrix = new THREE.Matrix4();
  const sphere = new THREE.Sphere();
  root.traverse(node => {
    if (!node.isInstancedMesh || node.morphTexture || hasAncestor(node, parent => dynamicNames.has(parent.name) || parent.isBone || parent.isSkinnedMesh)) return;
    if (!node.geometry.boundingSphere) node.geometry.computeBoundingSphere();
    // Keep an aggregate sphere for all ORIGINAL instances. It remains valid
    // when visible matrices are compacted into the front of the GPU buffer.
    node.computeBoundingSphere();
    const count = node.count;
    const originalMatrices = new Float32Array(node.instanceMatrix.array);
    const originalColors = node.instanceColor ? new Float32Array(node.instanceColor.array) : null;
    const spheres = new Float64Array(count * 4);
    for (let i = 0; i < count; i++) {
      localMatrix.fromArray(originalMatrices, i * 16);
      worldMatrix.multiplyMatrices(node.matrixWorld, localMatrix);
      sphere.copy(node.geometry.boundingSphere).applyMatrix4(worldMatrix);
      spheres.set([sphere.center.x, sphere.center.y, sphere.center.z, sphere.radius + 1e-5], i * 4);
    }
    batches.push({ node, count, originalMatrices, originalColors, spheres });
  });
  instanceCullingCache.set(root, batches);
  return batches;
}
async function loadModel(metadata, base) {
  const sceneURL = resolveURL(metadata.scene_url, base);
  const key = sceneURL + '|' + (metadata.scene_sha256 || metadata.model_sha256 || '');
  if (modelCache.has(key)) return modelCache.get(key);
  const promise = (async () => {
    const info = await fetchJSON(resolveURL(metadata.scene_info_url, base));
    const url = metadata.scene_gzip_url && typeof DecompressionStream !== 'undefined' ? resolveURL(metadata.scene_gzip_url, base) : sceneURL;
    const buffer = await fetchBinary(url, url.endsWith('.gz') ? 'gzip' : null, '농장 3D 장면', info.compressed_bytes || info.bytes);
    $('loadMessage').textContent = '농장과 로봇의 3D 부품 조립 중…';
    const gltf = await new GLTFLoader().parseAsync(buffer, new URL('.', sceneURL).href);
    // A skin's deformation can exceed its cached reference-pose sphere.
    // Measured plant skins stay visible throughout the recorded movement.
    gltf.scene.traverse(node => { if (node.isSkinnedMesh) node.frustumCulled = false; });
    const dynamicNames = new Set((info.body_nodes || []).map(binding => binding.node));
    for (const name of metadata.transmission?.nodes || []) dynamicNames.add(name);
    batchStaticInstances(gltf.scene, dynamicNames);
    const instanceBatches = prepareInstanceVisibility(gltf.scene, dynamicNames);
    return { model: gltf.scene, info, instanceBatches };
  })();
  modelCache.set(key, promise);
  try { return await promise; }
  catch (error) { modelCache.delete(key); throw error; }
}
function toDisplayPoint(point) { return new THREE.Vector3(point[0] - xyzOrigin[0], point[2] - xyzOrigin[2], -(point[1] - xyzOrigin[1])); }
function configuredBox(value) {
  if (!value) return null;
  if (Array.isArray(value) && value.length === 2 && value[0].length === 3) {
    const box = new THREE.Box3();
    // Exported scene bounds already use glTF Y-up.
    box.set(new THREE.Vector3(...value[0]), new THREE.Vector3(...value[1]));
    return box;
  }
  return null;
}
function defineBounds() {
  model.updateMatrixWorld(true);
  fullBounds = configuredBox(sceneInfo.bounds_y_up_m || sceneInfo.bounds_m || sceneInfo.bounds) || new THREE.Box3().setFromObject(model);
  robotBounds = liveRobotBounds();
  // Fit the entire recorded robot movement, including rail travel, so playback
  // never leaves a camera framed around only the initial position.
  recordedRobotBounds = new THREE.Box3();
  const robotIndices = bodyBindings.filter(binding => /HarvestRobot|ActuatedTool/.test(binding.bodyPath)).map(binding => binding.bodyIndex);
  const bodyCount = replay.motion.shape[1];
  const point = new THREE.Vector3();
  for (let frame = 0; frame < times.length; frame++) {
    for (const bodyIndex of robotIndices) {
      const offset = (frame * bodyCount + bodyIndex) * 7;
      point.set(poses[offset] - xyzOrigin[0], poses[offset + 2] - xyzOrigin[2], -(poses[offset + 1] - xyzOrigin[1]));
      recordedRobotBounds.expandByPoint(point);
    }
  }
  if (recordedRobotBounds.isEmpty()) recordedRobotBounds.copy(robotBounds);
  else recordedRobotBounds.expandByScalar(.30);
  const targetPoint = replay.target_center_world_m || replay.target?.center_world_m || sceneInfo.target_center_world_m;
  const targetBody = replay.target?.body_path || replay.target_body_path;
  const targetBinding = bodyBindings.find(binding => binding.bodyPath === targetBody);
  let center;
  if (targetPoint) center = toDisplayPoint(targetPoint);
  else if (targetBinding) center = targetBinding.node.getWorldPosition(new THREE.Vector3());
  else {
    const plants = bodyBindings.filter(binding => !/HarvestRobot|ActuatedTool/.test(binding.bodyPath));
    const robotCenter = robotBounds.getCenter(new THREE.Vector3());
    plants.sort((a, b) => a.node.position.distanceToSquared(robotCenter) - b.node.position.distanceToSquared(robotCenter));
    center = plants[0] ? plants[0].node.getWorldPosition(new THREE.Vector3()) : robotCenter.clone();
  }
  targetBounds = new THREE.Box3().setFromCenterAndSize(center, new THREE.Vector3(.8, 1.0, .8));
  farmContentBounds = recordedRobotBounds.clone().union(targetBounds);
  const background = model.getObjectByName(sceneInfo.groups?.background_plants?.node || 'background_plants');
  if (background) farmContentBounds.union(new THREE.Box3().setFromObject(background));
  const greenhouse = model.getObjectByName(sceneInfo.groups?.greenhouse?.node || 'greenhouse');
  greenhouse?.traverse(node => {
    if (!node.isMesh) return;
    const box = new THREE.Box3().setFromObject(node);
    const size = box.getSize(new THREE.Vector3());
    // The source includes a 16×24 m floor around a much smaller house. Keep
    // drawing it, while fitting the actual farm structure and crop rows.
    if (size.y < .2 && Math.max(size.x, size.z) > 6) return;
    farmContentBounds.union(box);
  });
  if (farmContentBounds.isEmpty()) farmContentBounds.copy(fullBounds);
}
function details() {
  const rows = [
    ['실행', selectedRun.label || selectedRun.id], ['엔진', engineLabel(replay.engine || selectedRun.engine)], ['장면', '기록 당시 모델'],
    ['기간', `${times[0].toFixed(3)}–${times[times.length - 1].toFixed(3)} s`],
    ['상태', `${times.length.toLocaleString()} 프레임 · ${replay.motion.shape[1]}개 바디`],
    ['결과', selectedRun.result || replay.result_summary || replay.status],
    ['모델', replay.scene_sha256 || replay.model_sha256 || sceneInfo.source_scene_sha256 || sceneInfo.source_sha256],
  ].filter(([, value]) => value !== undefined && value !== null && value !== '');
  $('runDetails').replaceChildren(...rows.flatMap(([label, value]) => {
    const term = document.createElement('dt'), desc = document.createElement('dd');
    term.textContent = label; desc.textContent = typeof value === 'object' ? JSON.stringify(value) : String(value);
    return [term, desc];
  }));
  $('runDescription').textContent = selectedRun.description || replay.description || '';
}
function failure(error) {
  console.error(error);
  $('loadTitle').textContent = '실행 기록을 불러오지 못했습니다';
  $('loadMessage').textContent = error.message || String(error);
  $('progress').hidden = true; $('retry').hidden = false; $('loading').hidden = false;
  enableControls(false);
}
async function selectRun(id) {
  const run = catalog.runs.find(item => item.id === id);
  if (!run) throw new Error('선택한 실행 기록이 없습니다: ' + id);
  const sequence = ++loadSequence;
  pause(); ready = false; enableControls(false);
  selectedRun = run;
  $('loading').hidden = false; $('retry').hidden = true; $('progress').hidden = false; $('progress').removeAttribute('value');
  $('loadTitle').textContent = '농장과 실행 기록 불러오는 중';
  $('loadMessage').textContent = run.label || run.id;
  $('runSelect').value = run.id;
  try {
    const metadataURL = resolveURL(run.replay_url, new URL('./catalog.json', location.href));
    const metadata = await fetchJSON(metadataURL);
    if (!metadata.motion?.shape || metadata.motion.shape.length !== 3 || metadata.motion.shape[2] !== 7) throw new Error('위치·회전 기록의 형식을 확인할 수 없습니다.');
    const loaded = await loadModel(metadata, metadataURL);
    if (sequence !== loadSequence) return;
    const [motionBuffer, timeBuffer, transmissionBuffer] = await Promise.all([
      fetchBinary(resolveURL(metadata.motion.url, metadataURL), metadata.motion.compression, '실제 실행 상태', metadata.motion.bytes_compressed),
      fetchBinary(resolveURL(metadata.time.url, metadataURL), metadata.time.compression, '실제 시뮬레이션 시간'),
      metadata.transmission ? fetchBinary(resolveURL(metadata.transmission.url, metadataURL), metadata.transmission.compression, '그리퍼 연결부 움직임') : null,
    ]);
    if (sequence !== loadSequence) return;
    const frameCount = metadata.motion.shape[0], bodyCount = metadata.motion.shape[1];
    if (motionBuffer.byteLength !== frameCount * bodyCount * 7 * 4) throw new Error('위치·회전 기록의 데이터 크기가 일치하지 않습니다.');
    if (timeBuffer.byteLength !== frameCount * 8) throw new Error('타임스탬프와 상태 프레임 수가 일치하지 않습니다.');
    const nextTimes = new Float64Array(timeBuffer);
    for (let i = 0; i < nextTimes.length; i++) {
      if (!Number.isFinite(nextTimes[i]) || (i && nextTimes[i] <= nextTimes[i - 1])) throw new Error('실제 시뮬레이션 시간이 올바른 순서로 기록되지 않았습니다.');
    }
    if (model) scene.remove(model);
    model = loaded.model; sceneInfo = loaded.info; replay = metadata; selectedRun = run;
    staticInstanceBatches = loaded.instanceBatches;
    retainedStaticInstances = staticInstanceBatches.reduce((sum, batch) => sum + batch.count, 0);
    scene.add(model);
    poses = new Float32Array(motionBuffer); times = nextTimes;
    transmission = transmissionBuffer ? new Float32Array(transmissionBuffer) : null;
    if (transmission && transmission.length !== metadata.transmission.shape.reduce((a, b) => a * b, 1)) throw new Error('그리퍼 연결부 기록의 크기가 일치하지 않습니다.');
    const origin = sceneInfo.origin_world_m || replay.origin_world_m || [0, 0, 0];
    xyzOrigin.splice(0, 3, ...origin);
    bodyBindings.length = 0; transmissionBindings.length = 0;
    for (const binding of sceneInfo.body_nodes || []) {
      const node = model.getObjectByName(binding.node);
      if (!node) throw new Error('3D 장면에서 실행 바디를 찾지 못했습니다: ' + binding.node);
      if (binding.body_index < 0 || binding.body_index >= bodyCount) throw new Error('실행 바디 인덱스가 기록 범위를 벗어났습니다.');
      bodyBindings.push({ node, bodyIndex: binding.body_index, bodyPath: binding.body_path });
    }
    for (const [matrixIndex, name] of (metadata.transmission?.nodes || []).entries()) {
      const node = model.getObjectByName(name);
      if (!node) throw new Error('그리퍼 연결부를 찾지 못했습니다: ' + name);
      transmissionBindings.push({ node, matrixIndex });
    }
    ready = true;
    $('timeline').min = String(times[0]); $('timeline').max = String(times[times.length - 1]);
    $('runTitle').textContent = run.label || run.id;
    $('durationInfo').textContent = `${(times[times.length - 1] - times[0]).toFixed(1)}초 기록 · ${times.length.toLocaleString()} 프레임`;
    seek(times[0]); defineBounds(); details(); view('robot');
    enableControls(true); $('loading').hidden = true; $('readout').hidden = false;
    const url = new URL(location.href); url.searchParams.set('run', run.id); history.replaceState(null, '', url);
    if (window.parent !== window) window.parent.postMessage({ type: 'farmily-replay:run', run: run.id }, location.origin);
    // Free GPU upload and render before automated readiness checks inspect pixels.
    renderNow();
    return snapshot();
  } catch (error) {
    if (sequence === loadSequence) { ready = false; failure(error); }
    throw error;
  }
}
function snapshot(options = {}) {
  const stats = { meshes: 0, skinnedMeshes: 0, instancedMeshes: 0, instances: 0 };
  model?.traverse(node => {
    if (node.isMesh) stats.meshes++;
    if (node.isSkinnedMesh) stats.skinnedMeshes++;
    if (node.isInstancedMesh) { stats.instancedMeshes++; stats.instances += node.count; }
  });
  const result = { ready, playing, run: selectedRun?.id || null, time_s: currentTime, speed, frame: frameIndex, frameIndex, interpolation: frameAlpha,
    start_s: times?.[0], end_s: times?.[times.length - 1], frames: times?.length || 0, bodies: bodyBindings.length,
    phase: replay?.phase?.[frameIndex], releases: (replay?.releases || []).filter(event => event.time_s <= currentTime),
    camera: camera?.position.toArray(), target: controls?.target.toArray(), view: currentView,
    renderer: { drawCalls: renderer?.info.render.calls || 0, triangles: renderer?.info.render.triangles || 0, ...stats,
      retainedStaticInstances, visibleStaticInstances, culledStaticInstances: retainedStaticInstances - visibleStaticInstances },
    scene_sha256: replay?.scene_sha256 || sceneInfo?.source_scene_sha256, source: replay?.source, availableRuns: catalog.runs.map(({ id, label }) => ({ id, label })) };
  if (options.includeBodies) result.bodyTransforms = bodyBindings.map(binding => ({ body_path: binding.bodyPath, body_index: binding.bodyIndex, node: binding.node.name, position: binding.node.position.toArray(), quaternion: binding.node.quaternion.toArray() }));
  return result;
}
function bodyPose(bodyPath) {
  const binding = bodyBindings.find(item => item.bodyPath === bodyPath || item.node.name === bodyPath);
  if (!binding) return null;
  model.updateMatrixWorld(true);
  return { body_path: binding.bodyPath, body_index: binding.bodyIndex, node: binding.node.name,
    position: binding.node.getWorldPosition(new THREE.Vector3()).toArray(),
    quaternion: binding.node.getWorldQuaternion(new THREE.Quaternion()).toArray(),
    time_s: currentTime, frameIndex, interpolation: frameAlpha };
}
window.HARVEST_REPLAY_API = { snapshot, seek, play, pause, selectRun, view, bodyPose };

$('playPause').onclick = () => playing ? pause() : play();
$('restart').onclick = () => { pause(); if (ready) seek(times[0]); };
$('timeline').oninput = event => { pause(); seek(Number(event.target.value)); };
$('speed').onchange = event => { speed = Number(event.target.value); lastWallTime = null; };
$('detailsButton').onclick = () => { $('details').hidden = !$('details').hidden; $('detailsButton').setAttribute('aria-expanded', String(!$('details').hidden)); };
$('runSelect').onchange = event => { selectRun(event.target.value).catch(() => {}); };
$('retry').onclick = () => selectedRun ? selectRun(selectedRun.id).catch(() => {}) : location.reload();
document.querySelectorAll('[data-view]').forEach(button => { button.onclick = () => view(button.dataset.view); });
document.addEventListener('visibilitychange', () => { if (document.hidden) pause(); });
window.addEventListener('message', event => {
  if (event.origin !== location.origin || event.source !== window.parent) return;
  if (event.data?.type === 'farmily-replay:pause') pause();
  else if (event.data?.type === 'farmily-replay:select' && typeof event.data.run === 'string') {
    const run = event.data.run;
    if (catalog.runs.some(item => item.id === run) && selectedRun?.id !== run) selectRun(run).catch(() => {});
  }
});
document.addEventListener('keydown', event => {
  if (!ready || /INPUT|SELECT|BUTTON|TEXTAREA/.test(event.target.tagName)) return;
  if (event.code === 'Space') { event.preventDefault(); playing ? pause() : play(); }
  else if (event.key === 'ArrowLeft' || event.key === 'ArrowRight') { event.preventDefault(); pause(); seek(currentTime + (event.key === 'ArrowLeft' ? -1 : 1)); }
});

try {
  renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false, powerPreference: 'high-performance' });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 1.5));
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.15;
  viewport.prepend(renderer.domElement);
  renderer.domElement.setAttribute('aria-label', '저장된 수확 동작을 자유로운 시점으로 보는 3D 장면');
  camera = new THREE.PerspectiveCamera(42, 1, .005, 200);
  controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = false;
  controls.addEventListener('change', () => { updateInstanceVisibility(); render(); });
  scene.add(new THREE.HemisphereLight(0xffffff, 0x81968a, 2.0));
  for (const [position, intensity] of [[[6, 12, 5], 2.5], [[-8, 6, -5], 1.8], [[2, 4, -8], 1.0]]) {
    const light = new THREE.DirectionalLight(0xffffff, intensity); light.position.set(...position); scene.add(light);
  }
  new ResizeObserver(() => {
    const { width, height } = viewport.getBoundingClientRect();
    if (!width || !height) return;
    camera.aspect = width / height; camera.updateProjectionMatrix(); renderer.setSize(width, height); updateInstanceVisibility(); render();
  }).observe(viewport);
  catalog = await fetchJSON(new URL('./catalog.json', location.href));
  if (!Array.isArray(catalog.runs) || !catalog.runs.length) throw new Error('아직 등록된 실행 기록이 없습니다.');
  $('runSelect').replaceChildren(...catalog.runs.map(run => { const option = document.createElement('option'); option.value = run.id; option.textContent = run.label || run.id; return option; }));
  $('runSelect').disabled = false;
  const requested = new URLSearchParams(location.search).get('run');
  await selectRun(catalog.runs.some(run => run.id === requested) ? requested : catalog.runs[0].id);
} catch (error) { failure(error); }
