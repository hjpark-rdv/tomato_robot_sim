import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';

const $ = id => document.getElementById(id);
const viewport = $('viewport');
const scene = new THREE.Scene();
scene.background = new THREE.Color('#edf1f3');
const modelCache = new Map();
let activeModelKey = null, disposedModelCount = 0;
// Keep one upcoming execution, rather than retaining every campaign's arrays
// and decoded scene. A foreground selection can adopt this same promise.
let prefetchedRun = null, activeRunLoad = null;
let activeSensorImages = [];
const instanceCullingCache = new WeakMap();
let staticInstanceBatches = [];
let retainedStaticInstances = 0, visibleStaticInstances = 0;
const bodyBindings = [];
const transmissionBindings = [];
let renderer, camera, controls, model, sceneInfo, replay, selectedRun;
let catalog = { runs: [] }, poses, times, transmission;
let selectedCollectionId = '';
let ready = false, playing = false, currentTime = 0, speed = 1;
let animationFrame = null, renderQueued = false, lastWallTime = null, loadSequence = 0;
let currentView = 'robot', frameIndex = 0, frameAlpha = 0;
let fullBounds = new THREE.Box3(), robotBounds = new THREE.Box3(), targetBounds = new THREE.Box3();
let recordedRobotBounds = new THREE.Box3();
let farmContentBounds = new THREE.Box3();
let sensorCamera = null, sensorFrameIndex = -1, sensorImageKey = '', sensorImageRequest = 0;
let sensorMode = 'rgb', sensorStatus = 'empty', sensorImageState = 'empty';
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

function isPartialDiagnostic(run = selectedRun, metadata = replay) {
  return run?.record_kind === 'partial_diagnostic' || metadata?.source?.record_kind === 'partial_diagnostic';
}
function isStoppedExecution(run = selectedRun, metadata = replay) {
  const result = metadata?.result_summary || run?.result;
  return (result?.completed === false && Boolean(result?.stop_reason)) ||
    (run?.completed === false && Boolean(run?.physical_stop_reason));
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
  for (const id of ['timeline', 'collectionTimeline', 'playPause', 'restart', 'speed']) $(id).disabled = !enabled;
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
  if (currentTime >= times[times.length - 1]) {
    pause();
    const group = currentCollection(), ids = group?.runs || [], index = ids.indexOf(selectedRun?.id);
    if (isStoppedExecution()) {
      $('branchNotice').textContent = '실제 물리 실행이 중단된 마지막 저장 상태입니다. 이후 구간으로 자동 재생하지 않습니다.';
      $('branchNotice').hidden = false;
    } else if (!isPartialDiagnostic() && $('collectionNext').checked && index >= 0 && index + 1 < ids.length) {
      const nextId = ids[index + 1], branch = independentBranchBoundary(group, nextId);
      if (branch) showIndependentBranchNotice(branch);
      else if (catalog.runs.find(run => run.id === nextId)?.record_kind === 'partial_diagnostic') {
        $('branchNotice').textContent = '다음 구간은 오류로 중단된 일부 진단 기록입니다. 기록 선택에서 직접 선택해 확인하세요.';
        $('branchNotice').hidden = false;
      } else selectRun(nextId, { automatic: true }).then(result => {
        if (result && ready && selectedRun?.id === nextId) play();
      }).catch(() => {});
    }
    return;
  }
  animationFrame = requestAnimationFrame(animate);
}
function play() {
  if (!ready || playing) return;
  if (currentTime >= times[times.length - 1]) seek(times[0]);
  playing = true;
  lastWallTime = null;
  $('playPause').textContent = 'Ⅱ 일시정지';
  $('playPause').setAttribute('aria-label', '일시정지');
  refreshRunPrefetch();
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
  updateSensorCamera();
  updateCollectionTimeline();
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
function emptySensorCamera(message, state = 'empty') {
  sensorStatus = state;
  $('cameraPip').dataset.state = state;
  $('cameraImage').hidden = true;
  $('cameraImage').removeAttribute('src');
  $('cameraEmpty').textContent = message;
  $('cameraEmpty').hidden = false;
  $('cameraStamp').textContent = '';
  $('cameraState').textContent = state === 'error' ? '3D 동작 기록은 계속 확인할 수 있습니다.' : '이 실행에서 저장한 카메라 프레임만 표시합니다.';
}
async function loadSensorCamera(metadata, metadataURL, options = {}) {
  if (!metadata.camera) return null;
  if (metadata.camera.schema !== 'mounted-rgbd-replay-v1' || typeof metadata.camera.frames !== 'string') throw new Error('카메라 기록 형식을 확인할 수 없습니다.');
  const framesURL = resolveURL(metadata.camera.frames, metadataURL);
  const data = await fetchJSON(framesURL, options);
  if (!Array.isArray(data.frames) || !data.frames.length) throw new Error('저장된 카메라 프레임이 없습니다.');
  let previous = -Infinity;
  const intervals = [];
  for (const [index, frame] of data.frames.entries()) {
    if (!Number.isFinite(frame.timestamp_s) || frame.timestamp_s <= previous) throw new Error('카메라 촬영 시간이 올바른 순서가 아닙니다.');
    if (index) intervals.push(frame.timestamp_s - previous);
    previous = frame.timestamp_s;
  }
  intervals.sort((a, b) => a - b);
  const interval = intervals.length ? intervals[Math.floor(intervals.length / 2)] : .5;
  const configuredAge = metadata.camera.max_frame_age_s ?? data.max_frame_age_s;
  const maxAge = Number.isFinite(configuredAge) && configuredAge >= 0 ? configuredAge : interval * 1.6;
  return { frames: data.frames, framesURL, metadata: metadata.camera, maxAge };
}
function updateSensorCamera() {
  if (!sensorCamera) return;
  const frames = sensorCamera.frames;
  let lo = 0, hi = frames.length;
  // Causal playback: a future exposure is never shown before its timestamp.
  while (lo < hi) {
    const mid = (lo + hi) >>> 1;
    if (frames[mid].timestamp_s <= currentTime + 1e-9) lo = mid + 1;
    else hi = mid;
  }
  sensorFrameIndex = lo - 1;
  if (sensorFrameIndex < 0) {
    sensorImageKey = ''; sensorImageRequest++;
    emptySensorCamera(`첫 촬영 대기 · ${frames[0].timestamp_s.toFixed(3)} s`);
    return;
  }
  const frame = frames[sensorFrameIndex], age = Math.max(0, currentTime - frame.timestamp_s);
  const stale = age > sensorCamera.maxAge + 1e-9;
  const phase = phaseLabels[frame.phase] || frame.phase || '';
  $('cameraStamp').textContent = `촬영 ${frame.timestamp_s.toFixed(3)} s · 재생 ${currentTime.toFixed(3)} s${phase ? ` · ${phase}` : ''}`;
  $('cameraState').textContent = stale ? `이 구간 촬영 누락 · ${age.toFixed(2)}초 전 프레임을 유지합니다.` : `저장된 관측 · ${age.toFixed(2)}초 경과${sensorCamera.metadata.mode === 'measured_state_render' ? ' · 저장 상태 렌더' : ''}`;
  const path = frame[sensorMode];
  const key = `${selectedRun?.id}:${sensorFrameIndex}:${sensorMode}`;
  sensorStatus = key === sensorImageKey && ['error', 'missing'].includes(sensorImageState) ? sensorImageState : stale ? 'stale' : 'ready';
  $('cameraPip').dataset.state = sensorStatus;
  if (key === sensorImageKey) return;
  sensorImageKey = key;
  sensorImageState = 'pending';
  const request = ++sensorImageRequest;
  $('cameraImage').hidden = true;
  $('cameraEmpty').hidden = false;
  if (typeof path !== 'string' || !path) {
    $('cameraEmpty').textContent = '이 프레임의 선택한 영상은 저장되지 않았습니다.';
    sensorImageState = sensorStatus = 'missing'; $('cameraPip').dataset.state = 'missing';
    return;
  }
  $('cameraEmpty').textContent = '촬영 프레임 불러오는 중…';
  const imageURL = resolveURL(path, sensorCamera.framesURL);
  const pending = activeSensorImages.find(item => item.url === imageURL && item.loaded)?.image || new Image();
  const show = () => {
    if (request !== sensorImageRequest) return;
    sensorImageState = 'ready';
    $('cameraImage').src = imageURL;
    $('cameraImage').hidden = false; $('cameraEmpty').hidden = true;
    // Warm only the next exposure in this mode, keeping long campaigns bounded.
    const nextPath = sensorCamera?.frames[sensorFrameIndex + 1]?.[sensorMode];
    if (typeof nextPath === 'string' && nextPath) {
      const preload = new Image(); preload.src = resolveURL(nextPath, sensorCamera.framesURL);
    }
  };
  pending.onload = show;
  pending.onerror = () => {
    if (request !== sensorImageRequest) return;
    sensorImageState = sensorStatus = 'error'; $('cameraPip').dataset.state = 'error';
    $('cameraEmpty').textContent = '촬영 프레임을 불러오지 못했습니다.';
  };
  if (pending.complete && pending.naturalWidth > 0 && pending.src === imageURL) show();
  else pending.src = imageURL;
}
async function fetchJSON(url, options = {}) {
  const response = await fetch(url, { cache: 'no-cache', ...options });
  if (!response.ok) throw new Error(`기록 HTTP ${response.status}: ${new URL(url).pathname}`);
  return response.json();
}
function reportLoadProgress(label, loaded, total) {
  $('loadMessage').textContent = `${label} · ${(loaded / 1048576).toFixed(1)}${total ? ` / ${(total / 1048576).toFixed(1)}` : ''} MB`;
  if (total) $('progress').value = Math.min(100, loaded / total * 100);
  else $('progress').removeAttribute('value');
}
async function fetchBinary(url, compression, label, expectedBytes, options = {}) {
  const { onProgress = reportLoadProgress, ...fetchOptions } = options;
  const response = await fetch(url, fetchOptions);
  if (!response.ok) throw new Error(`데이터 HTTP ${response.status}: ${new URL(url).pathname}`);
  const total = Number(response.headers.get('content-length')) || expectedBytes;
  const reader = response.body.getReader(), chunks = [];
  let loaded = 0;
  while (true) {
    const { value, done } = await reader.read();
    if (done) break;
    chunks.push(value); loaded += value.byteLength;
    onProgress?.(label, loaded, total);
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
function disposeModelResources(root) {
  const geometries = new Set(), materials = new Set(), textures = new Set(), skeletons = new Set();
  root.traverse(node => {
    if (node.geometry) geometries.add(node.geometry);
    for (const material of Array.isArray(node.material) ? node.material : node.material ? [node.material] : []) materials.add(material);
    if (node.skeleton) skeletons.add(node.skeleton);
    // Instance buffers have their own disposal event, beyond shared geometry.
    if (node.isInstancedMesh) node.dispose();
  });
  for (const material of materials) {
    for (const value of Object.values(material)) if (value?.isTexture) textures.add(value);
  }
  const retainedImages = new Set();
  for (const entry of modelCache.values()) {
    if (!entry.value || entry.value.model === root) continue;
    entry.value.model.traverse(node => {
      for (const material of Array.isArray(node.material) ? node.material : node.material ? [node.material] : []) {
        for (const value of Object.values(material)) if (value?.isTexture && value.source?.data) retainedImages.add(value.source.data);
      }
    });
  }
  const images = new Set([...textures].map(texture => texture.source?.data).filter(Boolean));
  for (const skeleton of skeletons) skeleton.dispose();
  for (const geometry of geometries) geometry.dispose();
  for (const texture of textures) texture.dispose();
  for (const material of materials) material.dispose();
  // GLTFLoader's ImageBitmaps need explicit release. Respect a globally owned
  // Three.js cache or an image still used by another retained model.
  if (!THREE.Cache.enabled) for (const bitmap of images) if (!retainedImages.has(bitmap) && typeof bitmap.close === 'function') bitmap.close();
  instanceCullingCache.delete(root);
  root.clear();
  disposedModelCount++;
}
function trimModelCache() {
  for (const [key, entry] of modelCache) {
    if (key === activeModelKey || (entry.requestedSequence != null && (entry.requestedSequence === activeRunLoad?.token || entry.requestedSequence === prefetchedRun?.token))) continue;
    if (entry.value) disposeModelResources(entry.value.model);
    else entry.controller.abort();
    modelCache.delete(key);
  }
}
async function loadModel(metadata, base, sequence, onProgress = reportLoadProgress) {
  const sceneURL = resolveURL(metadata.scene_url, base);
  const key = sceneURL + '|' + (metadata.scene_sha256 || metadata.model_sha256 || '');
  if (modelCache.has(key)) {
    const entry = modelCache.get(key); entry.requestedSequence = sequence;
    return entry.promise;
  }
  const entry = { value: null, controller: new AbortController(), requestedSequence: sequence };
  entry.promise = (async () => {
    const info = await fetchJSON(resolveURL(metadata.scene_info_url, base), { signal: entry.controller.signal });
    const url = metadata.scene_gzip_url && typeof DecompressionStream !== 'undefined' ? resolveURL(metadata.scene_gzip_url, base) : sceneURL;
    const buffer = await fetchBinary(url, url.endsWith('.gz') ? 'gzip' : null, '농장 3D 장면', info.compressed_bytes || info.bytes, { signal: entry.controller.signal, onProgress });
    onProgress?.('농장과 로봇의 3D 부품 조립 중…', buffer.byteLength);
    const gltf = await new GLTFLoader().parseAsync(buffer, new URL('.', sceneURL).href);
    if (entry.controller.signal.aborted) { disposeModelResources(gltf.scene); throw new DOMException('Superseded scene request', 'AbortError'); }
    // A skin's deformation can exceed its cached reference-pose sphere.
    // Measured plant skins stay visible throughout the recorded movement.
    gltf.scene.traverse(node => { if (node.isSkinnedMesh) node.frustumCulled = false; });
    const dynamicNames = new Set((info.body_nodes || []).map(binding => binding.node));
    for (const name of metadata.transmission?.nodes || []) dynamicNames.add(name);
    batchStaticInstances(gltf.scene, dynamicNames);
    const instanceBatches = prepareInstanceVisibility(gltf.scene, dynamicNames);
    entry.value = { model: gltf.scene, info, instanceBatches, cacheKey: key };
    trimModelCache();
    return entry.value;
  })();
  modelCache.set(key, entry);
  trimModelCache();
  try { return await entry.promise; }
  catch (error) { if (modelCache.get(key) === entry) modelCache.delete(key); throw error; }
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
  const partial = isPartialDiagnostic(), stopped = isStoppedExecution();
  $('recordBadge').hidden = !partial && !stopped;
  $('recordBadge').textContent = partial ? '오류로 중단된 일부 기록 · 수확 실적·재개 상태 아님' :
    stopped ? '실제 물리 실행 중단 · 마지막 저장 상태에서 종료' : '';
  const rows = [
    ['실행', selectedRun.label || selectedRun.id], ['엔진', engineLabel(replay.engine || selectedRun.engine)], ['장면', '기록 당시 모델'],
    ['기간', `${times[0].toFixed(3)}–${times[times.length - 1].toFixed(3)} s`],
    ['상태', `${times.length.toLocaleString()} 프레임 · ${replay.motion.shape[1]}개 바디`],
    ['결과', selectedRun.result || replay.result_summary || replay.status],
    ['모델', replay.scene_sha256 || replay.model_sha256 || sceneInfo.source_scene_sha256 || sceneInfo.source_sha256],
    ['테스트 묶음', currentCollection()?.label],
    ['기록 유형', partial ? '오류 중단 진단 · 저장된 일부 프레임만 표시' : stopped ? '물리 중단 기록 · 완주·수확 완료 아님' : null],
  ].filter(([, value]) => value !== undefined && value !== null && value !== '');
  $('runDetails').replaceChildren(...rows.flatMap(([label, value]) => {
    const term = document.createElement('dt'), desc = document.createElement('dd');
    term.textContent = label; desc.textContent = typeof value === 'object' ? JSON.stringify(value) : String(value);
    return [term, desc];
  }));
  $('runDescription').textContent = selectedRun.description || replay.description || '';
}
function initialReplaySelection(replayCatalog, requestedRun, requestedCollection) {
  const runs = replayCatalog.runs || [], collections = replayCatalog.collections || [];
  const defaultCollection = collections.find(group => group.id === replayCatalog.default_collection);
  const explicitCollection = collections.find(group => group.id === requestedCollection);
  const explicitRun = runs.find(run => run.id === requestedRun);
  const inCollection = (group, id) => group && group.runs.includes(id);
  const containing = id => inCollection(defaultCollection, id) ? defaultCollection : collections.find(group => group.runs.includes(id));
  if (explicitCollection) return {
    collectionId: explicitCollection.id,
    runId: inCollection(explicitCollection, explicitRun?.id) ? explicitRun.id : explicitCollection.runs[0],
  };
  if (explicitRun) return { collectionId: containing(explicitRun.id)?.id || '', runId: explicitRun.id };
  if (defaultCollection) return {
    collectionId: defaultCollection.id,
    runId: inCollection(defaultCollection, replayCatalog.default_run) ? replayCatalog.default_run : defaultCollection.runs[0],
  };
  const defaultRun = runs.find(run => run.id === replayCatalog.default_run);
  if (defaultRun) return { collectionId: containing(defaultRun.id)?.id || '', runId: defaultRun.id };
  return { collectionId: '', runId: runs[0]?.id };
}
function currentCollection() { return (catalog.collections || []).find(item => item.id === selectedCollectionId) || null; }
function independentBranchBoundary(group, targetId) {
  const edges = group?.provenance?.history_graph?.edges;
  return Array.isArray(edges) ? edges.find(edge => edge?.kind === 'software_fault_independent_branch' && edge.target === targetId) || null : null;
}
function showIndependentBranchNotice(branch) {
  const label = catalog.runs.find(run => run.id === branch.target)?.label || branch.target;
  $('branchNotice').textContent = `자동 재생을 멈췄습니다. 다음 기록 “${label}”은 소프트웨어 오류 뒤 정상 측정 상태에서 시작한 독립 재시험입니다. 기록 선택에서 직접 선택해 확인하세요.`;
  $('branchNotice').hidden = false;
}
function collectionContinuityText(group) {
  if (!group) return '';
  const continuity = group.continuity === 'measured_state_handoff' ? '측정 상태를 이어받은 실행 구간입니다.' : group.continuity === 'independent_trials' ? '개별 시험이며, 구간마다 기록된 초기 상태로 전환됩니다.' : '기록 구간 전환 · 연속성은 시험 설명을 참조하세요.';
  const note = typeof group.provenance?.continuity_note === 'string' ? group.provenance.continuity_note.trim() : '';
  return note ? `${continuity} ${note}` : continuity;
}
function collectionSegments(group, runs) {
  let total = 0;
  const segments = (group?.runs || []).map(id => {
    const run = runs.find(item => item.id === id), duration = Number(run?.duration_s);
    if (!run || !Number.isFinite(duration) || duration < 0) throw new Error('기록 구간의 길이가 올바르지 않습니다: ' + id);
    const segment = {id, start: total, end: total + duration, duration};
    total += duration;
    return segment;
  });
  return {segments, total};
}
function collectionPosition(layout, value) {
  if (!Number.isFinite(value) || !layout.segments.length) throw new Error('전체 기록 탐색 시간이 올바르지 않습니다.');
  const time = Math.min(layout.total, Math.max(0, value));
  const segment = layout.segments.find(row => time < row.end) || layout.segments.at(-1);
  return {id: segment.id, localTime: Math.min(segment.duration, Math.max(0, time - segment.start))};
}
function durationLabel(seconds) {
  const tenths = Math.round(seconds * 10);
  return `${Math.floor(tenths / 600)}:${((tenths % 600) / 10).toFixed(1).padStart(4, '0')}`;
}
function updateCollectionTimeline() {
  const group = currentCollection(), layout = collectionSegments(group, catalog.runs);
  const segment = layout.segments.find(row => row.id === selectedRun?.id);
  $('collectionTimelineRow').hidden = !group;
  if (!group) return;
  const time = segment ? segment.start + Math.min(segment.duration, Math.max(0, currentTime - (times?.[0] || 0))) : 0;
  $('collectionTimeline').max = String(layout.total);
  $('collectionTimeline').value = String(time);
  $('collectionTime').textContent = `${durationLabel(time)} / ${durationLabel(layout.total)}`;
  $('collectionTimeline').setAttribute('aria-valuetext', `${durationLabel(time)} / ${durationLabel(layout.total)} · 구간별 기록 합계`);
}
async function seekCollection(value) {
  pause();
  const group = currentCollection();
  const position = collectionPosition(collectionSegments(group, catalog.runs), Number(value));
  if (selectedRun?.id !== position.id) await selectRun(position.id);
  if (!ready || currentCollection()?.id !== group.id || selectedRun?.id !== position.id) return;
  seek(times[0] + position.localTime);
  // Manual seeking may cross an independent trial, but never interpolates
  // state across it or changes the automatic-playback boundary guard.
  if (independentBranchBoundary(group, position.id)) {
    $('branchNotice').textContent = '독립 재시험 구간을 직접 선택했습니다. 이전 구간과 하나의 연속 물리 실행이 아닙니다.';
    $('branchNotice').hidden = false;
  }
  return snapshot();
}
function updateRunPicker() {
  $('branchNotice').hidden = true;
  $('branchNotice').textContent = '';
  const group = currentCollection(), ids = group?.runs;
  const runs = ids ? ids.map(id => catalog.runs.find(item => item.id === id)).filter(Boolean) : catalog.runs;
  $('runSelect').replaceChildren(...runs.map((run, index) => {
    const option = document.createElement('option'); option.value = run.id;
    option.textContent = `${group ? `${index + 1}/${runs.length} · ` : ''}${run.label || run.id}`;
    return option;
  }));
  if (runs.some(run => run.id === selectedRun?.id)) $('runSelect').value = selectedRun.id;
  $('collectionSelect').value = selectedCollectionId;
  $('collectionNextLabel').hidden = !group;
  $('collectionInfo').hidden = !group;
  const links = group ? [
    ...(typeof group.report_url === 'string' ? [{ label: '전체 결과', url: group.report_url }] : []),
    ...(Array.isArray(group.diagnostics_urls) ? group.diagnostics_urls : []),
  ].filter(item => typeof item?.url === 'string') : [];
  $('collectionLinks').replaceChildren(...links.map(item => {
    const link = document.createElement('a'); link.textContent = item.label || '진단 기록';
    link.href = resolveURL(item.url, new URL('./catalog.json', location.href));
    link.target = '_blank'; link.rel = 'noopener'; return link;
  }));
  $('collectionLinks').hidden = !links.length;
  if (group) {
    const {total} = collectionSegments(group, catalog.runs);
    $('collectionInfo').textContent = `${group.label || group.id} · ${runs.length}개 구간 · 기록 합계 ${durationLabel(total)}. 전체 기록 막대로 구간을 이동할 수 있습니다. ${collectionContinuityText(group)}`;
  }
  updateCollectionTimeline();
  return runs;
}
async function selectCollection(id) {
  if (id && !(catalog.collections || []).some(item => item.id === id)) throw new Error('테스트 묶음을 찾지 못했습니다: ' + id);
  selectedCollectionId = id;
  const runs = updateRunPicker();
  if (!runs.length) return;
  if (!runs.some(run => run.id === selectedRun?.id)) return selectRun(runs[0].id);
  details();
  refreshRunPrefetch();
  return snapshot();
}
function failure(error) {
  console.error(error);
  $('loadTitle').textContent = '실행 기록을 불러오지 못했습니다';
  $('loadMessage').textContent = error.message || String(error);
  $('progress').hidden = true; $('retry').hidden = false; $('loading').hidden = false;
  enableControls(false);
}
function cancelRunPreparation(entry) {
  if (!entry) return;
  entry.controller.abort();
  for (const item of entry.sensorImages) {
    item.image.onload = item.image.onerror = null;
    item.image.removeAttribute('src');
  }
  entry.sensorImages.length = 0;
}
function prepareSensorImages(entry, sensor) {
  if (!sensor) return;
  // Warm at most two exposures, in the three selectable PIP modes. All other
  // frames remain demand-loaded; never download an entire camera campaign.
  for (const frame of sensor.frames.slice(0, 2)) for (const mode of ['rgb', 'recognition', 'depth_overlay']) {
    const path = frame[mode];
    if (typeof path !== 'string' || !path) continue;
    const url = resolveURL(path, sensor.framesURL), image = new Image();
    const item = { url, image, loaded: false };
    entry.sensorImages.push(item);
    image.onload = () => { item.loaded = true; };
    image.onerror = () => {};
    image.src = url;
  }
}
function createRunPreparation(run, background = false) {
  const entry = { run, token: Symbol(run.id), controller: new AbortController(), sensorImages: [], value: null, status: 'loading' };
  const checkCurrent = () => {
    if (entry.controller.signal.aborted) throw new DOMException('Superseded execution request', 'AbortError');
  };
  const progress = (...args) => { if (activeRunLoad === entry && !background) reportLoadProgress(...args); };
  entry.promise = (async () => {
    const metadataURL = resolveURL(run.replay_url, new URL('./catalog.json', location.href));
    const metadata = await fetchJSON(metadataURL, { signal: entry.controller.signal });
    checkCurrent();
    if (!metadata.motion?.shape || metadata.motion.shape.length !== 3 || metadata.motion.shape[2] !== 7) throw new Error('위치·회전 기록의 형식을 확인할 수 없습니다.');
    const options = { signal: entry.controller.signal, onProgress: progress };
    // Scene and arrays are independent after metadata. Fetch them concurrently,
    // also for an ordinary manual selection; background work never changes UI.
    const [loaded, motionBuffer, timeBuffer, transmissionBuffer, sensorResult] = await Promise.all([
      loadModel(metadata, metadataURL, entry.token, progress),
      fetchBinary(resolveURL(metadata.motion.url, metadataURL), metadata.motion.compression, '실제 실행 상태', metadata.motion.bytes_compressed, options),
      fetchBinary(resolveURL(metadata.time.url, metadataURL), metadata.time.compression, '실제 시뮬레이션 시간', undefined, options),
      metadata.transmission ? fetchBinary(resolveURL(metadata.transmission.url, metadataURL), metadata.transmission.compression, '그리퍼 연결부 움직임', undefined, options) : null,
      loadSensorCamera(metadata, metadataURL, { signal: entry.controller.signal }).then(value => ({ value }), error => ({ error })),
    ]);
    checkCurrent();
    const frameCount = metadata.motion.shape[0], bodyCount = metadata.motion.shape[1];
    if (motionBuffer.byteLength !== frameCount * bodyCount * 7 * 4) throw new Error('위치·회전 기록의 데이터 크기가 일치하지 않습니다.');
    if (timeBuffer.byteLength !== frameCount * 8) throw new Error('타임스탬프와 상태 프레임 수가 일치하지 않습니다.');
    const nextTimes = new Float64Array(timeBuffer);
    for (let i = 0; i < nextTimes.length; i++) {
      if (!Number.isFinite(nextTimes[i]) || (i && nextTimes[i] <= nextTimes[i - 1])) throw new Error('실제 시뮬레이션 시간이 올바른 순서로 기록되지 않았습니다.');
    }
    const nextTransmission = transmissionBuffer ? new Float32Array(transmissionBuffer) : null;
    if (nextTransmission && nextTransmission.length !== metadata.transmission.shape.reduce((a, b) => a * b, 1)) throw new Error('그리퍼 연결부 기록의 크기가 일치하지 않습니다.');
    // Validate bindings before replacing a visible scene with a prepared one.
    for (const binding of loaded.info.body_nodes || []) {
      if (!loaded.model.getObjectByName(binding.node)) throw new Error('3D 장면에서 실행 바디를 찾지 못했습니다: ' + binding.node);
      if (binding.body_index < 0 || binding.body_index >= bodyCount) throw new Error('실행 바디 인덱스가 기록 범위를 벗어났습니다.');
    }
    for (const name of metadata.transmission?.nodes || []) if (!loaded.model.getObjectByName(name)) throw new Error('그리퍼 연결부를 찾지 못했습니다: ' + name);
    prepareSensorImages(entry, sensorResult.value);
    // Compile shaders without applying future poses to the active scene.
    if (background && loaded.model !== model && renderer?.compileAsync) await renderer.compileAsync(loaded.model, camera, scene);
    checkCurrent();
    entry.value = { metadataURL, metadata, loaded, poses: new Float32Array(motionBuffer), times: nextTimes, transmission: nextTransmission, sensorResult };
    entry.status = 'ready';
    return entry.value;
  })().catch(error => {
    const cancelled = entry.controller.signal.aborted;
    cancelRunPreparation(entry);
    entry.status = cancelled ? 'cancelled' : 'error';
    throw error;
  });
  return entry;
}
function nextPrefetchRun() {
  const group = currentCollection(), ids = group?.runs || [], index = ids.indexOf(selectedRun?.id);
  if (!ready || !$('collectionNext').checked || isPartialDiagnostic() || isStoppedExecution() || index < 0 || index + 1 >= ids.length) return null;
  const id = ids[index + 1];
  if (independentBranchBoundary(group, id)) return null;
  const run = catalog.runs.find(item => item.id === id);
  // A diagnostic can be selected manually, but must not become an implicit
  // continuation merely because it was downloaded ahead of time.
  return run?.record_kind === 'partial_diagnostic' ? null : run || null;
}
function refreshRunPrefetch() {
  const next = nextPrefetchRun();
  if (prefetchedRun?.run.id === next?.id) return;
  const old = prefetchedRun;
  prefetchedRun = null;
  cancelRunPreparation(old);
  trimModelCache();
  if (!next) return;
  const entry = prefetchedRun = createRunPreparation(next, true);
  entry.promise.catch(error => {
    if (prefetchedRun === entry && error.name !== 'AbortError') console.warn('다음 실행 기록 미리 읽기 실패; 선택 시 다시 시도합니다.', next.id, error);
  }).finally(() => { trimModelCache(); });
}
async function selectRun(id, { automatic = false } = {}) {
  const run = catalog.runs.find(item => item.id === id);
  if (!run) throw new Error('선택한 실행 기록이 없습니다: ' + id);
  $('branchNotice').hidden = true;
  $('branchNotice').textContent = '';
  if (currentCollection() && !currentCollection().runs.includes(id)) { selectedCollectionId = ''; updateRunPicker(); }
  const sequence = ++loadSequence;
  const candidate = prefetchedRun?.run.id === id && prefetchedRun.status !== 'error' ? prefetchedRun : null;
  const prepared = candidate?.value;
  const oldLoad = activeRunLoad, oldPrefetch = prefetchedRun;
  activeRunLoad = candidate || createRunPreparation(run);
  prefetchedRun = null;
  if (oldLoad && oldLoad !== activeRunLoad) cancelRunPreparation(oldLoad);
  if (oldPrefetch && oldPrefetch !== activeRunLoad) cancelRunPreparation(oldPrefetch);
  trimModelCache();
  const previousOrigin = [...xyzOrigin];
  pause(); ready = false; enableControls(false);
  sensorCamera = null; sensorFrameIndex = -1; sensorImageKey = ''; sensorImageState = 'empty'; sensorImageRequest++;
  $('cameraMode').disabled = true;
  emptySensorCamera('카메라 기록을 확인하는 중…');
  selectedRun = run;
  $('loading').hidden = Boolean(prepared) || automatic; $('retry').hidden = true; $('progress').hidden = false; $('progress').removeAttribute('value');
  $('loadTitle').textContent = '농장과 실행 기록 불러오는 중';
  $('loadMessage').textContent = run.label || run.id;
  $('runSelect').value = run.id;
  try {
    const request = activeRunLoad;
    const data = prepared || await request.promise;
    if (sequence !== loadSequence) return;
    const { metadata, loaded, sensorResult } = data;
    const bodyCount = metadata.motion.shape[1];
    activeSensorImages = request.sensorImages;
    if (model) scene.remove(model);
    model = loaded.model; sceneInfo = loaded.info; replay = metadata; selectedRun = run;
    activeModelKey = loaded.cacheKey;
    staticInstanceBatches = loaded.instanceBatches;
    retainedStaticInstances = staticInstanceBatches.reduce((sum, batch) => sum + batch.count, 0);
    scene.add(model);
    poses = data.poses; times = data.times;
    sensorCamera = sensorResult.value || null;
    if (sensorResult.error) emptySensorCamera(sensorResult.error.message, 'error');
    else if (!sensorCamera) emptySensorCamera('카메라 기록 없음');
    $('cameraMode').disabled = !sensorCamera;
    transmission = data.transmission;
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
    seek(times[0]); defineBounds(); details();
    if (automatic) {
      // Keep the user's viewpoint through a transition, including exporters
      // whose local display origins differ. Do not interpolate recorded states.
      const delta = new THREE.Vector3(previousOrigin[0] - xyzOrigin[0], previousOrigin[2] - xyzOrigin[2], -(previousOrigin[1] - xyzOrigin[1]));
      camera.position.add(delta); controls.target.add(delta); controls.update(); updateInstanceVisibility();
    } else view('robot');
    enableControls(true); $('loading').hidden = true; $('readout').hidden = false;
    const url = new URL(location.href); url.searchParams.set('run', run.id);
    if (selectedCollectionId) url.searchParams.set('collection', selectedCollectionId); else url.searchParams.delete('collection');
    history.replaceState(null, '', url);
    if (window.parent !== window) window.parent.postMessage({ type: 'farmily-replay:run', run: run.id }, location.origin);
    // Free GPU upload and render before automated readiness checks inspect pixels.
    renderNow();
    refreshRunPrefetch();
    return snapshot();
  } catch (error) {
    if (sequence !== loadSequence) return;
    ready = false; failure(error);
    throw error;
  } finally {
    if (sequence === loadSequence) activeRunLoad = null;
    trimModelCache();
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
    mountedCamera: { available: Boolean(sensorCamera), mode: sensorMode, status: sensorStatus, frame: sensorFrameIndex,
      timestamp_s: sensorCamera?.frames[sensorFrameIndex]?.timestamp_s ?? null,
      frameCount: sensorCamera?.frames.length || 0, sourceMode: sensorCamera?.metadata.mode || null },
    collection: currentCollection() ? { id: selectedCollectionId, label: currentCollection().label, runs: [...currentCollection().runs], index: currentCollection().runs.indexOf(selectedRun?.id), continuity: currentCollection().continuity || 'unknown', autoNext: $('collectionNext').checked } : null,
    renderer: { drawCalls: renderer?.info.render.calls || 0, triangles: renderer?.info.render.triangles || 0, ...stats,
      memory: renderer ? { ...renderer.info.memory } : null,
      retainedStaticInstances, visibleStaticInstances, culledStaticInstances: retainedStaticInstances - visibleStaticInstances },
    modelResources: { cachedModels: [...modelCache.values()].filter(entry => entry.value).length,
      loadingModels: [...modelCache.values()].filter(entry => !entry.value).length, disposedModels: disposedModelCount,
      prefetchedRun: prefetchedRun?.run.id || null, prefetchStatus: prefetchedRun?.status || 'none',
      prefetchedCameraImages: prefetchedRun?.sensorImages.filter(item => item.loaded).length || 0 },
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
window.HARVEST_REPLAY_API = { snapshot, seek, seekCollection, play, pause, selectRun, selectCollection, view, bodyPose };

$('collectionNext').onchange = () => refreshRunPrefetch();
$('cameraMode').onchange = event => { sensorMode = event.target.value; updateSensorCamera(); };
$('cameraToggle').onclick = () => {
  const content = $('cameraContent'); content.hidden = !content.hidden;
  $('cameraToggle').textContent = content.hidden ? '펼치기' : '접기';
  $('cameraToggle').setAttribute('aria-expanded', String(!content.hidden));
};

$('playPause').onclick = () => playing ? pause() : play();
$('restart').onclick = () => { pause(); if (ready) seek(times[0]); };
$('timeline').oninput = event => { pause(); seek(Number(event.target.value)); };
$('collectionTimeline').oninput = event => { pause(); $('collectionTime').textContent = `${durationLabel(Number(event.target.value))} / ${durationLabel(Number(event.target.max))}`; };
$('collectionTimeline').onchange = event => { seekCollection(event.target.value).catch(failure); };
$('speed').onchange = event => { speed = Number(event.target.value); lastWallTime = null; };
$('detailsButton').onclick = () => { $('details').hidden = !$('details').hidden; $('detailsButton').setAttribute('aria-expanded', String(!$('details').hidden)); };
$('runSelect').onchange = event => { selectRun(event.target.value).catch(() => {}); };
$('collectionSelect').onchange = event => { selectCollection(event.target.value).catch(error => failure(error)); };
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
  const knownRuns = new Set(catalog.runs.map(run => run.id));
  catalog.collections = (catalog.collections || []).filter(group => group.id && Array.isArray(group.runs) && group.runs.length && group.runs.every(id => knownRuns.has(id)));
  $('collectionPicker').hidden = !catalog.collections.length;
  $('collectionSelect').replaceChildren(...[{ id: '', label: '전체 기록' }, ...catalog.collections].map(group => {
    const option = document.createElement('option'); option.value = group.id; option.textContent = group.label || group.id; return option;
  }));
  const params = new URLSearchParams(location.search), requestedCollection = params.get('collection'), requested = params.get('run');
  const initialSelection = initialReplaySelection(catalog, requested, requestedCollection);
  selectedCollectionId = initialSelection.collectionId;
  updateRunPicker();
  $('runSelect').disabled = false;
  await selectRun(initialSelection.runId);
} catch (error) { failure(error); }
