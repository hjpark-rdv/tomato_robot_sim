import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';

const viewport = document.getElementById('viewport');
const status = document.getElementById('loadMessage');
let renderer, model, controls, bounds, camera;
let currentView = 'all';
const scene = new THREE.Scene();
scene.background = new THREE.Color('#edf1f3');
const components = {};
let queued = false;
function render() {
  if (queued || !renderer) return;
  queued = true;
  requestAnimationFrame(() => {
    queued = false;
    renderer.render(scene, camera);
  });
}
function fit(box, direction, padding = 1.22) {
  if (!box || box.isEmpty()) return;
  const center = box.getCenter(new THREE.Vector3());
  const radius = box.getSize(new THREE.Vector3()).length() / 2;
  const angle = Math.min(THREE.MathUtils.degToRad(camera.fov / 2), Math.atan(Math.tan(THREE.MathUtils.degToRad(camera.fov / 2)) * camera.aspect));
  const distance = radius / Math.sin(angle) * padding;
  camera.position.copy(center).add(new THREE.Vector3(...direction).normalize().multiplyScalar(distance));
  camera.near = .002;
  camera.far = Math.max(100, distance * 10);
  camera.updateProjectionMatrix();
  controls.target.copy(center);
  controls.minDistance = .08;
  controls.maxDistance = 20;
  controls.update();
  render();
}
function view(name) {
  if (!model) return;
  currentView = name;
  camera.up.set(0, 1, 0);
  const directions = { all: [-2.5, 1.3, 3], front: [0, .08, 1], back: [0, .08, -1], side: [-1, .08, 0], top: [0, 1, .001], gripper: [-2, 1.2, 1] };
  if (name === 'gripper') {
    components.gripper.visible = true;
    document.querySelector('[data-component="gripper"]').checked = true;
    fit(new THREE.Box3().setFromObject(components.gripper), directions.gripper, 1.5);
  } else {
    fit(bounds, directions[name] || directions.all);
  }
}
function failure(error) {
  console.error(error);
  status.textContent = '모델을 불러오지 못했습니다. 연결을 확인하고 다시 시도해 주세요.';
  document.getElementById('progress').hidden = true;
  document.getElementById('loading').hidden = false;
  document.getElementById('retry').hidden = false;
}
document.getElementById('retry').onclick = () => location.reload();

try {
  renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 1.75));
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.1;
  viewport.appendChild(renderer.domElement);
  renderer.domElement.setAttribute('aria-label', '회전과 확대가 가능한 수확로봇 모델');
  camera = new THREE.PerspectiveCamera(36, 1, .002, 100);
  controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = false;
  controls.addEventListener('change', render);
  scene.add(new THREE.HemisphereLight(0xffffff, 0x84988e, 2.0));
  for (const [position, intensity] of [[[3, 6, 4], 2.4], [[-4, 3, -2], 1.7], [[1, 2, -4], 1.0]]) {
    const light = new THREE.DirectionalLight(0xffffff, intensity);
    light.position.set(...position);
    scene.add(light);
  }
  function resize() {
    const { width, height } = viewport.getBoundingClientRect();
    if (!width || !height) return;
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    renderer.setSize(width, height);
    if (model && bounds) view(currentView);
    render();
  }
  new ResizeObserver(resize).observe(viewport);
  resize();
  const response = await fetch('./models/model.json', { cache: 'no-cache' });
  if (!response.ok) throw new Error(`Model metadata: HTTP ${response.status}`);
  const info = await response.json();
  const loader = new GLTFLoader();
  const progress = (loaded, total) => {
    const pct = Math.min(100, loaded / total * 100);
    document.getElementById('progress').value = pct;
    status.textContent = `${(loaded / 1048576).toFixed(1)} / ${(total / 1048576).toFixed(1)} MB · ${Math.round(pct)}%`;
  };
  let gltf;
  if (info.compressed_file && typeof DecompressionStream !== 'undefined') {
    const response = await fetch('./models/' + info.compressed_file + '?v=' + info.sha256.slice(0, 12));
    if (!response.ok) throw new Error(`Model download: HTTP ${response.status}`);
    const reader = response.body.getReader();
    const chunks = [];
    let received = 0;
    while (true) {
      const { value, done } = await reader.read();
      if (done) break;
      chunks.push(value);
      received += value.byteLength;
      progress(received, info.compressed_bytes);
    }
    status.textContent = '부품 조립 중…';
    const stream = new Blob(chunks).stream().pipeThrough(new DecompressionStream('gzip'));
    gltf = await loader.parseAsync(await new Response(stream).arrayBuffer(), new URL('./models/', location.href).href);
  } else {
    gltf = await loader.loadAsync('./models/' + info.file + '?v=' + info.sha256.slice(0, 12), event => progress(event.loaded, event.total || info.bytes));
  }
  model = gltf.scene;
  scene.add(model);
  for (const name of Object.keys(info.components)) {
    components[name] = model.getObjectByName(name);
    if (!components[name]) throw new Error('Missing component: ' + name);
  }
  bounds = new THREE.Box3().setFromObject(model);
  let liftHeight = info.lift?.reference_height_m ?? 0;
  let demoFrame = null;
  function setLift(height) {
    if (!info.lift || !Number.isFinite(Number(height))) return;
    liftHeight = THREE.MathUtils.clamp(Number(height), ...info.lift.range_m);
    for (const name of info.lift.moving_nodes) components[name].position.y = liftHeight - info.lift.reference_height_m;
    model.updateMatrixWorld(true);
    bounds = new THREE.Box3().setFromObject(model);
    document.getElementById('liftHeight').value = String(Math.round(liftHeight*1000));
    document.getElementById('liftValue').textContent = `리프트 상승량 ${Math.round(liftHeight*1000)} mm`;
    const dimensions = bounds.getSize(new THREE.Vector3());
    document.getElementById('dimensions').textContent = `현재 자세 높이 ${dimensions.y.toFixed(2)} m · 폭 ${dimensions.x.toFixed(2)} m`;
    render();
  }
  function stopDemo() {
    if (demoFrame !== null) cancelAnimationFrame(demoFrame);
    demoFrame = null;
    document.getElementById('liftDemo').textContent = '승강 보기';
  }
  if (info.lift) {
    document.getElementById('liftControl').hidden = false;
    const slider = document.getElementById('liftHeight');
    slider.min = String(info.lift.range_m[0]*1000); slider.max = String(info.lift.range_m[1]*1000);
    slider.oninput = () => { stopDemo(); setLift(Number(slider.value)/1000); };
    document.getElementById('liftReset').onclick = () => { stopDemo(); setLift(info.lift.reference_height_m); };
    document.getElementById('liftDemo').onclick = () => {
      if (demoFrame !== null) { stopDemo(); return; }
      setLift(info.lift.range_m[0]); const fullTravel = bounds.clone();
      setLift(info.lift.range_m[1]); fullTravel.union(bounds);
      fit(fullTravel, [-2.5,1.3,3]);
      const start = performance.now();
      document.getElementById('liftDemo').textContent = '정지';
      const animate = now => {
        const u = Math.min(1,(now-start)/10000);
        const sweep = .5-.5*Math.cos(2*Math.PI*u);
        setLift(info.lift.range_m[0] + sweep*(info.lift.range_m[1]-info.lift.range_m[0]));
        if (u < 1) demoFrame = requestAnimationFrame(animate); else stopDemo();
      };
      demoFrame = requestAnimationFrame(animate);
    };
    document.addEventListener('visibilitychange', () => { if(document.hidden) stopDemo(); });
    setLift(liftHeight);
  }
  const size = bounds.getSize(new THREE.Vector3());
  const floor = new THREE.Mesh(new THREE.PlaneGeometry(200, 200), new THREE.MeshStandardMaterial({ color: 0xe1e7e9, roughness: 1 }));
  floor.rotation.x = -Math.PI / 2;
  floor.position.y = bounds.min.y - .005;
  scene.add(floor);
  const grid = new THREE.GridHelper(6, 30, 0xb9c8c5, 0xd0dbd8);
  grid.position.y = bounds.min.y - .004;
  grid.material.transparent = true;
  grid.material.opacity = .5;
  scene.add(grid);
  document.getElementById('dimensions').textContent = `현재 자세 높이 ${size.y.toFixed(2)} m · 폭 ${size.x.toFixed(2)} m`;
  document.querySelectorAll('[data-view]').forEach(button => {
    button.disabled = false;
    button.onclick = () => view(button.dataset.view);
  });
  document.querySelectorAll('[data-component]').forEach(input => {
    input.disabled = false;
    input.onchange = () => { components[input.dataset.component].visible = input.checked; render(); };
  });
  view('all');
  document.getElementById('loading').hidden = true;
  window.ROBOT_ASSEMBLY_API = {
    view,
    setLift: height => { stopDemo(); setLift(height); },
    snapshot: () => ({ ready: true, source: info.source_sha256, model: info.sha256,
      components: Object.fromEntries(Object.entries(components).map(([key, value]) => [key, value.visible])),
      triangles: info.triangles, sourceMeshes: info.visual_source_meshes,
      camera: camera.position.toArray(), target: controls.target.toArray(),
      liftHeight_m: liftHeight,
      componentBounds: Object.fromEntries(Object.entries(components).map(([key,value]) => { const b=new THREE.Box3().setFromObject(value); return [key,[b.min.toArray(),b.max.toArray()]]; })),
      renderer: 'WebGL', bounds: [bounds.min.toArray(), bounds.max.toArray()] })
  };
} catch (error) { failure(error); }
