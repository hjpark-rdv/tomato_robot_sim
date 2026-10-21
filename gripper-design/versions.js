const designs = {
  v5: {revision: 'V5.1', viewer: 'inaho_gripper/inaho_gripper_v5_viewer.html',
    step: 'inaho_gripper/inaho_gripper_v5_assembly.step',
    glb: 'inaho_gripper/inaho_gripper_v5_assembly.glb',
    print: 'inaho_gripper/inaho_gripper_v5_printable.zip',
    guide: 'inaho_gripper/inaho_gripper_v5_assembly_guide_KO.pdf'},
  v6: {revision: 'V6.0', viewer: 'inaho_gripper_v6/inaho_gripper_v6_viewer.html',
    step: 'inaho_gripper_v6/models/inaho_gripper_v6_assembly.step',
    glb: 'inaho_gripper_v6/models/inaho_gripper_v6_assembly.glb',
    print: 'inaho_gripper_v6/inaho_gripper_v6_printable.zip',
    guide: 'inaho_gripper_v6/docs/inaho_gripper_v6_assembly_guide_KO.pdf'}
};
const selector = document.getElementById('designVersion');
function showVersion(key) {
  if (!Object.hasOwn(designs, key)) key = 'v6';
  const design = designs[key];
  selector.value = key;
  document.getElementById('revision').textContent = design.revision;
  document.title = `Inaho Gripper ${design.revision} · Farmily`;
  document.getElementById('pageHeading').textContent = 'Inaho Gripper 디자인';
  const viewer = document.getElementById('gripperViewer');
  viewer.title = `듀얼 벨트 그리퍼 ${design.revision} 3D 설계 뷰어`;
  // Keep one history entry per version; iframe navigation must not add its
  // own entry between the parent page's V5/V6 entries.
  if (viewer.dataset.design !== key) {
    if (!viewer.getAttribute('src')) viewer.src = './' + design.viewer;
    else viewer.contentWindow.location.replace(new URL(design.viewer, location.href).href);
    viewer.dataset.design = key;
  }
  for (const [id, field] of [['downloadStep', 'step'], ['downloadGlb', 'glb'], ['downloadPrint', 'print'], ['downloadGuide', 'guide']]) {
    document.getElementById(id).href = './' + design[field];
  }
}
selector.addEventListener('change', () => {
  const url = new URL(location.href);
  url.searchParams.set('version', selector.value);
  history.pushState(null, '', url);
  fromURL();
});
function showTab(tab, version) {
  if (!['robot', 'replay'].includes(tab)) tab = 'gripper';
  const robot = tab === 'robot';
  const replay = tab === 'replay';
  if (!replay) document.getElementById('replayViewer').contentWindow?.postMessage({ type: 'farmily-replay:pause' }, location.origin);
  for (const name of ['gripper', 'robot', 'replay']) document.getElementById(name + 'Panel').hidden = tab !== name;
  document.querySelector('.version-picker').hidden = tab !== 'gripper';
  document.querySelector('nav[aria-label="설계 파일 다운로드"]').hidden = replay;
  for (const button of document.querySelectorAll('[data-tab]')) {
    const active = button.dataset.tab === tab;
    button.setAttribute('aria-selected', String(active));
    button.tabIndex = active ? 0 : -1;
  }
  for (const id of ['downloadStep', 'downloadPrint', 'downloadGuide']) document.getElementById(id).hidden = robot;
  if (replay) {
    document.title = '농장 수확 재생 · Farmily';
    document.getElementById('pageHeading').textContent = 'Farmily 농장 수확 재생';
    document.getElementById('revision').textContent = '저장된 실제 실행';
    const viewer = document.getElementById('replayViewer');
    if (!viewer.getAttribute('src')) {
      const url = new URL('./harvest-replay/', location.href);
      const run = new URLSearchParams(location.search).get('run');
      if (run) url.searchParams.set('run', run);
      viewer.src = url.href;
    } else {
      const run = new URLSearchParams(location.search).get('run');
      if (run) viewer.contentWindow?.postMessage({ type: 'farmily-replay:select', run }, location.origin);
    }
  } else if (robot) {
    document.title = '수확로봇 조립 · Farmily';
    document.getElementById('pageHeading').textContent = 'Farmily 수확로봇';
    document.getElementById('revision').textContent = 'S300 · RB5';
    document.getElementById('downloadGlb').href = './robot-assembly/models/s300_lift_rb5_gripper.glb';
    const viewer = document.getElementById('robotViewer');
    if (!viewer.getAttribute('src')) viewer.src = './robot-assembly/';
  } else {
    showVersion(version);
  }
}
const fromURL = () => {
  const params = new URLSearchParams(location.search);
  showTab(params.get('tab'), params.get('version'));
};
const tabs = Array.from(document.querySelectorAll('[data-tab]'));
for (const [i, button] of tabs.entries()) {
  button.addEventListener('click', () => {
    if (button.getAttribute('aria-selected') === 'true') return;
    const url = new URL(location.href);
    if (button.dataset.tab !== 'gripper') url.searchParams.set('tab', button.dataset.tab);
    else url.searchParams.delete('tab');
    history.pushState(null, '', url);
    fromURL();
  });
  button.addEventListener('keydown', event => {
    let next;
    if (event.key === 'ArrowRight') next = (i + 1) % tabs.length;
    else if (event.key === 'ArrowLeft') next = (i + tabs.length - 1) % tabs.length;
    else if (event.key === 'Home') next = 0;
    else if (event.key === 'End') next = tabs.length - 1;
    else return;
    event.preventDefault();
    tabs[next].focus();
    tabs[next].click();
  });
}
window.addEventListener('popstate', fromURL);
window.addEventListener('message', event => {
  const viewer = document.getElementById('replayViewer');
  if (event.origin !== location.origin || event.source !== viewer.contentWindow || event.data?.type !== 'farmily-replay:run' || typeof event.data.run !== 'string') return;
  const url = new URL(location.href);
  if (url.searchParams.get('tab') !== 'replay') return;
  url.searchParams.set('run', event.data.run);
  history.replaceState(null, '', url);
});
fromURL();
