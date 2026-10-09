from pathlib import Path
import base64
ROOT=Path(__file__).resolve().parents[1]
s=(ROOT/'source/viewer_template.html').read_text()
font=ROOT/'vendor/NotoSansKR-Regular.woff'
fontcss="@font-face{font-family:'Noto Sans KR';src:url(data:font/woff;base64,"+base64.b64encode(font.read_bytes()).decode()+") format('woff');font-weight:100 900;font-display:swap;}"
data={'/*__FONT__*/':fontcss,'/*__VENDOR__*/':(ROOT/'vendor/three-r140-with-loaders.js').read_text(),'/*__PARAMS__*/':(ROOT/'parameters.json').read_text(),'/*__MANIFEST__*/':(ROOT/'parts_manifest.json').read_text(),'/*__MODEL__*/':base64.b64encode((ROOT/'models/inaho_gripper_v5_assembly.glb').read_bytes()).decode(),'/*__APP__*/':(ROOT/'source/app.js').read_text(),'/*__FALLBACK__*/':(ROOT/'source/canvas_fallback.js').read_text()}
data['/*__VENDOR__*/']+='\n'+(ROOT/'vendor/OrbitControls.js').read_text()
for k,v in data.items():s=s.replace(k,v)
temporary=ROOT/'inaho_gripper_v5_viewer.tmp.html'
temporary.write_text(s)
temporary.replace(ROOT/'inaho_gripper_v5_viewer.html')
print('Viewer bytes',len(s.encode()))
