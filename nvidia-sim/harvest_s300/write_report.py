#!/usr/bin/env python3
"""Create a local review page from actual Isaac captures and validation files."""
import argparse
import html
import json
from pathlib import Path

p=argparse.ArgumentParser()
p.add_argument('root',type=Path)
p.add_argument('--assembly',default='assembly_v3')
p.add_argument('--render',default='runtime61')
p.add_argument('--physics',default='physics61_retry')
a=p.parse_args()
manifest=json.loads((a.root/a.assembly/'assembly_manifest.json').read_text())
validation=json.loads((a.root/a.assembly/'geometry_validation.json').read_text())
runtime=a.root/a.physics/'runtime.json'
physics=json.loads(runtime.read_text()) if runtime.exists() else {'status':'미완료 — 로그 확인 필요'}
cards=''.join(f'<figure><a href="{a.render}/{name}.png"><img src="{a.render}/{name}.png"></a><figcaption>{label}</figcaption></figure>'
 for name,label in [('RobotCamera','S300 + 리프트 + RB5 + V5.1 그리퍼'),('FarmCamera','원본 54식물 온실 전체'),('ToolCamera','V5.1 그리퍼 실제 부착 모습')])
page=f'''<!doctype html><html lang="ko"><meta charset="utf-8"><title>S300 수확로봇 · Isaac Sim 6.1</title>
<style>body{{background:#111b20;color:#eaf2f4;font:17px/1.7 system-ui;margin:40px auto;max-width:1200px;padding:0 20px}}a{{color:#86d7c6}}img{{width:100%;border-radius:10px}}figure{{margin:28px 0}}pre{{white-space:pre-wrap;background:#1a2a31;padding:20px;overflow:auto}}.note{{border-left:4px solid #e8bf76;padding:16px;background:#302a20}}</style>
<h1>S300 수확로봇 · Isaac Sim 6.1</h1>
<p>원본 온실 54식물과 S300 V10 외형을 사용했다. 물통·펌프·분무부를 제거한 장착판 위에 리프트·RB5를 올리고, 기존 고리 대신 V5.1 그리퍼를 부착했다.</p>
<p>농장 메시·잎·송이는 원본 그대로이며, MuJoCo light/ultralight 변환본을 사용하지 않았다. 공간 배율은 원본 1.0이다.</p>
<p class="note">현재는 조립·고정 장애물 환경 확인 단계다. 농장 식물은 고정 충돌체이며 탄성/휘어짐은 아직 연결하지 않았다. 새 그리퍼도 44mm 개방 고정 상태다. 수확 성공, 실측 물성, GPU 속도 향상 검증 결과가 아니다.</p>
{cards}<h2>검증</h2>
<p>원본 메시·법선, 입력 파일, 부착 좌표, 기존 관절, 재질 연결 검사: {sum(validation['checks'].values())}/{len(validation['checks'])} 통과.</p>
<p>농장 원본 고유 메시 1,440개 / 고정 충돌 인스턴스 77,614개. 그리퍼 356부품. S300 부품별 제거 이유는 manifest에 기록했다.</p>
<a href="{a.assembly}/assembly_manifest.json">assembly_manifest.json</a> · <a href="{a.assembly}/geometry_validation.json">geometry_validation.json</a>
<h2>짧은 실제 물리 실행</h2><pre>{html.escape(json.dumps(physics,ensure_ascii=False,indent=2))}</pre>
<p><a href="{a.physics}.log">실행 로그</a> · <a href="{a.physics}/contacts.json">접촉 기록</a></p>
<h2>GUI 실행</h2><pre>cd /root/farmily_tomato
DISPLAY=:0 ./nvidia-sim/run_s300_harvest.sh \\
  {html.escape(str(a.root/a.assembly/'scene.usda'))} \\
  --output "/root/docker_share/mujoko_debugging_data/$(date +%Y%m%d_%H%M%S)_s300_view"</pre>
<p>기존 Isaac Sim 5.1, 원본 CAD/USD 및 과거 결과는 보존했다. 로봇 주행, 새 그리퍼의 하중/관성 및 RB5 체결판 강도는 미검증이다.</p></html>'''
(a.root/'index.html').write_text(page,encoding='utf-8')
print(a.root/'index.html')
