#!/usr/bin/env python3
"""Create the V6 receiver addendum; common mechanism guide remains V5.1."""
from pathlib import Path
from reportlab.pdfgen import canvas
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.lib.colors import HexColor
import json

ROOT = Path(__file__).resolve().parents[1]
pdfmetrics.registerFont(TTFont('KR', str(ROOT/'vendor/NotoSansKR-Regular.ttf')))
pdf = ROOT/'docs/inaho_gripper_v6_assembly_guide_KO.pdf'
c = canvas.Canvas(str(pdf), pagesize=(595,842))
c.setTitle('Inaho Gripper V6 · 개방형 경사 받침 조립 안내')
ink = HexColor('#173d33')
def text(x,y,s,size=10):
    c.setFillColor(ink);c.setFont('KR',size);c.drawString(x,y,s)
def page(title, number):
    c.setFillColor(HexColor('#f7f8f5'));c.rect(0,0,595,842,fill=1,stroke=0)
    text(42,799,'FARMILY / INAHO GRIPPER / V6.0',10)
    text(42,760,title,23)
    text(42,28,'사진 참고 시안 · 공통 기구는 V5.1 기준 · 실물 시험 전',9)
    text(533,28,str(number),9)
def lines(y, items):
    for item in items:
        text(42,y,item);y-=21
    return y
page('정면이 열린 경사 U자 받침',1)
lines(724,['정면 막음판을 제거하고, 벌어진 돌출 입구와 뒤쪽으로 낮아지는 경사를 적용했습니다.',
           '벨트·롤러·모터·개폐 링크는 V5.1과 같습니다. 받침은 좌측과 함께 움직입니다.'])
c.drawImage(str(ROOT/'previews/home.png'),42,355,width=511,height=341,preserveAspectRatio=True,anchor='c')
rows=[('구분','V5.1','V6 수정'),('받침 폭','약146 mm','본체122 / 입구138'),('앞뒤 길이','약204 mm','188 mm'),
      ('최저 바닥','약Z-92 mm','약Z-168 mm'),('받침 형상','경사 반원형 받침','돌출·확장 입구'),
      ('고정 방식','좌측 2점 M3','동일 체결점'),('개구부 / 경사','후방 개방 /10도','앞뒤 개방 /12도')]
y=325
for i,row in enumerate(rows):
    c.setFillColor(HexColor('#e5eeea') if i%2==0 else HexColor('#ffffff'))
    c.rect(42,y-10,511,27,fill=1,stroke=0)
    for x,value in zip([52,232,407],row):text(x,y,value,10)
    y-=27
lines(102,['입구가 높고 모터 쪽이 낮은12도 경사는 사진 참고 설계 가정입니다.',
           '치수·경사는 실측값이 아니며 실제 inaho 양산 도면의 복원이 아닙니다.'])
c.showPage()
page('장착·출력 및 확인 항목',2)
lines(720,['1. 기존 좌측 하부 데크의 M3x16 볼트 2개와 구형 받침을 분리합니다.',
 '2. 새 받침의 옆으로 열린 포켓에 기존 M3 너트 2개를 넣습니다.',
 '3. 기둥의 두 구멍을 데크에 맞추고 기존 볼트 2개로 체결합니다.',
 '   체결 중심 (X,Y): (-54,46), (-50,144) mm / 개폐 간격44 기준.',
 '4. 수동으로 개폐하며 호스트 로봇·주변 구조물·작물과의 여유를 확인합니다.'])
text(42,591,'본체 단면 / 입구 변경',14)
# U cross-section diagram, true proportions in a 1.2:1 illustration.
c.saveState();c.translate(183,407);c.scale(1.2,1.2)
c.setStrokeColor(HexColor('#315e58'));c.setLineWidth(3)
p=c.beginPath();p.moveTo(-61,57);p.lineTo(-61,0)
p.curveTo(-61,-33.69,-33.69,-61,0,-61);p.curveTo(33.69,-61,61,-33.69,61,0)
p.lineTo(61,57);c.drawPath(p);c.restoreState()
text(295,497,'외측 폭122 / 벽3 mm',11)
text(295,475,'내측 반경 R58',11)
text(295,453,'정면 막음판 없음 / 앞뒤 개방',11)
text(295,431,'입구28 mm 돌출 · 좌우8 mm 확장',11)
text(295,409,'선단 R18 / 모터 쪽으로12도 하향',11)
text(295,387,'전방 Y202 / 후방 Y14',11)
lines(309,['출력 부품: L_T01_deep_U_receiver.stl / PETG / mm',
 '출력 방향: 앞면 기준, 외접 크기 약138×166×188 mm.',
 '플레어·경사 옆벽·너트 포켓의 서포트와 좁은 접지면의 브림을 검토하세요.',
 '공통 기구 조립은 동봉 common_mechanism_guide_V5.1.pdf를 참고합니다.',
 '그 문서의 구형 받침 관련 내용은 본 V6 변경 안내로 대체합니다.'])
result=json.loads((ROOT/'checks/v6_receiver.json').read_text())
lines(179,[f"CAD: 개폐28~96 mm의35개 위치에서 새 받침 간섭 {len(result['collisions'])}건.",
 '판정 기준: 교집합0.01 mm³ 초과. STEP 솔리드와 STL 폐곡면 확인.',
 '실제 과실 이송·배출·낙하 충격·강도·공차·과실 손상은 미검증입니다.',
 '경사면 이송과 최대 개방 시 과실 받침 범위는 실물에서 확인하세요.'])
c.save()
print(pdf)
