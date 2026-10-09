#!/usr/bin/env python3
"""Korean assembly guide. All verification figures are loaded from check reports."""
from pathlib import Path
from collections import Counter
from math import pi
import csv,json,html
from reportlab.pdfgen import canvas
from reportlab.platypus import SimpleDocTemplate,Paragraph,Spacer,Table,TableStyle,PageBreak,Image,KeepTogether
from reportlab.lib import colors
from reportlab.lib.styles import getSampleStyleSheet,ParagraphStyle
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.lib.enums import TA_LEFT
ROOT=Path(__file__).resolve().parents[1]
FONT=ROOT/'vendor/NotoSansKR-Regular.ttf'
pdfmetrics.registerFont(TTFont('KR',str(FONT)))
INK=colors.HexColor('#163b48');TEAL=colors.HexColor('#17788a');MUTED=colors.HexColor('#627d89');PALE=colors.HexColor('#eaf1f4');GOLD=colors.HexColor('#e3a04e')
styles=getSampleStyleSheet()
styles.add(ParagraphStyle('TitleKR',fontName='KR',fontSize=25,leading=36,textColor=INK,spaceAfter=13))
styles.add(ParagraphStyle('HeadKR',fontName='KR',fontSize=19,leading=28,textColor=INK,spaceAfter=12,keepWithNext=True))
styles.add(ParagraphStyle('SubKR',fontName='KR',fontSize=12,leading=19,textColor=TEAL,spaceBefore=10,spaceAfter=6,keepWithNext=True))
styles.add(ParagraphStyle('BodyKR',fontName='KR',fontSize=9.2,leading=15.8,textColor=INK,spaceAfter=7,wordWrap='CJK'))
styles.add(ParagraphStyle('SmallKR',fontName='KR',fontSize=7.7,leading=12.1,textColor=MUTED,spaceAfter=5,wordWrap='CJK'))
styles.add(ParagraphStyle('CellKR',fontName='KR',fontSize=8.0,leading=12.5,textColor=INK,wordWrap='CJK'))
styles.add(ParagraphStyle('CellSmall',fontName='KR',fontSize=6.9,leading=10.5,textColor=INK,wordWrap='CJK'))
story=[]
def para(s,style='BodyKR'):return Paragraph(s,styles[style])
def p(s,style='BodyKR'):story.append(para(s,style))
def title(n,s):p(f'{n:02d} / V5 ASSEMBLY','SmallKR');p(s,'HeadKR')
def sub(s):p(s,'SubKR')
def table(rows,widths,small=False):
    cell='CellSmall' if small else 'CellKR'
    data=[[para(html.escape(str(x)).replace('\n','<br/>'),cell) for x in row] for row in rows]
    t=Table(data,colWidths=widths,repeatRows=1,hAlign='LEFT')
    t.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),PALE),('LINEBELOW',(0,0),(-1,0),1,TEAL),('VALIGN',(0,0),(-1,-1),'TOP'),('LEFTPADDING',(0,0),(-1,-1),7),('RIGHTPADDING',(0,0),(-1,-1),7),('TOPPADDING',(0,0),(-1,-1),6),('BOTTOMPADDING',(0,0),(-1,-1),6),('LINEBELOW',(0,1),(-1,-1),.3,colors.HexColor('#d7e1e5'))]))
    story.append(t);story.append(Spacer(1,7))
def note(s):
    t=Table([[para(s,'SmallKR')]],colWidths=[503]);t.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,-1),PALE),('BOX',(0,0),(-1,-1),.5,colors.HexColor('#cbdae0')),('LEFTPADDING',(0,0),(-1,-1),11),('RIGHTPADDING',(0,0),(-1,-1),11),('TOPPADDING',(0,0),(-1,-1),9),('BOTTOMPADDING',(0,0),(-1,-1),8)]));story.append(t);story.append(Spacer(1,8))
def page():story.append(PageBreak())
def shot(name,w=503,h=None,crop=None):
    from PIL import Image as PILImage
    src=ROOT/'previews'/name
    if src.exists():
        im=PILImage.open(src);
        if crop:
            import io
            im=im.crop(crop);buffer=io.BytesIO();im.save(buffer,format='PNG');buffer.seek(0);src=buffer
        scale=min(w/im.width,h/im.height) if h else w/im.width;story.append(Image(str(src) if isinstance(src,Path) else src,im.width*scale,im.height*scale));story.append(Spacer(1,9))
def header(c,d):
    c.setFillColor(INK);c.rect(0,806,595,36,fill=1,stroke=0);c.setFillColor(colors.white);c.setFont('KR',8);c.drawString(46,819,'FARMILY / DUAL-BELT GRIPPER V5.1');c.setFillColor(MUTED);c.setFont('KR',7);c.drawString(46,24,'단위 mm | 원형 배치 반영 / 기구 조립용 | 실물 시험 전');c.drawRightString(549,24,f'{d.page}');

manifest=json.loads((ROOT/'parts_manifest.json').read_text());P=json.loads((ROOT/'parameters.json').read_text())
checks={n:json.loads((ROOT/'checks'/f'{n}.json').read_text()) for n in ['interference_full','interfaces_and_tension','assembly_paths','export_checks','double_check','browser_checks']}
np=sum(r['kind']=='print' for r in manifest);ni=len(manifest); ex=checks['export_checks']; inter=checks['interference_full'];audit=checks['double_check']
p('inaho 형태의<br/>듀얼 벨트 그리퍼 V5.1','TitleKR')
p('같은 높이의 본체 · 비대칭 벨트 끝단 · 독립 커버 체결','SubKR')
shot('00_model.png',503,h=365)
table([['설계 상태','전달 기준'],['본체 높이 차이','좌우 0 mm. 상·하 데크와 외측 커버 기준면 일치'],['벨트 끝단','L Z9~61 / R Z13~65. 폭52, 끝단 차이4'],['구성',f'{ni} CAD 인스턴스 / 출력 부품 {np}개 + 끼워맞춤 쿠폰5개'],['출력물',f'STL {len(ex["stl"])}개 / STEP {len(ex["step"])}개 / GLB / 오프라인 Three.js']], [120,383])
note('4 mm 비대칭은 V5에서 정한 설계값이며 inaho 원본 실측값이 아닙니다. 제작 목적은 조립 가능한 기구 시제품입니다. 아래 CAD 검증을 통과했지만 실제 출력·조립·파지력·수명 시험을 대신하지 않습니다.')
page();title(1,'높이 기준과 V4 변경점')
p('좌우는 후방에서 전방(+Y)을 바라본 기준입니다. X는 개폐 방향, Z는 높이입니다. 앞뒤 위치 차이 14 mm는 유지하고 좌측 카세트 전체의 Z=-8 mm 이동은 제거했습니다.')
table([['항목','좌측 L','우측 R'],['하부 데크','Z -2~6','Z -2~6'],['상부 데크','Z 69~77','Z 69~77'],['외측 핑거 커버','Z -4~79','Z -4~79'],['후방 커버 천장','Z 102~104','Z 102~104'],['접촉 벨트','Z 9~61','Z 13~65'],['롤러 하부 플랜지','Z 6~8','Z 10~12'],['롤러 접촉 드럼','Z 8~62','Z 12~66'],['롤러 상부 플랜지','Z 62~64','Z 66~68'],['벨트 높이 겹침','Z 13~61, 48 mm','같음']], [167,168,168])
sub('함께 바뀐 연결 부품')
p('벨트 폭을56에서52 mm로 줄이고 루트·노즈 롤러와 좌우 축 스페이서를 각각 수정했습니다. 본체 공간 안에서 양쪽 벨트에 플랜지 여유1 mm를 두었습니다. 좌측 경사 받침도 같은 카세트 기준면에 연결되므로 V4보다8 mm 위로 이동합니다.')
p('20T 풀리는 실제 고정 나사가 있는 RobotDigg 제품으로 교체했습니다. 풀리 밑면은 서로 달라도 벨트 홈 중심면은 Z86.75 mm로 맞춥니다. 상부 베어링 캡은 낮은 버튼헤드 볼트를 사용합니다.')
p('C14 커버는 전용 M3x12 세 개로만 고정합니다. 커버를 벗겨도 C11의 구조 M4 두 개와 모터 슬라이드 M3 네 개는 남습니다. F08 원점 스위치 브래킷, 우측 새들 작동 돌기, 모터선 타이 고정부를 추가했습니다.')
note('V5.1: 노즈 축 고정 네 개를 ISO4026 M3x6 평끝 무두나사로 바꾸고 작은 와셔 네 개를 제거했습니다. 머리 착좌로 조임이 멈추던 조건을 없애 포켓 M3 너트에서 축 접촉 위치를 조정합니다. 상부 나사는 상부 암 장착 후 설치합니다. 텐셔너 M3x25 두 개는 잭 잠금너트까지 나사산이 이어지는 전나사산 제품을 사용합니다. 체결 부품 분해 위치도 정정했습니다.')
page();title(2,'출력·공차·구매 준비')
table([['항목','권장 시작 조건 / 확인'],['강성 출력물','PETG. 노즐0.4 / 층0.2 / 외벽6 / 채움50%. 링크·클램프·새들80~100%'],['C07 접촉 벨트','TPU95A, 폭52 / 두께2. 연속 루프. free C143.2 형상이 STL에 반영됨'],['최대 출력 크기','STL 외접 치수는 220x220x220 mm 안에 있음. 제공 방향 유지. 브림·서포트·장비 금지 영역은 슬라이서에서 확인'],['쿠폰','625:16.00~16.30 / 608:22.00~22.30 / 축:5.00~5.30 / 핀:6.00~6.30 / M3·M4·M5 너트 포켓'],['핏 선택','동일 재료·슬라이서로 쿠폰을 먼저 출력. 축은 부드러운 통과, 베어링 외륜은 손상 없이 장착, 너트는 회전하지 않는 치수 선택'],['후가공','서포트와 코끼리발 제거. 구멍만 규격 리머로 정리. 금속 샤프트 절단면 모따기·버 제거']], [121,382])
p('표준 CAD의 베어링 좌석은 직경 공차 여유0.15 mm입니다. 프린터 수축·재료·방향에 따라 실제 끼워맞춤이 바뀝니다. STL 전체를 임의 확대하면 모터·레일·볼트 피치도 변하므로 구멍 보정이나 소스 치수 수정으로 대응합니다.')
sub('필수 구매품')
table([['규격','수량'],['MGN12R L195, 양 끝 구멍10 / 피치25 + MGN12H','레일1 + 블록2'],['625-2Z 5x16x5 / 608-2Z 8x22x7','8 / 2'],['Ø5h6 L105 / Ø5h6 L90 / Ø8h6 L76 샤프트','2 / 2 / 1'],['11HS12-0674S / Pololu2267 SY42STH38-1684A','2 / 1'],['RobotDigg GT2-20T-5B-6 / MISUMI GPA32GT2060-A-P5','2 / 2'],['220-2GT 폭6, 닫힌 루프110치','2'],['GN705-5-E / GN705-8-E / ST-FC04 5x8','2 / 2 / 1'],['ISO7379-6-M5-12 / D2F-01F','4 / 1']], [400,103],True)
p('전체 볼트·너트 수량은 PURCHASE_BOM.csv, 출력 목록은 PRINT_LIST.csv를 사용합니다. 판매처에서 구하기 어려운 14/18/22/28 mm 나사는 완성 길이로 주문하거나 긴 볼트를 절단·모따기합니다. 막힌 구멍에는 더 긴 나사를 그대로 넣지 않습니다. 타이 폭2.5 x100 mm 4개와 모터/스위치 배선은 소모품 목록에 별도로 포함했습니다. 모터 내부 축·스위치 작동자는 별도 주문하지 않습니다.','SmallKR')
page();title(3,'프레임과 개폐 구동부 조립')
p('1. F01 뒤쪽 포켓에 레일용 M3 너트를 넣고 레일 F02를 M3x14 여덟 개로 고정합니다. 레일 양 끝10 mm와 구멍 피치25 mm를 확인합니다. 블록이 레일에서 빠지지 않게 유지한 상태로 장착합니다.')
p('2. F04 타워에 608 두 개를 넣고 캡 F05, 1 mm 내부 스페이서 F06, Ø8 L76 축, GN705-8 칼라를 조립합니다. 축이 자유롭게 돌고 축 방향 유격이 과하지 않도록 칼라를 조정합니다. 베어링 외륜을 잡은 채 내륜을 강제로 압축하지 않습니다.')
p('3. F07 요크와 NEMA17 모터를 설치합니다. 모터 나사는 M3x10 +0.5 mm 와셔, 모터 진입3.5 mm / 도면 깊이4.5 mm입니다. 5/8 커플링 ST-FC04에 모터와 별도 지지축을 연결합니다.')
p('4. V5.1의 C01 새들을 MGN12H에 M3x8 네 개씩 고정합니다. 두 새들은 힌지 너트 포켓이 수정되었으므로 V5.0 출력물은 V5.1 STL로 다시 출력합니다. 새들 패드5 mm + 블록 나사 진입3 mm, 블록 깊이3.5 mm입니다. 더 긴 나사를 사용하지 않습니다.')
p('5. K01 크랭크와 K02 링크를 연결합니다. 크랭크 반경25, 링크 중심거리45 mm입니다. ISO7379 숄더 Ø6x12 위에 와셔1 + 부시10 + 와셔1 mm가 놓입니다. 부시는 고정되고 링크가 부시 외경에서 회전하며 명목 축 방향 여유는2 mm입니다. 새들측 M5 너트는 수정 포켓을 사용하여 숄더 뒤 최대2 mm의 민축 구간을 피합니다. 출력한 스택을 실측하고 너트 체결 후에도 링크가 자유롭게 회전하는지 확인합니다.')
p('6. 개폐 스토퍼와 A01 뒤쪽 어댑터를 조립합니다. 스토퍼는 접촉면 간격28 /96 mm용이며 충돌 에너지를 견디는 정격 부품은 아닙니다. 어댑터는 PCD50에4xØ6.6, 중앙Ø26의 사용자 정의 플랜지입니다. 상대 로봇 플레이트와 체결 나사는 실제 호스트에 맞춰 별도 선정합니다.')
shot('07_front_height.png',503,h=220,crop=(620,330,1130,720))
page();title(4,'카세트 조립: 루트 롤러와 벨트')
p('카세트는 작업대에서 먼저 조립한 뒤 새들에 체결합니다. 좌우 부품을 섞지 않도록 L/R 파일 이름을 유지합니다. 두 벨트의 STL 크기는 같아도 롤러와 스페이서의 장착 높이는 다릅니다.')
p('1. C02 하부 데크에 하부625, C05 캡과 너트를 설치합니다. C03 벨트 백킹의 하부 나사와 너트를 체결합니다. 측면 너트 창은 접근 가능하게 둡니다.')
p('2. Ø5 L105 루트 축과 하부 칼라, C04 스페이서를 넣습니다. C06 드럼 두 반쪽을 축에 끼우고 M3x20 네 개로 교대로 조입니다. 드럼 고정은 벨트로 공구 접근이 가려지기 전에 완료합니다.')
p('3. 아직 노즈 롤러를 고정하지 않은 상태에서 TPU 루프를 루트 드럼에 걸어 둡니다. 노즈 롤러도 벨트 안쪽에 넣은 다음 축·캐리어를 조립합니다. 상부 데크와 노즈 상부 암은 이 단계에서 열려 있어야 합니다. 플랜지를 억지로 벌리거나 벨트 한쪽 끝을 잘라 연결하지 않습니다.')
table([['루트 축 스택','좌측 L','우측 R'],['칼라 / 하부 외측 스페이서','-8~-2 / -2~0','동일'],['하부625','0~5','0~5'],['하부 내측 스페이서','5~6, t1','5~10, t5'],['드럼+플랜지','6~64','10~68'],['상부 내측 스페이서','64~70, t6','68~70, t2'],['상부625','70~75','70~75'],['상부 외측 스페이서','75~81.4, t6.4','동일'],['32T 풀리','81.6~91.9','동일']], [193,155,155])
note('루트 상부 스페이서와 풀리 사이0.2 mm를 축 방향 여유로 남깁니다. 조립 후 드럼이 축에 고정되어도 베어링은 자유 회전해야 합니다. 스페이서와 외륜을 함께 눌러 고정하지 않습니다.')
page();title(5,'노즈 롤러·텐셔너·상부 데크')
table([['노즈 축 스택','좌측 L','우측 R'],['Ø5 L90 축 / 하부 캐리어','-8~82 / -8~-2','동일'],['하부 스페이서','-2~8, t10','-2~12, t14'],['하부625','8~13','12~17'],['중앙 스페이서','13~57, t44','17~61, t44'],['상부625','57~62','61~66'],['상부 스페이서','62~76.85, t14.85','66~76.85, t10.85'],['상부 암','77~83','77~83'],['상부 끝 여유','76.85~77, 0.15','동일']], [193,155,155])
p('4. 베어링 두 개와 중앙 스페이서를 C08 롤러에 넣고, 외측 스페이서와 함께 C10 하부 캐리어에 올립니다. Ø5 L90 축은 탭 없는 평축입니다. 하부 Z=-5 포켓에 M3 너트를 넣고 ISO4026 M3x6 평끝 무두나사(AF1.5)를 가볍게 축에 접촉시켜 임시 고정합니다. 작은 와셔는 사용하지 않습니다. 상부 암과 상부 고정 나사는 아직 설치하지 않습니다.')
p('5. 상부625와 캡을 미리 설치한 C02 상부 데크를 위에서 내려 조립하고 C03 백킹 상부의 M3x14 두 개와 너트를 체결합니다. C10 상부 암은 분리된 상태여야 합니다. 이후 키를 맞춰 상부 암을 내려 M3x16 +너트로 연결하고 상부 Z80의 포켓 M3 너트와 ISO4026 M3x6을 설치합니다. 축 높이와 스페이서 스택을 맞춘 뒤 두 무두나사를 조정하여 축을 고정합니다. 상부 베어링 캡은 반드시 ISO7380-1 M3x10 버튼헤드입니다.')
p('6. C10 상·하 장공의 M4x14 +1 mm 와셔를 느슨하게 두고 캐리어를 C=141 mm 쪽으로 이동하여 벨트를 안착시킵니다. 전나사산 M3x25 잭으로 양쪽 축 끝이 함께 움직이게 조절합니다. Bossard BN3 또는 동등 제품을 사용하여 머리 가까운 잠금너트에도 나사산이 물려야 합니다. 명목 C=144 mm이며 사용 가능 범위141~147 mm입니다.')
p('7. 회전 저항이 증가하지 않는 최소 장력으로 맞추고 캐리어 M4 두 개와 잭 잠금너트를 고정합니다. 잭은 상부 서비스 창을 통해 AF2.5 L렌치로 접근합니다. 가동 범위를 억지로 끝까지 밀어 벨트를 팽팽하게 만들지 않습니다.')
p('8. 노즈 롤러를 손으로 돌리고 축을 당겨 롤러의 자유 회전과 축의 미끄러짐 방지를 확인합니다. 무두나사가 축을 옆으로 밀어 위치가 바뀌는 경우 캐리어 정렬과 장력을 다시 맞춥니다. 확인 후 C13 외측 커버를 좌우 각각 M3x10 네 개와 데크의 포켓 너트로 체결하고 벨트 접촉 여부를 다시 확인합니다.')
note('자유 루프 C143.2 대비 C144의 기하학적 늘어남은 약0.423%입니다. 이것은 장력(N)을 뜻하지 않습니다. C141은 설치용 느슨한 상태이며 처짐·재료 크리프는 강체 CAD에 포함되지 않습니다.')
page();title(6,'후방 타이밍 구동과 독립 커버')
table([['기준','20T 모터 풀리','32T 루트 풀리'],['품번','RobotDigg GT2-20T-5B-6','MISUMI GPA32GT2060-A-P5'],['장착 방향','Ø10 허브가 위쪽','보스 없는 A형'],['밑면 / 윗면 Z','82 /99.5','81.6 /91.9'],['벨트 홈 Z','83~90.5','83.25~90.25'],['홈 중심 Z','86.75','86.75'],['벨트 폭 / Z 범위','6 /83.75~89.75','동일'],['나사 고정','2xM3, 허브 끝에서4','포함된 M3x3 두 개']], [129,187,187],True)
p('1. 완성된 카세트를 C01 새들에 맞춥니다. 하부 데크는 M4x18 두 개, 상부 데크와 C11 브래킷은 함께 M4x25 두 개로 새들의 포켓 M4 너트에 고정합니다. C12 슬라이드는 C11 포켓 너트와 M3x12 네 개로 연결하며 장력 조절 전에는 느슨하게 둡니다. NEMA11은 C12에 M2.5x8 +0.5 mm 와셔로 고정합니다. 나사 진입3.5 mm로 도면 최소 깊이4 mm를 넘지 않습니다.')
p('2. 220-2GT 벨트를 두 풀리 홈에 먼저 끼운 뒤, 두 풀리와 벨트를 함께 위에서 평행하게 내려 두 축에 장착합니다. 이 조립 경로를 양측 CAD에서 검사했습니다. 20T와32T의 밑면을 같게 맞추지 않습니다.')
p('3. 모터축 평면에 허브 나사를 맞춥니다. 20T 세트스크루 중심 Z95.5, 모터축 끝 Z98입니다. 32T 나사는 벨트를 느슨하게 두고 공구가 닿는 각도로 풀리를 돌려 고정합니다. 풀리 고정 후 루트 축 유격0.2 mm와 자유 회전을 다시 확인합니다.')
p(f'4. 피치 길이220 mm의 기하학적 중심거리 {P["timing_centres"]:.3f} mm를 기준으로 모터 슬라이드를 조절하고 M3x12 네 개를 고정합니다. 슬라이드 이동 ±3 mm는 벨트를 제거한 상태의 조절 여유 검사입니다. 실제 벨트를 ±3 mm 모두 늘려 사용할 수 있다는 뜻이 아닙니다.')
p('5. 모터선은 C11 후방 타이 고정부 두 곳에 묶고 후방 하단 출구로 뺍니다. 슬라이드 조절과 개폐 이동에 필요한 여유선을 둡니다. V4의 공유 커버 볼트 대신 전용 체결부를 사용합니다.')
p('6. C14를 전용 M3x12 세 개로 고정합니다. 커버를 열 때는 이 세 개만 제거합니다. C11 구조 볼트와 슬라이드 볼트는 남아 있습니다. 커버 내면과20T 상단 명목 간격은2.5 mm입니다.')
page();title(7,'원점 스위치와 받침 조립')
p('F08과 D2F-01F는 프레임에 달기 전에 작업대에서 먼저 조립합니다. M2x12 두 개, 0.5 mm 와셔, M2 너트를 사용합니다. 브래킷 장공은 X 방향 ±0.8 mm 조정용입니다. 두 장착 구멍의 피치는6.5 mm입니다.')
p('F08을 F01의 추가 구멍에 M3x16 두 개로 고정합니다. 스위치는 레일 위쪽에 놓이고 우측 C01의 돌기가 누릅니다. 스위치 PCB 단자 세 개에 납땜·절연한 배선을 별도로 준비하고 고정합니다. 외부 제어기는 COM/NC 연속성 확인으로 배선 단선을 함께 감지하도록 구성할 수 있습니다.')
sub('전원을 넣기 전 원점 위치 교정')
p('1. 캘리퍼로 접촉면 간격95.6 mm를 맞추고, 멀티미터로 스위치 전환점을 확인하며 장공 위치를 조정합니다. 규격 OP의 공차가±0.3 mm이므로 CAD의 명목 위치만 믿고 통전하지 않습니다.')
p('2. 96 mm 기계 스토퍼까지 손으로 이동하여 스위치가 과도하게 눌리거나 본체가 돌기와 부딪히지 않는지 확인합니다. 명목 전환 후 스위치 추가 눌림은0.2 mm입니다. 제조사가 보장하는 최소 OT 0.25 mm보다 작지만 여유가0.05 mm뿐이므로, 전환점과 스토퍼를 실측하여 과도한 눌림이 없도록 확인합니다.')
p('3. 원점 탐색은 개방 방향으로 저속 접근하고 신호 전환 즉시 정지·후퇴합니다. 스위치는 힘 센서가 아니며 모터 전류 한도만으로 과실 파지력을 보장할 수 없습니다. 제어기·드라이버·프로그램은 본 기구 파일에 포함되지 않습니다.')
sub('T01 좌측 받침')
p('받침은 좌측 하부 데크의 두 위치에 M3x16 +포켓 너트로 연결합니다. 앞부분은 닫히고 후방 배출구로10° 경사를 줍니다. 높이를 맞춘 새 데크와 함께 이동하며 개폐 범위에서 프레임·상대편 부품과 간섭 여부를 검사했습니다. 과실이 실제로 구르고 빠지는 경로는 더미와 실제 작물로 시험해야 합니다.')
p('배선과 타이는 STL에 포함되지 않습니다. 소모품은 AUXILIARY_BOM.txt를 확인합니다. 배선은 링크·풀리·벨트 경로와 떨어뜨리고 개폐 전 구간에서 당겨지지 않도록 여유 루프를 남깁니다.')
page();title(8,'검증 결과와 더블 체크')
table([['검사','결과 / 범위'],['강체 간섭',f'간격28~96 mm, {len(inter["positions"])} 위치. {inter["exact_boolean_tests"]}회 정밀 교집합. 0.01 mm³ 초과 간섭0'],['벨트 장력 조절','노즈 -3 /0 /+3 mm, 간섭0. TPU 변형·힘은 제외'],['모터 위치 조절','슬라이드 -3 /0 /+3 mm, 간섭0. 타이밍 벨트 제거 조건'],['나사 길이',f'{len(checks["interfaces_and_tension"]["interfaces"])} 체결 스택의 너트 물림 / 막힌 구멍 깊이 확인'],['조립·서비스 경로',f'좌우 데크·상부 암·후드·타이밍 세트 {sum(len(p["positions"]) for p in checks["assembly_paths"]["paths"])} 경로 위치 검사, 간섭0'],['공구 접근','장력 잭 AF2.5 L렌치 및 구조·모터 슬라이드 수직 접근 확인'],['2차 회전 검사',f'개폐 최소/최대에서 {len(audit["rotation_samples"])} 조합. 드럼·축·노즈·풀리 회전 간섭0'],['2차 STEP 실측','출력된 STEP에서 좌우 본체 기준면 및 독립 벨트 높이 재측정'],['내보내기',f'STL {len(ex["stl"])}개 폐곡면·일체·양의 체적·220 mm 확인. STEP {len(ex["step"])}개 재오픈 유효'],['뷰어',f'{checks["browser_checks"]["render_path"]}. 개폐·벨트/축 회전·부품선택·분해·정투상 시험']], [124,379],True)
sub('실제 형상에서 다시 측정한 여유')
table([['위치','명목 최소 간격'],['32T 하부 플랜지 / 버튼헤드',f'{audit["clearances_mm"]["R_32T_flange_to_cap_button"]:.3f} mm'],['20T 상단 / 후드',f'{audit["clearances_mm"]["R_20T_top_to_cover"]:.3f} mm'],['원점 브래킷 / 블록, gap96',f'{audit["clearances_mm"]["home_bracket_to_block_at96"]:.3f} mm'],['원점 스위치 본체 / 새들, gap96',f'{audit["clearances_mm"]["home_body_to_carriage_at96"]:.3f} mm']], [370,133],True)
note('커버·슬라이드, 캡 볼트·풀리, 새들·원점 브래킷의 충돌을 수정했습니다. V5.1 체결 치수와 노즈 나사 조정 여유는 checks/repair_audit.json, 분해 소속 관계는 checks/explosion_audit.json에 있습니다. 샘플 사이 연속 경로, 모든 공차 조합과 변형 해석을 증명한 결과는 아닙니다.')
page();title(9,'STEP·STL·뷰어 사용 방법')
p('inaho_gripper_v5_viewer.html은 네트워크 요청 없이 모델·폰트·Three.js를 포함합니다. 전면·후면·측면·상면은 정투상입니다. 좌측 높이 표와 부품 클릭 정보로 본체와 벨트의 기준면을 구분할 수 있습니다. 회전 애니메이션은 미끄럼 없는 기구학 표시이며 재료 접촉 시뮬레이션이 아닙니다.')
p('분해도는 부품과 체결 세트의 배치를 확인하는 표시입니다. 실제 조립 순서는 이 가이드를 따릅니다. 노즈 무두나사와 축의 접촉 위치·조임 상태는 분해0% 조립 상태에서 확인합니다. 분해100% 위치를 장착 치수로 사용하지 않습니다.')
p('models의 전체 STEP은 mm 단위, 부품별 명칭과 색을 포함한 조립체입니다. L/R/fixed/linkage 하위 STEP과 step_parts의 개별 출력 부품 STEP도 제공합니다. SolidWorks에서 STEP을 열고 가져오기 진단 및 간섭 검사를 실행할 수 있습니다. STEP에 SolidWorks 네이티브 메이트·모터 제약은 포함되지 않습니다.')
p('개별 STEP은 조립 좌표를 유지합니다. STL은 출력 방향을 적용하여 원점과 바닥에 옮겼습니다. 두 파일을 그대로 포개면 일치하지 않는 것이 정상입니다. C07 STL은 자유 루프 C143.2, 조립 STEP과 GLB는 장착 C144입니다.')
p('GLB는 미터/Y-up 규약이며 뷰어는 같은 형상을 mm/Z-up으로 변환하여 표시합니다. STL을 슬라이서에서 mm로 열고 임의 자동 스케일링이 적용되지 않았는지 확인합니다.')
sub('파일 안내')
table([['경로','내용'],['source/','CadQuery 설계·검증·내보내기 / Three.js 뷰어 소스'],['stl/','출력 부품66개 + 핏 쿠폰5개'],['step_parts/','각 출력 부품·핏 쿠폰의 조립 좌표 STEP'],['models/','전체 및4개 하위 조립체 STEP / GLB'],['docs/','조립 가이드, 구매 BOM, 출력 목록, 체결 스택, 기준 치수·출처'],['checks/','간섭·회전·조립 경로·공구·재오픈·뷰어 검증 JSON'],['FILE_SHA256.json','전체 패키지 파일 해시']], [135,368])
p('SolidWorks 프로그램 자체에서는 실행하지 못했습니다. STEP 유효성·솔리드 개수·기준 높이와 재오픈은 OpenCascade로 검증했습니다. 실제 사용 중인 SolidWorks 버전에서 가져오기 진단을 완료한 후 제작에 사용하십시오.','SmallKR')
page();title(10,'출력 후 조립·동작 시험 기록')
p('아래는 완료한 시험이 아니라 제작 후 수행할 인수 확인 절차입니다. 기록에 실제 값을 채우고 다음 단계로 이동합니다.')
table([['순서','확인 / 기록'],['1. 부품 검사','핏 쿠폰, 구멍 피치, 뒤틀림, 체결부 균열 확인. 선정한 홀 보정값 기록'],['2. 무전원 수동 검사','벨트 없는 상태에서 축과 링크가 걸리지 않는지 확인. gap28/44/96 실측'],['3. 벨트 장착','루프 안착·좌우 높이·플랜지 여유·백킹0.6 mm 확인. C값과 유격 기록'],['4. 원점·배선','스위치 전환 gap95.6 교정. 기계 끝96까지 실제 남은 작동량 확인. 단선·당김 확인'],['5. 낮은 속도 무부하','양측 벨트를 각각 구동해 편주·마찰·풀리 미끄럼·베어링 발열 확인. 기준 마크로 벨트 이동 비교'],['6. 더미 파지력','목표 과실 크기의 탄성 더미와 외부 힘계로 좌우 누름 힘(N)을 측정. 모터 전류·간격·속도와 함께 기록'],['7. 반복 시험','우선100회 개폐/이송 후 나사 풀림, TPU 늘어남, 트래킹과 크랙 확인. 실패 시 보완 후 재시험'],['8. 실제 작물','더미 시험을 통과한 설정부터 시작. 수확 성공·압흔·표피 손상·낙하·받침 배출 결과 기록']], [114,389])
sub('판정 기준')
p('금속축과 출력부가 계속 마찰하거나, 플랜지에 벨트가 지속 접촉하거나, 나사를 풀어야만 회전하는 상태는 조립 완료로 판정하지 않습니다. 벨트 이탈·동작 누락·균열·커버 탈착 후 정렬 변화가 없어야 합니다. 더미 힘 측정과 과실 손상 결과를 바탕으로 허용 파지력·구동 조건을 정합니다.')
p('이 모델은 모든 부품을 플라스틱으로 출력하는 구성이 아닙니다. 모터·레일·베어링·금속축·타이밍 벨트·볼트류는 BOM 규격품입니다. 출력물만 연결하여 동작하는 모형과 달리 실제 회전·미끄럼 접점은 해당 구매 부품을 사용합니다.')
page();title(11,'치수 근거와 재현 방법')
p('공개 자료에서 확인할 수 없는 inaho 생산 치수는 복원값으로 단정하지 않았습니다. 본체 동일 높이와 벨트 끝단 비대칭은 사용자가 동영상에서 확인한 관찰을 설계 전제로 적용했습니다. 앞뒤14 mm, 벨트 끝단4 mm, 폭52 mm는 본 설계에서 선정한 값입니다.')
sources=json.loads((ROOT/'docs/SOURCES.json').read_text())
table([['대상','제조사·공개 자료']]+[[item['component'],item['url']] for item in sources if item['component']!='벨트 모터 도면 사본'],[105,398],True)
p('재현: Python3.12 + source/requirements.txt 의존성을 설치하고 build_v5.py --export, verify_v5.py, verify_interfaces.py, verify_repair.py, verify_assembly_paths.py, check_exports.py, verify_double_check.py, build_viewer.py 순으로 실행합니다. node source/test_explosion_ownership.cjs와 브라우저 검사 후 build_guide.py, check_pdf.py와 package_release.py를 실행합니다. 상세 명령과 PDF 검토 조건은 README_KO.txt를 따릅니다.','SmallKR')
p('parameters.json은 내보낸 설계 요약입니다. 모든 형상이 이 JSON으로 완전 구속되는 범용 파라메트릭 모델은 아닙니다. 치수를 바꿀 때는 source/build_v5.py의 관련 형상·스택을 함께 수정하고 전체 검증을 재실행합니다.','SmallKR')
out=ROOT/'docs/inaho_gripper_v5_assembly_guide_KO.pdf'
temporary=out.with_suffix('.tmp.pdf')
SimpleDocTemplate(str(temporary),title='FARMILY V5.1 그리퍼 조립 가이드',author='FARMILY / Mechanical Development',pagesize=(595,842),leftMargin=46,rightMargin=46,topMargin=52,bottomMargin=44).build(story,onFirstPage=header,onLaterPages=header)
temporary.replace(out)
print(out)
