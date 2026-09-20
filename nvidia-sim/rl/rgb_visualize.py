"""Render exact online RGB guard diagnostics; never recompute tracks from video.

Input: lossless RGB, rgb_analysis.json and camera/ROI configuration.
No evaluator positions, contacts, force or simulator depth are read.
"""
import argparse
import json
from functools import lru_cache
from pathlib import Path

import cv2
import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw, ImageFont

BG = (15, 21, 32)
FG = (231, 237, 246)
MUTED = (159, 175, 195)
CYAN = (46, 217, 240)
YELLOW = (255, 220, 65)
PINK = (255, 97, 150)
GREEN = (103, 223, 153)
FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'


@lru_cache(None)
def font(size):
    return ImageFont.truetype(FONT, size)


def text(draw, xy, value, size=25, color=FG):
    draw.text(xy, value, font=font(size), fill=color)


def overlay(rgb, visual, roi=None):
    out = rgb.copy()
    if roi is not None:
        x,y,w,h = roi
        cv2.rectangle(out, (x,y), (x+w,y+h), GREEN, 2)
    previous = np.asarray(visual.get('previous_pixels', []))
    actual = np.asarray(visual.get('observed_pixels', []))
    predicted = np.asarray(visual.get('predicted_pixels', []))
    pixel = lambda p: tuple(np.rint(p).astype(int))
    for i, point in enumerate(actual):
        if len(predicted):
            cv2.arrowedLine(out, pixel(previous[i]), pixel(predicted[i]), CYAN, 1, cv2.LINE_AA, tipLength=.3)
            cv2.arrowedLine(out, pixel(previous[i]), pixel(point), YELLOW, 1, cv2.LINE_AA, tipLength=.3)
            cv2.line(out, pixel(predicted[i]), pixel(point), PINK, 2, cv2.LINE_AA)
            cv2.circle(out, pixel(predicted[i]), 3, CYAN, 1, cv2.LINE_AA)
        cv2.circle(out, pixel(point), 2, YELLOW, -1, cv2.LINE_AA)
    return out


def zoom(rgb, visual, roi):
    points = np.asarray(visual.get('observed_pixels', []))
    center = np.median(points,axis=0) if len(points) else np.array(roi[:2])+np.array(roi[2:])/2
    w,h = 288,120
    x = int(np.clip(center[0]-w/2,0,rgb.shape[1]-w))
    y = int(np.clip(center[1]-h/2,0,rgb.shape[0]-h))
    return cv2.resize(rgb[y:y+h,x:x+w],(864,360),interpolation=cv2.INTER_CUBIC), (x,y,w,h)


def validate(records):
    window = []
    hits = 0
    checked = 0
    for row in records:
        if not row['active']:
            continue
        visual = row.get('visual') or {}
        if 'predicted_pixels' not in visual:
            continue
        errors = np.asarray(visual['observed_pixels'])-np.asarray(visual['predicted_pixels'])
        assert np.allclose(errors,visual['innovation_vectors_px'],atol=1e-9)
        median = np.median(errors,axis=0)
        window = (window+[median])[-6:]
        residual = float(np.linalg.norm(np.sum(window,axis=0)))
        decision = row['decision']
        assert np.isclose(residual,decision['residual_px'],atol=1e-8)
        hits = hits+1 if residual>4 else 0
        assert hits == decision['persistence']
        checked += 1
    # An uncertainty stop can happen before a usable motion model exists.
    # Such a run has no residual curve; do not invent one for the video.
    assert any(r.get('visual') for r in records), 'No online feature diagnostics found'
    return checked


def panel(rgb, row, roi, history, duration, held, playback):
    canvas = Image.new('RGB',(1920,1080),BG)
    draw = ImageDraw.Draw(canvas)
    decision = row.get('decision') or {}
    visual = row.get('visual') or {}
    active = row['active']
    stopped = bool(decision.get('stop'))
    t = row['time_s']
    stop_reasons = {'tracking_lost':'특징점 추적 실패',
                    'insufficient_visible_features':'보이는 특징점 부족',
                    'unobservable_motion_model':'초기 모델 추정 실패',
                    'inconsistent_visual_tracks':'특징점 이동 불일치',
                    'invalid_motion_prediction':'유효하지 않은 이동 예측',
                    'motion_or_model_mismatch':'움직임 또는 모델 불일치'}
    stop_reason = stop_reasons.get(decision.get('reason'),'판단 불확실')
    state = (stop_reason+' → 후퇴' if stopped else
             '초기 모델 추정' if decision.get('reason') == 'calibrating' else
             'RGB 추적·판단 중' if active else '접근 중 · 추적 시작 전')
    if row['phase'] in ('retreat','hold'):
        state = '후퇴 / 유지 · 추가 추적 없음'
    text(draw,(24,18),'RGB + 관절 | 밀림 판단 시각화',39)
    text(draw,(1050,25),f"시뮬레이션 {t:05.2f}초  ·  STEP {row['step']}  ·  {playback}",25)
    text(draw,(24,78),f'실제 카메라 영상  |  {state}',25,PINK if stopped else GREEN)
    full = overlay(rgb,visual,roi if decision.get('reason')=='calibrating' and len(visual.get('previous_pixels',[]))==0 else None) if active else rgb
    canvas.paste(Image.fromarray(full),(24,120))
    # After the stop, show the decision frame in a clearly labeled frozen inset.
    frozen = not active and held is not None
    detail_image,detail_visual = held if frozen else (full,visual)
    detail, crop = zoom(detail_image,detail_visual,roi)
    canvas.paste(Image.fromarray(detail),(1020,160))
    draw = ImageDraw.Draw(canvas)
    text(draw,(1020,113),'정지 판단 프레임 고정 · 3배 확대' if frozen else '선택한 특징점 주변 · 3배 확대',25)
    draw.rectangle((1020,160,1884,520),outline=PINK if frozen else MUTED,width=2)
    if active:
        x,y,w,h=crop
        draw.rectangle((24+x,120+y,24+x+w,120+y+h),outline=MUTED,width=1)
    text(draw,(24,862),'● 실제 관측 위치 / 이동',25,YELLOW)
    text(draw,(480,862),'○ 관절로 예측한 위치 / 이동',25,CYAN)
    text(draw,(24,905),'분홍 선: 예측과 실제의 차이 (픽셀)',25,PINK)
    text(draw,(24,950),'각 점의 차이 → 중앙값 → 최근 6프레임 벡터 합의 크기',24)
    text(draw,(24,991),'4 px 초과가 3회 연속이면 정지 · 관측은 30 Hz',25)
    text(draw,(24,1036),'Depth·힘·정답 위치 미사용 | 초기 정지 가정 + 수동 ROI | 수확 성공 판정 아님',21,MUTED)
    # Draw only past measurements; no future values or privileged contact marker.
    text(draw,(1020,547),'정지 지표: 최근 6프레임 누적 오차 [px]',25)
    x0,y0,w,h = 1080,604,788,160
    ymax=20.
    for val in (0,4,10,20):
        yy=y0+h-h*val/ymax
        draw.line((x0,yy,x0+w,yy),fill=PINK if val==4 else (48,62,80),width=2 if val==4 else 1)
        text(draw,(1024,yy-16),str(val),22,PINK if val==4 else MUTED)
    pts=[(x0+w*tm/duration,y0+h-h*min(v,ymax)/ymax) for tm,v in history]
    if len(pts)>1:
        draw.line(pts,fill=YELLOW,width=3)
    xx=x0+w*t/duration
    draw.line((xx,y0,xx,y0+h),fill=FG,width=1)
    for sec in range(0,int(duration)+1,2):
        text(draw,(x0+w*sec/duration-7,y0+h+4),str(sec),20,MUTED)
    text(draw,(1785,799),'시간 [초]',18,MUTED)
    residual=decision.get('residual_px')
    text(draw,(1020,824),f"현재 지표  {'--' if residual is None else f'{residual:.2f}'} px  /  기준 4.00 px",29)
    hits=decision.get('persistence',0)
    text(draw,(1020,875),f'연속 초과  {min(hits,3)} / 3',29,PINK if hits else GREEN)
    for i in range(3):
        draw.rounded_rectangle((1530+i*65,880,1573+i*65,917),radius=6,fill=PINK if i<hits else (49,64,84))
    coherence=decision.get('coherence')
    text(draw,(1020,931),f"유효 특징점 {decision.get('tracks','--')}개  |  이동 일치율 {'--' if coherence is None else f'{coherence*100:.0f}%'}",25)
    note = ('마지막 판단값 유지 · 확대 화면은 정지 순간' if frozen else
            stop_reason+' → 정지 요청' if stopped else
            '특징점 8개 미만 / 추적 불안정도 중단 사유')
    text(draw,(1020,979),note,24,PINK if stopped else MUTED)
    text(draw,(1020,1022),'실제 실행 중 저장한 특징점·판단값 (재추정 없음)',22,MUTED)
    return np.asarray(canvas)


def render(directory):
    if (directory/'TARGET_IDENTITY_CORRECTION.txt').exists():
        raise ValueError('This historical run tracked the wrong fruit; preserve its correction banner and use the corrected ROI run.')
    config=json.loads((directory/'config.json').read_text())
    records=json.loads((directory/'rgb_analysis.json').read_text())
    checked=validate(records)
    stop=next((r for r in records if r['active'] and (r.get('decision') or {}).get('stop')),None)
    if stop is None:
        raise ValueError('This explanation expects a recorded online stop event')
    capture=cv2.VideoCapture(str(directory/(config['camera']+'_raw_2x.mkv')))
    outputs=[directory/'rgb_decision_2x.mp4',directory/'rgb_decision_explained.mp4']
    writers=[imageio.get_writer(str(p),fps=60,codec='libx264',quality=8,macro_block_size=1,
                               ffmpeg_params=['-movflags','+faststart']) for p in outputs]
    counts=[0,0]; history=[]; held=None
    try:
        for row in records:
            ok,bgr=capture.read()
            if not ok:
                raise RuntimeError('Lossless RGB / diagnostic frame count mismatch')
            rgb=cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB)
            decision=row.get('decision') or {}
            if row['active'] and decision.get('residual_px') is not None:
                history.append((row['time_s'],decision['residual_px']))
            visual=row.get('visual') or {}
            if row['active'] and decision.get('stop'):
                held=(overlay(rgb,visual),visual)
            regular=panel(rgb,row,config['roi'],history,records[-1]['time_s'],held,'2x')
            writers[0].append_data(regular); counts[0]+=1
            slow=stop['time_s']-.75 <= row['time_s'] <= stop['time_s']+.15
            frame=panel(rgb,row,config['roi'],history,records[-1]['time_s'],held,'0.25x · 판단 구간' if slow else '2x') if slow else regular
            for _ in range(8 if slow else 1):
                writers[1].append_data(frame); counts[1]+=1
            if row['active'] and decision.get('persistence') in (1,2,3):
                still=panel(rgb,row,config['roi'],history,records[-1]['time_s'],held,'설명용 정지 화면')
                for _ in range(120 if decision['persistence']==3 else 45):
                    writers[1].append_data(still);counts[1]+=1
                imageio.imwrite(directory/f"decision_hit_{decision['persistence']}.png",still)
        if capture.read()[0]:
            raise RuntimeError('Lossless RGB has extra frames')
    finally:
        capture.release()
        for writer in writers:
            writer.close()
    report=dict(source='Exact online diagnostics + lossless RGB; no compressed-video retracking',
                input_frames=len(records),validated_decision_frames=checked,stop_step=stop['step'],
                stop_time_s=stop['time_s'],stop_residual_px=stop['decision']['residual_px'],
                fps=60,output_frames=counts,outputs=[p.name for p in outputs],
                explained_video='2x normally; 0.25x around decision; intentional freezes at hits 1, 2, 3',
                no_ground_truth_inputs=True)
    (directory/'visualization_report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(__doc__)
    parser.add_argument('directory',type=Path)
    render(parser.parse_args().directory)
