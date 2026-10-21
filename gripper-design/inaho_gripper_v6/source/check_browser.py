#!/usr/bin/env python3
"""Verify V6 and V5 selection, motion, exact downloads and mobile navigation."""
import argparse
import hashlib
import json
import os
from pathlib import Path
from urllib.request import urlopen
from playwright.sync_api import sync_playwright

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--url', default='http://127.0.0.1:8090/gripper-design/')
parser.add_argument('--browser', default='/root/.cache/ms-playwright/chromium-1208/chrome-linux64/chrome')
args = parser.parse_args()
os.environ.pop('DISPLAY', None)
errors, checks = [], []
with sync_playwright() as p:
    browser = p.chromium.launch(executable_path=args.browser, headless=True,
        args=['--no-sandbox','--ozone-platform=headless','--use-angle=swiftshader','--enable-unsafe-swiftshader'])
    page = browser.new_page(viewport={'width': 1500, 'height': 1000}, accept_downloads=True)
    page.on('pageerror', lambda error: errors.append(str(error)))
    page.on('response', lambda r: errors.append(f'{r.status} {r.url}') if r.status >= 400 else None)
    def ready(revision):
        page.wait_for_function('''revision=>{
          const w=document.querySelector('iframe').contentWindow;
          return w?.GRIPPER_API && w.document.querySelector('#viewtitle')?.textContent.startsWith(revision+' /');
        }''', arg=revision, timeout=90000)
        return page.frames[1]
    page.goto(args.url)
    viewer = ready('V6.0')
    controls = page.frame_locator('#gripperViewer')
    assert page.locator('#designVersion').input_value() == 'v6'
    state = viewer.evaluate('GRIPPER_API.snapshot()')
    assert state['parts'] == state['visible'] == 356
    assert any(p['name'] == 'L_T01_deep_U_receiver' for p in viewer.evaluate('GRIPPER_API.partStates()'))
    page.screenshot(path=str(ROOT/'previews/page-desktop.png'))
    for gap in [28,44,96]:
        controls.locator('#gap').fill(str(gap))
        assert viewer.evaluate('GRIPPER_API.snapshot().gap') == gap
    before = viewer.evaluate('GRIPPER_API.snapshot().cameraPosition')
    controls.locator('#explode').fill('1')
    assert all(abs(a-b)<1e-7 for a,b in zip(before,viewer.evaluate('GRIPPER_API.snapshot().cameraPosition')))
    states = {p['name']:p for p in viewer.evaluate('GRIPPER_API.partStates()')}
    tray = states['L_T01_deep_U_receiver']
    assert all(p['offset'] == tray['offset'] for name,p in states.items() if 'T01' in name)
    controls.locator('#showtray').uncheck()
    assert all(not p['visible'] for p in viewer.evaluate('GRIPPER_API.partStates()') if 'T01' in p['name'])
    controls.locator('#reset').click()
    controls.locator('#cycle').click()
    viewer.wait_for_function('GRIPPER_API.snapshot().gap!==44')
    controls.locator('#reset').click()
    controls.locator('#beltrun').click()
    viewer.wait_for_function('GRIPPER_API.snapshot().phase>0')
    controls.locator('#reset').click()
    controls.locator('[data-view="receiver"]').click()
    assert controls.locator('#viewtitle').inner_text().startswith('V6.0 / RECEIVER')
    checks.append('V6 356 parts; full gap range, opening/belt playback, tray toggle and fastener explosion ownership; no camera jump')
    for selector in ['#downloadStep','#downloadGlb','#downloadPrint']:
        with page.expect_download() as download:
            page.locator(selector).click()
        d = download.value
        assert d.failure() is None and 'v6' in d.suggested_filename
        candidates = list(ROOT.rglob(d.suggested_filename))
        assert len(candidates) == 1
        assert hashlib.sha256(Path(d.path()).read_bytes()).digest() == hashlib.sha256(candidates[0].read_bytes()).digest()
    with urlopen(page.locator('#downloadGuide').evaluate('(link)=>link.href')) as r:
        assert r.read().startswith(b'%PDF')
    checks.append('V6 STEP/GLB/ZIP downloaded with matching content and original names; V6 PDF opens')
    page.locator('#designVersion').select_option('v5')
    viewer = ready('V5.1')
    assert 'version=v5' in page.url
    assert any(p['name'] == 'L_T01_sloped_receiver' for p in viewer.evaluate('GRIPPER_API.partStates()'))
    assert 'inaho_gripper_v5_assembly.step' in page.locator('#downloadStep').get_attribute('href')
    page.reload();ready('V5.1')
    page.locator('#designVersion').select_option('v6');ready('V6.0')
    page.go_back();ready('V5.1')
    page.go_forward();viewer=ready('V6.0')
    page.set_viewport_size({'width':390,'height':844})
    assert not page.evaluate('document.documentElement.scrollWidth>innerWidth')
    assert not viewer.evaluate('document.documentElement.scrollWidth>innerWidth')
    page.screenshot(path=str(ROOT/'previews/page-mobile.png'))
    page.locator('#projectHomeLink').click()
    page.wait_for_selector('[data-project="gripper-design"]')
    checks.append('V5.1 and V6 selection updates models/downloads; URL, reload, back/forward, mobile and project-home navigation work')
    assert not errors, errors
    browser.close()
report = dict(passed=True,checks=checks,browser_errors=errors)
(ROOT/'checks/browser_checks.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
print(json.dumps(report,ensure_ascii=False))
