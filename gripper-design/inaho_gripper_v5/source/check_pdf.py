#!/usr/bin/env python3
"""Run after PDF generation. --visual-reviewed records completed rendered-page review."""
from pathlib import Path
import argparse,hashlib,json
import fitz
root=Path(__file__).resolve().parents[1]
args=argparse.ArgumentParser();args.add_argument('--visual-reviewed',action='store_true');args=args.parse_args()
p=root/'docs/inaho_gripper_v5_assembly_guide_KO.pdf';doc=fitz.open(p);outside=[];text=''
for i,page in enumerate(doc):
 text+=page.get_text()
 for block in page.get_text('dict')['blocks']:
  if block['type']!=0:continue
  for line in block['lines']:
   for span in line['spans']:
    x0,y0,x1,y1=span['bbox']
    if x0<40 or x1>555 or y0<0 or y1>842:outside.append({'page':i+1,'text':span['text'],'bbox':span['bbox']})
expected=['출력 부품 66','STL 71','STEP 76','39 위치','16 조합','95.6','99.5','0.950','AUXILIARY_BOM.txt','실물 시험 전']
missing=[s for s in expected if s not in text]
report=dict(pages=len(doc),pdf_sha256=hashlib.sha256(p.read_bytes()).hexdigest(),text_bounds_pass=not outside,outside_text=outside,missing_expected_text=missing,missing_glyph_count=text.count('\x00'),visual_review_pass=args.visual_reviewed,visually_reviewed_pages=list(range(1,len(doc)+1)) if args.visual_reviewed else [])
(root/'checks/pdf_checks.json').write_text(json.dumps(report,ensure_ascii=False,indent=2));print(json.dumps(report,ensure_ascii=False,indent=2))
assert not outside and not missing and '\x00' not in text
