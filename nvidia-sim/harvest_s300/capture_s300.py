#!/usr/bin/env python3
"""Read the existing S300 V10 viewer's assembled mesh snapshot (no source edits)."""
import argparse
import gzip
import json
from pathlib import Path


def main():
    from playwright.sync_api import sync_playwright
    p = argparse.ArgumentParser()
    p.add_argument('--url', default='http://127.0.0.1:8090/s300/?version=v10')
    p.add_argument('--chromium', required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    if a.output.exists():
        raise FileExistsError(a.output)
    with sync_playwright() as pw:
        browser = pw.chromium.launch(executable_path=a.chromium, headless=True,
            args=['--no-sandbox', '--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader'])
        page = browser.new_page()
        page.goto(a.url, wait_until='networkidle', timeout=120000)
        page.wait_for_function('window.__viewerReady', timeout=120000)
        payload = page.evaluate('''() => {
          const allowed = new Set(__s300.versions.allowedMeshes().map(m=>m.uuid));
          const shot = __s300.captureExportScene('v10', {hideScans:true,hideGrid:true});
          const pack = a => { const u = new Uint8Array(a.buffer,a.byteOffset,a.byteLength);
            let s=''; for(let i=0;i<u.length;i+=16384)s+=String.fromCharCode(...u.subarray(i,i+16384));
            return {type:a.constructor.name,data:btoa(s)}; };
          const meshes=[];
          shot.scene.traverse(m => {if(!m.isMesh || !allowed.has(m.userData.exportSourceId))return;
            const g=m.geometry, mat=Array.isArray(m.material)?m.material[0]:m.material;
            if(Array.isArray(m.material)&&m.material.length>1)throw Error('Multiple materials: '+m.name);
            meshes.push({name:m.name,source:m.userData,matrix:m.matrixWorld.toArray(),
              positions:pack(g.attributes.position.array),indices:g.index?pack(g.index.array):null,
              normals:g.attributes.normal?pack(g.attributes.normal.array):null,
              colors:mat.vertexColors&&g.attributes.color?pack(g.attributes.color.array):null,
              color:mat.color.toArray(),roughness:mat.roughness??0.5,metallic:mat.metalness??0});
          }); shot.cleanup();
          return {schema:'s300-viewer-mesh-snapshot-v1',version:'v10',units:'m',up_axis:'Y',meshes};
        }''')
        browser.close()
    payload['source_url'] = a.url
    a.output.parent.mkdir(parents=True, exist_ok=True)
    with gzip.open(a.output, 'wt', encoding='utf-8') as stream:
        json.dump(payload, stream, ensure_ascii=False)
    print(json.dumps({'output':str(a.output),'meshes':len(payload['meshes'])}), flush=True)


if __name__ == '__main__':
    main()
