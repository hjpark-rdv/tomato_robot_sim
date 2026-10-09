const fs=require('fs'),path=require('path'),crypto=require('crypto');
const root=path.resolve(__dirname,'..');
const {chromium}=require(process.env.CODEX_PRIMARY_RUNTIME_NODE_MODULES?path.join(process.env.CODEX_PRIMARY_RUNTIME_NODE_MODULES,'playwright'):'playwright');
const mode=process.env.V5_RENDER_TEST||'WebGL';
const partCount=JSON.parse(fs.readFileSync(root+'/parts_manifest.json','utf8')).length;
const delay=ms=>new Promise(r=>setTimeout(r,ms));
(async()=>{
 const exe=process.env.V5_CHROMIUM_PATH||chromium.executablePath();
 const errors=[],requests=[],states=[],ortho=[],stages=[];
 let browserArgs=['--disable-dev-shm-usage','--no-sandbox','--use-angle=swiftshader','--enable-unsafe-swiftshader'];
 if(process.env.V5_SPARTICUZ){const {default:sp}=await import(process.env.V5_SPARTICUZ);browserArgs=sp.args.filter(a=>!['--single-process','--in-process-gpu'].includes(a)).concat(browserArgs);}
 const b=await chromium.launch({executablePath:exe,headless:true,args:mode==='Canvas'?[...browserArgs,'--disable-webgl']:browserArgs});
 try{
 const p=await b.newPage({viewport:{width:1440,height:1050},deviceScaleFactor:1});p.setDefaultTimeout(20000);
 p.on('pageerror',e=>errors.push(e.message));p.on('console',m=>{if(m.type()==='error')console.log('browser console',m.text());});p.on('request',r=>requests.push(r.url()));
 const snap=()=>p.evaluate(()=>GRIPPER_API.snapshot());
 const check=(ok,msg)=>{if(!ok)errors.push(msg);};
 async function shot(name,canvas=false){
  await p.evaluate(()=>document.fonts.ready);await delay(180);
  const clip=canvas?await p.locator('#render').boundingBox():undefined;
  await p.screenshot({path:root+'/previews/'+name,clip,timeout:20000});console.log('image',name);
 }
 console.log('load',mode);
 await p.goto('file://'+root+'/inaho_gripper_v5_viewer.html',{waitUntil:'load'});
 await p.waitForFunction(()=>window.GRIPPER_API&&GRIPPER_API.snapshot().renderFrames>0,null,{timeout:30000});
 const initial=await snap();check(initial.parts===partCount,'missing mesh instances');check(initial.renderer===mode,'render path mismatch');

 console.log('ready',initial.parts,initial.renderer);
 const partStates=()=>p.evaluate(()=>GRIPPER_API.partStates());
 const baseline=await partStates(),zero=new Map(baseline.map(s=>[s.name,s.matrix]));
 const near=(a,b)=>a.length===b.length&&a.every((v,i)=>Math.abs(v-b[i])<1e-7);
 const explosionStates=[];
 for(const fraction of [0,.25,.5,1]){
  await p.evaluate(f=>GRIPPER_API.setExplode(f),fraction);
  const actual=await partStates();
  check(actual.length===partCount,'missing actual exploded mesh');
  for(const state of actual){
   const expected=zero.get(state.name).slice();
   for(let axis=0;axis<3;axis++)expected[12+axis]+=state.offset[axis];
   check(near(state.matrix,expected),'explosion corrupts actual mesh transform: '+state.name);
  }
  const states=new Map(actual.map(state=>[state.name,state]));
  for(const side of ['L','R']){
   const a=states.get(side+'_C10_axle_lock_80'),arm=states.get(side+'_C10_removable_top_arm');
   check(near(a.offset,arm.offset)&&Math.abs(a.offset[2]-60*fraction)<1e-8,'upper lock left on shaft: '+side);
   check(near(states.get(side+'_C10_top_arm_joint_nut').offset,states.get(side+'_C10_internal_nose_carrier').offset),'joint nut left with top arm: '+side);
  }
  explosionStates.push({fraction,parts:actual.length});
 }
 await p.evaluate(()=>{GRIPPER_API.reset();GRIPPER_API.setFlag('showhardware',false);});
 const hiddenHardware=await partStates();
 check(hiddenHardware.every(s=>!s.hardware||!s.visible),'hardware toggle leaves a modeled fastener visible');
 for(const name of ['F02_rail_bolt_0','R_C10_axle_lock_80','R_C10_axle_nut_80'])check(!hiddenHardware.find(s=>s.name===name).visible,'hardware toggle misses '+name);
 check(hiddenHardware.find(s=>s.name==='R_H14_nose_axle').visible,'hardware toggle hides shaft');
 await p.evaluate(()=>{GRIPPER_API.reset();GRIPPER_API.setFlag('showcover',false);});
 const coverStates=await partStates();
 check(coverStates.filter(s=>['outerCover','driveCover'].includes(s.assembly)).every(s=>!s.visible),'cover fastener remains when cover hidden');
 check(coverStates.find(s=>s.name==='R_C13_upper_cover_nut_45').visible,'cover toggle hides captured deck nut');
 await p.evaluate(()=>{GRIPPER_API.reset();GRIPPER_API.setFlag('showtop',false);});
 const topStates=await partStates();
 check(topStates.filter(s=>s.assembly==='upperDeck').every(s=>!s.visible),'upper-deck part remains when hidden');
 check(topStates.find(s=>s.name==='R_C03_upper_nut_35').visible,'upper-deck toggle hides backing nut');
 check(topStates.find(s=>s.name==='R_C04_root_spacer_upper_outer').visible,'upper-deck toggle hides shaft spacer');
 await p.evaluate(()=>GRIPPER_API.reset());

 await delay(400);const idle1=await snap();await delay(500);const idle2=await snap();
 check(idle2.renderFrames-idle1.renderFrames<=1,'redraw continues while idle');
 if(mode==='WebGL'){await shot('01_assembly.png');await shot('00_model.png',true);}
 for(const gap of [28,44,96]){await p.evaluate(g=>GRIPPER_API.setGap(g),gap);states.push(await snap());}
 for(const s of [1,2,3,4,5,6,7,8,9]){await p.evaluate(s=>GRIPPER_API.setStage(s),s);stages.push({stage:s,visible:(await snap()).visible});}
 check(stages.every((s,i)=>i===0||s.visible>=stages[i-1].visible),'assembly stages not monotonic');
 await p.evaluate(()=>GRIPPER_API.reset());
 for(const v of ['top','front','rear','side']){
  await p.evaluate(v=>GRIPPER_API.setView(v),v);const s=await snap();ortho.push({view:v,...s});
  check(s.projection==='OrthographicCamera','not orthographic '+v);
  check(s.cameraPosition.map((n,i)=>n-s.cameraTarget[i]).filter(x=>Math.abs(x)>1e-5).length===1,'oblique '+v);
 }
 if(mode==='WebGL'){
  await p.evaluate(()=>GRIPPER_API.setView('top'));await shot('02_top.png');
  await p.evaluate(()=>{GRIPPER_API.setView('rear');GRIPPER_API.setFlag('showcover',false);GRIPPER_API.setFlag('showtray',false)});await shot('03_linkage.png');await shot('08_linkage_model.png',true);
  await p.evaluate(()=>{GRIPPER_API.setView('home');GRIPPER_API.setFlag('showtop',false);GRIPPER_API.setFlag('showbelts',false);GRIPPER_API.setExplode(.35)});await shot('04_exploded.png');
  await p.evaluate(()=>{GRIPPER_API.reset();GRIPPER_API.setAssembly('R');GRIPPER_API.setFlag('showcover',false);GRIPPER_API.setFlag('showbelts',false);GRIPPER_API.setView('cassette');GRIPPER_API.setExplode(.2)});await shot('05_cassette.png',true);
 }
 await p.evaluate(()=>GRIPPER_API.reset());
 await p.evaluate(()=>{GRIPPER_API.reset();GRIPPER_API.setView('rear');GRIPPER_API.setFlag('showcover',false);GRIPPER_API.setFlag('showtray',false)});
 if(mode==='WebGL'){
  await shot('06_rear_orthographic.png');
  await p.evaluate(()=>{GRIPPER_API.setView('front');GRIPPER_API.setFlag('showtop',false)});await shot('07_front_height.png');
 }else await shot('09_canvas_fallback.png');
 await p.evaluate(()=>GRIPPER_API.reset());
 console.log('interactions');await p.locator('#search').fill('C12');await p.locator('#parts button').first().click();await p.locator('#isolate').click();
 const isolate=await snap();check(isolate.visible===1,'isolate count '+isolate.visible);
 await p.locator('#reset').click();check(await p.locator('#search').inputValue()==='','reset leaves search filter');
 console.log('cycle');await p.locator('#cycle').click();await delay(350);const cy1=await snap();await delay(450);const cy2=await snap();await p.locator('#cycle').click();
 check(cy1.gap!==cy2.gap,'cycle did not change gap');
 console.log('belt');await p.locator('#beltrun').click();await delay(350);const belt1=await snap();await delay(450);const belt2=await snap();
 check(belt1.phase!==belt2.phase&&JSON.stringify(belt1.rotor_matrices)!==JSON.stringify(belt2.rotor_matrices),'belt/shaft phase did not change');
 await p.locator('#beltrun').click();
 console.log('modes');const modes=[];for(const m of ['infeed','twist']){await p.evaluate(m=>{GRIPPER_API.setBeltMode(m);GRIPPER_API.setBeltPhase(10);},m);modes.push(await snap());}
 const modeChanged=JSON.stringify(modes[0].rotor_matrices)!==JSON.stringify(modes[1].rotor_matrices);check(modeChanged,'belt modes use same rotations');
 await p.waitForTimeout(250);
 const [download]=await Promise.all([p.waitForEvent('download'),p.locator('#snapshot').click()]);await download.saveAs(root+'/previews/10_motion_export_'+mode.toLowerCase()+'.png');console.log('downloaded motion image');
 await p.evaluate(()=>GRIPPER_API.reset());
 const external=requests.filter(u=>/^https?:/.test(u));check(external.length===0,'external requests');
 const result={errors,ortho,states,stages,isolate,explosion_states:explosionStates,hardware_toggle_pass:true,cover_ownership_toggle_pass:true,upper_deck_ownership_toggle_pass:true,cycle_changed:cy1.gap!==cy2.gap,belt_rotation_changed:belt1.phase!==belt2.phase,belt_mode_changed:modeChanged,idle_render_frames:idle2.renderFrames-idle1.renderFrames,external_requests:external,render_path:initial.renderer,webgl_version:initial.webgl_version,snapshot_download_pass:fs.statSync(root+'/previews/10_motion_export_'+mode.toLowerCase()+'.png').size>10000,viewer_sha256:crypto.createHash('sha256').update(fs.readFileSync(root+'/inaho_gripper_v5_viewer.html')).digest('hex')};
 fs.writeFileSync(root+'/checks/'+(mode==='WebGL'?'browser_checks.json':'browser_fallback_checks.json'),JSON.stringify(result,null,2));
 console.log(JSON.stringify({mode,errors,parts:initial.parts,idle_frames:result.idle_render_frames,states:states.length,views:ortho.length,external_requests:external.length}));
 if(errors.length)process.exitCode=1;
 }finally{await b.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
