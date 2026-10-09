// Run with node source/test_explosion_ownership.cjs after build_v5/build_viewer.
// The parent relationships below come from the bearing seats, nut traps and
// fastening stacks in build_v5.py, independently of app.js ownership rules.
const fs=require('fs'),path=require('path'),vm=require('vm'),crypto=require('crypto');
const root=path.resolve(__dirname,'..');
const manifest=JSON.parse(fs.readFileSync(path.join(root,'parts_manifest.json'),'utf8'));
const params=JSON.parse(fs.readFileSync(path.join(root,'parameters.json'),'utf8'));
const app=fs.readFileSync(path.join(__dirname,'app.js'),'utf8');
const begin=app.indexOf('// BEGIN EXPLODED_ASSEMBLY_OWNERSHIP');
const end=app.indexOf('// END EXPLODED_ASSEMBLY_OWNERSHIP');
const code=app.slice(begin,end)+'\nthis.auditAPI={explodedAssembly,explodedOffset,hardware,explodedOwners};';
const context={MANIFEST:manifest};vm.createContext(context);vm.runInContext(code,context);
const api=context.auditAPI,byName=new Map(manifest.map(p=>[p.name,p])),errors=[],relationships=[],exceptions=[];
const same=(a,b)=>JSON.stringify(a)===JSON.stringify(b);
const check=(ok,message)=>{if(!ok)errors.push(message);};
function relation(child,parent,reason){
  check(byName.has(child)&&byName.has(parent),'missing relationship part: '+child+' / '+parent);
  if(!byName.has(child)||!byName.has(parent))return;
  for(const amount of [0,.25,.5,1])check(same(api.explodedOffset(byName.get(child),amount),api.explodedOffset(byName.get(parent),amount)),'child detached from owning assembly: '+child+' / '+parent+' at '+amount);
  relationships.push({child,parent,reason});
}
function children(prefix,pattern,parent,reason){
  const selected=manifest.filter(p=>p.name.startsWith(prefix)&&pattern.test(p.name.slice(prefix.length)));
  check(selected.length>0,'empty relationship family: '+prefix+pattern);
  for(const p of selected)relation(p.name,prefix+parent,reason);
}
for(const side of ['L','R']){
  const prefix=side+'_';
  children(prefix,/^C01_rail_bolt_/,'C01_carriage_saddle','Screw heads seat on the saddle.');
  children(prefix,/^C02_(?:lower|upper)_root_nut_/,'C01_carriage_saddle','Both root nut traps are in the saddle.');
  children(prefix,/^C02_lower_root_bolt_/,'C02_lower_deck','Lower-deck root screw head.');
  children(prefix,/^C03_(?:lower|upper)_nut_/,'C03_inner_belt_backing','Upper and lower nuts are trapped in the full-height backing.');
  for(const tag of ['lower','upper']){
    children(prefix,new RegExp('^C03_'+tag+'_bolt_'),'C02_'+tag+'_deck','The screw head seats on the corresponding deck.');
    relation(prefix+'H10_625_root_'+tag,prefix+'C02_'+tag+'_deck','Root bearing is retained in its deck seat.');
    relation(prefix+'C05_root_bearing_cap_'+tag,prefix+'C02_'+tag+'_deck','Bearing cap belongs to its deck.');
    children(prefix,new RegExp('^C05_'+tag+'_(?:bolt|nut)_'),'C02_'+tag+'_deck','Bearing-cap fastening stack and deck nut traps.');
    relation(prefix+'H13_625_nose_'+tag,prefix+'C08_nose_roller','Nose bearing is retained in the roller bore, not the deck.');
    relation(prefix+'C10_clamp_nut_'+tag,prefix+'C02_'+tag+'_deck','Carrier clamp nut is trapped in the deck.');
    const arm=tag==='upper'?'C10_removable_top_arm':'C10_internal_nose_carrier';
    relation(prefix+'C10_clamp_'+tag,prefix+arm,'Carrier-clamp screw head seats on the carrier/arm.');
    relation(prefix+'C10_clamp_washer_'+tag,prefix+arm,'Carrier-clamp washer seats on the carrier/arm.');
    children(prefix,new RegExp('^C13_'+tag+'_cover_nut_'),'C02_'+tag+'_deck','Cover nuts are trapped in the corresponding deck.');
  }
  children(prefix,/^C04_root_spacer_/,'H11_root_shaft','Loose axial spacers remain in the displayed root-shaft train.');
  relation(prefix+'H12_root_collar',prefix+'H11_root_shaft','Retaining collar accompanies the root shaft.');
  children(prefix,/^C06_clamp_(?:bolt|nut)_/,'C06_drive_drum_outer','Split drum clamp hardware stays with the displayed drum assembly.');
  relation(prefix+'C06_drive_drum_inner',prefix+'C06_drive_drum_outer','The split drum is displayed as one removable assembly.');
  children(prefix,/^C09_nose_spacer_/,'H14_nose_axle','Nose spacers remain in the axle train.');
  relation(prefix+'C08_nose_roller',prefix+'H14_nose_axle','Nose roller and axle separate from the cassette together.');
  for(const [z,arm] of [['-5','C10_internal_nose_carrier'],['80','C10_removable_top_arm']]){
    relation(prefix+'C10_axle_lock_'+z,prefix+arm,'The radial axle-lock screw belongs to the carrier at that height.');
    relation(prefix+'C10_axle_nut_'+z,prefix+arm,'The radial axle-lock nut trap is in that carrier/arm.');
    const washer=prefix+'C10_axle_washer_'+z;
    if(byName.has(washer))relation(washer,prefix+arm,'Historical V5.0 washer follows its carrier/arm.');
  }
  relation(prefix+'C10_top_arm_joint_bolt',prefix+'C10_removable_top_arm','Joint screw head seats on the removable arm.');
  relation(prefix+'C10_top_arm_joint_nut',prefix+'C10_internal_nose_carrier','Joint nut trap is in the lower carrier.');
  children(prefix,/^C10_jack_(?:screw|captive|locknut)$/,'C03_inner_belt_backing','Tension jack is supported by the backing reaction lug.');
  children(prefix,/^C11_root_bolt_/,'C11_rear_motor_bracket','Shared root screw head seats on the motor bracket.');
  children(prefix,/^C11_plate_bolt_/,'C12_motor_slide_plate','Slide-plate screw heads sit on the slide.');
  children(prefix,/^C11_plate_nut_/,'C11_rear_motor_bracket','Slide-plate nuts are trapped in the bracket.');
  children(prefix,/^C12_motor_(?:bolt|washer)_/,'C12_motor_slide_plate','Motor screws and washers seat on the slide plate.');
  relation(prefix+'H15_NEMA11_belt_motor',prefix+'C12_motor_slide_plate','Motor/slide subassembly stays together in the exploded display.');
  relation(prefix+'H15_rotor_shaft',prefix+'H15_NEMA11_belt_motor','Included rotor shaft remains with its motor.');
  children(prefix,/^C13_(?:lower|upper)_cover_bolt_/,'C13_outer_finger_cover','Both screw heads belong to the same unbroken outer cover.');
  children(prefix,/^C14_service_bolt_/,'C14_rear_drive_cover','Only the service screws accompany the removable hood.');
  children(prefix,/^C14_service_nut_/,'C11_rear_motor_bracket','Hood nuts stay in integral bracket posts.');
  const sign=side==='L'?-1:1;
  for(const j of [0,1]){
    for(const stem of ['K03_shoulder','K03_nut','K03_rear_washer','K03_front_washer'])relation(stem+'_'+sign+'_'+j,j===0?'K01_opening_crank':prefix+'C01_carriage_saddle','Pivot fasteners remain with their crank/saddle mounting.');
    relation('K04_bush_'+sign+'_'+j,'K02_link_'+sign,'Display convention: loose bushes accompany the separated link; actual motion group is unchanged.');
  }
  for(const [nut,bolt,reason] of [
    ['C10_top_arm_joint_nut','C10_top_arm_joint_bolt','Joint nut remains in the lower carrier when the arm rises.'],
    ['C10_clamp_nut_upper','C10_clamp_upper','Upper clamp nut remains in the deck when the nose assembly moves forward.'],
    ['C10_clamp_nut_lower','C10_clamp_lower','Lower clamp nut remains in the deck when the nose assembly moves forward.'],
  ]){
    check(!same(api.explodedOffset(byName.get(prefix+nut),1),api.explodedOffset(byName.get(prefix+bolt),1)),'stationary nut incorrectly follows removable screw: '+prefix+nut);
    exceptions.push({nut:prefix+nut,screw:prefix+bolt,reason});
  }
}
children('L_',/^T01_post_(?:bolt|nut)_/,'T01_sloped_receiver','Tray attachment hardware accompanies the removed tray.');
for(const p of manifest.filter(p=>/^F07_/.test(p.name)&&p.name!=='F07_opening_motor_yoke'))relation(p.name,'F07_opening_motor_yoke','Opening-motor hardware stays with the yoke subassembly.');
for(const p of manifest.filter(p=>/^A01_(?:bolt|nut)_/.test(p.name)))relation(p.name,'A01_rear_fixture_adapter','Adapter attachment hardware stays with the adapter.');
for(const p of manifest){
  check(api.explodedOwners.has(p.name),'unclassified instance: '+p.name);
  check(same(api.explodedOffset(p,0),[0,0,0]),'assembled position changed: '+p.name);
  const full=api.explodedOffset(p,1);
  check(full.every(Number.isFinite),'non-finite offset: '+p.name);
  for(const fraction of [.25,.5])check(same(api.explodedOffset(p,fraction),full.map(x=>x*fraction)),'nonlinear explosion: '+p.name);
}
try{api.explodedAssembly({name:'R_C99_new_part',group:'R'});errors.push('unknown part silently assigned an owner');}catch(e){check(/ownership missing/.test(e.message),'unknown part failed for an unrelated reason');}
const hardwareFixtures=['F02_rail_bolt_0','L_C01_rail_bolt_-10_22','R_C10_axle_lock_80','R_C10_axle_nut_80','K03_shoulder_1_0'];
for(const name of hardwareFixtures)check(api.hardware(byName.get(name)),'hardware toggle misses actual fastener: '+name);
for(const name of ['F02_MGN12R_195','R_H14_nose_axle','R_H12_root_collar','R_H15_NEMA11_belt_motor','R_H16_2GT_pulley_32T'])check(!api.hardware(byName.get(name)),'hardware toggle hides a major purchased component: '+name);
const motion=app.slice(app.indexOf('  function motion(p){'),app.indexOf('  function visibility(p){')).replace(/\s+/g,'');
const motionHash=crypto.createHash('sha256').update(motion).digest('hex');
check(motionHash==='26fc11647e8660cd6c8eb3bb03161cab0e4adbdf55032b9bf1603c81e63229c8','assembled motion implementation changed from reviewed V5.0');
check(manifest.length===(params.revision==='V5.1'?356:360),'unexpected manifest instance count for '+params.revision);
// Historical V5.0 offset, retained only to make the repair report reproducible.
function oldOffset(p){
  const n=p.name,side=p.group==='R'?1:p.group==='L'?-1:0,d=[0,0,0];
  if(n.includes('T01'))return [-25,0,-85];
  if(side){d[0]=side*65;if(/upper|top_arm|C14/.test(n))d[2]=60;if(/C11|C12|H15|H16|H17/.test(n))d[1]=-45;if(/C08|C09|C10|H13|H14/.test(n))d[1]=45;if(n.includes('C13'))d[0]+=side*30;}
  else if(n.startsWith('K'))d[1]=65;
  else if(/F07|H05|H06|A01/.test(n))d[1]=-70;
  return d;
}
const changed=manifest.filter(p=>!same(oldOffset(p),api.explodedOffset(p,1))).map(p=>({name:p.name,assembly:api.explodedAssembly(p),old_offset_mm:oldOffset(p),new_offset_mm:api.explodedOffset(p,1)}));
const ownershipCases=[
  ['upper_axle_lock_hardware',/^[LR]_C10_axle_(?:lock|nut|washer)_80$/,'Radial lock hardware follows the removable top arm.'],
  ['top_arm_joint_nut',/^[LR]_C10_top_arm_joint_nut$/,'Joint nut remains in the lower carrier.'],
  ['upper_root_captive_nuts',/^[LR]_C02_upper_root_nut_/,'Root nuts remain in the saddle.'],
  ['upper_backing_captive_nuts',/^[LR]_C03_upper_nut_/,'Backing nuts remain in the full-height backing.'],
  ['upper_root_spacers',/^[LR]_C04_root_spacer_upper_/,'Loose root spacers remain with the shaft train.'],
  ['upper_nose_bearings',/^[LR]_H13_625_nose_upper$/,'Upper nose bearings remain with the roller.'],
  ['upper_nose_spacers',/^[LR]_C09_nose_spacer_upper$/,'Upper nose spacers remain with the axle.'],
  ['carrier_clamp_captive_nuts',/^[LR]_C10_clamp_nut_(?:lower|upper)$/,'Carrier-clamp nuts stay in their deck traps.'],
  ['backing_tension_jack',/^[LR]_C10_jack_(?:screw|captive|locknut)$/,'Tension jack stays on the backing reaction lug.'],
  ['upper_outer_cover_screws',/^[LR]_C13_upper_cover_bolt_/,'Upper cover screws follow the unbroken cover laterally.'],
  ['outer_cover_captive_nuts',/^[LR]_C13_(?:lower|upper)_cover_nut_/,'Cover nuts remain in the decks.'],
  ['drive_cover_captive_nuts',/^[LR]_C14_service_nut_/,'Drive-cover nuts remain in bracket posts.'],
  ['drive_cover_and_service_screws',/^[LR]_C14_(?:rear_drive_cover|service_bolt_.*)$/,'Removable hood and its screws stay above the rear-drive assembly.'],
  ['loose_link_endpoint_bushes',/^K04_bush_(?:-1|1)_1$/,'Display convention places loose bushes with removed links.'],
].map(([issue,pattern,reason])=>({issue,reason,affected_instances:changed.filter(p=>pattern.test(p.name)).map(p=>p.name)}));
check(ownershipCases.reduce((sum,issue)=>sum+issue.affected_instances.length,0)===changed.length,'changed ownership is not fully explained by the named audit findings');
// A regression must continue to lift the arm, and must separate entire covers
// and roller trains; equal child/parent offsets alone cannot establish that.
for(const side of ['L','R']){
  const x=side==='L'?-65:65;
  for(const [name,expected] of [
    ['C10_removable_top_arm',[x,45,60]],['H14_nose_axle',[x,45,0]],
    ['C02_upper_deck',[x,0,60]],['C03_inner_belt_backing',[x,0,0]],
    ['C13_outer_finger_cover',[side==='L'?-95:95,0,0]],
    ['C14_rear_drive_cover',[x,-45,60]],['C11_rear_motor_bracket',[x,-45,0]],
  ])check(same(api.explodedOffset(byName.get(side+'_'+name),1),expected),'exploded placement contract changed: '+side+'_'+name);
}
const summary={revision:params.revision,all_checks_pass:errors.length===0,errors,instances:manifest.length,classified_instances:api.explodedOwners.size,changed_instances:changed.length,parent_relationships_checked:relationships.length,explosion_fractions_checked:[0,.25,.5,1],assembled_motion_sha256:motionHash,hardware_fixtures_checked:hardwareFixtures,assembly_counts:Object.fromEntries([...new Set(manifest.map(p=>api.explodedAssembly(p)))].map(owner=>[owner,manifest.filter(p=>api.explodedAssembly(p)===owner).length])),corrected_ownership_cases:ownershipCases,stationary_nut_exceptions:exceptions,relationships,changed};
fs.mkdirSync(path.join(root,'checks'),{recursive:true});fs.writeFileSync(path.join(root,'checks','explosion_audit.json'),JSON.stringify(summary,null,2));
console.log(JSON.stringify({revision:summary.revision,all_checks_pass:summary.all_checks_pass,instances:summary.instances,changed_instances:summary.changed_instances,parent_relationships_checked:summary.parent_relationships_checked,errors},null,2));
if(errors.length)process.exitCode=1;
