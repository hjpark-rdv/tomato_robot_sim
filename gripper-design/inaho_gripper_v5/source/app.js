(() => {
  // BEGIN EXPLODED_ASSEMBLY_OWNERSHIP
  // Offsets describe removable assemblies, not words in part names. In
  // particular, a captive nut belongs to the printed part containing its trap.
  const EXPLODED_ASSEMBLIES={
    frame:{sideX:0,y:0,z:0},
    openingMotor:{sideX:0,y:-70,z:0},
    linkage:{sideX:0,y:65,z:0},
    cassette:{sideX:65,y:0,z:0},
    upperDeck:{sideX:65,y:0,z:60},
    rearDrive:{sideX:65,y:-45,z:0},
    noseTrain:{sideX:65,y:45,z:0},
    noseTopArm:{sideX:65,y:45,z:60},
    outerCover:{sideX:95,y:0,z:0},
    driveCover:{sideX:65,y:-45,z:60},
    tray:{x:-25,y:0,z:-85},
  };
  const CASSETTE_OWNERSHIP=[
    {assembly:'cassette',names:[
      'H01_MGN12H','C01_carriage_saddle','C02_lower_deck',
      'C03_inner_belt_backing','H10_625_root_lower',
      'C04_root_spacer_lower_outer','C04_root_spacer_lower_inner',
      'C04_root_spacer_upper_inner','C04_root_spacer_upper_outer',
      'C05_root_bearing_cap_lower','H11_root_shaft','H12_root_collar',
      'C06_drive_drum_inner','C06_drive_drum_outer','C07_contact_belt_TPU',
      'C10_clamp_nut_lower','C10_jack_screw','C10_jack_captive','C10_jack_locknut',
    ],patterns:[
      /^C01_rail_bolt_/,
      /^C02_lower_root_bolt_/,/^C02_(?:lower|upper)_root_nut_/,
      /^C03_lower_bolt_/,/^C03_(?:lower|upper)_nut_/,
      /^C05_lower_(?:bolt|nut)_/,/^C06_clamp_(?:bolt|nut)_/,
      /^C13_lower_cover_nut_/,
    ]},
    {assembly:'upperDeck',names:[
      'C02_upper_deck','H10_625_root_upper','C05_root_bearing_cap_upper',
      'C10_clamp_nut_upper',
    ],patterns:[/^C03_upper_bolt_/,/^C05_upper_(?:bolt|nut)_/,/^C13_upper_cover_nut_/]},
    {assembly:'rearDrive',names:[
      'C11_rear_motor_bracket','C12_motor_slide_plate',
      'H15_NEMA11_belt_motor','H15_rotor_shaft',
      'H16_2GT_pulley_32T','H16_2GT_pulley_20T','H17_timing_belt_220',
    ],patterns:[
      /^C11_plate_(?:bolt|nut)_/,/^C11_root_bolt_/,
      /^C12_motor_(?:bolt|washer)_/,/^C14_service_nut_/,
    ]},
    {assembly:'noseTrain',names:[
      'C08_nose_roller','H13_625_nose_lower','H13_625_nose_upper',
      'C09_nose_spacer_lower','C09_nose_spacer_centre','C09_nose_spacer_upper',
      'C10_internal_nose_carrier','H14_nose_axle',
      'C10_axle_lock_-5','C10_axle_nut_-5','C10_axle_washer_-5',
      'C10_top_arm_joint_nut','C10_clamp_lower','C10_clamp_washer_lower',
    ],patterns:[]},
    {assembly:'noseTopArm',names:[
      'C10_removable_top_arm','C10_top_arm_joint_bolt',
      'C10_axle_lock_80','C10_axle_nut_80','C10_axle_washer_80',
      'C10_clamp_upper','C10_clamp_washer_upper',
    ],patterns:[]},
    {assembly:'outerCover',names:['C13_outer_finger_cover'],patterns:[/^C13_(?:lower|upper)_cover_bolt_/]},
    {assembly:'driveCover',names:['C14_rear_drive_cover'],patterns:[/^C14_service_bolt_/]},
    {assembly:'tray',names:['T01_sloped_receiver'],patterns:[/^T01_post_(?:bolt|nut)_/]},
  ];
  function explodedAssembly(p){
    const n=p.name;
    // Bushes accompany the displayed loose links by convention. Their actual
    // motion groups still follow the corresponding crank or saddle pivot.
    if(/^K04_bush_(?:-1|1)_[01]$/.test(n))return 'linkage';
    if(/^K0[123]_/.test(n))return p.group==='R'||p.group==='L'?'cassette':'linkage';
    if(p.group==='R'||p.group==='L'){
      const localName=n.replace(/^[LR]_/,''),matches=CASSETTE_OWNERSHIP.filter(rule=>rule.names.includes(localName)||rule.patterns.some(pattern=>pattern.test(localName)));
      if(matches.length!==1)throw Error('Exploded assembly ownership missing or ambiguous: '+n);
      return matches[0].assembly;
    }
    if(/^(?:F07|H05|H06|A01)_/.test(n))return 'openingMotor';
    if(/^(?:F0[1-6]|F08|H0[2-4]|H1[89])_/.test(n))return 'frame';
    throw Error('Exploded assembly ownership missing: '+n);
  }
  function explodedOffset(p,amount){
    const owner=EXPLODED_ASSEMBLIES[explodedAssembly(p)],side=p.group==='R'?1:p.group==='L'?-1:0;
    return [owner.x===undefined?owner.sideX*side:owner.x,owner.y,owner.z].map(value=>value*amount);
  }
  const explodedOwners=new Map(MANIFEST.map(p=>[p.name,explodedAssembly(p)]));
  function hardware(p){return p.kind==='buy'&&/^(?:ISO(?:\s|\d)|Washer\b|Ganter ISO7379\b)/.test(p.spec);}
  // END EXPLODED_ASSEMBLY_OWNERSHIP
  const $=id=>document.getElementById(id), host=$('render');
  const scene=new THREE.Scene();scene.background=new THREE.Color('#e9eef0');
  const camera=new THREE.OrthographicCamera(-300,300,230,-230,.5,5000);camera.up.set(0,0,1);
  // Multisample antialiasing is disabled for software WebGL compatibility.
  let renderer,webgl=false,needsRender=true,renderFrames=0,lastRender=0;
  try{const c=document.createElement('canvas');const ctx=c.getContext('webgl2',{antialias:false,preserveDrawingBuffer:true})||c.getContext('webgl',{antialias:false,preserveDrawingBuffer:true});if(!ctx)throw Error('No WebGL');renderer=new THREE.WebGLRenderer({canvas:c,context:ctx,antialias:false,preserveDrawingBuffer:true});webgl=true;renderer.outputEncoding=THREE.sRGBEncoding;renderer.setPixelRatio(Math.min(devicePixelRatio,2));}
  catch(e){renderer=new EngineeringCanvasRenderer();}
  host.appendChild(renderer.domElement);$('rendererbadge').textContent=webgl?'Three.js / WebGL':'Three.js geometry / Canvas 호환 렌더러';
  let controls;function configureControls(){if(controls)controls.dispose();controls=new THREE.OrbitControls(camera,renderer.domElement);controls.enableDamping=false;controls.maxZoom=8;controls.minZoom=.35;controls.addEventListener('change',()=>{needsRender=true;});}configureControls();controls.target.set(0,5,2);
  scene.add(new THREE.HemisphereLight(0xffffff,0x5c7980,1.05));const sun=new THREE.DirectionalLight(0xffffff,.85);sun.position.set(-250,-250,500);scene.add(sun);
  const fill=new THREE.DirectionalLight(0xc5e5ef,.4);fill.position.set(250,140,90);scene.add(fill);
  const grid=new THREE.GridHelper(800,40,0xaabfc5,0xd6e0e4);grid.rotation.x=Math.PI/2;grid.position.z=-110;scene.add(grid);
  const byName=new Map(MANIFEST.map(p=>[p.name,p])), byGLTFName=new Map(MANIFEST.map(p=>[THREE.PropertyBinding.sanitizeNodeName(p.name),p])), objects=[],groups={};let selected=null,isolated=false,cycle=false,belts=false,phase=0,last=performance.now(),gap=44,explode=0;
  const v5=(x,y,z=0)=>new THREE.Vector3(x,y,z), pivot=v5(0,-31,-18);
  function crank(g){const s=g/2+18;return Math.acos((s*s+25*25-45*45)/(2*s*25));}
  const th0=crank(44);
  function stageOf(n){
    if(n.includes('T01'))return 9;
    if(/^[LR]_/.test(n)){
      if(/C13|C14/.test(n))return 8;
      if(/C11|C12|H15|H16|H17/.test(n))return 7;
      if(/C07|C10|H14/.test(n))return 6;
      if(/C06|C08|C09|H10|H11|H12|H13/.test(n))return 5;
      if(/C02|C03|C04|C05/.test(n))return 4;
      return 3;
    }
    if(n.startsWith('K'))return 3;
    if(/F04|F05|F06|F07|H02|H03|H04|H05|H06/.test(n))return 2;
    return 1;
  }
  function offset(p){
    return v5(...explodedOffset(p,explode));
  }
  function motion(p){
    const m=new THREE.Matrix4(),g=p.group,th=crank(gap),s=gap/2+18;
    if(p.name==='H19_home_switch_actuator'){const travel=Math.max(0,gap/2+40.5-87.8);m.makeTranslation(travel,0,0);}
    else if(g==='R'||g==='L')m.makeTranslation((g==='R'?1:-1)*(gap-44)/2,0,0);
    else if(g==='crank')m.makeTranslation(0,-31,-18).multiply(new THREE.Matrix4().makeRotationY(-(th-th0))).multiply(new THREE.Matrix4().makeTranslation(0,31,18));
    else if(g==='linkR'||g==='linkL'){
      const sign=g==='linkR'?1:-1,a0=new THREE.Vector3(...p.motion.a0),b0=new THREE.Vector3(...p.motion.b0),a=v5(sign*25*Math.cos(th),-31,-18+sign*25*Math.sin(th)),b=v5(sign*s,-31,-18);
      const a1=Math.atan2(b.z-a.z,b.x-a.x),a2=Math.atan2(b0.z-a0.z,b0.x-a0.x);
      m.makeTranslation(a.x,a.y,a.z).multiply(new THREE.Matrix4().makeRotationY(-(a1-a2))).multiply(new THREE.Matrix4().makeTranslation(-a0.x,-a0.y,-a0.z));
    }
    if(explode===0 && /C06_|H11_root_shaft|H12_root_collar|H16_2GT_pulley|H15_rotor_shaft|C08_nose_roller/.test(p.name)){
      const side=p.group==='R'?1:-1;
      const twist=$('beltmode').value==='twist';
      const sign=twist?1:side;
      let ang=sign*phase/16,x=side*40,y=side<0?14:0;
      if(/20T|rotor_shaft/.test(p.name)){ang*=32/20;y-=PARAMS.timing_centres;}
      if(p.name.includes('C08')){ang*=16/11;x-=side*5;y+=144;}
      m.multiply(new THREE.Matrix4().makeTranslation(x,y,0)).multiply(new THREE.Matrix4().makeRotationZ(ang)).multiply(new THREE.Matrix4().makeTranslation(-x,-y,0));
    }
    return new THREE.Matrix4().makeTranslation(...offset(p).toArray()).multiply(m);
  }
  function visibility(p){
    const n=p.name,stage=+$('stage').value,af=$('assemblyFilter').value;
    if(af!=='all'&&(af==='linkage'?!['crank','linkR','linkL'].includes(p.group):p.group!==af))return false;
    if(stage && stageOf(n)>stage)return false;
    if(isolated && selected!==n)return false;
    if(!$('showbelts').checked&&n.includes('C07'))return false;
    if(!$('showtray').checked&&n.includes('T01'))return false;
    if(!$('showcover').checked&&['outerCover','driveCover'].includes(explodedOwners.get(n)))return false;
    if(!$('showhardware').checked&&hardware(p))return false;
    if($('onlyprint').checked&&p.kind!=='print')return false;
    if(!$('showtop').checked&&explodedOwners.get(n)==='upperDeck')return false;
    return true;
  }
  const dimMat=new THREE.LineBasicMaterial({color:0x167b8b}),dimGeom=new THREE.BufferGeometry().setFromPoints([v5(-30,70,93),v5(30,70,93)]),dim=new THREE.Line(dimGeom,dimMat);scene.add(dim);
  const markers=[];
  for(const side of [-1,1])for(let i=0;i<6;i++){
    const m=new THREE.Mesh(new THREE.BoxGeometry(.5,2,40),new THREE.MeshBasicMaterial({color:0xf3b86e}));m.userData={side,i};m.visible=false;scene.add(m);markers.push(m);
  }
  function update(){
    needsRender=true;
    $('gapval').textContent=gap.toFixed(1)+' mm';$('gap').value=gap;$('theta').textContent=(crank(gap)*180/Math.PI).toFixed(1)+'°';$('expval').textContent=Math.round(explode*100)+'%';
    objects.forEach(o=>{o.matrix.copy(motion(o.userData.meta));o.matrixWorldNeedsUpdate=true;o.visible=visibility(o.userData.meta);});
    dim.geometry.setFromPoints([v5(-gap/2,75,87),v5(gap/2,75,87)]);dim.visible=!isolated&&explode===0&&+$('stage').value!==1&&+$('stage').value!==2;
    markers.forEach(m=>{const{side,i}=m.userData;m.position.set(side*(gap/2-.3),((i*24-phase*($('beltmode').value==='twist'?side:1))%144+144)%144+(side<0?14:0),(side<0?PARAMS.belt_z_left:PARAMS.belt_z_right)+PARAMS.belt_width/2);const af=$('assemblyFilter').value,stage=+$('stage').value;m.visible=belts&&explode===0&&$('showbelts').checked&&!isolated&&(af==='all'||af===(side<0?'L':'R'))&&(!stage||stage>=6);});
    updateBadge();
  }
  function updateBadge(){
    const q=v5(0,75,94).project(camera),r=host.getBoundingClientRect();$('gapbadge').style.left=((q.x+1)*r.width/2-25)+'px';$('gapbadge').style.top=((1-q.y)*r.height/2)+'px';$('gapbadge').textContent=gap.toFixed(1)+' mm';$('gapbadge').style.display=dim.visible?'block':'none';
  }
  function setView(view){
    const look=view==='cassette'?v5(40,35,40):v5(0,5,2);
    const dir={cassette:[190,310,190],home:[370,490,300],top:[0,0,800],front:[0,800,0],side:[800,0,0],rear:[0,-800,0]}[view]||[370,490,300];
    controls.enableDamping=false;camera.up.set(0,view==='top'?1:0,view==='top'?0:1);configureControls();
    controls.target.copy(look);camera.position.copy(look).add(v5(...dir));camera.zoom=view==='cassette'?1.4:1;camera.updateProjectionMatrix();camera.lookAt(look);controls.update();
    $('viewtitle').textContent=PARAMS.revision+' / '+view.toUpperCase()+' / ORTHOGRAPHIC';document.querySelectorAll('[data-view]').forEach(b=>b.classList.toggle('active',b.dataset.view===view));update();
  }
  function resize(){const r=host.getBoundingClientRect(),a=r.width/r.height;camera.left=-230*a;camera.right=230*a;camera.top=230;camera.bottom=-230;camera.updateProjectionMatrix();renderer.setSize(r.width,r.height);needsRender=true;}
  function partInfo(n){
    const p=byName.get(n);if(!p)return;
    selected=n;objects.forEach(o=>{if(o.material.emissive)o.material.emissive.setHex(o.name===n?0x3d2010:0x000000);});
    const size=(p.print_bbox_mm||p.bbox_mm).map(x=>x.toFixed(1)).join(' × ');
    $('partinfo').innerHTML='<b>'+n+'</b><br>'+(p.kind==='print'?'출력 부품 · '+p.material:'구매 부품 · '+p.spec)+'<br>'+size+' mm<br><span class="small">'+p.note+'</span>';
    update();
  }
  function list(){const q=$('search').value.toLowerCase();$('parts').replaceChildren();MANIFEST.filter(p=>p.kind==='print'&&(!q||p.name.toLowerCase().includes(q))).forEach(p=>{const b=document.createElement('button');b.textContent=p.name;b.onclick=()=>partInfo(p.name);$('parts').appendChild(b);});}
  const ray=new THREE.Raycaster();let down=null;
  renderer.domElement.addEventListener('pointerdown',e=>{down=[e.clientX,e.clientY];});
  renderer.domElement.addEventListener('pointerup',e=>{if(!down||Math.hypot(e.clientX-down[0],e.clientY-down[1])>5)return;const r=renderer.domElement.getBoundingClientRect();ray.setFromCamera(new THREE.Vector2((e.clientX-r.left)/r.width*2-1,-(e.clientY-r.top)/r.height*2+1),camera);const hit=ray.intersectObjects(objects.filter(o=>o.visible),false)[0];if(hit)partInfo(hit.object.name);});
  $('gap').oninput=e=>{gap=+e.target.value;cycle=false;$('cycle').textContent='개폐 재생';update();};
  $('explode').oninput=e=>{explode=+e.target.value;update();};
  $('cycle').onclick=()=>{cycle=!cycle;$('cycle').textContent=cycle?'개폐 정지':'개폐 재생';};
  $('beltrun').onclick=()=>{belts=!belts;$('beltrun').classList.toggle('active',belts);};
  $('beltmode').onchange=update;$('assemblyFilter').onchange=update;$('stage').onchange=update;document.querySelectorAll('input[type=checkbox]').forEach(el=>el.onchange=update);
  $('search').oninput=list;$('isolate').onclick=()=>{if(!selected)return;isolated=!isolated;$('isolate').classList.toggle('active',isolated);update();};
  $('clear').onclick=()=>{selected=null;isolated=false;$('isolate').classList.remove('active');objects.forEach(o=>o.material.emissive?.setHex(0));$('partinfo').textContent='부품을 클릭하면 재질, 규격, 출력 크기를 표시합니다.';update();};
  function reset(){$('search').value='';$('beltmode').value='infeed';list();gap=44;explode=0;cycle=false;belts=false;phase=0;$('explode').value=0;$('cycle').textContent='개폐 재생';$('beltrun').classList.remove('active');$('stage').value='0';$('assemblyFilter').value='all';document.querySelectorAll('input[type=checkbox]').forEach(el=>el.checked=el.id!=='onlyprint');$('clear').click();setView('home');update();}
  $('reset').onclick=reset;document.querySelectorAll('[data-view]').forEach(b=>b.onclick=()=>setView(b.dataset.view));
  $('snapshot').onclick=()=>{renderer.render(scene,camera);const a=document.createElement('a');a.href=renderer.domElement.toDataURL('image/png');a.download='inaho_gripper_v5_view.png';a.click();};
  new ResizeObserver(resize).observe(host);resize();
  const buf=Uint8Array.from(atob(MODEL),c=>c.charCodeAt(0)).buffer;
  new THREE.GLTFLoader().parse(buf,'',gltf=>{
    gltf.scene.traverse(obj=>{if(!obj.isMesh)return;const p=byGLTFName.get(obj.name);if(!p)return;const geo=obj.geometry.clone().scale(1000,1000,1000),mat=obj.material.clone();mat.side=THREE.DoubleSide;const mesh=new THREE.Mesh(geo,mat);mesh.name=p.name;mesh.userData.meta=p;mesh.matrixAutoUpdate=false;objects.push(mesh);scene.add(mesh);});
    $('loading').remove();$('printcount').textContent=MANIFEST.filter(p=>p.kind==='print').length+' + '+PARAMS.fit_coupon_count;$('partcount').textContent=objects.length;list();setView('home');
    window.GRIPPER_API={setGap:g=>{gap=Math.max(28,Math.min(96,g));update();},setExplode:e=>{explode=e;$('explode').value=e;update();},setView,setBeltPhase:v=>{phase=v;update();},setBeltMode:v=>{$('beltmode').value=v;update();},setAssembly:g=>{$('assemblyFilter').value=g;update();},select:partInfo,reset,setFlag:(id,v)=>{$(id).checked=v;update();},setStage:s=>{$('stage').value=s;update();},snapshot:()=>({renderer:webgl?'WebGL':'Canvas',webgl_version:webgl?renderer.getContext().getParameter(renderer.getContext().VERSION):null,projection:camera.type,cameraPosition:camera.position.toArray(),cameraTarget:controls.target.toArray(),body_height_difference_mm:PARAMS.beta,belt_z_ranges_mm:[[PARAMS.belt_z_left,PARAMS.belt_z_left+PARAMS.belt_width],[PARAMS.belt_z_right,PARAMS.belt_z_right+PARAMS.belt_width]],renderFrames,phase,rotor_matrices:objects.filter(o=>/H16/.test(o.name)).map(o=>o.matrix.elements.slice()),gap,crank_deg:crank(gap)*180/Math.PI,parts:objects.length,visible:objects.filter(o=>o.visible).length,explode,selected,link_length:45}),update};
    window.GRIPPER_API.partStates=()=>objects.map(o=>({name:o.name,group:o.userData.meta.group,assembly:explodedOwners.get(o.name),offset:explodedOffset(o.userData.meta,explode),hardware:hardware(o.userData.meta),visible:o.visible,matrix:o.matrix.elements.slice()}));
  },e=>{$('loading').textContent='모델을 불러오지 못했습니다: '+e.message;console.error(e);});
  function animate(t){const dt=Math.min((t-last)/1000,.1);last=t;controls.update();if(cycle||belts){if(cycle)gap=62+34*Math.sin(t*.0007);if(belts)phase+=20*dt;update();}if(needsRender&&t-lastRender>=32){updateBadge();renderer.render(scene,camera);needsRender=false;lastRender=t;renderFrames++;}requestAnimationFrame(animate);}requestAnimationFrame(animate);
})();
