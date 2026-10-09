/* CPU compatibility renderer for environments without WebGL.
 * Uses THREE geometry/camera transforms. This is not part of Three.js itself.
 * Depth-buffered rasterization (approximate shading); intended for inspection fallback.
 */
class EngineeringCanvasRenderer {
 constructor(){this.domElement=document.createElement('canvas');this.ctx=this.domElement.getContext('2d');this.shadowMap={};this.isSoftware=true;this.ratio=1;this.last=0;this.cache=new WeakMap();}
 setPixelRatio(r){this.ratio=1;}
 setSize(w,h){w=Math.max(1,Math.round(w));h=Math.max(1,Math.round(h));this.w=w;this.h=h;this.domElement.width=Math.round(w*this.ratio);this.domElement.height=Math.round(h*this.ratio);this.domElement.style.width=w+'px';this.domElement.style.height=h+'px';this.last=0;}
 getContext(){return this.ctx;}
 render(scene,camera){
  this.last=performance.now();
  if(!this.w||!this.h)return;
  const ctx=this.ctx,w=this.w,h=this.h;ctx.setTransform(this.ratio,0,0,this.ratio,0,0);ctx.fillStyle='#e9eef0';ctx.fillRect(0,0,w,h);
  scene.updateMatrixWorld(true);camera.updateMatrixWorld(true);camera.matrixWorldInverse.copy(camera.matrixWorld).invert();
  const vp=new THREE.Matrix4().multiplyMatrices(camera.projectionMatrix,camera.matrixWorldInverse);
  function project(v){const q=v.clone().applyMatrix4(vp);return [(q.x+1)*w/2,(1-q.y)*h/2,q.z];}
  // Ground plane grid, purely a drawing aid.
  ctx.strokeStyle='#d5dfe2';ctx.lineWidth=.55;
  for(let k=-500;k<=500;k+=20){
   for(const pair of [[[k,-500,-159],[k,500,-159]],[[-500,k,-159],[500,k,-159]]]){
    const a=project(new THREE.Vector3(...pair[0])),b=project(new THREE.Vector3(...pair[1]));
    if(a[2]>1||b[2]>1||a[2]<-1||b[2]<-1)continue;
    ctx.beginPath();ctx.moveTo(a[0],a[1]);ctx.lineTo(b[0],b[1]);ctx.stroke();
   }
  }
  const shadow=project(new THREE.Vector3(5,0,-159));
  const grd=ctx.createRadialGradient(shadow[0],shadow[1],5,shadow[0],shadow[1],Math.min(w,h)*.29);grd.addColorStop(0,'rgba(60,85,93,.13)');grd.addColorStop(1,'rgba(60,85,93,0)');ctx.fillStyle=grd;ctx.fillRect(0,0,w,h);
  const faces=[],light=new THREE.Vector3(.25,-.5,1).normalize();
  const nmat=new THREE.Matrix3(),proj=new THREE.Matrix4();
  const a=new THREE.Vector3(),b=new THREE.Vector3(),c=new THREE.Vector3(),n=new THREE.Vector3();
  scene.traverseVisible(o=>{
   if(!o.isMesh||!o.geometry||o.geometry.type==='PlaneGeometry')return;
   const geometry=o.geometry,attr=geometry.getAttribute('position'),nor=geometry.getAttribute('normal');if(!attr)return;
   const ix=geometry.index?.array;const mat=Array.isArray(o.material)?o.material[0]:o.material;if(!mat)return;
   proj.multiplyMatrices(vp,o.matrixWorld);nmat.getNormalMatrix(o.matrixWorld);
   let info=this.cache.get(geometry);
   if(!info){info={screen:new Float32Array(attr.count*3)};this.cache.set(geometry,info);}
   const xyz=info.screen;
   for(let i=0;i<attr.count;i++){a.fromBufferAttribute(attr,i).applyMatrix4(proj);xyz[3*i]=(a.x+1)*w/2;xyz[3*i+1]=(1-a.y)*h/2;xyz[3*i+2]=a.z;}
   const base=(mat.color||new THREE.Color(.5,.5,.5)).clone();if(o.name&&o.name.length)base.convertLinearToSRGB();if(mat.emissive)base.add(mat.emissive);
   const count=ix?ix.length:attr.count;
   for(let j=0;j<count;j+=3){
    const i0=ix?ix[j]:j,i1=ix?ix[j+1]:j+1,i2=ix?ix[j+2]:j+2;
    const x0=xyz[3*i0],y0=xyz[3*i0+1],z0=xyz[3*i0+2],x1=xyz[3*i1],y1=xyz[3*i1+1],z1=xyz[3*i1+2],x2=xyz[3*i2],y2=xyz[3*i2+1],z2=xyz[3*i2+2];
    if(z0>1||z1>1||z2>1||z0<-1||z1<-1||z2<-1)continue;
    if(Math.max(x0,x1,x2)<0||Math.min(x0,x1,x2)>w||Math.max(y0,y1,y2)<0||Math.min(y0,y1,y2)>h)continue;
    const area=(x1-x0)*(y2-y0)-(y1-y0)*(x2-x0);
    if(mat.side!==THREE.DoubleSide&&area>=0)continue;
    if(Math.abs(area)<.035)continue;
    if(nor){n.fromBufferAttribute(nor,i0);a.fromBufferAttribute(nor,i1);b.fromBufferAttribute(nor,i2);n.add(a).add(b).multiplyScalar(1/3).applyMatrix3(nmat).normalize();}
    else{a.fromBufferAttribute(attr,i0);b.fromBufferAttribute(attr,i1);c.fromBufferAttribute(attr,i2);n.crossVectors(b.sub(a),c.sub(a)).applyMatrix3(nmat).normalize();}
    const shade=.56+.44*Math.max(0,n.dot(light));
    const rr=Math.min(255,Math.round(base.r*255*shade)),gg=Math.min(255,Math.round(base.g*255*shade)),bb=Math.min(255,Math.round(base.b*255*shade));
    faces.push([x0,y0,z0,x1,y1,z1,x2,y2,z2,rr,gg,bb]);
   }
  });
  const image=ctx.getImageData(0,0,w,h), pixels=image.data;
  if(!this.depth||this.depth.length!==w*h)this.depth=new Float32Array(w*h);
  const depth=this.depth;depth.fill(Infinity);
  for(const f of faces){
   const [x0,y0,z0,x1,y1,z1,x2,y2,z2,rr,gg,bb]=f;
   const area=(x1-x0)*(y2-y0)-(y1-y0)*(x2-x0);if(Math.abs(area)<1e-6)continue;
   const A=(y1-y2)/area,B=(x2-x1)/area,C=(x1*y2-y1*x2)/area;
   const D=(y2-y0)/area,E=(x0-x2)/area,F=(x2*y0-y2*x0)/area;
   const xmin=Math.max(0,Math.floor(Math.min(x0,x1,x2))),xmax=Math.min(w-1,Math.ceil(Math.max(x0,x1,x2)));
   const ymin=Math.max(0,Math.floor(Math.min(y0,y1,y2))),ymax=Math.min(h-1,Math.ceil(Math.max(y0,y1,y2)));
   for(let yy=ymin;yy<=ymax;yy++){
    let wa=A*(xmin+.5)+B*(yy+.5)+C,wb=D*(xmin+.5)+E*(yy+.5)+F,ind=yy*w+xmin;
    for(let xx=xmin;xx<=xmax;xx++,ind++,wa+=A,wb+=D){
     const wc=1-wa-wb;if(wa<-.0002||wb<-.0002||wc<-.0002)continue;
     const z=wa*z0+wb*z1+wc*z2;if(z>=depth[ind])continue;depth[ind]=z;
     const n=ind*4;pixels[n]=rr;pixels[n+1]=gg;pixels[n+2]=bb;pixels[n+3]=255;
    }
   }
  }
  ctx.putImageData(image,0,0);
  ctx.fillStyle='#647b83';ctx.font='10px Arial';ctx.fillText('Canvas compatibility renderer / approximate shading',18,h-86);
 }
}
