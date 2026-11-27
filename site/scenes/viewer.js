import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';

const { Vector3, Quaternion } = THREE;
const UP = new Vector3(0, 1, 0);
const LEFT = new Vector3(0, 0, -1);
const FORWARD = new Vector3(1, 0, 0);
const color = {
  fixture: 0x919d9d, guard: 0x8b9e9e, terrain: 0x8f9e9d, dock: 0x9ba7a4,
  surface: 0x515d5d, actor: 0xb76d36, mast: 0x99a9aa, trim: 0xeeeade, edge: 0x6d7774,
  path: 0x1b8c77, pad: 0x5b7872, amber: 0xe2a54d,
};
const ZERO = new Vector3();

const xyz = (x, y, z) => new Vector3(x, z, -y);

function locate(samples, t) {
  let low = 0, high = samples.length - 1;
  while (low < high) {
    const mid = Math.ceil((low + high) / 2);
    if (samples[mid][0] <= t) low = mid;
    else high = mid - 1;
  }
  return low;
}

export function sampleAt(samples, t) {
  if (!samples.length) return null;
  const i = locate(samples, t);
  const a = samples[i], b = samples[i + 1];
  if (!b || t <= a[0]) return a;
  const f = (t - a[0]) / (b[0] - a[0]);
  return a.map((v, j) => {
    const delta = b[j] - v;
    return j === 4 || j === 5 ? v + Math.atan2(Math.sin(delta), Math.cos(delta)) * f : v + delta * f;
  });
}

export function latestAt(events, t) {
  return events.length && events[0][0] <= t ? events[locate(events, t)] : null;
}

function fit(track) {
  const { setup } = track;
  const points = track.samples.map(p => p.slice(1, 3));
  points.push(...setup.markers, ...setup.targets);
  for (const b of setup.boxes) {
    points.push([b.p[0] - b.d[0]/2, b.p[1] - b.d[1]/2], [b.p[0] + b.d[0]/2, b.p[1] + b.d[1]/2]);
  }
  const x = points.map(p=>p[0]), y = points.map(p=>p[1]);
  const x0 = Math.min(...x), x1 = Math.max(...x), y0 = Math.min(...y), y1 = Math.max(...y);
  return { cx: (x0 + x1)/2, cy:(y0+y1)/2, w:Math.max(1.63,x1-x0+.75), h:Math.max(1.43,y1-y0+.72) };
}

function markInstances(group) {
  group.traverse(obj => {
    if (obj.isMesh) { obj.castShadow = true; obj.receiveShadow = true; }
  });
  return group;
}

export class JanusViewer {
  constructor(host) {
    this.host = host;
    this.renderer = new THREE.WebGLRenderer({antialias:true,alpha:true, powerPreference:'high-performance'});
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio,2));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFShadowMap;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.2;
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.domElement.setAttribute('aria-label','Simulation robot and test fixture');
    host.prepend(this.renderer.domElement);
    this.scene = new THREE.Scene();
    this.sensorBackground = new THREE.Color(0x202320);
    const environment = new RoomEnvironment();
    const pmrem = new THREE.PMREMGenerator(this.renderer);
    this.scene.environment = pmrem.fromScene(environment, .055).texture;
    this.scene.environmentIntensity = .62;
    environment.dispose();
    pmrem.dispose();

    this.camera = new THREE.OrthographicCamera(-1, 1, 1, -1, .01, 40);
    this.sensor = new THREE.PerspectiveCamera(48, 1, .016, 20);
    this.controls = new OrbitControls(this.camera,this.renderer.domElement);
    this.controls.enableDamping = false;
    this.controls.zoomToCursor = true;
    this.controls.minZoom = .6;
    this.controls.maxZoom = 5;
    this.controls.minPolarAngle = .001;
    this.controls.maxPolarAngle = Math.PI/2 - .075;
    this.controls.addEventListener('change', () => this.render());
    const fill = new THREE.HemisphereLight(0xf6f7e9,0x5e6867,1.36);
    this.key = new THREE.DirectionalLight(0xfff3dc, 2.25);
    this.key.castShadow = true;
    this.key.shadow.mapSize.set(2048,2048);
    this.key.shadow.camera.near = .18;
    this.key.shadow.camera.far = 14;
    this.key.shadow.normalBias = .009;
    this.key.shadow.bias = -.00015;
    this.scene.add(fill,this.key,this.key.target);
    this.materials = new Map();
    this.view = 'overview';
    this.traceVisible = true;
    this.propsVisible = true;
    this.sectionVisible = true;
    this.sectionWalls = [];
    this.onResize = new ResizeObserver(()=>this.resize());
    this.onResize.observe(host);
  }

  async load() {
    const loader = new GLTFLoader();
    const textureLoader = new THREE.TextureLoader();
    const [kit, concrete] = await Promise.all([
      loader.loadAsync(new URL('./assets/janus-kit.glb', import.meta.url).href),
      textureLoader.loadAsync(new URL('./assets/concrete.png', import.meta.url).href),
    ]);
    this.kit = kit.scene;
    this.surface = concrete;
    concrete.colorSpace = THREE.SRGBColorSpace;
    concrete.wrapS = concrete.wrapT = THREE.RepeatWrapping;
    concrete.repeat.set(2.5,2.2);
    concrete.anisotropy = Math.min(8,this.renderer.capabilities.getMaxAnisotropy());
    for (const name of ['ROBOT','CAMERA_HEAD','DOCK_STAND','PAYLOAD','QR_PRIMARY','QR_SECONDARY','PROP_CONE','PROP_RACK','PROP_PALLET','PROP_STACK','PROP_JACK']) {
      if (!this.kit.getObjectByName(name)) throw new Error('Model parçası bulunamadı: '+name);
    }
  }

  resize() {
    const width = this.host.clientWidth, height = this.host.clientHeight;
    if (!width || !height) return;
    this.aspect = width / height;
    this.renderer.setSize(width,height,false);
    this.sensor.aspect = this.aspect;
    this.sensor.updateProjectionMatrix();
    this.updateFrustum();
    this.render();
  }

  updateFrustum() {
    const aspect = this.aspect || 1;
    const d = this.bounds;
    if (!d) return;
    const controlsMargin = this.host.clientWidth < 450 && !document.documentElement.dataset.embed ? 1.12 : 1;
    const span = this.view === 'top' ? Math.max(d.h+0.14,(d.w+0.12)/aspect) :
      Math.max(d.h*.58 + d.w*.37 + .46, (d.w*.84+d.h*.54+.25)/aspect)*1.06;
    const halfSpan = span * controlsMargin / 2;
    this.camera.top = halfSpan; this.camera.bottom = -halfSpan;
    this.camera.left = -halfSpan*aspect; this.camera.right = halfSpan*aspect;
    this.camera.updateProjectionMatrix();
  }

  setView(view) {
    this.view = view;
    this.controls.enabled = view !== 'sensor';
    this.scene.background = view === 'sensor' ? this.sensorBackground : null;
    this.applySection();
    this.camera.zoom = 1;
    if (!this.bounds) return;
    const {cx,cy,w,h} = this.bounds;
    this.controls.target.copy(xyz(cx, cy, .10));
    if (view === 'top') {
      this.camera.position.copy(xyz(cx,cy-.001,4.4));
      this.camera.up.set(0,1,0);
      this.controls.enableRotate = false;
    } else {
      this.camera.position.copy(xyz(cx-w*.69,cy-h*.96,Math.max(w,h)*.95));
      this.camera.up.set(0,1,0);
      this.controls.enableRotate = true;
    }
    this.updateFrustum();
    this.setPanMode(view === 'top');
    this.controls.update();
    this.render();
  }

  zoom(increment) {
    const camera = this.camera;
    camera.zoom = THREE.MathUtils.clamp(
      camera.zoom * 1.25 ** increment, this.controls.minZoom, this.controls.maxZoom
    );
    camera.updateProjectionMatrix();
    this.controls.update();
    this.render();
  }

  setPanMode(active) {
    this.controls.mouseButtons.LEFT = active ? THREE.MOUSE.PAN : THREE.MOUSE.ROTATE;
    this.controls.touches.ONE = active ? THREE.TOUCH.PAN : THREE.TOUCH.ROTATE;
  }

  pan(x, y) {
    this.controls.pan(x, y);
  }

  material(name) {
    if (!this.materials.has(name)) {
      const mat = new THREE.MeshStandardMaterial({
        color: name === 'concrete' ? 0xd7d8d0 : color[name],
        map: name === 'concrete' ? this.surface : null,
        roughness: ['terrain','dock','guard'].includes(name) ? .61 : .86,
        metalness: ['terrain','dock','guard'].includes(name) ? .22 : .03,
      });
      this.materials.set(name,mat);
    }
    return this.materials.get(name);
  }

  box(name, size, pos, style, parent=this.stage, rotation=[0,0,0]) {
    const [l,w,h] = size;
    const materials = style === 'concrete' ? Array.from({length:6},(_,i)=>this.material(i===2?'concrete':'edge')) : this.material(style);
    const ob = new THREE.Mesh(new THREE.BoxGeometry(l,h,w),materials);
    ob.name = name;
    ob.position.copy(xyz(...pos));
    ob.quaternion.setFromAxisAngle(UP,rotation[2])
      .multiply(new Quaternion().setFromAxisAngle(LEFT,rotation[1]))
      .multiply(new Quaternion().setFromAxisAngle(FORWARD,rotation[0]));
    ob.receiveShadow = true;
    ob.castShadow = style !== 'concrete';
    ob.userData.localMesh = true;
    parent.add(ob);
    return ob;
  }

  copy(name,parent) {
    const group = markInstances(this.kit.getObjectByName(name).clone(true));
    parent.add(group);
    return group;
  }

  placeProp(name, pos, scale, yaw=0) {
    const obj = this.copy('PROP_'+name.toUpperCase(),this.props);
    obj.position.copy(xyz(...pos)); obj.scale.setScalar(scale); obj.rotation.y = yaw;
  }

  line(points, size, style, parent) {
    if (points.length < 2) return;
    const count = Math.max(2,points.length-1);
    const mesh = new THREE.Mesh(new THREE.TubeGeometry(new THREE.CatmullRomCurve3(points),count,size,5,false),this.material(style));
    mesh.userData.localMesh = true;
    parent.add(mesh);
  }

  ring(x,y,r,style,parent=this.stage) {
    const geom = new THREE.RingGeometry(r-.008,r,64);
    const mesh = new THREE.Mesh(geom,this.material(style));
    mesh.rotation.x=-Math.PI/2;
    mesh.position.copy(xyz(x,y,.005));
    mesh.userData.localMesh = true;
    parent.add(mesh);
  }

  setupCase(meta, track) {
    this.track = track; this.caseId = meta.id;
    if (this.stage) {
      this.stage.traverse(obj => {
        if (obj.userData.localMesh && obj.geometry) obj.geometry.dispose();
        if (obj.userData.localMaterial) obj.material.dispose();
      });
      this.scene.remove(this.stage);
    }
    this.stage = new THREE.Group();
    this.stage.name='Case '+meta.id;
    this.scene.add(this.stage);
    this.props = new THREE.Group();
    this.path = new THREE.Group();
    this.stage.add(this.props,this.path);
    this.props.visible = this.propsVisible;
    this.path.visible = this.traceVisible;
    this.bounds = fit(track);
    const {cx,cy,w,h} = this.bounds;
    this.key.position.copy(xyz(cx-2.4,cy-3.1,5));
    this.key.target.position.copy(xyz(cx,cy,0));
    const shadowSpan = Math.max(w,h)*1.2;
    Object.assign(this.key.shadow.camera,{left:-shadowSpan,right:shadowSpan,bottom:-shadowSpan,top:shadowSpan});
    this.key.shadow.camera.updateProjectionMatrix();

    this.box('Concrete slab',[w,h,.078],[cx,cy,-.04],'concrete');
    this.box('Slab lower band',[w+.024,h+.024,.016],[cx,cy,-.085],'edge');
    for (const side of [-1,1]) {
      this.box('Floor trim',[w*.87,.011,.001],[cx,cy+side*(h/2-.051),.001],'trim');
      for (let step=0;step<5;step++) {
        const xx=cx+(step-2)*w*.126;
        this.box('Scale notch',[.003,.035,.001],[xx,cy+side*(h/2-.069),.0012],'trim');
      }
    }

    const {setup} = track;
    if (setup.border !== null) {
      const width = cx+w/2-setup.border;
      if (width>0) this.box('High drag zone',[width,h-.06,.004],[setup.border+width/2,cy,0],'surface');
    } else if (setup.surface.includes('high')) {
      this.box('High drag surface',[w-.10,h-.10,.004],[cx,cy,0],'surface');
    }

    if (setup.profile.length) {
      const [y0,y1] = setup.profileWidth;
      const vertices=[],indices=[];
      for (const [x,z] of setup.profile) for (const y of [y0,y1]) vertices.push(x,z+.00015,-y);
      for (let i=0;i<setup.profile.length-1;i++) indices.push(i*2,i*2+2,i*2+3,i*2,i*2+3,i*2+1);
      const g=new THREE.BufferGeometry();
      g.setAttribute('position',new THREE.Float32BufferAttribute(vertices,3));g.setIndex(indices);g.computeVertexNormals();
      const model=new THREE.Mesh(g,this.material('terrain'));model.receiveShadow=true;model.userData.localMesh=true;this.stage.add(model);
    }

    this.actor = null;
    this.sectionWalls = [];
    for (const b of setup.boxes) {
      const panel=this.box(b.name,b.d,b.p,b.style,this.stage,b.r);
      if (b.style==='actor' && setup.actor.length) this.actor=panel;
      if (b.style==='guard') {
        const cap=this.box('Wall top aluminium cap',[b.d[0],b.d[1],.004],[0,0,0],'trim',panel);
        cap.position.set(0,b.d[2]/2-.001,0);
        if (b.name.includes('south') || b.name.includes('west')) {
          this.sectionWalls.push({panel,cap});
        }
      }
    }

    this.markers=[];
    for (const data of setup.markers) {
      const [x,y,z,a,type,width]=data;
      const stand=this.copy('DOCK_STAND',this.stage);
      stand.position.copy(xyz(x,y,0));stand.scale.set(.68,z/.31,.68);stand.rotation.y=a;
      const mark=this.copy(type==='distractor'?'QR_SECONDARY':'QR_PRIMARY',this.stage);
      mark.scale.setScalar(width);
      mark.position.copy(xyz(x-.003*Math.cos(a),y-.003*Math.sin(a),z));
      mark.rotation.y=a;
      this.markers.push({data,mark,stand});
    }

    for (const [x,y] of setup.targets) {
      this.ring(x,y,.154,'trim');
      this.box('Survey cross line',[.31,.005,.002],[x,y,.003],'trim');
      this.box('Survey cross line',[.005,.31,.002],[x,y,.003],'trim');
    }

    if (!setup.boxes.length) {
      this.placeProp('cone',[cx+w*.41,cy-h*.41,0],.29);
      this.placeProp('cone',[cx-w*.41,cy+h*.41,0],.29);
    }
    if (setup.markers.length || meta.id.startsWith('O')) this.placeProp('rack',[cx+w*.37,cy+h*.36,0],.36);
    const placements={
      M10:[['stack',-.32,.30,.32]], O01:[['pallet',-.32,.32,.31]],
      O06:[['stack',-.28,.26,.36],['jack',.06,.31,.31]],
      O07:[['pallet',-.35,.27,.32],['jack',.04,.34,.32]],
    };
    for (const [role,dx,dy,scale] of placements[meta.id]||[]) this.placeProp(role,[cx+dx*w,cy+dy*h,0],scale);

    this.robot=this.copy('ROBOT',this.stage);
    this.head=this.copy('CAMERA_HEAD',this.robot);
    this.head.position.copy(xyz(...setup.camera));
    this.head.rotation.y=setup.cameraYaw;
    const [cameraX,cameraY,cameraZ]=setup.camera;
    const supportTop=cameraZ-.028, supportBottom=.110;
    if (supportTop>supportBottom+.004) {
      this.box('Camera mast base',[.051,.058,.007],[cameraX-.027,cameraY,.113],'edge',this.robot);
      for (const side of [-1,1]) {
        this.box('Camera mast rail',[.017,.011,supportTop-supportBottom],
          [cameraX-.026,cameraY+side*.017,(supportTop+supportBottom)/2],'mast',this.robot);
      }
    }
    this.joints=['CASTER_SWIVEL','WHEEL_L','WHEEL_R','CASTER_ROLL'].map(n=>this.robot.getObjectByName(n));
    if (setup.payload) {
      const p=this.copy('PAYLOAD',this.robot);
      p.position.copy(xyz(-.01,0,.124));
    }

    const raw=track.samples;
    const route=[xyz(raw[0][1],raw[0][2],Math.max(.006,raw[0][3]-.069))];
    for (const [,x,y,z] of raw) {
      const point=xyz(x,y,Math.max(.006,z-.069));
      if (route.at(-1).distanceToSquared(point)>.001**2) route.push(point);
    }
    if (route.length>1) {
      // Each segment follows the measured axle trace. A Line avoids rounding sharp turns.
      const g=new THREE.BufferGeometry().setFromPoints(route);
      const m=new THREE.LineBasicMaterial({color: color.path});
      const ob=new THREE.Line(g,m);
      ob.userData.localMesh=true;
      ob.userData.localMaterial=true;
      this.path.add(ob);
    }
    this.ring(raw[0][1],raw[0][2],.055,'path',this.path);
    this.ring(raw.at(-1)[1],raw.at(-1)[2],.072,'path',this.path);
    this.setView(this.view);
    this.setTime(0);
  }

  setSection(show) {
    this.sectionVisible=show;
    this.applySection();
    this.render();
  }

  applySection() {
    const enabled=this.sectionVisible && this.view==='overview';
    if (!this.wallSectionMaterial) {
      this.wallSectionMaterial=this.material('guard').clone();
      this.wallSectionMaterial.transparent=true;
      this.wallSectionMaterial.opacity=.26;
      this.wallSectionMaterial.depthWrite=false;
      this.wallCapSectionMaterial=this.material('trim').clone();
      this.wallCapSectionMaterial.transparent=true;
      this.wallCapSectionMaterial.opacity=.46;
      this.wallCapSectionMaterial.depthWrite=false;
    }
    for (const {panel,cap} of this.sectionWalls) {
      panel.material=enabled?this.wallSectionMaterial:this.material('guard');
      cap.material=enabled?this.wallCapSectionMaterial:this.material('trim');
      panel.castShadow=!enabled;
      cap.castShadow=!enabled;
    }
  }

  setTrace(show) {
    this.traceVisible=show;
    if (this.path) this.path.visible=show;
    this.render();
  }

  setProps(show) {
    this.propsVisible=show;
    if (this.props) this.props.visible=show;
    this.render();
  }

  setTime(t) {
    if (!this.robot) return;
    const p=sampleAt(this.track.samples,t);
    const [,x,y,z,yaw,steer,l,r,c,,pitch,roll] = p;
    this.robot.position.copy(xyz(x,y,z));
    this.robot.quaternion.setFromAxisAngle(UP,yaw)
      .multiply(new Quaternion().setFromAxisAngle(LEFT,pitch))
      .multiply(new Quaternion().setFromAxisAngle(FORWARD,roll));
    this.joints[0].rotation.y=steer;
    this.joints[1].rotation.z=-l;this.joints[2].rotation.z=-r;this.joints[3].rotation.z=-c;
    if (this.actor) {
      const q=sampleAt(this.track.setup.actor,t);
      if (q) this.actor.position.copy(xyz(q[1],q[2],q[3]));
    }
    if (this.caseId==='Q06' || this.caseId==='Q07') {
      const changes=this.track.events.filter(e=>['hide','restore_service_ack','visible_loss','qr_lost','lost'].includes(e[1]));
      const state=latestAt(changes,t)?.[1];
      const haveMarker=!state || state==='restore_service_ack';
      for (const {data, mark, stand} of this.markers) {
        mark.visible=haveMarker;
        if (this.caseId==='Q07') {
          const restore=this.track.events.find(e=>e[1]==='restore_service_ack');
          const y=restore&&t>=restore[0] ? .12 : data[1];
          stand.position.z=-y;mark.position.z=-y;
        }
      }
    }
    if (this.caseId==='Q09' && this.markers.length>1) {
      const event=latestAt(this.track.events,t);
      const show=event?.[1]==='duplicate_ids';
      this.markers[1].mark.visible=show;
      this.markers[1].stand.visible=show;
    }
    this.robot.updateMatrixWorld(true);
    this.sensor.position.copy(this.head.getWorldPosition(ZERO.clone()));
    const direction=this.head.getWorldQuaternion(new Quaternion());
    this.sensor.up.copy(UP).applyQuaternion(direction);
    this.sensor.lookAt(FORWARD.clone().applyQuaternion(direction).add(this.sensor.position));
    this.render();
    return p;
  }

  render() {
    if (!this.stage || !this.aspect) return;
    this.renderer.render(this.scene,this.view==='sensor'?this.sensor:this.camera);
  }
}
