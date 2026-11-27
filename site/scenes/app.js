import { JanusViewer, sampleAt, latestAt } from './viewer.js';

const $ = id => document.getElementById(id);
const query = selector => [...document.querySelectorAll(selector)];
const filters = query('[data-filter]');
const views = query('[data-view]');
const categories = { M:'Motion', Q:'QR docking', O:'Operations' };
const runLabels = {
  'Kayıtlı koşu':'Recorded run', 'Canlı fren':'Live braking', 'Python fren basamağı':'Brake step',
  'Dar alanda düzeltilen rota':'Revised corridor route', 'İlk rota · temas':'Original route · wall contact',
  '90° · yerinde dönüş':'90° caster · pivot', '180° · yerinde dönüş':'180° caster · pivot',
  '180° · ileri komutsuz kontrol':'180° caster · zero forward request', 'Nominal kütle':'Nominal mass',
  'Yüzey sınırı':'Surface boundary', 'Nominal zemin':'Nominal surface', 'Yüksek sürükleme':'High drag surface',
  'Pahlı 5 mm eşik':'Beveled 5 mm threshold', 'Rampa ve 5 mm eşik':'Ramp and 5 mm threshold',
  '20 mm duyarlılık':'20 mm threshold', 'Dar koridor':'Narrow corridor', 'Açık arka çıkış':'Open rear exit',
  'İzinli devam':'Restart authorized', 'İzin verilmedi':'Restart withheld',
  'Komut watchdog':'Command timeout', 'Operatör iptali':'Operator cancellation',
  'Odom geri geldi · yeni izin':'Odometry restored · new authorization', 'Odometri kesintisi':'Odometry interrupted',
  'Rijit 4 kg yük':'Fixed 4 kg payload', 'Saat geri geldi · yeni izin':'Clock restored · new authorization',
  'Dört köşe ile hazırlık':'Four corners with staging', 'Merkezleme + menzil':'Center and range',
  'Soldan sınırlı arama':'Bounded search from left', 'Sağdan sınırlı arama':'Bounded search from right',
  'İşaret geri gelir':'Marker returns', 'Yanlış kimlik · zaman aşımı':'Wrong ID · timeout',
  'Akış kopması ve tekrar':'Camera interruption and repeat', 'Geri zaman damgası':'Backward timestamp',
  'Dock koridorunda engel':'Obstacle in the dock corridor', 'BMS teyidi gelmez':'No BMS confirmation',
  'Sanal BMS teyidi':'Virtual BMS confirmation', 'Yanal ve açılı QR':'Offset and angled QR',
  'Kamera ve hedef sütunu':'Camera and target offsets', 'Sönümlü 55 ms denetim':'55 ms · damped controller',
  '55 ms referans':'55 ms · reference controller',
};
const poseLabels = {
  'Python aks pozu':'Python axle pose', 'Simscape aks pozu':'Simscape axle pose',
  'Dünya pozu':'World pose', 'Replay dünya pozu':'Replayed world pose',
};
const eventLabels = {
  case_start:'Run started', trace_start:'Recording started', trace_end:'Recording ended',
  acquired:'QR detected', center:'Centering', advance:'Approaching', stage_align:'Aligning before approach',
  stage_turn_to_position:'Turn toward approach position', stage_translate:'Move to approach position',
  stage_turn_to_dock:'Turn toward dock', optical_center:'Center in camera', case_end:'Run ended',
  terminal_gate_verified:'End criteria checked', final_brake:'Final brake', no_qr:'QR out of view',
  fresh_matching_id:'Matching QR seen', brake_step:'Brake command', terminal:'Run ended',
  terminal_hold:'Stopped at target', standoff:'Stopped at optical range', optical_stop:'Stopped at optical range',
  visible_loss:'QR out of view', qr_lost:'QR lost', lost:'QR lost', hide:'Marker removed',
  restore:'Marker restored', restore_service_ack:'Marker position updated',
  contact:'Contact', virtual_contact:'Simulated contact', charge_confirmed:'Charge confirmed in simulation',
  duplicate_ids:'Duplicate markers', wrong_id:'Wrong marker', timeout:'Timed out',
  search_timeout:'Search timed out', search:'Searching', reacquired:'QR reacquired',
  stop:'Stopped', guard_stop:'Guard stopped motion', hold:'Holding', fault:'Fault',
  odom_stale:'Odometry stale', odom_fresh:'Odometry fresh',
  operator_authorize:'Operator authorization', unauthorized:'Command rejected',
};
const fmt = new Intl.NumberFormat('en-GB',{minimumFractionDigits:2,maximumFractionDigits:2});
const short = new Intl.NumberFormat('en-GB',{maximumFractionDigits:1});
const eventName = key => eventLabels[key] || String(key).replaceAll('_',' ');
const embedded = document.documentElement.dataset.embed === 'true';
const assetVersion = new URL(import.meta.url).search;
const compactLayout = matchMedia('(max-width: 760px)');
const cache=new Map();
let list=[],filter='',current=null,track=null,viewer=null,view='overview',renderError=null,requestNumber=0;
let time=0, direction=0, lastTick=null, requestFrame=0, activeEvent=null, recording=null;
let engineError=null, pendingId=null;
const viewport=$('viewport');
const catalog=$('catalog');
const cameraTools=query('.camera-tools button:not(#fullscreen)');
const playbackTools=query('.transport button, .transport select, .transport input');
const caseTitle=c=>c.titleEn || c.title;
const video=$('camera-video');
const cameraInput=$('camera-input');
const sensorButton=views.find(button => button.dataset.view === 'sensor');

function button(text, classes) {
  const node=document.createElement('button');
  node.type='button';node.className=classes;node.textContent=text;
  return node;
}

function renderCatalog() {
  const search=$('search').value.trim().toLocaleLowerCase('en-GB');
  const matches=list.filter(c => (!filter || c.id.startsWith(filter)) &&
    (!search || [c.id, caseTitle(c), c.title, c.environment].join(' ').toLocaleLowerCase('en-GB').includes(search)));
  $('case-count').textContent=matches.length+' '+(matches.length===1?'case':'cases');
  $('case-list').replaceChildren();
  if (!matches.length) {
    const empty=document.createElement('p');
    empty.className='empty';empty.textContent='No cases match this search.';
    $('case-list').append(empty);
  }
  for (const c of matches) {
    const b=button('','case-row');
    b.dataset.case=c.id;
    const id=document.createElement('span'),name=document.createElement('span');
    id.className='mono'; id.textContent=c.id; name.textContent=caseTitle(c);
    b.setAttribute('aria-current', String(c.id===current?.meta.id));
    b.append(id,name);
    b.addEventListener('click',()=>{
      catalog.close();
      $('open-catalog').focus();
      selectCase(c.id,0,true);
    });
    $('case-list').append(b);
  }
}

function updateCaseNavigation(id) {
  const index = list.findIndex(item => item.id === id);
  $('case-position').textContent = (index + 1) + ' / ' + list.length;
  $('prev-case').disabled = index <= 0;
  $('next-case').disabled = index >= list.length - 1;
}

function shiftCase(step) {
  const index=list.findIndex(c=>c.id===(pendingId || current?.meta.id));
  const next=list[index+step];
  if (next) selectCase(next.id,0,true);
}

function setCameraTools() {
  cameraTools.forEach(control => {control.disabled=Boolean(renderError || !track || view==='sensor');});
}
async function getJson(path) {
  const result=await fetch(path+assetVersion);
  if (!result.ok) throw new Error(path+' ('+result.status+')');
  return result.json();
}

function syncAddress(replace=false) {
  const link=new URL(window.location.href);
  link.searchParams.set('vaka',current.meta.id);
  const variant=+$('variant').value;
  if (variant) link.searchParams.set('kosu', String(variant+1));
  else link.searchParams.delete('kosu');
  history[replace?'replaceState':'pushState'](null,'',link);
}

async function selectCase(id,startVariant=0,update=false) {
  const entry=list.find(item=>item.id===id);
  if (!entry) return;
  const request=++requestNumber;
  pendingId=id;
  pause();
  if (recording) stopRecording();
  track=null;
  viewport.setAttribute('aria-busy','true');
  $('tracking-status').textContent='Loading '+id+'…';
  $('viewer-fallback').hidden=true;
  playbackTools.forEach(control=>{control.disabled=true;});
  views.forEach(control=>{control.disabled=true;});
  $('variant').disabled=true;
  const clip='clips/'+id+'.mp4'+assetVersion;
  $('clip-link').href=clip;
  $('fallback-clip').href=clip;
  $('result-link').href='../results.html#case-'+id;
  setCameraTools();
  if (!embedded) updateCaseNavigation(id);
  if (!cache.has(id)) cache.set(id,getJson('./data/'+id+'.json'));
  try {
    const next=await cache.get(id);
    if (request!==requestNumber) return;
    current=next;
    renderError=engineError;
    const {meta,tracks}=current;
    for (const [target,content] of [['stage-case',id],['case-title',caseTitle(entry)],['case-note',entry.noteEn || ''],['environment',meta.environment],['category',categories[id[0]]],['scenario',meta.scenario]]) $(target).textContent=content;
    document.title=id+' · '+caseTitle(entry)+' · Arche';
    $('variant').replaceChildren();
    tracks.forEach((t,i)=>{
      const o=document.createElement('option');
      o.value=i;
      o.textContent=(runLabels[t.name] || t.name)+' / '+t.environment;
      $('variant').append(o);
    });
    $('variant').disabled=tracks.length===1;
    $('variant').value=Math.max(0,Math.min(tracks.length-1,startVariant));
    const proof=$('published');
    proof.replaceChildren();
    for (const entry of meta.archive) {
      const p=document.createElement('p'),strong=document.createElement('strong');
      strong.textContent=entry.name+' — ';
      p.append(strong,document.createTextNode(entry.text));
      proof.append(p);
    }
    const p=document.createElement('p');p.textContent=meta.acceptance;proof.append(p);
    if (!embedded && catalog.open) renderCatalog();
    installTrack();
    if (update) syncAddress();
  } catch (error) {
    cache.delete(id);
    if (request!==requestNumber) return;
    showRenderError(error, true);
    $('retry-case').onclick=()=>selectCase(id,startVariant,update);
  } finally {
    if (request===requestNumber) {
      pendingId=null;
      viewport.setAttribute('aria-busy','false');
    }
  }
}

function showRenderError(error, retry=false) {
  renderError=error;
  delete viewport.dataset.loaded;
  $('fallback-message').textContent=retry?'This case could not load.':'3D is unavailable on this device.';
  $('viewer-fallback').hidden=false;
  $('retry-case').hidden=!retry;
  $('tracking-status').textContent='';
  $('record').disabled=true;
  setCameraTools();
  console.error(error);
}

function installTrack() {
  pause();
  if (recording) stopRecording();
  track=current.tracks[+$('variant').value];
  time=0;
  $('viewer-fallback').hidden=true;
  renderError=engineError;
  playbackTools.forEach(control=>{control.disabled=false;});
  $('record').disabled=Boolean(engineError || !window.MediaRecorder || !viewer?.renderer.domElement.captureStream);
  views.forEach(control=>{control.disabled=Boolean(engineError);});
  activeEvent=null;
  $('current-event').textContent='—';
  $('origin').textContent=poseLabels[track.basis] || track.basis;
  $('raw-path').textContent=track.sourceSha256;
  $('fixture-path').textContent=track.setup.fixtureProvenance;
  const fixtureNote = $('fixture-note');
  fixtureNote.hidden = !track.setup.fixtureNote;
  fixtureNote.textContent = track.setup.fixtureNote || '';
  const clip = track.media || 'clips/' + current.meta.id + '.mp4' + assetVersion;
  $('clip-link').href = clip;
  $('clip-link').textContent = track.media ? 'Camera input MP4 ↗' : 'MP4 ↗';
  $('fallback-clip').href = clip;
  sensorButton.textContent = track.media ? 'Camera input' : 'Camera';
  $('seek').max=track.end;
  $('event-count').textContent=track.events.length || '';
  $('show-section').hidden=!track.setup.boxes.some(b=>b.style==='guard');
  $('events').replaceChildren();
  $('event-marks').replaceChildren();
  for (const e of track.events) {
    const node=button('','event-row');
    node.dataset.time=e[0];
    const t=document.createElement('time'),msg=document.createElement('span');
    t.textContent=fmt.format(e[0])+' s';msg.textContent=eventName(e[1]);
    node.append(t,msg);
    node.title=e[2]?eventName(e[2]):eventName(e[1]);
    node.addEventListener('click',()=>{pause();seek(e[0]);});
    $('events').append(node);
    const marker=document.createElement('i');
    marker.style.left=(e[0]*100/track.end)+'%';
    $('event-marks').append(marker);
  }
  video.pause();
  video.removeAttribute('src');
  if (!cameraInput.hidden) video.load();
  cameraInput.hidden = true;
  if (viewer && !renderError) {
    try {
      viewer.setupCase(current.meta,track);
      setView(view);
      viewport.dataset.loaded=current.meta.id+'-'+$('variant').value;
    } catch (error) {
      showRenderError(error);
    }
  } else if (engineError) {
    showRenderError(engineError);
  }
  if (renderError) {
    playbackTools.forEach(control=>{control.disabled=true;});
    views.forEach(control=>{control.disabled=true;});
  }
  seek(embedded ? track.end*.64 : 0);
}

function seek(t) {
  if (!track) return;
  time=Math.max(0,Math.min(track.end,t));
  const p=(!renderError&&viewer?.setTime(time)) || sampleAt(track.samples,time);
  $('seek').value=String(time);
  $('time').textContent=fmt.format(time)+' / '+fmt.format(track.end)+' s';
  $('seek').setAttribute('aria-valuetext',fmt.format(time)+' seconds');
  if (p) {
    $('axis').textContent=fmt.format(p[1])+' / '+fmt.format(p[2])+' m';
    const angle=theta=>short.format(Math.atan2(Math.sin(theta),Math.cos(theta))*180/Math.PI)+'°';
    $('angle').textContent=angle(p[4])+' / '+(track.jointSignals.includes('caster')?angle(p[5]):'—');
    $('velocity').textContent=fmt.format(p[9])+' m/s';
  }
  const e=latestAt(track.events,time);
  if (e!==activeEvent) {
    activeEvent=e;
    $('current-event').textContent=e?eventName(e[1]):'—';
    const index=track.events.indexOf(e);
    [...$('events').children].forEach((b,i)=>b.setAttribute('aria-current',String(i===index)));
  }
  if (!renderError) {
    const qr=latestAt(track.qr,time);
    let state=track.environment;
    if (qr && (track.setup.markers.length || track.media)) state=qr[1] ? 'QR · '+(qr[2]===null?'valid':short.format(qr[2])+' px') : 'QR · '+eventName(qr[3]||'invalid');
    const markerShift = latestAt(track.setup.markerTimeline || [],time);
    if (markerShift && markerShift[1] === null) state = 'Test fixture · tag hidden';
    else if (markerShift) state = 'Test fixture · tag returned 120 mm left';
    $('tracking-status').textContent=state;
  }
}

function tick(now) {
  if (!direction) return;
  if (lastTick!==null) seek(time+Math.min(.10,(now-lastTick)/1000)*(+$('speed').value)*direction);
  lastTick=now;
  if ((direction>0 && time>=track.end) || (direction<0 && time<=0)) {
    pause();
    if (recording) stopRecording();
    return;
  }
  requestFrame=requestAnimationFrame(tick);
}

function play(nextDirection=1) {
  if (!track || renderError || !cameraInput.hidden) return;
  if (direction===nextDirection) { pause(); return; }
  pause();
  if (nextDirection>0 && time>=track.end) seek(0);
  if (nextDirection<0 && time<=0) seek(track.end);
  direction=nextDirection;
  $('play').setAttribute('aria-label',direction===1?'Pause':'Play');
  $('reverse').setAttribute('aria-pressed',String(direction===-1));
  requestFrame=requestAnimationFrame(tick);
}

function pause() {
  direction=0;lastTick=null;
  cancelAnimationFrame(requestFrame);
  $('play').setAttribute('aria-label','Play');
  $('reverse').setAttribute('aria-pressed','false');
}

function setView(next) {
  view=next;
  const cameraReplay = view === 'sensor' && Boolean(track?.media);
  cameraInput.hidden = !cameraReplay;
  if (cameraReplay) {
    pause();
    if (!video.getAttribute('src')) video.src = track.media;
  } else {
    video.pause();
  }
  document.getElementById('scene').classList.toggle('camera-input-view',cameraReplay);
  playbackTools.forEach(control => {control.disabled = cameraReplay || Boolean(renderError);});
  viewer?.setView(view);
  for (const b of views) b.setAttribute('aria-pressed',String(b.dataset.view===view));
  $('pan-mode').setAttribute('aria-pressed',String(view==='top'));
  viewport.classList.toggle('is-panning',view==='top');
  $('gesture-help').textContent=view==='sensor'?'View from the simulated robot camera':
    view==='top'?'Drag to pan · scroll to zoom':'Drag to orbit · pinch or scroll to zoom';
  setCameraTools();
  $('pan-mode').disabled=view!=='overview' || Boolean(renderError);
}

function stopRecording() {
  if (recording?.recorder.state==='recording') recording.recorder.stop();
}

function record() {
  if (recording) { stopRecording(); return; }
  if (renderError || !track || !cameraInput.hidden || !window.MediaRecorder || !viewer.renderer.domElement.captureStream) return;
  pause();seek(0);
  const stream=viewer.renderer.domElement.captureStream(30);
  const format=['video/webm;codecs=vp9','video/webm;codecs=vp8','video/mp4'].find(type=>MediaRecorder.isTypeSupported(type));
  const recorder=new MediaRecorder(stream,format?{mimeType:format,videoBitsPerSecond:4500000}:undefined);
  const chunks=[];
  const id=current.meta.id, variant=+$('variant').value+1;
  recording={recorder,stream};
  recorder.ondataavailable=e=>{if(e.data.size) chunks.push(e.data);};
  function endRecording() {
    stream.getTracks().forEach(t=>t.stop());
    recording=null;
    $('record').textContent='Record viewport';
    $('record').setAttribute('aria-pressed','false');
  }
  recorder.onerror=()=>{
    endRecording();
    $('tracking-status').textContent='Recording could not start';
  };
  recorder.onstop=()=>{
    if (chunks.length) {
      const type=recorder.mimeType||chunks[0].type;
      const url=URL.createObjectURL(new Blob(chunks,{type}));
      const a=document.createElement('a');
      a.href=url;a.download='JANUS-'+id+'-kosu-'+variant+(type.includes('mp4')?'.mp4':'.webm');
      a.click();
      setTimeout(()=>URL.revokeObjectURL(url),3000);
    }
    endRecording();
  };
  recorder.start(250);
  $('record').textContent='Stop recording';
  $('record').setAttribute('aria-pressed','true');
  play();
}

function openCatalog() {
  renderCatalog();
  catalog.showModal();
}
$('open-catalog').addEventListener('click',openCatalog);
const runDialog = $('run-dialog');
const runInspector = document.querySelector('.inspector');
function placeInspector() {
  if (compactLayout.matches) {
    $('run-sheet').append(runInspector);
  } else {
    if (runDialog.open) runDialog.close();
    document.querySelector('.lab-layout').append(runInspector);
  }
}
if (!embedded) {
  placeInspector();
  compactLayout.addEventListener('change', placeInspector);
  $('show-inspector').addEventListener('click', () => runDialog.showModal());
}
$('prev-case').addEventListener('click',()=>shiftCase(-1));
$('next-case').addEventListener('click',()=>shiftCase(1));
for (const b of filters) b.addEventListener('click',()=>{
  filter=b.dataset.filter;
  for (const a of filters) a.setAttribute('aria-pressed',String(a===b));
  renderCatalog();
  $('case-list').scrollTop=0;
});
$('search').addEventListener('input',renderCatalog);
for(const b of views) b.addEventListener('click',()=>setView(b.dataset.view));
for(const [name,action] of [['show-trace','setTrace'],['show-props','setProps'],['show-section','setSection']]) {
  $(name).addEventListener('click',e=>{
    const show=e.currentTarget.getAttribute('aria-pressed')!=='true';
    e.currentTarget.setAttribute('aria-pressed',String(show));
    viewer?.[action](show);
  });
}
$('play').addEventListener('click',()=>play());
$('reverse').addEventListener('click',()=>play(-1));
$('restart').addEventListener('click',()=>{pause();seek(0);});
$('step-back').addEventListener('click',()=>{pause();seek(time-.25);});
$('step-forward').addEventListener('click',()=>{pause();seek(time+.25);});
$('seek').addEventListener('input',e=>{pause();seek(+e.target.value);});
$('variant').addEventListener('change',()=>{installTrack();syncAddress();});
$('record').addEventListener('click',record);
$('zoom-in').addEventListener('click',()=>viewer?.zoom(1));
$('zoom-out').addEventListener('click',()=>viewer?.zoom(-1));
$('fit-camera').addEventListener('click',()=>setView(view));
$('pan-mode').addEventListener('click',()=>{
  const active=$('pan-mode').getAttribute('aria-pressed')!=='true';
  $('pan-mode').setAttribute('aria-pressed',String(active));
  viewport.classList.toggle('is-panning',active);
  viewer?.setPanMode(active);
  $('gesture-help').textContent=active?'Drag to pan · scroll to zoom':'Drag to orbit · scroll to zoom · right-drag to pan';
});
const replay=document.querySelector('.replay');
if (document.fullscreenEnabled && replay.requestFullscreen) {
  $('fullscreen').addEventListener('click',async()=>{
    try {
      if (document.fullscreenElement) await document.exitFullscreen();
      else await replay.requestFullscreen();
    } catch {
      $('tracking-status').textContent='Full screen is unavailable';
    }
  });
  document.addEventListener('fullscreenchange',()=>{
    const label=document.fullscreenElement?'Exit full screen':'Full screen';
    $('fullscreen').setAttribute('aria-label',label);
    $('fullscreen').title=label;
  });
} else {
  $('fullscreen').hidden=true;
}
const panKeys={ArrowLeft:[26,0],ArrowRight:[-26,0],ArrowUp:[0,26],ArrowDown:[0,-26]};
viewport.addEventListener('keydown',e=>{
  if (e.target!==viewport) return;
  if (e.code==='Space') { e.preventDefault(); play(); return; }
  if (!viewer || renderError || view==='sensor') return;
  if (view !== 'robot' && panKeys[e.key]) viewer.pan(...panKeys[e.key]);
  else if (['+', '=','-'].includes(e.key)) viewer.zoom(e.key==='-'?-1:1);
  else if (e.key==='0') setView(view);
  else return;
  e.preventDefault();
});

function fromAddress() {
  const params=new URLSearchParams(location.search);
  const id=params.get('vaka')?.toUpperCase();
  const run=Number(params.get('kosu'));
  return [list.some(c=>c.id===id)?id:'Q04',Number.isSafeInteger(run)?Math.max(0,run-1):0];
}
window.addEventListener('popstate',()=>{if(list.length) selectCase(...fromAddress());});
playbackTools.forEach(control=>{control.disabled=true;});
views.forEach(control=>{control.disabled=true;});
setCameraTools();

try {
  const index=await getJson('./data/index.json');
  list=index.cases;
  try {
    viewer=new JanusViewer(viewport);
    if (!embedded && compactLayout.matches) view = 'robot';
    viewer.controls.enableZoom=!embedded;
    await viewer.load();
  } catch(error) {
    engineError=error;
  }
  await selectCase(...fromAddress());
} catch(error) {
  showRenderError(error);
  viewport.setAttribute('aria-busy','false');
}
