import { JanusViewer, sampleAt, latestAt } from './viewer.js';

const $ = id => document.getElementById(id);
const query = selector => [...document.querySelectorAll(selector)];
const filters = query('[data-filter]');
const views = query('[data-view]');
const categories = {M:'Hareket',Q:'QR yaklaşımı',O:'Operasyon'};
const label = {
  case_start:'Koşu başladı', trace_start:'Kayıt başladı', trace_end:'Kayıt sona erdi',
  acquired:'QR algılandı', center:'Merkezleme', advance:'İleri yaklaşım', center_dwell_met:'Merkez bekleme tamam',
  staging_plan:'Hazırlık hedefi', staging_complete:'Hazırlık tamam', stage_align:'Hiza hazırlığı',
  stage_turn_to_position:'Hazırlık yönüne dönüş', stage_translate:'Hazırlık pozuna ilerleme',
  stage_turn_to_dock:'Dock yönüne dönüş', optical_center:'Optik merkezleme', case_end:'Koşu sona erdi',
  terminal_gate_verified:'Terminal kapısı doğrulandı', obstacle_frame_saved:'Engel karesi kaydedildi',
  final_brake:'Son fren', no_qr:'QR görünmüyor', fresh_matching_id:'Taze doğru QR',
  brake_step:'Fren komutu', terminal:'Koşu sonu', terminal_hold:'Terminal duruş', standoff:'Optik duruş',
  optical_stop:'Optik duruş', visible_loss:'QR görüntüden kayboldu', qr_lost:'QR kaybı', lost:'QR kaybı',
  hide:'İşaret kaldırıldı', restore:'İşaret geri kondu', restore_service_ack:'İşaret konumu güncellendi',
  contact:'Temas olayı', virtual_contact:'Sanal temas', virtual_charge:'Sanal şarj olayı', bms_charge_confirmed:'Sanal BMS teyidi',
  charge_confirmed:'Şarj teyidi', bms_timeout:'BMS zaman aşımı', wrong_id:'Yanlış QR kimliği',
  abort:'İptal / güvenli duruş', search_timeout:'Arama zaman aşımı', gate_accepted:'Geçiş kapısı tamam',
  surface_change:'İkinci yüzeye geçiş', odom_stream_restored:'Odometri geri geldi',
  regressive_odometry:'Geri giden odometri', odometry_stream_cut:'Odometri akışı kesildi', guard_stop:'Komut kesildi',
  control_stream:'Komut akışı', camera_offset:'Kamera montaj ofseti', arrived:'Hedefe varış', mission_reset:'Görev değişti',
  goal_started:'Hedef etkin', operator_consent:'Operatör izni', authorized_resume:'İzinli devam',
  unauthorized_probe:'İzinsiz istek', authorization_signal_sent:'Açık izin verildi', recovery_ack_received:'İzin teyidi',
  authorized_restart_input:'Yeni hareket isteği', clock_cut:'Simülasyon saati durdu', clock_restore:'Simülasyon saati geri geldi',
  hazard_present:'Tehlike olayının başlangıcı', obstacle_detected:'Engel algılandı', pedestrian_detected:'Geçiş algılandı',
  fixture_clear_observed:'Geçit açıldı', resume_gate_open:'Devam kapısı açıldı',
  duplicate_ids:'Aynı kimlikli iki işaret', low_light:'Düşük ışık', motion_blur:'Hareket bulanıklığı',
  low_light_motion_blur:'Düşük ışık ve bulanıklık', stimulus_recovery:'Görüntü uyarısı sona erdi',
  occlusion:'Görsel örtme', crop:'Kadrajdan kesilme', hotspot:'Parlama', whiteout:'Beyaz doygunluk',
};
const fmt = new Intl.NumberFormat('tr-TR',{minimumFractionDigits:2,maximumFractionDigits:2});
const short = new Intl.NumberFormat('tr-TR',{maximumFractionDigits:1});
const tr = text => {
  if (label[text]) return label[text];
  const segment=text.match(/^segment_(\d+)$/);
  return segment ? segment[1]+'. rota bölümü' : text.replaceAll('_',' ');
};
const cache=new Map();
let list=[],filter='',current=null,track=null,viewer=null,view='overview',renderError=null,requestNumber=0;
let time=0,playing=false,lastTick=null,requestFrame=0,activeEvent=null,recording=null;
const video=$('camera-video');

function button(text, classes) {
  const node=document.createElement('button');
  node.type='button';node.className=classes;node.textContent=text;
  return node;
}

function renderCatalog() {
  const search=$('search').value.trim().toLocaleLowerCase('tr-TR');
  const matches=list.filter(c => (!filter || c.id.startsWith(filter)) &&
    (!search || (c.id+' '+c.title+' '+c.environment).toLocaleLowerCase('tr-TR').includes(search)));
  $('case-count').textContent=matches.length;
  $('case-list').replaceChildren();
  if (!matches.length) {
    const empty=document.createElement('p');
    empty.className='empty';empty.textContent='Eşleşen vaka yok.';
    $('case-list').append(empty);
  }
  for (const c of matches) {
    const b=button('','case-row');
    b.dataset.case=c.id;
    const id=document.createElement('span'),name=document.createElement('span');
    id.textContent=c.id;name.textContent=c.title;
    b.setAttribute('aria-current', c.id===current?.meta.id ? 'true':'false');
    b.append(id,name);
    b.addEventListener('click',()=>selectCase(c.id,0,true));
    $('case-list').append(b);
  }
}

function markCatalog() {
  for (const b of query('[data-case]')) b.setAttribute('aria-current',b.dataset.case===current.meta.id?'true':'false');
}

async function getJson(path) {
  const result=await fetch(path);
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
  const request=++requestNumber;
  pause();
  if (recording) stopRecording();
  if (!cache.has(id)) cache.set(id,getJson('./data/'+id+'.json'));
  try {
    const next=await cache.get(id);
    if (request!==requestNumber) return;
    current=next;
    const {meta,tracks}=current;
    for (const [target,content] of [['case-id',meta.id],['case-title',meta.title],['environment',meta.environment],['category',categories[meta.id[0]]],['scenario',meta.scenario]]) $(target).textContent=content;
    document.title=meta.id+' · JANUS 3B Test Arşivi';
    $('result-link').href='../results.html#case-'+meta.id;
    $('clip-link').href='clips/'+meta.id+'.mp4';
    $('variant').replaceChildren();
    tracks.forEach((t,i)=>{
      const o=document.createElement('option');
      o.value=i;o.textContent= t.name+' · '+t.environment;
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
    markCatalog();
    await installTrack();
    if (update) syncAddress();
  } catch (error) {
    cache.delete(id);
    showRenderError(error);
  }
}

function showRenderError(error) {
  renderError=error;
  $('viewer-fallback').textContent='3B görünüm açılamadı. Üstteki MP4 bağlantısından koşuyu izleyebilirsin.';
  $('viewer-fallback').hidden=false;
  $('tracking-status').textContent='3B görünüm kullanılamıyor';
  $('record').disabled=true;
  console.error(error);
}

async function installTrack() {
  pause();
  if (recording) stopRecording();
  track=current.tracks[+$('variant').value];
  time=0;
  activeEvent=null;
  $('current-event').textContent='—';
  $('origin').textContent=track.basis+' · '+track.setup.fixtureProvenance;
  $('raw-path').textContent=track.sourceSha256;
  $('fixture-path').textContent=track.setup.fixtureProvenance;
  $('seek').max=track.end;
  $('event-count').textContent=track.events.length;
  $('show-section').hidden=!track.setup.boxes.some(b=>b.style==='guard');
  $('events').replaceChildren();
  $('event-marks').replaceChildren();
  for (const e of track.events) {
    const node=button('','event-row');
    node.dataset.time=e[0];
    const t=document.createElement('time'),msg=document.createElement('span');
    t.textContent=fmt.format(e[0])+' s';msg.textContent=tr(e[1]);
    node.append(t,msg);
    node.title=e[2]?tr(e[2]):tr(e[1]);
    node.addEventListener('click',()=>{pause();seek(e[0]);});
    $('events').append(node);
    const marker=document.createElement('i');
    marker.style.left=(e[0]*100/track.end)+'%';
    $('event-marks').append(marker);
  }
  video.pause();
  $('video-box').hidden=!track.media;
  $('video-box').open=false;
  if (track.media) video.src=track.media;
  else {video.removeAttribute('src');video.load();}
  if (viewer && !renderError) {
    try {
      viewer.setupCase(current.meta,track);
      if (view==='sensor' && !track.setup.markers.length) setView('overview');
      $('viewport').dataset.loaded=current.meta.id+'-'+$('variant').value;
    } catch (error) {
      showRenderError(error);
    }
  }
  seek(0);
}

function seek(t) {
  if (!track) return;
  time=Math.max(0,Math.min(track.end,t));
  const p=(!renderError&&viewer?.setTime(time)) || sampleAt(track.samples,time);
  $('seek').value=String(time);
  $('time').textContent=fmt.format(time)+' / '+fmt.format(track.end)+' s';
  $('seek').setAttribute('aria-valuetext',fmt.format(time)+' saniye');
  if (p) {
    $('axis').textContent=fmt.format(p[1])+' / '+fmt.format(p[2])+' m';
    const angle=theta=>short.format(Math.atan2(Math.sin(theta),Math.cos(theta))*180/Math.PI)+'°';
    $('angle').textContent=angle(p[4])+' / '+(track.jointSignals.includes('caster')?angle(p[5]):'—');
    $('velocity').textContent=fmt.format(p[9])+' m/s';
  }
  const e=latestAt(track.events,time);
  if (e!==activeEvent) {
    activeEvent=e;
    $('current-event').textContent=e?tr(e[1]):'—';
    const index=track.events.indexOf(e);
    [...$('events').children].forEach((b,i)=>b.setAttribute('aria-current',String(i===index)));
  }
  if (!renderError) {
    const qr=latestAt(track.qr,time);
    let state=track.environment+' · kayıt';
    if (qr && track.setup.markers.length) state=qr[1] ? 'QR · '+(qr[2]===null?'geçerli':short.format(qr[2])+' px') : 'QR · '+tr(qr[3]||'geçersiz');
    $('tracking-status').textContent=state;
  }
}

function tick(now) {
  if (!playing) return;
  const speed=+$('speed').value;
  if (lastTick!==null) seek(time+Math.min(.10,(now-lastTick)/1000)*speed);
  lastTick=now;
  if(time>=track.end) {
    pause();
    if (recording) stopRecording();
    return;
  }
  requestFrame=requestAnimationFrame(tick);
}

function play() {
  if (!track || playing) return;
  if (time>=track.end) seek(0);
  playing=true;lastTick=null;
  $('play').textContent='Ⅱ Duraklat';
  $('play').setAttribute('aria-label','Duraklat');
  requestFrame=requestAnimationFrame(tick);
}

function pause() {
  playing=false;lastTick=null;
  cancelAnimationFrame(requestFrame);
  $('play').textContent='▶ Oynat';
  $('play').setAttribute('aria-label','Oynat');
}

function setView(next) {
  view=next;
  viewer?.setView(view);
  for(const b of views) b.setAttribute('aria-pressed',b.dataset.view===view?'true':'false');
}

function stopRecording() {
  if (recording?.recorder.state==='recording') recording.recorder.stop();
}

function record() {
  if (recording) { stopRecording(); return; }
  if (renderError || !track || !window.MediaRecorder || !viewer.renderer.domElement.captureStream) return;
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
    $('record').textContent='Video kaydet';
    $('record').setAttribute('aria-pressed','false');
  }
  recorder.onerror=()=>{
    endRecording();
    $('tracking-status').textContent='Video kaydı başlatılamadı';
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
  $('record').textContent='Kaydı bitir';
  $('record').setAttribute('aria-pressed','true');
  play();
}

for (const b of filters) b.addEventListener('click',()=>{
  filter=b.dataset.filter;
  for (const a of filters) a.setAttribute('aria-pressed',a===b?'true':'false');
  renderCatalog();
  $('case-list').scrollTop=0;
});
$('search').addEventListener('input',renderCatalog);
for(const b of views) b.addEventListener('click',()=>setView(b.dataset.view));
for(const [name,action] of [['show-trace','setTrace'],['show-props','setProps'],['show-section','setSection']]) {
  $(name).addEventListener('click',e=>{
    const show=e.target.getAttribute('aria-pressed')!=='true';
    e.target.setAttribute('aria-pressed',String(show));viewer?.[action](show);
  });
}
$('play').addEventListener('click',()=>playing?pause():play());
$('restart').addEventListener('click',()=>{pause();seek(0);});
$('seek').addEventListener('input',e=>{pause();seek(+e.target.value);});
$('variant').addEventListener('change',()=>{installTrack();syncAddress();});
$('record').addEventListener('click',record);
document.addEventListener('keydown',e=>{
  if (e.code==='Space' && (e.target===document.body || e.target===$('viewport'))) {
    e.preventDefault();playing?pause():play();
  }
});

function fromAddress() {
  const params=new URLSearchParams(location.search);
  const id=params.get('vaka')?.toUpperCase();
  return [list.some(c=>c.id===id)?id:'Q04',Math.max(0,(Number(params.get('kosu'))||1)-1)];
}
window.addEventListener('popstate',()=>{if(list.length) selectCase(...fromAddress());});

try {
  const index=await getJson('./data/index.json');
  list=index.cases;
  renderCatalog();
  try {
    viewer=new JanusViewer($('viewport'));
    await viewer.load();
    if(!window.MediaRecorder || !viewer.renderer.domElement.captureStream) $('record').disabled=true;
  } catch(error) { showRenderError(error); }
  await selectCase(...fromAddress());
  const panel=$('case-list'), active=panel.querySelector('[aria-current=true]');
  if (active) panel.scrollTop=Math.max(0,active.offsetTop-panel.offsetTop-panel.clientHeight*.74);
} catch(error) {
  showRenderError(error);
}
