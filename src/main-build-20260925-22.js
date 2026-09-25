import {D435RgbFeatureSource} from './rgb-feature-source-build-20260925-21.js?v=20260925.21';
import {drawDepth,TrajectoryRenderer} from './render.js?v=20260925.8';
const $=id=>document.getElementById(id);
const rgbSource=new D435RgbFeatureSource();
const trajView=new TrajectoryRenderer($('trajectoryCanvas'));
const worker=new Worker(new URL('./slam-worker-build-20260925-22.js?v=20260925.22',import.meta.url),{type:'classic'});
const HUB_CHANNEL='realsense-sensor-api-v1';
const hubChannel=new BroadcastChannel(HUB_CHANNEL);
let running=false,busy=false,dropped=0,lastSent=0,lastRgbFeatures=null,lastRgbFeaturesAt=0,rgbFeatureSeq=0,path=[],keyframePath=[],originMatrix=null,trajectoryRows=[];
let latestT265=null,depthFps=0,lastDepthArrival=0,depthFrames=0,hubOnline=false,hubDepthActive=false,hubT265Active=false,autoStarted=false,sessionInitialized=false,lastOrientationBuild='',warnedOrientationBuild=false;
let diagnosticLines=[],lastTrackingState='',lastHealthLogAt=0,lastLoggedLoops=0,lastLoggedRelocalized=0,lastLoggedKeyframes=0,lastLoggedMapMatches=0,lastLoggedMapRejects=0,lastLoggedMapRepairs=0,mapMode='mapping';
rgbSource.setLogger((level,message)=>log(level,message));
$('startBtn').onclick=()=>running?pauseSlam():startSlam();
$('restartHubBtn').onclick=()=>{log('INFO','Sensor Hub restart requested.');hubCommand('sensor_api_restart');};
$('resetOriginBtn').onclick=()=>{originMatrix=currentMatrix?currentMatrix.slice():null;log('INFO','Origin reset to current 6DoF pose. Map coordinates are unchanged internally.');};
$('resetMapBtn').onclick=()=>{log('WARN','Persistent SLAM map reset requested.');worker.postMessage({type:'reset'});sessionInitialized=true;};
$('buildMapBtn').onclick=()=>setMapMode('mapping');
$('localizeModeBtn').onclick=()=>setMapMode('localization');
$('saveMapBtn').onclick=()=>{log('INFO','Map export requested.');worker.postMessage({type:'exportMap'});};
$('loadMapBtn').onclick=()=>$('loadMapInput').click();
$('loadMapInput').onchange=loadMapFile;
$('benchmarkBtn').onclick=()=>{log('INFO','Browser benchmark started.');worker.postMessage({type:'benchmark'});};
$('exportCsvBtn').onclick=exportCsv;
$('copyLogBtn').onclick=copyLog;
$('clearLogBtn').onclick=clearLog;
window.addEventListener('beforeunload',()=>{rgbSource.stop();try{hubChannel.postMessage({type:'sensor_api_goodbye',protocol:1,client_id:'realsense-browser-slam'});}catch{}try{hubChannel.close();}catch{}});
window.addEventListener('error',e=>log('ERROR',`Window error: ${e.message}${e.filename?` @ ${e.filename}:${e.lineno||0}`:''}`));
window.addEventListener('unhandledrejection',e=>log('ERROR',`Unhandled promise rejection: ${formatReason(e.reason)}`));
let currentRawPose=null,currentMatrix=null;
worker.onmessage=e=>{
  const m=e.data;
  if(m.type==='result'){busy=false;renderResult(m);}
  else if(m.type==='benchmark'){
    const el=$('benchmarkState');el.textContent=`Synthetic 320×240: median ${m.median.toFixed(1)} ms · p95 ${m.p95.toFixed(1)} ms · recommended ${m.recommendedHz} Hz`;
    el.style.color=m.median<40?'#3fb950':'#d29922';log('INFO',`Benchmark: median=${m.median.toFixed(1)} ms p95=${m.p95.toFixed(1)} ms recommended=${m.recommendedHz} Hz`);
  }else if(m.type==='mapExport'){
    const blob=new Blob([JSON.stringify(m.map)],{type:'application/json'});
    const a=document.createElement('a');a.href=URL.createObjectURL(blob);a.download=`d435-map-${new Date().toISOString().replace(/[:.]/g,'-')}.json`;a.click();setTimeout(()=>URL.revokeObjectURL(a.href),1500);
    log('INFO',`Map saved: ${m.keyframes} keyframes, ${(blob.size/1024/1024).toFixed(1)} MiB.`);
    setMapState(`Map saved · ${m.keyframes} keyframes`,'ok');
  }else if(m.type==='mapImported'){
    busy=false;mapMode=m.mapMode||'localization';originMatrix=null;path=[];trajectoryRows=[];lastTrackingState='';lastLoggedMapMatches=0;lastLoggedKeyframes=m.keyframes||0;
    updateMapUi(mapMode,m.keyframes,0);trajView.render([],[]);
    log('INFO',`Map loaded: ${m.keyframes} keyframes (${m.rgbKeyframes||0} with RGB features). Localization mode enabled.`);
    setMapState(`Loaded ${m.keyframes} keyframes · ready to localize`,'ok');
  }else if(m.type==='mapError'){log('ERROR',m.message);setMapState(m.message,'bad');
  }else if(m.type==='mode'){mapMode=m.mapMode;updateMapUi(mapMode,m.keyframes,null);log('INFO',`Map mode changed to ${mapMode}. keyframes=${m.keyframes}`);
  }else if(m.type==='sessionReset'){busy=false;path=[];trajectoryRows=[];originMatrix=null;lastTrackingState='';lastLoggedMapMatches=0;updateMapUi(m.mapMode,m.keyframes,0);log('INFO',`Tracking session reset. Persistent map preserved (${m.keyframes} keyframes).`);
  }else if(m.type==='reset'){
    busy=false;path=[];keyframePath=[];originMatrix=null;trajectoryRows=[];lastTrackingState='';lastLoggedLoops=0;lastLoggedRelocalized=0;lastLoggedKeyframes=0;lastLoggedMapMatches=0;lastLoggedMapRejects=0;lastLoggedMapRepairs=0;mapMode='mapping';trajView.render([],[]);updateMapUi('mapping',0,0);setMapState('Map cleared. Build a new map.');log('INFO','Persistent SLAM map reset complete.');
  }
};
worker.onerror=e=>log('ERROR',`SLAM worker error: ${e.message||'unknown error'}${e.filename?` @ ${e.filename}:${e.lineno||0}`:''}`);
function hubCommand(type,options={}){
  hubChannel.postMessage({type,protocol:1,client_id:'realsense-browser-slam',request_id:`slam-${Date.now()}-${Math.random().toString(16).slice(2)}`,options});
}
function updateHubStatusFromSnapshot(st){
  hubOnline=true;
  const d=st?.d435||{},t=st?.t265||{};
  hubDepthActive=!!d.active;hubT265Active=!!t.active;
  $('hubState').textContent='ONLINE';
  $('hubDepth').textContent=d.active?`LIVE · ${d.width||0}×${d.height||0} · ${Number(d.hz||0).toFixed(1)} Hz`:'WAITING';
  $('hubT265').textContent=t.active?`LIVE · ${Number(t.hz||0).toFixed(1)} Hz`:'WAITING';
  $('hubT265Conf').textContent=t.active?confidenceName(t.confidence):'—';
}
hubChannel.onmessage=e=>{
  const m=e.data;if(!m||m.protocol!==1)return;
  if(m.type==='sensor_api_status'){updateHubStatusFromSnapshot(m.status||{});return;}
  if(m.type==='d435_status'){hubOnline=true;hubDepthActive=!!m.status?.active;$('hubState').textContent='ONLINE';$('hubDepth').textContent=hubDepthActive?'LIVE':'WAITING';return;}
  if(m.type==='t265_status'){hubOnline=true;hubT265Active=!!m.status?.active;$('hubState').textContent='ONLINE';$('hubT265').textContent=hubT265Active?'LIVE':'WAITING';return;}
  if(m.type==='t265_pose'){
    latestT265=m;hubOnline=true;hubT265Active=true;
    $('hubState').textContent='ONLINE';
    $('hubT265').textContent=`LIVE · ${Number(m.raw_pose_hz||0).toFixed(1)} Hz`;
    $('hubT265Conf').textContent=confidenceName(m.tracker_confidence);
    return;
  }
  if(m.type==='d435_depth'){handleDepthFrame(m);return;}
  if(m.type==='sensor_api_start_result'&&m.authorization_required){
    log('WARN','T265 authorization is required once in the RealSense Web Viewer / Boot Lab.');
  }
};
hubChannel.postMessage({type:'sensor_api_hello',protocol:1,client_id:'realsense-browser-slam',request_id:'slam-hello'});
hubChannel.postMessage({type:'sensor_api_status_request',protocol:1,client_id:'realsense-browser-slam',request_id:'slam-status'});
setInterval(()=>{try{hubChannel.postMessage({type:'sensor_api_status_request',protocol:1,client_id:'realsense-browser-slam',request_id:'slam-status'});}catch{}},2000);

async function initRgb(){
  $('rgbState').textContent='STARTING';
  try{
    const info=await rgbSource.startAuto();
    $('rgbState').textContent=`LIVE · ${info.width}×${info.height}`;
    log('INFO',`D435 RGB ready: ${info.label} ${info.width}x${info.height}`);
  }catch(e){
    $('rgbState').textContent='UNAVAILABLE';
    log('WARN',`D435 RGB auto-start failed: ${e.name||'Error'}: ${e.message}. Depth/T265 tracking can continue, but map recovery is weaker.`);
  }
}
function startSlam(){
  if(running)return;
  const resume=sessionInitialized;
  running=true;busy=false;dropped=0;lastSent=0;
  if(!resume){
    originMatrix=null;path=[];keyframePath=[];trajectoryRows=[];lastTrackingState='';lastHealthLogAt=0;lastLoggedLoops=0;lastLoggedRelocalized=0;lastLoggedKeyframes=0;lastLoggedMapMatches=0;lastLoggedMapRejects=0;lastLoggedMapRepairs=0;
    worker.postMessage({type:'startSession'});
    sessionInitialized=true;
  }
  $('startBtn').textContent='Pause SLAM';
  setBadge('TRACKING','live');
  setCam(`Sensor Hub input · Depth ${hubDepthActive?'LIVE':'WAITING'} · T265 ${hubT265Active?'LIVE':'WAITING'} · RGB ${rgbSource.active?'ON':'OFF'}`,'ok');
  log('INFO',resume?`SLAM resumed without session reset. T265=${latestT265?'available':'waiting'}, RGB=${rgbSource.active?'ON':'OFF'}`:`SLAM processing started automatically. T265=${latestT265?'available':'waiting'}, RGB=${rgbSource.active?'ON':'OFF'}`);
  if(!resume){const k=scaledK();log('INFO',`SLAM intrinsics 320x240: fx=${k.fx.toFixed(3)} fy=${k.fy.toFixed(3)} cx=${k.cx.toFixed(3)} cy=${k.cy.toFixed(3)}`);}
}
function pauseSlam(){
  running=false;busy=false;$('startBtn').textContent='Resume SLAM';setBadge('IDLE','idle');setCam('SLAM paused. Sensor Hub remains active.');log('INFO',`SLAM paused. dropped=${dropped}, trajectoryRows=${trajectoryRows.length}`);
}
function handleDepthFrame(m){
  const now=performance.now();hubOnline=true;hubDepthActive=true;depthFrames++;
  if(lastDepthArrival){const dt=now-lastDepthArrival;if(dt>0){const hz=1000/dt;depthFps=depthFps?depthFps*.88+hz*.12:hz;}}
  lastDepthArrival=now;
  $('hubState').textContent='ONLINE';
  $('hubDepth').textContent=`LIVE · ${m.width||0}×${m.height||0} · ${depthFps.toFixed(1)} Hz`;
  lastOrientationBuild=m.orientation_build||'';
  $('depthOrientation').textContent=m.orientation==='physical-upright'?`UPRIGHT · ${lastOrientationBuild||m.pixel_transform||''}`:(m.orientation||'UNKNOWN');
  if(m.orientation_build!=='20260925.7'&&!warnedOrientationBuild){
    warnedOrientationBuild=true;log('WARN',`Unexpected Sensor Hub Depth orientation build: ${m.orientation_build||'(none)'} transform=${m.pixel_transform||'(none)'}; expected 20260925.7.`);
  }
  const raw=m.data instanceof Uint16Array?m.data:new Uint16Array(m.data?.buffer||m.data||0);
  if(raw.length!==320*240){log('WARN',`Ignoring unexpected Depth frame size ${m.width}x${m.height} samples=${raw.length}`);return;}
  const scale=num('depthScale',Number(m.depth_scale_m)||.001),maxRange=num('maxRange',4);
  const depth=new Float32Array(raw.length);
  for(let i=0;i<raw.length;i++){const z=raw[i]*scale;depth[i]=(raw[i]>0&&z<=maxRange)?z:0;}
  drawDepth($('depthCanvas'),depth,320,240,maxRange);
  $('cameraFps').textContent=`${depthFps.toFixed(1)} Hz`;
  if(!autoStarted){autoStarted=true;startSlam();}
  if(!running)return;
  const target=Math.max(5,Math.min(20,num('targetHz',10))),period=1000/target;
  if(now-lastRgbFeaturesAt>250){
    lastRgbFeatures=rgbSource.rgbFeatures();lastRgbFeaturesAt=now;if(lastRgbFeatures)rgbFeatureSeq++;
  }
  if(now-lastSent<period)return;
  if(busy){dropped++;return;}
  busy=true;lastSent=now;
  const K=scaledK(),rf=lastRgbFeatures;
  const rg=rf?.global?.slice()||null,rxy=rf?.xy?.slice()||null,rb=rf?.brief?.slice()||null;
  const transfer=[depth.buffer];if(rg)transfer.push(rg.buffer);if(rxy)transfer.push(rxy.buffer);if(rb)transfer.push(rb.buffer);
  const q=latestT265?.quaternion,p265=latestT265?.position;
  worker.postMessage({
    type:'frame',depth:depth.buffer,w:320,h:240,K,maxCorr:num('maxCorr',.08),
    rgbGlobal:rg?.buffer||null,rgbXY:rxy?.buffer||null,rgbBrief:rb?.buffer||null,rgbCount:rf?.count||0,rgbW:rf?.width||160,rgbH:rf?.height||120,
    rgbStreamActive:rgbSource.active,rgbSeq:rgbFeatureSeq,
    t265Quaternion:q?[Number(q.x),Number(q.y),Number(q.z),Number(q.w)]:null,
    t265Position:p265?[Number(p265.x),Number(p265.y),Number(p265.z)]:null,
    t265Confidence:Number(latestT265?.tracker_confidence)||0,
    t265Seq:Number(latestT265?.seq)||0,
    t265RawHz:Number(latestT265?.raw_pose_hz)||0,
    timestamp:now
  },transfer);
}
function scaledK(){
  return{fx:num('fx',381.902)*.5,fy:num('fy',381.902)*.5,cx:num('ppx',318.229)*.5,cy:num('ppy',239.945)*.5};
}
function confidenceName(v){return['FAILED','LOW','MEDIUM','HIGH'][Math.max(0,Math.min(3,Number(v)||0))];}
function renderResult(m){currentRawPose=m.pose;currentMatrix=m.matrix;mapMode=m.mapMode||mapMode;const p=relativePose(m.pose,m.matrix);$('trackingState').textContent=m.state;$('slamHz').textContent=`${m.slamHz.toFixed(1)} Hz`;$('computeMs').textContent=`${m.computeMs.toFixed(1)} ms`;$('dropped').textContent=String(dropped);$('inliers').textContent=String(m.inliers);$('rmse').textContent=Number.isFinite(m.rmse)?`${(m.rmse*1000).toFixed(1)} mm`:'—';$('keyframes').textContent=String(m.keyframes);$('loops').textContent=String(m.loops);$('relocalized').textContent=String(m.relocalized);$('mapMatches').textContent=String(m.mapMatches||0);$('mapRejects').textContent=String(m.mapRejects||0);$('mapRepairs').textContent=String(m.mapRepairs||0);$('localAnchors').textContent=String(m.localAnchors||0);$('localAnchorRejects').textContent=String(m.localAnchorRejects||0);$('localCorr').textContent=m.localLocked?`${(m.localCorrectionM*1000).toFixed(1)} mm / ${(m.localCorrectionDeg||0).toFixed(2)}°`:'—';$('localRef').textContent=m.localLocked?(m.localTrusted?'TRUSTED MAP':'RECENT KF'):'—';$('localAnchorFrame').textContent=m.localAnchorFrame?String(m.localAnchorFrame):'—';$('trustedFrame').textContent=String(m.lastTrustedFrame||0);$('t265Fusion').textContent=m.t265Bridge?`BRIDGE · ${confidenceName(m.t265Confidence)}`:(m.t265Used?`USED · ${confidenceName(m.t265Confidence)}`:(m.t265Available?`FALLBACK · ${confidenceName(m.t265Confidence)}`:'NO POSE'));$('t265Delta').textContent=Number.isFinite(m.t265DeltaDeg)?`${m.t265DeltaDeg.toFixed(2)}° / ${Number.isFinite(m.t265DeltaM)?(m.t265DeltaM*1000).toFixed(1):'—'} mm`:'—';$('t265IcpDiff').textContent=Number.isFinite(m.t265IcpDisagreeDeg)?`${m.t265IcpDisagreeDeg.toFixed(2)}° / ${Number.isFinite(m.t265IcpTranslationDiffM)?(m.t265IcpTranslationDiffM*1000).toFixed(1):'—'} mm`:'—';$('t265Counts').textContent=`${m.t265UsedFrames||0} / ${m.t265FallbackFrames||0}`;$('t265Retry').textContent=`${m.t265RetryFrames||0} · bridge ${m.t265BridgeFrames||0}`;$('staticState').textContent=m.state==='STATIC'?'LOCKED':`${m.staticStreak||0}/4`;$('staticDepth').textContent=Number.isFinite(m.staticMedianMm)?`${m.staticMedianMm.toFixed(1)} / ${m.staticP90Mm.toFixed(1)} mm`:'—';$('staticRatio').textContent=`${((m.staticCloseRatio||0)*100).toFixed(0)}%`;$('staticOdom').textContent=Number.isFinite(m.staticOdomMm)?`${m.staticOdomMm.toFixed(1)} mm / ${m.staticOdomDeg.toFixed(2)}°`:'—';$('staticRgb').textContent=Number.isFinite(m.staticRgbMotionPx)?`${m.staticRgbInliers||0}/${m.staticRgbMatches||0} · ${m.staticRgbMotionPx.toFixed(2)} px / ${m.staticRgbAngleDeg.toFixed(2)}°`:'—';$('staticNoise').textContent=Number.isFinite(m.depthNoiseMedMm)?`${m.depthNoiseMedMm.toFixed(1)} / ${m.depthNoiseP90Mm.toFixed(1)} mm (${m.depthNoiseSamples||0})`:'—';$('staticGateDepth').textContent=Number.isFinite(m.staticDepthMedLimitMm)?`${m.staticDepthMedLimitMm.toFixed(1)} / ${m.staticDepthP90LimitMm.toFixed(1)} mm`:'—';$('rgbFresh').textContent=m.staticFreshRgb?'NEW':'HOLD';$('staticLatch').textContent=m.staticLatched?'LATCHED':'FREE';$('staticMove').textContent=`${m.staticMoveScore||0} · exit ${m.staticExitStreak||0}/2`;$('mapModeValue').textContent=mapMode==='mapping'?'BUILD':'LOCALIZE';$('rgbFeatures').textContent=String(m.rgbFeatures||0);$('rgbMatches').textContent=String(m.rgbMatches||0);$('rgbInliers').textContent=String(m.rgbInliers||0);$('confirmHits').textContent=String(m.confirmHits||0);$('recoveryGate').textContent=m.needsMapRecovery?'MAP REQUIRED':'FREE';$('posX').textContent=fmtM(p.x);$('posY').textContent=fmtM(p.y);$('posZ').textContent=fmtM(p.z);$('roll').textContent=fmtD(p.roll);$('pitch').textContent=fmtD(p.pitch);$('yaw').textContent=fmtD(p.yaw);setBadge(m.state,(m.state==='TRACKING'||m.state==='T265_BRIDGE'||m.state==='RELOCALIZED'||m.state==='MAP_LOCKED'||m.state==='STATIC')?'live':(m.state==='LOST'||m.state==='SEARCHING_MAP'||m.state==='RECOVERY_CONFIRM'||m.state==='MAP_CONFIRM')?'warn':'idle');path=m.path||path;keyframePath=m.keyframePath||keyframePath;trajView.render(path,keyframePath);trajectoryRows.push([performance.now(),p.x,p.y,p.z,p.roll,p.pitch,p.yaw,m.state,m.rmse,m.inliers]);
  if(m.state!==lastTrackingState){const suffix=m.staticExitReason?`, staticExit=${m.staticExitReason}`:'';log((m.state==='LOST'||m.state==='RECOVERY_CONFIRM'||m.state==='MAP_CONFIRM')?'WARN':'INFO',`Tracking state: ${lastTrackingState||'NONE'} -> ${m.state} (inliers=${m.inliers}, rmse=${fmtRmse(m.rmse)}, confirm=${m.confirmHits||0}/2, static=${m.staticStreak||0}/4${suffix})`);lastTrackingState=m.state;}
  if(m.loops>lastLoggedLoops){log('INFO',`Loop closure accepted. total=${m.loops}`);lastLoggedLoops=m.loops;}
  if(m.relocalized>lastLoggedRelocalized){log('INFO',`Relocalization succeeded. total=${m.relocalized}`);lastLoggedRelocalized=m.relocalized;}
  if((m.mapMatches||0)>lastLoggedMapMatches){log('INFO',`Map anchor accepted. total=${m.mapMatches} correction=${(m.mapDriftM||0).toFixed(3)}m / ${(m.mapDriftDeg||0).toFixed(2)}deg RGB=${m.rgbInliers||0}/${m.rgbMatches||0}`);lastLoggedMapMatches=m.mapMatches||0;setMapState(`MAP LOCK · ${(m.mapDriftM||0).toFixed(3)} m / ${(m.mapDriftDeg||0).toFixed(1)}° · RGB ${m.rgbInliers||0}/${m.rgbMatches||0}`,'ok');}
  if((m.mapRejects||0)>lastLoggedMapRejects){log('WARN',`Map candidate rejected by confidence gate. total=${m.mapRejects} reason=${m.lastRejectReason||'unspecified'}`);lastLoggedMapRejects=m.mapRejects||0;}
  if((m.mapRepairs||0)>lastLoggedMapRepairs){log('INFO',`Map repaired after relocalization. total=${m.mapRepairs}`);lastLoggedMapRepairs=m.mapRepairs||0;}
  if(m.keyframes>lastLoggedKeyframes){log('INFO',`Keyframe added. total=${m.keyframes}`);lastLoggedKeyframes=m.keyframes;}
  const now=performance.now();if(now-lastHealthLogAt>3000){log('INFO',`Health: state=${m.state} camera=${depthFps.toFixed(1)}Hz slam=${m.slamHz.toFixed(1)}Hz compute=${m.computeMs.toFixed(1)}ms dropped=${dropped} inliers=${m.inliers} rmse=${fmtRmse(m.rmse)} RGBfeatures=${m.rgbFeatures||0} RGBmatch=${m.rgbInliers||0}/${m.rgbMatches||0} rejects=${m.mapRejects||0} confirm=${m.confirmHits||0}/2 recoveryGate=${m.needsMapRecovery?'MAP':'FREE'} repairs=${m.mapRepairs||0} local=${m.localAnchors||0}/${m.localAnchorRejects||0} localRef=${m.localLocked?(m.localTrusted?'TRUSTED':'RECENT'):'na'} localKF=${m.localAnchorFrame||0} localCorr=${m.localLocked?`${(m.localCorrectionM*1000).toFixed(1)}mm/${(m.localCorrectionDeg||0).toFixed(2)}deg`:'na'} t265=${m.t265Bridge?'BRIDGE':m.t265Used?'USED':'FALLBACK'} conf=${m.t265Confidence||0} dR=${Number.isFinite(m.t265DeltaDeg)?m.t265DeltaDeg.toFixed(2):'na'}deg dT=${Number.isFinite(m.t265DeltaM)?(m.t265DeltaM*1000).toFixed(1):'na'}mm icpDiff=${Number.isFinite(m.t265IcpDisagreeDeg)?m.t265IcpDisagreeDeg.toFixed(2):'na'}deg/${Number.isFinite(m.t265IcpTranslationDiffM)?(m.t265IcpTranslationDiffM*1000).toFixed(1):'na'}mm t265Count=${m.t265UsedFrames||0}/${m.t265FallbackFrames||0} bridge=${m.t265BridgeFrames||0} trustedFrame=${m.lastTrustedFrame||0} static=${m.staticStreak||0}/4 blockDepthΔ=${Number.isFinite(m.staticMedianMm)?m.staticMedianMm.toFixed(1):'na'}/${Number.isFinite(m.staticP90Mm)?m.staticP90Mm.toFixed(1):'na'}mm close=${((m.staticCloseRatio||0)*100).toFixed(0)}% odom=${Number.isFinite(m.staticOdomMm)?m.staticOdomMm.toFixed(1):'na'}mm/${Number.isFinite(m.staticOdomDeg)?m.staticOdomDeg.toFixed(2):'na'}deg rgb=${m.staticRgbInliers||0}/${m.staticRgbMatches||0}:${Number.isFinite(m.staticRgbMotionPx)?m.staticRgbMotionPx.toFixed(2):'na'}px fresh=${m.staticFreshRgb?'Y':'N'} noise=${Number.isFinite(m.depthNoiseMedMm)?m.depthNoiseMedMm.toFixed(1):'na'}/${Number.isFinite(m.depthNoiseP90Mm)?m.depthNoiseP90Mm.toFixed(1):'na'}mm gate=${Number.isFinite(m.staticDepthMedLimitMm)?m.staticDepthMedLimitMm.toFixed(1):'na'}/${Number.isFinite(m.staticDepthP90LimitMm)?m.staticDepthP90LimitMm.toFixed(1):'na'}mm latch=${m.staticLatched?'Y':'N'} moveScore=${m.staticMoveScore||0} exit=${m.staticExitStreak||0}/2 pose=[${p.x.toFixed(3)},${p.y.toFixed(3)},${p.z.toFixed(3)}]m rpy=[${p.roll.toFixed(2)},${p.pitch.toFixed(2)},${p.yaw.toFixed(2)}]deg`);lastHealthLogAt=now;}
}
function setMapMode(mode){mapMode=mode==='localization'?'localization':'mapping';worker.postMessage({type:'setMode',mode:mapMode});updateMapUi(mapMode,null,null);setMapState(mapMode==='mapping'?'Building/extending map. Walk through the area you want to remember.':'Localization mode. The stored map is fixed and current pose will be anchored to matching keyframes.');}
async function loadMapFile(e){const file=e.target.files?.[0];e.target.value='';if(!file)return;try{setMapState('Reading map…');const text=await file.text();const map=JSON.parse(text);log('INFO',`Loading map file "${file.name}" (${(file.size/1024/1024).toFixed(1)} MiB).`);worker.postMessage({type:'importMap',map});}catch(err){log('ERROR',`Map load failed: ${err.message}`);setMapState(`Map load failed: ${err.message}`,'bad');}}
function updateMapUi(mode,keyframes,matches){if(mode)mapMode=mode;if($('mapModeValue'))$('mapModeValue').textContent=mapMode==='mapping'?'BUILD':'LOCALIZE';if(keyframes!=null&&$('keyframes'))$('keyframes').textContent=String(keyframes);if(matches!=null&&$('mapMatches'))$('mapMatches').textContent=String(matches);$('buildMapBtn')?.classList.toggle('primary',mapMode==='mapping');$('localizeModeBtn')?.classList.toggle('primary',mapMode==='localization');}
function setMapState(text,kind=''){const el=$('mapState');if(!el)return;el.textContent=text;el.style.color=kind==='ok'?'#3fb950':kind==='bad'?'#f85149':'';}
function relativePose(p,matrix){if(!originMatrix||!matrix)return p;const R=mul4(invRigid(originMatrix),matrix),e=euler4(R);return{x:R[3],y:R[7],z:R[11],roll:e[0],pitch:e[1],yaw:e[2]};}
function invRigid(T){return[T[0],T[4],T[8],-(T[0]*T[3]+T[4]*T[7]+T[8]*T[11]),T[1],T[5],T[9],-(T[1]*T[3]+T[5]*T[7]+T[9]*T[11]),T[2],T[6],T[10],-(T[2]*T[3]+T[6]*T[7]+T[10]*T[11]),0,0,0,1];}
function mul4(a,b){const o=new Array(16).fill(0);for(let r=0;r<4;r++)for(let c=0;c<4;c++)for(let k=0;k<4;k++)o[r*4+c]+=a[r*4+k]*b[k*4+c];return o;}
function euler4(T){const pitch=Math.asin(Math.max(-1,Math.min(1,-T[8]))),roll=Math.atan2(T[9],T[10]),yaw=Math.atan2(T[4],T[0]),d=180/Math.PI;return[roll*d,pitch*d,yaw*d];}
function fmtM(v){return `${v>=0?'+':''}${v.toFixed(3)} m`;}function fmtD(v){return `${v>=0?'+':''}${v.toFixed(2)}°`;}
function num(id,f){const v=Number($(id).value);return Number.isFinite(v)?v:f;}function setCam(s,k=''){const e=$('cameraState');e.textContent=s;e.style.color=k==='ok'?'#3fb950':k==='bad'?'#f85149':'';}function setBadge(s,k){const e=$('statusBadge');e.textContent=s;e.className=`badge ${k||'idle'}`;}
function exportCsv(){let s='time_ms,x_m,y_m,z_m,roll_deg,pitch_deg,yaw_deg,state,rmse_m,inliers\n';for(const r of trajectoryRows)s+=r.join(',')+'\n';const a=document.createElement('a');a.href=URL.createObjectURL(new Blob([s],{type:'text/csv'}));a.download='d435-browser-slam-trajectory.csv';a.click();setTimeout(()=>URL.revokeObjectURL(a.href),1000);log('INFO',`Trajectory CSV exported: ${trajectoryRows.length} rows.`);}
function log(level,message){const stamp=new Date().toISOString();const line=`[${stamp}] [${level}] ${message}`;diagnosticLines.push(line);if(diagnosticLines.length>2000)diagnosticLines.splice(0,diagnosticLines.length-2000);const pre=$('diagnosticLog');if(pre){pre.textContent=diagnosticLines.join('\n');pre.scrollTop=pre.scrollHeight;}$('logCount').textContent=`${diagnosticLines.length} lines`;console[level==='ERROR'?'error':level==='WARN'?'warn':'log'](line);}
async function copyLog(){const text=diagnosticLines.join('\n');if(!text){$('copyLogState').textContent='Log is empty.';return;}try{await navigator.clipboard.writeText(text);$('copyLogState').textContent=`Copied ${diagnosticLines.length} log lines to clipboard.`;log('INFO','Diagnostic log copied to clipboard.');}catch(e){const ta=document.createElement('textarea');ta.value=text;ta.style.position='fixed';ta.style.left='-9999px';document.body.appendChild(ta);ta.select();const ok=document.execCommand('copy');ta.remove();$('copyLogState').textContent=ok?`Copied ${diagnosticLines.length} log lines to clipboard.`:`Copy failed: ${e.message}`;if(!ok)log('ERROR',`Clipboard copy failed: ${e.message}`);}}
function clearLog(){diagnosticLines=[];$('diagnosticLog').textContent='';$('logCount').textContent='0 lines';$('copyLogState').textContent='Log cleared.';log('INFO','Diagnostic log cleared.');}
function selectedLabel(id){const e=$(id),o=e?.options?.[e.selectedIndex];return o?.textContent||'';}
function shortId(id){if(!id)return'(none)';return id.length>18?`${id.slice(0,8)}…${id.slice(-6)}`:id;}
function formatReason(r){if(r instanceof Error)return `${r.name}: ${r.message}`;try{return typeof r==='string'?r:JSON.stringify(r);}catch{return String(r);}}
let warnedProfile='';
function logOnceProfileWarning(w,h){const k=`${w}x${h}`;if(k===warnedProfile)return;warnedProfile=k;log('WARN',`Depth stream is ${k}, not 640x480. Intrinsics are being scaled from the 640x480 D435 preset; metric accuracy is provisional until per-profile calibration is supplied.`);}
function fmtRmse(v){return Number.isFinite(v)?`${(v*1000).toFixed(1)}mm`:'n/a';}
log('INFO',`Build=20260925.22 · Sensor Hub + T265 relative-pose guard/bridge`);
log('INFO',`Page loaded. UA=${navigator.userAgent}`);
log('INFO',`Capabilities: BroadcastChannel=${typeof BroadcastChannel!=='undefined'} getUserMedia=${!!navigator.mediaDevices?.getUserMedia} WebGL2=${!!document.createElement('canvas').getContext('webgl2')} Worker=${typeof Worker!=='undefined'} clipboard=${!!navigator.clipboard}`);
setCam('Waiting for Sensor Hub Depth. SLAM will start automatically.');
$('hubState').textContent='WAITING';$('hubDepth').textContent='WAITING';$('hubT265').textContent='WAITING';
initRgb();
updateMapUi('mapping',0,0);
