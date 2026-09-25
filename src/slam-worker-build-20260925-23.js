let prev=null,w=320,h=240,K=null,pose=I(),lastRel=I(),frame=0,keyframes=[],loops=0,relocalized=0,lostCount=0,lastTime=0,slamHz=0,path=[],keyframePath=[];
let mapMode='mapping',mapMatches=0,mapRejects=0,mapRepairs=0,localAnchors=0,localAnchorRejects=0,lastMapMatchFrame=-999,mapRevision=0,keyframeHoldUntil=0,pendingAnchor=-1,pendingHits=0,pendingLastFrame=-999,needsMapRecovery=false,lastRejectReason='',lastTrustedFrame=0,lastTrustedAnchorFrame=0,staticStreak=0,staticFrames=0,staticRefDepth=null,staticRefRgb=null,prevRgb=null,staticLastRgbSeq=-1,depthNoiseMed=.010,depthNoiseP90=.028,depthNoiseSamples=0,staticLatched=false,staticExitStreak=0,staticLatchDepth=null,staticLatchRgb=null,t265PrevQ=null,t265PrevP=null,t265CurrentQ=null,t265CurrentP=null,t265CurrentConf=0,t265UsedFrames=0,t265FallbackFrames=0,t265LastConf=0,t265LastDeltaDeg=NaN,t265LastDeltaM=NaN,t265IcpDisagreeDeg=NaN,t265IcpTranslationDiffM=NaN,t265RetryFrames=0,t265BridgeFrames=0;
const MAP_CHECK_INTERVAL=12,MAX_KEYFRAMES=120,STATIC_STREAK_N=4;
const POP8=new Uint8Array(256);for(let i=0;i<256;i++){let x=i,c=0;while(x){x&=x-1;c++;}POP8[i]=c;}
onmessage=e=>{const m=e.data;
  if(m.type==='reset'){reset();postMessage({type:'reset',mapMode,keyframes:0});return;}
  if(m.type==='startSession'){resetTracking();postMessage({type:'sessionReset',mapMode,keyframes:keyframes.length});return;}
  if(m.type==='setMode'){mapMode=m.mode==='localization'?'localization':'mapping';postMessage({type:'mode',mapMode,keyframes:keyframes.length});return;}
  if(m.type==='exportMap'){exportMap();return;}
  if(m.type==='importMap'){importMap(m.map);return;}
  if(m.type==='benchmark'){benchmark();return;}
  if(m.type==='frame')processFrame(m);
};
function reset(){prev=null;pose=I();lastRel=I();frame=0;keyframes=[];loops=0;relocalized=0;lostCount=0;lastTime=0;slamHz=0;path=[];keyframePath=[];mapMode='mapping';mapMatches=0;mapRejects=0;mapRepairs=0;localAnchors=0;localAnchorRejects=0;lastMapMatchFrame=-999;mapRevision++;keyframeHoldUntil=0;pendingAnchor=-1;pendingHits=0;pendingLastFrame=-999;needsMapRecovery=false;lastRejectReason='';lastTrustedFrame=0;lastTrustedAnchorFrame=0;staticStreak=0;staticFrames=0;staticRefDepth=null;staticRefRgb=null;prevRgb=null;staticLastRgbSeq=-1;depthNoiseMed=.010;depthNoiseP90=.028;depthNoiseSamples=0;staticLatched=false;staticExitStreak=0;staticLatchDepth=null;staticLatchRgb=null;t265PrevQ=null;t265PrevP=null;t265CurrentQ=null;t265CurrentP=null;t265CurrentConf=0;t265UsedFrames=0;t265FallbackFrames=0;t265LastConf=0;t265LastDeltaDeg=NaN;t265LastDeltaM=NaN;t265IcpDisagreeDeg=NaN;t265IcpTranslationDiffM=NaN;t265RetryFrames=0;t265BridgeFrames=0;}
function resetTracking(){prev=null;pose=I();lastRel=I();frame=keyframes.reduce((n,q)=>Math.max(n,q.frame||0),0)+1;lostCount=0;lastTime=0;slamHz=0;path=[];keyframePath=keyframes.map(q=>({x:q.pose[3],y:q.pose[7],z:q.pose[11]}));lastMapMatchFrame=-999;mapMatches=0;mapRejects=0;mapRepairs=0;localAnchors=0;localAnchorRejects=0;keyframeHoldUntil=keyframes.length?frame+12:frame;pendingAnchor=-1;pendingHits=0;pendingLastFrame=-999;needsMapRecovery=keyframes.length>0;lastRejectReason='';lastTrustedFrame=keyframes.length?keyframes[keyframes.length-1].frame:0;lastTrustedAnchorFrame=keyframes.length?keyframes[keyframes.length-1].frame:0;staticStreak=0;staticFrames=0;staticRefDepth=null;staticRefRgb=null;prevRgb=null;staticLastRgbSeq=-1;depthNoiseMed=.010;depthNoiseP90=.028;depthNoiseSamples=0;staticLatched=false;staticExitStreak=0;staticLatchDepth=null;staticLatchRgb=null;t265PrevQ=null;t265PrevP=null;t265CurrentQ=null;t265CurrentP=null;t265CurrentConf=0;t265UsedFrames=0;t265FallbackFrames=0;t265LastConf=0;t265LastDeltaDeg=NaN;t265LastDeltaM=NaN;t265IcpDisagreeDeg=NaN;t265IcpTranslationDiffM=NaN;t265RetryFrames=0;t265BridgeFrames=0;}

function processFrame(m){
  const t0=performance.now(),depth=new Float32Array(m.depth),maxCorr=m.maxCorr||.08;
  const rgb=m.rgbGlobal?{global:new Uint8Array(m.rgbGlobal),xy:m.rgbXY?new Uint16Array(m.rgbXY):new Uint16Array(0),brief:m.rgbBrief?new Uint8Array(m.rgbBrief):new Uint8Array(0),count:m.rgbCount||0,width:m.rgbW||160,height:m.rgbH||120}:null;
  const rgbStreamActive=!!m.rgbStreamActive,rgbSeq=Number(m.rgbSeq)||0,rgbFresh=rgbSeq>0&&rgbSeq!==staticLastRgbSeq;
  const t265Q=validQuat(m.t265Quaternion)?normalizeQuat(m.t265Quaternion):null;
  const t265P=validVec3(m.t265Position)?m.t265Position.map(Number):null;
  const t265Conf=Number(m.t265Confidence)||0;
  let t265Rel=null,t265Usable=false;
  t265LastConf=t265Conf;
  if(t265Q&&t265P){t265CurrentQ=t265Q;t265CurrentP=t265P;t265CurrentConf=t265Conf;}
  if(t265Q&&t265P&&t265Conf>=2){
    if(t265PrevQ&&t265PrevP){
      t265Rel=t265RelativeOptical(t265PrevQ,t265PrevP,t265Q,t265P);
      t265LastDeltaDeg=rot(t265Rel)*180/Math.PI;
      t265LastDeltaM=trans(t265Rel);
      t265Usable=Number.isFinite(t265LastDeltaDeg)&&Number.isFinite(t265LastDeltaM)&&t265LastDeltaDeg<=95&&t265LastDeltaM<=.60;
    }
    t265PrevQ=t265Q;t265PrevP=t265P;
  }else if(t265Q&&t265P&&t265Conf>0&&!t265PrevQ){
    t265PrevQ=t265Q;t265PrevP=t265P;
  }
  w=m.w;h=m.h;K=m.K;frame++;
  let state='INITIALIZED',inliers=0,rmse=0,mapLocked=false,localLocked=false,localTrusted=false,localAnchorFrame=0,localCorrectionM=0,localCorrectionDeg=0,mapDriftM=0,mapDriftDeg=0,rgbMatches=0,rgbInliers=0,rgbInlierRatio=0,confirmHits=0,staticMedianMm=Infinity,staticP90Mm=Infinity,staticCloseRatio=0,staticOdomMm=Infinity,staticOdomDeg=Infinity,staticRgbMatches=0,staticRgbInliers=0,staticRgbMotionPx=Infinity,staticRgbAngleDeg=Infinity,staticDepthMedLimitMm=Infinity,staticDepthP90LimitMm=Infinity,staticFreshRgb=rgbFresh,staticMoveScore=0,staticExitReason='',t265Used=false,t265Retry=false,t265Bridge=false;

  const forceRecovery=needsMapRecovery&&keyframes.length>0;
  if(prev&&!forceRecovery){
    let odom=track(depth,prev,poseGuess(t265Usable?t265Rel:null),maxCorr);
    if(t265Usable&&(!odom.ok||odom.inliers<120||odom.rmse>Math.max(.040,maxCorr*.70))){
      const retry=track(depth,prev,t265Rel,Math.max(maxCorr,.10),[14,9,5,4],[4,4,5,5]);
      t265Retry=true;t265RetryFrames++;
      if(retry.ok||retry.inliers>odom.inliers||(retry.inliers===odom.inliers&&retry.rmse<odom.rmse))odom=retry;
    }
    if(t265Usable){
      const icpRotOnly=rotationOnly(odom.T);
      t265IcpDisagreeDeg=odom.ok?rot(rel(t265Rel,icpRotOnly))*180/Math.PI:NaN;
      t265IcpTranslationDiffM=odom.ok?translationDiff(odom.T,t265Rel):NaN;
      const leverAllowance=.035+2*.10*Math.sin(Math.min(Math.PI,t265LastDeltaDeg*Math.PI/180)/2);
      const translationGate=Math.max(.075,leverAllowance);
      const icpTranslationTrusted=odom.ok&&trans(odom.T)<.25&&t265IcpTranslationDiffM<=translationGate;
      if(icpTranslationTrusted){
        odom.T=withRotation(odom.T,t265Rel);
        t265Used=true;t265UsedFrames++;
      }else{
        // Keep tracking through large rotations using the T265 relative rigid motion.
        // Absolute map coordinates are still corrected only by RGB+Depth map locks.
        odom={ok:true,T:t265Rel.slice(),inliers:odom.inliers||0,rmse:odom.rmse};
        t265Used=true;t265Bridge=true;t265UsedFrames++;t265BridgeFrames++;
      }
    }else{
      t265FallbackFrames++;
      t265IcpDisagreeDeg=NaN;t265IcpTranslationDiffM=NaN;
    }
    inliers=odom.inliers;rmse=odom.rmse;
    const sig=depthBlockSignature(depth);
    const odomTrans=trans(odom.T),odomRot=rot(odom.T);
    staticOdomMm=odomTrans*1000;staticOdomDeg=odomRot*180/Math.PI;

    if(staticLatched){
      let moveEvidence=0,reason=[];
      if(odom.ok&&odomTrans>=.035){moveEvidence++;reason.push('ICP translation');}
      if(odom.ok&&odomRot>=3.0*Math.PI/180){moveEvidence++;reason.push('ICP rotation');}
      if(t265Usable&&t265LastDeltaDeg>=1.0){moveEvidence++;reason.push('T265 rotation');}
      if(rgbFresh&&rgb&&staticLatchRgb&&rgb.count>=12&&staticLatchRgb.count>=12){
        const rrMove=matchRgbFeatures(rgb,staticLatchRgb);
        staticRgbMatches=rrMove.matches||0;staticRgbInliers=rrMove.inliers||0;staticRgbMotionPx=rrMove.motionPx??Infinity;staticRgbAngleDeg=Math.abs(rrMove.angleDeg??Infinity);
        const rgbReliable=rrMove.matches>=10&&rrMove.inliers>=7&&rrMove.ratio>=.50&&rrMove.error<=8;
        if(rgbReliable&&staticRgbMotionPx>=1.8){moveEvidence++;reason.push('RGB translation');}
        if(rgbReliable&&staticRgbAngleDeg>=1.0){moveEvidence++;reason.push('RGB rotation');}
        if(rgbReliable&&Math.abs((rrMove.scale??1)-1)>=.018){moveEvidence++;reason.push('RGB scale');}
      }
      if(rgbFresh&&staticLatchDepth){
        const dsMove=depthSignatureStability(sig,staticLatchDepth);
        staticMedianMm=dsMove.median*1000;staticP90Mm=dsMove.p90*1000;staticCloseRatio=dsMove.closeRatio;
        const moveMed=Math.max(.030,depthNoiseMed*2.8),moveP90=Math.max(.070,depthNoiseP90*2.5);
        staticDepthMedLimitMm=moveMed*1000;staticDepthP90LimitMm=moveP90*1000;
        if(dsMove.valid>=80&&(dsMove.median>moveMed||dsMove.p90>moveP90)){moveEvidence++;reason.push('Depth geometry');}
      }
      staticMoveScore=moveEvidence;
      const immediate=(odom.ok&&(odomTrans>=.060||odomRot>=5*Math.PI/180))||(t265Usable&&t265LastDeltaDeg>=4.0);
      if(immediate||(rgbFresh&&moveEvidence>=2))staticExitStreak++; else if(rgbFresh)staticExitStreak=0;
      if(staticExitStreak<2&&!immediate){
        lastRel=I();state='STATIC';lostCount=0;staticFrames++;clearPending();
      }else{
        staticLatched=false;staticExitReason=reason.join('+')||'strong ICP motion';staticExitStreak=0;staticStreak=0;
        staticRefDepth=sig.slice();staticRefRgb=rgb?cloneRgb(rgb):null;staticLatchDepth=null;staticLatchRgb=null;
      }
    }

    let staticCandidate=false;
    if(!staticLatched&&!rgbStreamActive){
      if(!staticRefDepth)staticRefDepth=sig.slice();
      const ds=depthSignatureStability(sig,staticRefDepth);
      staticMedianMm=ds.median*1000;staticP90Mm=ds.p90*1000;staticCloseRatio=ds.closeRatio;
      const medLimit=Math.max(.014,depthNoiseMed*1.55),p90Limit=Math.max(.038,depthNoiseP90*1.55);
      staticDepthMedLimitMm=medLimit*1000;staticDepthP90LimitMm=p90Limit*1000;
      staticCandidate=odom.ok&&ds.valid>=80&&ds.median<=medLimit&&ds.p90<=p90Limit&&ds.closeRatio>=.50&&odomTrans<=.018&&odomRot<=1.2*Math.PI/180;
      if(staticCandidate)staticStreak++;else staticStreak=0;
      staticRefDepth=sig.slice();
    }else if(!staticLatched&&rgbFresh&&rgb){
      if(staticRefDepth&&staticRefRgb&&rgb.count>=12&&staticRefRgb.count>=12){
        const ds=depthSignatureStability(sig,staticRefDepth);
        staticMedianMm=ds.median*1000;staticP90Mm=ds.p90*1000;staticCloseRatio=ds.closeRatio;
        const rr=matchRgbFeatures(rgb,staticRefRgb);
        staticRgbMatches=rr.matches||0;staticRgbInliers=rr.inliers||0;staticRgbMotionPx=rr.motionPx??Infinity;staticRgbAngleDeg=Math.abs(rr.angleDeg??Infinity);
        const rgbStable=rr.matches>=12&&rr.inliers>=9&&rr.ratio>=.65&&rr.error<=6.5&&staticRgbMotionPx<=.60&&staticRgbAngleDeg<=.35&&Math.abs((rr.scale??1)-1)<=.008;
        const odomSmall=odom.ok&&odomTrans<=.015&&odomRot<=.8*Math.PI/180;
        if(rgbStable&&odomSmall&&Number.isFinite(ds.median)&&ds.median<.025&&ds.p90<.060){
          const a=depthNoiseSamples<4 ? .35 : .12;
          depthNoiseMed=depthNoiseMed*(1-a)+ds.median*a;
          depthNoiseP90=depthNoiseP90*(1-a)+ds.p90*a;
          depthNoiseSamples++;
        }
        const medLimit=Math.max(.014,depthNoiseMed*1.55),p90Limit=Math.max(.038,depthNoiseP90*1.55);
        staticDepthMedLimitMm=medLimit*1000;staticDepthP90LimitMm=p90Limit*1000;
        const depthStable=ds.valid>=80&&ds.median<=medLimit&&ds.p90<=p90Limit&&ds.closeRatio>=.48;
        staticCandidate=rgbStable&&odomSmall&&depthStable;
        if(staticCandidate)staticStreak++;else staticStreak=0;
      }else staticStreak=0;
      staticRefDepth=sig.slice();staticRefRgb=cloneRgb(rgb);staticLastRgbSeq=rgbSeq;
    }
    const staticHold=!staticLatched&&staticStreak>=STATIC_STREAK_N&&odom.ok&&odomTrans<=.030&&odomRot<=2.5*Math.PI/180;
    const sane=odom.ok&&(t265Bridge?(odomTrans<.60&&odomRot<95*Math.PI/180):(odomTrans<.25&&odomRot<25*Math.PI/180));
    if(staticLatched&&!needsMapRecovery){
      // already frozen above; do not run odometry integration or map anchoring
    }else if(staticHold&&!needsMapRecovery){
      staticLatched=true;staticExitStreak=0;staticLatchDepth=sig.slice();staticLatchRgb=rgb?cloneRgb(rgb):null;
      if(keyframes.length===1&&keyframes[0].t265&&t265CurrentQ&&t265CurrentP&&t265CurrentConf>=2){
        const originRel=t265RelativeOptical(keyframes[0].t265.q,keyframes[0].t265.p,t265CurrentQ,t265CurrentP);
        if(trans(originRel)<=.050&&rot(originRel)<=3*Math.PI/180)pose=keyframes[0].pose.slice();
      }
      lastRel=I();state='STATIC';lostCount=0;staticFrames++;clearPending();
    }else if(sane){
      const prevPose=pose.slice();
      pose=mul(pose,odom.T);lastRel=odom.T.slice();state=t265Bridge?'T265_BRIDGE':'TRACKING';lostCount=0;
      if(!t265Bridge&&keyframes.length&&frame%2===0){
        const la=findLocalAnchor(depth,pose,maxCorr);
        if(la){
          const corr=rel(pose,la.pose);localCorrectionM=trans(corr);localCorrectionDeg=rot(corr)*180/Math.PI;
          if(localCorrectionM<=.12&&localCorrectionDeg<=7){
            pose=la.pose;lastRel=rel(prevPose,pose);localAnchors++;localLocked=true;localTrusted=!!la.trusted;localAnchorFrame=la.anchorFrame||0;
            inliers=la.inliers;rmse=la.rmse;
          }else localAnchorRejects++;
        }
      }
      if(keyframes.length&&frame-lastMapMatchFrame>=MAP_CHECK_INTERVAL){
        lastMapMatchFrame=frame;
        const mm=findMapMatch(depth,rgb,maxCorr,false);
        if(mm){
          const before=pose.slice(),dr=rel(before,mm.pose);mapDriftM=trans(dr);mapDriftDeg=rot(dr)*180/Math.PI;
          const verdict=mapCorrectionAccept(mm,mapDriftM,mapDriftDeg,false,rgbStreamActive);
          if(verdict.ok){
            const needsConfirm=verdict.confirm;
            confirmHits=needsConfirm?confirmCandidate(mm.anchorFrame):2;
            if(!needsConfirm||confirmHits>=2){
              const correction=mul(mm.pose,inv(before));
              if(mapMode==='mapping')applyMapCorrection(correction,lastTrustedFrame);
              pose=mm.pose;lastRel=I();mapMatches++;mapLocked=true;state='MAP_LOCKED';lastTrustedFrame=frame;lastTrustedAnchorFrame=mm.anchorFrame;keyframeHoldUntil=Math.max(keyframeHoldUntil,frame+12);clearPending();
              inliers=mm.inliers;rmse=mm.rmse;rgbMatches=mm.rgbMatches||0;rgbInliers=mm.rgbInliers||0;rgbInlierRatio=mm.rgbInlierRatio||0;
            }else{
              state='MAP_CONFIRM';inliers=mm.inliers;rmse=mm.rmse;rgbMatches=mm.rgbMatches||0;rgbInliers=mm.rgbInliers||0;rgbInlierRatio=mm.rgbInlierRatio||0;
            }
          }else{
            mapRejects++;lastRejectReason=verdict.reason;mm.rejectReason=verdict.reason;clearPending();
          }
        }
      }
    }else{
      lostCount++;staticStreak=0;staticLatched=false;staticExitStreak=0;staticLatchDepth=null;staticLatchRgb=null;staticRefDepth=null;staticRefRgb=null;staticLastRgbSeq=-1;needsMapRecovery=keyframes.length>0;
      const r=findMapMatch(depth,rgb,maxCorr,true);
      if(r){
        const recDr=rel(pose,r.pose),recM=trans(recDr),recDeg=rot(recDr)*180/Math.PI;mapDriftM=recM;mapDriftDeg=recDeg;const verdict=mapCorrectionAccept(r,recM,recDeg,true,rgbStreamActive);
        if(verdict.ok){
          confirmHits=confirmCandidate(r.anchorFrame);
          if(confirmHits>=2){
            const beforeRecovery=pose.slice(),repair=mul(r.pose,inv(beforeRecovery));
            if(mapMode==='mapping'){applyMapCorrection(repair,lastTrustedFrame,false);mapRepairs++;}
            pose=r.pose;lastRel=I();inliers=r.inliers;rmse=r.rmse;rgbMatches=r.rgbMatches||0;rgbInliers=r.rgbInliers||0;rgbInlierRatio=r.rgbInlierRatio||0;relocalized++;mapMatches++;lostCount=0;staticStreak=0;staticLatched=false;staticExitStreak=0;staticLatchDepth=null;staticLatchRgb=null;mapLocked=true;needsMapRecovery=false;state='RELOCALIZED';lastTrustedFrame=frame;lastTrustedAnchorFrame=r.anchorFrame;keyframeHoldUntil=Math.max(keyframeHoldUntil,frame+30);lastMapMatchFrame=frame;clearPending();
          }else{
            state='RECOVERY_CONFIRM';inliers=r.inliers;rmse=r.rmse;rgbMatches=r.rgbMatches||0;rgbInliers=r.rgbInliers||0;rgbInlierRatio=r.rgbInlierRatio||0;
          }
        }else{mapRejects++;lastRejectReason=verdict.reason;state='LOST';clearPending();}
      }else state='LOST';
    }
  }else if(keyframes.length){
    const r=findMapMatch(depth,rgb,maxCorr,true);
    if(r){
      const dr=rel(pose,r.pose);mapDriftM=trans(dr);mapDriftDeg=rot(dr)*180/Math.PI;
      const verdict=mapCorrectionAccept(r,mapDriftM,mapDriftDeg,true,rgbStreamActive);
      if(verdict.ok){
        confirmHits=confirmCandidate(r.anchorFrame);
        if(confirmHits>=2){
          const beforeRecovery=pose.slice(),repair=mul(r.pose,inv(beforeRecovery));
          if(mapMode==='mapping'){applyMapCorrection(repair,lastTrustedFrame,false);mapRepairs++;}
          pose=r.pose;lastRel=I();inliers=r.inliers;rmse=r.rmse;rgbMatches=r.rgbMatches||0;rgbInliers=r.rgbInliers||0;rgbInlierRatio=r.rgbInlierRatio||0;relocalized++;mapMatches++;mapLocked=true;needsMapRecovery=false;state='RELOCALIZED';lastTrustedFrame=frame;lastTrustedAnchorFrame=r.anchorFrame;keyframeHoldUntil=Math.max(keyframeHoldUntil,frame+30);lastMapMatchFrame=frame;clearPending();
        } else{needsMapRecovery=true;state='RECOVERY_CONFIRM';inliers=r.inliers;rmse=r.rmse;rgbMatches=r.rgbMatches||0;rgbInliers=r.rgbInliers||0;rgbInlierRatio=r.rgbInlierRatio||0;}
      }else{mapRejects++;lastRejectReason=verdict.reason;needsMapRecovery=true;state=mapMode==='localization'?'SEARCHING_MAP':'LOST';clearPending();}
    }else{needsMapRecovery=true;state=mapMode==='localization'?'SEARCHING_MAP':'LOST';}
  }

  if(mapMode==='mapping'&&!needsMapRecovery&&state!=='LOST'&&state!=='SEARCHING_MAP'&&state!=='RECOVERY_CONFIRM'&&state!=='MAP_CONFIRM'&&state!=='STATIC'&&state!=='T265_BRIDGE'&&frame>=keyframeHoldUntil&&keyframes.length<MAX_KEYFRAMES){
    if(isKeyframe(pose)||keyframes.length===0)addKeyframe(depth,rgb,pose);
  }

  if(state==='TRACKING'||state==='MAP_LOCKED'||state==='RELOCALIZED'||state==='INITIALIZED'||state==='MAP_CONFIRM'||state==='STATIC'||state==='T265_BRIDGE'){prev=depth;prevRgb=rgb?cloneRgb(rgb):null;if(!rgbStreamActive&&!staticRefDepth)staticRefDepth=depthBlockSignature(depth);}
  const e=euler(pose);
  path.push({frame,x:pose[3],y:pose[7],z:pose[11]});if(path.length>3000)path.shift();
  const now=performance.now();if(lastTime){const hz=1000/(now-lastTime);slamHz=slamHz?slamHz*.9+hz*.1:hz;}lastTime=now;
  postMessage({type:'result',state,inliers,rmse,computeMs:performance.now()-t0,slamHz,keyframes:keyframes.length,loops,relocalized,mapMatches,mapRejects,mapRepairs,localAnchors,localAnchorRejects,mapMode,mapLocked,localLocked,localTrusted,localAnchorFrame,localCorrectionM,localCorrectionDeg,mapDriftM,mapDriftDeg,mapRevision,rgbFeatures:rgb?.count||0,rgbMatches,rgbInliers,rgbInlierRatio,confirmHits,pendingAnchor,needsMapRecovery,lastRejectReason,rgbStreamActive,lastTrustedFrame,lastTrustedAnchorFrame,staticStreak,staticFrames,staticMedianMm,staticP90Mm,staticCloseRatio,staticOdomMm,staticOdomDeg,staticRgbMatches,staticRgbInliers,staticRgbMotionPx,staticRgbAngleDeg,staticDepthMedLimitMm,staticDepthP90LimitMm,staticFreshRgb,depthNoiseSamples,depthNoiseMedMm:depthNoiseMed*1000,depthNoiseP90Mm:depthNoiseP90*1000,staticLatched,staticExitStreak,staticMoveScore,staticExitReason,t265Available:!!t265Q,t265Confidence:t265Conf,t265Used,t265Retry,t265Bridge,t265DeltaDeg:t265LastDeltaDeg,t265DeltaM:t265LastDeltaM,t265IcpDisagreeDeg,t265IcpTranslationDiffM,t265UsedFrames,t265FallbackFrames,t265RetryFrames,t265BridgeFrames,mapT265Init:!!(keyframes.length&&keyframes.some(q=>q.t265)),
    pose:{x:pose[3],y:pose[7],z:pose[11],roll:e[0],pitch:e[1],yaw:e[2]},matrix:Array.from(pose),path:path.slice(-1500),keyframePath:keyframePath.slice(-500)});
}
function depthBlockSignature(d){
  const bs=16,nx=Math.floor(w/bs),ny=Math.floor(h/bs),out=new Float32Array(nx*ny);
  for(let by=0;by<ny;by++)for(let bx=0;bx<nx;bx++){
    const vals=[];
    const x0=bx*bs,y0=by*bs;
    for(let y=y0+2;y<y0+bs-2;y+=4)for(let x=x0+2;x<x0+bs-2;x+=4){
      const z=d[y*w+x];if(z>.15&&z<6)vals.push(z);
    }
    if(vals.length>=5){vals.sort((a,b)=>a-b);out[by*nx+bx]=vals[Math.floor(vals.length/2)];}
  }
  return out;
}
function depthSignatureStability(a,b){
  const diffs=[];let valid=0,close=0;
  const n=Math.min(a.length,b.length);
  for(let i=0;i<n;i++){
    const za=a[i],zb=b[i];if(!(za>.15&&zb>.15))continue;
    const d=Math.abs(za-zb);valid++;if(d<=.010)close++;if(d<=.10)diffs.push(d);
  }
  if(diffs.length<50)return{valid,median:Infinity,p90:Infinity,closeRatio:valid?close/valid:0};
  diffs.sort((x,y)=>x-y);
  return{valid,median:diffs[Math.floor(diffs.length*.5)],p90:diffs[Math.floor(diffs.length*.9)],closeRatio:valid?close/valid:0};
}
function poseGuess(t265Rel=null){
  if(!t265Rel)return lastRel.slice();
  return withRotation(lastRel,t265Rel);
}
function isKeyframe(T){if(!keyframes.length)return true;const q=keyframes[keyframes.length-1],gap=frame-q.frame;if(gap<8)return false;if(gap>=30)return true;const d=rel(q.pose,T);return trans(d)>.10||rot(d)>7*Math.PI/180;}
function cloneRgb(r){return r?{global:r.global.slice(),xy:r.xy.slice(),brief:r.brief.slice(),count:r.count,width:r.width,height:r.height}:null;}
function t265FromKeyframe(q){
  if(!q?.t265||!t265CurrentQ||!t265CurrentP||t265CurrentConf<2)return null;
  const T=t265RelativeOptical(q.t265.q,q.t265.p,t265CurrentQ,t265CurrentP);
  return trans(T)<=2.5&&rot(T)<=150*Math.PI/180?T:null;
}
function addKeyframe(depth,rgb,T){
  if(keyframes.length>=MAX_KEYFRAMES)return false;
  const first=keyframes.length===0;
  const kf={frame,pose:T.slice(),depth:depth.slice(),dd:depthDesc(depth),rgb:cloneRgb(rgb),
    t265:(t265CurrentQ&&t265CurrentP&&t265CurrentConf>=2)?{q:t265CurrentQ.slice(),p:t265CurrentP.slice(),conf:t265CurrentConf}:null};
  keyframes.push(kf);
  if(first){lastTrustedFrame=frame;lastTrustedAnchorFrame=frame;}
  keyframePath=keyframes.map(q=>({x:q.pose[3],y:q.pose[7],z:q.pose[11]}));mapRevision++;return true;
}
function findLocalAnchor(depth,poseGuessWorld,maxCorr){
  if(!keyframes.length)return null;
  const idxs=[],seen=new Set();
  const add=i=>{if(i>=0&&i<keyframes.length&&!seen.has(i)){seen.add(i);idxs.push(i);}};
  // Always keep the last globally trusted map keyframe in the local candidate set.
  if(lastTrustedAnchorFrame){
    let ti=-1,bestDf=Infinity;
    for(let i=0;i<keyframes.length;i++){
      const df=Math.abs((keyframes[i].frame||0)-lastTrustedAnchorFrame);
      if(df<bestDf){bestDf=df;ti=i;}
    }
    add(ti);
    if(ti>=0){add(ti-1);add(ti+1);}
  }
  for(let i=keyframes.length-1;i>=Math.max(0,keyframes.length-3);i--)add(i);

  let bestTrusted=null,bestTrustedQ=-Infinity,bestRecent=null,bestRecentQ=-Infinity;
  for(const i of idxs){
    const q=keyframes[i],gap=frame-(q.frame||0),trusted=Math.abs((q.frame||0)-lastTrustedAnchorFrame)<=1;
    if(!trusted&&(gap<0||gap>55))continue;
    const tinit=t265FromKeyframe(q);
    const initial=tinit||rel(q.pose,poseGuessWorld);
    const initM=trans(initial),initDeg=rot(initial)*180/Math.PI;
    if(initM>(tinit?1.80:(trusted?0.80:0.55))||initDeg>(tinit?120:(trusted?45:35)))continue;
    const r=track(depth,q.depth,initial,Math.max(trusted ? .12 : .10,maxCorr*(trusted?1.5:1.3)),trusted?[14,9,5,4]:[12,7,4],trusted?[4,4,5,5]:[3,4,5]);
    if(!r.ok||r.inliers<(trusted?220:180)||r.rmse>(trusted ? .043 : .040))continue;
    const cyc=rel(initial,r.T),corrM=trans(cyc),corrDeg=rot(cyc)*180/Math.PI;
    if(corrM>(trusted ? .20 : .15)||corrDeg>(trusted?11:9))continue;
    const result={pose:mul(q.pose,r.T),inliers:r.inliers,rmse:r.rmse,anchorFrame:q.frame,trusted,corrM,corrDeg};
    const quality=r.inliers/(1+r.rmse*100+corrM*4+corrDeg*.12);
    if(trusted){if(quality>bestTrustedQ){bestTrustedQ=quality;bestTrusted=result;}}
    else if(quality>bestRecentQ){bestRecentQ=quality;bestRecent=result;}
  }

  // If the trusted map anchor can still see the current frame, prefer it unless it
  // strongly disagrees with the provisional trajectory by more than the allowed correction.
  if(bestTrusted){
    const c=rel(poseGuessWorld,bestTrusted.pose);
    if(trans(c)<=.18&&rot(c)<=10*Math.PI/180)return bestTrusted;
  }
  return bestRecent||bestTrusted;
}
function findMapMatch(depth,rgb,maxCorr,recovery){
  if(!keyframes.length)return null;
  const desc=depthDesc(depth),cand=[],threshold=recovery?0.34:0.24;
  for(const q of keyframes){
    if(mapMode==='mapping'&&!recovery&&frame-(q.frame||0)<45)continue;
    const ds=descDist(desc,q.dd),rs=(rgb&&q.rgb?.global)?rgbDist(rgb.global,q.rgb.global):.5,base=ds*.72+rs*.28;
    if(base<threshold)cand.push({base,ds,rs,q,rgbEval:null,rank:base});
  }
  cand.sort((a,b)=>a.base-b.base);
  const pre=cand.slice(0,recovery?14:10);
  for(const c of pre){
    if(rgb&&c.q.rgb&&rgb.count>=8&&c.q.rgb.count>=8){
      c.rgbEval=matchRgbFeatures(rgb,c.q.rgb);
      const e=c.rgbEval;
      const geomBonus=Math.min(.15,(e.inliers/24)*.10+(e.ratio||0)*.07);
      const contradiction=e.matches>=8&&e.inliers<3?.10:0;
      c.rank=c.base-geomBonus+contradiction;
    }
  }
  pre.sort((a,b)=>a.rank-b.rank);
  let best=null,bestQuality=-Infinity;
  for(const c of pre.slice(0,recovery?9:6)){
    const q=c.q,e=c.rgbEval;
    if(e&&e.matches>=8){
      const strong=e.inliers>=(recovery?6:8)&&e.ratio>=(recovery ? .24 : .30)&&e.error<=(recovery?8.0:6.5);
      if(!strong&&c.ds>(recovery ? .075 : .055))continue;
      if(e.inliers<3&&c.ds>(recovery ? .045 : .035))continue;
    }
    const tinit=t265FromKeyframe(q),initial=tinit||I();
    const r=track(depth,q.depth,initial,Math.max(recovery?0.14:0.11,maxCorr*(recovery?1.8:1.45)),recovery?[16,10,6,4]:[14,9,5,4],recovery?[5,5,6,6]:[4,4,5,5]);
    const minIn=recovery?120:190,maxRmse=recovery?0.055:0.040;
    if(!r.ok||r.inliers<minIn||r.rmse>maxRmse)continue;
    let t265PoseDiffM=Infinity,t265PoseDiffDeg=Infinity,t265Consistent=false;
    if(tinit){
      t265PoseDiffM=translationDiff(r.T,tinit);
      t265PoseDiffDeg=rot(rel(tinit,rotationOnly(r.T)))*180/Math.PI;
      const leverAllowance=.08+2*.10*Math.sin(Math.min(Math.PI,rot(tinit))/2);
      t265Consistent=t265PoseDiffM<=Math.max(.14,leverAllowance)&&t265PoseDiffDeg<=12;
    }
    const back=track(q.depth,depth,inv(r.T),Math.max(.12,maxCorr*1.55),[16,9,5],[3,4,5]);
    if(!back.ok||back.inliers<(recovery?90:130)||back.rmse>(recovery?0.065:0.052))continue;
    const cyc=mul(back.T,r.T);
    if(trans(cyc)>(recovery?0.10:0.065)||rot(cyc)>(recovery?10:6)*Math.PI/180)continue;
    const rgbBoost=e?e.inliers*1.8:0;
    const quality=(r.inliers+rgbBoost)/(1+r.rmse*90+c.rank*4);
    if(quality>bestQuality){
      bestQuality=quality;
      best={pose:mul(q.pose,r.T),inliers:r.inliers,rmse:r.rmse,anchorFrame:q.frame,score:c.rank,
        rgbMatches:e?.matches||0,rgbInliers:e?.inliers||0,rgbInlierRatio:e?.ratio||0,rgbError:e?.error??Infinity,rgbUsable:!!(rgb&&q.rgb&&rgb.count>=12&&q.rgb.count>=12),
        t265Consistent,t265PoseDiffM,t265PoseDiffDeg,t265Init:!!tinit};
    }
  }
  return best;
}
function mapCorrectionAccept(mm,driftM,driftDeg,recovery,rgbActive){
  const matches=mm.rgbMatches||0,inliers=mm.rgbInliers||0,ratio=mm.rgbInlierRatio||0,err=mm.rgbError??Infinity;
  const depthExcellent=mm.inliers>=2500&&mm.rmse<=.028;
  const rgbStrong=matches>=10&&inliers>=7&&ratio>=.55&&err<=7.0;
  const rgbModerate=matches>=8&&inliers>=5&&ratio>=.35&&err<=8.5;
  const rgbVeryStrong=matches>=20&&inliers>=16&&ratio>=.72&&err<=5.5;
  const t265DepthStrong=!!mm.t265Consistent&&mm.inliers>=900&&mm.rmse<=.036;
  if(recovery){
    if(rgbActive){
      if(mm.rgbUsable===false)return{ok:false,confirm:false,reason:'RGB stream active but feature support is insufficient'};
      if(!(rgbStrong||(rgbModerate&&t265DepthStrong)))return{ok:false,confirm:false,reason:'recovery requires RGB or T265-assisted geometry'};
      if(mm.rmse>.036)return{ok:false,confirm:false,reason:'recovery depth error high'};
      return{ok:true,confirm:true,reason:t265DepthStrong?'RGB+Depth+T265 recovery candidate':'RGB+Depth recovery candidate'};
    }
    if(!depthExcellent)return{ok:false,confirm:false,reason:'depth-only recovery not strong enough'};
    return{ok:true,confirm:true,reason:'depth-only recovery candidate'};
  }
  if(driftM>.45||driftDeg>20){
    const t265Verified=rgbModerate&&t265DepthStrong&&mm.inliers>=1200&&mm.rmse<=.034;
    if(!((rgbVeryStrong&&mm.rmse<=.026&&mm.inliers>=1500)||t265Verified))return{ok:false,confirm:false,reason:'very large correction not verified'};
    return{ok:true,confirm:true,reason:t265Verified?'T265-assisted verified large loop closure':'verified large loop closure'};
  }
  if(driftM>.15||driftDeg>5){
    const rgbDepthStrong=matches>=16&&inliers>=12&&ratio>=.65&&err<=6.0&&mm.rmse<=.032;
    const t265Verified=rgbModerate&&t265DepthStrong;
    if(!(rgbDepthStrong||t265Verified))return{ok:false,confirm:false,reason:'large correction lacks RGB/depth/T265 confidence'};
    return{ok:true,confirm:true,reason:t265Verified?'T265-assisted verified medium loop closure':'verified medium loop closure'};
  }
  if(rgbActive&&matches>=6&&(inliers<4||ratio<.40))return{ok:false,confirm:false,reason:'RGB geometry weak'};
  return{ok:true,confirm:false,reason:'small map lock accepted'};
}
function confirmCandidate(anchorFrame){
  if(pendingAnchor===anchorFrame&&frame-pendingLastFrame<=MAP_CHECK_INTERVAL+3)pendingHits++;
  else{pendingAnchor=anchorFrame;pendingHits=1;}
  pendingLastFrame=frame;return pendingHits;
}
function clearPending(){pendingAnchor=-1;pendingHits=0;pendingLastFrame=-999;}
function matchRgbFeatures(a,b){
  if(!a||!b||a.count<4||b.count<4||a.brief.length<a.count*16||b.brief.length<b.count*16)return{matches:0,inliers:0,ratio:0,error:Infinity};
  const raw=[];
  for(let i=0;i<a.count;i++){
    let best=999,second=999,bj=-1;
    for(let j=0;j<b.count;j++){const d=ham128(a.brief,i,b.brief,j);if(d<best){second=best;best=d;bj=j;}else if(d<second)second=d;}
    if(bj<0||best>48||best*100>=second*82)continue;
    let rev=999,ri=-1;for(let k=0;k<a.count;k++){const d=ham128(b.brief,bj,a.brief,k);if(d<rev){rev=d;ri=k;}}
    if(ri===i)raw.push({i,j:bj,d:best});
  }
  raw.sort((x,y)=>x.d-y.d);
  if(raw.length<3)return{matches:raw.length,inliers:0,ratio:0,error:Infinity};
  const test=raw.slice(0,Math.min(28,raw.length));let bestIn=0,bestErr=Infinity,bestScale=NaN,bestCs=NaN,bestSn=NaN,bestTx=NaN,bestTy=NaN;
  const pairLimit=Math.min(18,test.length);
  for(let u=0;u<pairLimit;u++)for(let v=u+1;v<pairLimit;v++){
    const m1=test[u],m2=test[v],ax1=a.xy[m1.i*2],ay1=a.xy[m1.i*2+1],ax2=a.xy[m2.i*2],ay2=a.xy[m2.i*2+1],bx1=b.xy[m1.j*2],by1=b.xy[m1.j*2+1],bx2=b.xy[m2.j*2],by2=b.xy[m2.j*2+1];
    const acx=ax2-ax1,acy=ay2-ay1,bcx=bx2-bx1,bcy=by2-by1,la=Math.hypot(acx,acy),lb=Math.hypot(bcx,bcy);
    if(la<5||lb<5)continue;const scale=lb/la;if(scale<.5||scale>2)continue;
    const cs=(acx*bcx+acy*bcy)/(la*lb),sn=(acx*bcy-acy*bcx)/(la*lb);
    const tx=bx1-scale*(cs*ax1-sn*ay1),ty=by1-scale*(sn*ax1+cs*ay1);
    let inn=0,err=0;
    for(const m of raw){
      const x=a.xy[m.i*2],y=a.xy[m.i*2+1],qx=b.xy[m.j*2],qy=b.xy[m.j*2+1];
      const px=scale*(cs*x-sn*y)+tx,py=scale*(sn*x+cs*y)+ty,e=Math.hypot(px-qx,py-qy);
      if(e<6.5){inn++;err+=e;}
    }
    const avg=inn?err/inn:Infinity;if(inn>bestIn||(inn===bestIn&&avg<bestErr)){bestIn=inn;bestErr=avg;bestScale=scale;bestCs=cs;bestSn=sn;bestTx=tx;bestTy=ty;}
  }
  let motionPx=Infinity,angleDeg=Infinity;
  if(Number.isFinite(bestScale)){
    const cx=(a.width||160)/2,cy=(a.height||120)/2;
    const px=bestScale*(bestCs*cx-bestSn*cy)+bestTx,py=bestScale*(bestSn*cx+bestCs*cy)+bestTy;
    motionPx=Math.hypot(px-cx,py-cy);angleDeg=Math.atan2(bestSn,bestCs)*180/Math.PI;
  }
  return{matches:raw.length,inliers:bestIn,ratio:raw.length?bestIn/raw.length:0,error:bestErr,scale:bestScale,motionPx,angleDeg};
}
function ham128(a,ai,b,bi){let n=0,oa=ai*16,ob=bi*16;for(let k=0;k<16;k++)n+=POP8[a[oa+k]^b[ob+k]];return n;}
function applyMapCorrection(C,anchorFrame,countLoop=true){
  for(const q of keyframes)if((q.frame||0)>anchorFrame)q.pose=mul(C,q.pose);
  for(const p of path)if((p.frame||0)>anchorFrame){const v=tpnt(C,[p.x,p.y,p.z]);p.x=v[0];p.y=v[1];p.z=v[2];}
  keyframePath=keyframes.map(q=>({x:q.pose[3],y:q.pose[7],z:q.pose[11]}));if(countLoop)loops++;mapRevision++;
}
function exportMap(){
  try{
    const obj={format:'d435-browser-map',version:3,created:new Date().toISOString(),w,h,K,mapRevision,keyframes:keyframes.map(q=>({
      frame:q.frame,pose:Array.from(q.pose),depth_mm_b64:depthToB64(q.depth),
      t265:q.t265?{q:Array.from(q.t265.q),p:Array.from(q.t265.p),conf:q.t265.conf||0}:null,
      rgb:q.rgb?{global_b64:bytesToB64(q.rgb.global),xy_b64:bytesToB64(new Uint8Array(q.rgb.xy.buffer,q.rgb.xy.byteOffset,q.rgb.xy.byteLength)),brief_b64:bytesToB64(q.rgb.brief),count:q.rgb.count,width:q.rgb.width,height:q.rgb.height}:null
    }))};
    postMessage({type:'mapExport',map:obj,keyframes:keyframes.length});
  }catch(e){postMessage({type:'mapError',message:'Map export failed: '+e.message});}
}
function importMap(obj){
  try{
    if(!obj||obj.format!=='d435-browser-map'||![1,2,3].includes(obj.version)||!Array.isArray(obj.keyframes))throw new Error('Unsupported map file.');
    if(obj.w!==320||obj.h!==240)throw new Error('Map resolution must be 320x240.');
    w=obj.w;h=obj.h;K=obj.K||K;
    const loaded=[];
    for(const q of obj.keyframes){
      const d=b64ToDepth(q.depth_mm_b64,w*h);let rgb=null;
      if(obj.version===2&&q.rgb){
        const xyb=b64ToBytes(q.rgb.xy_b64),brief=b64ToBytes(q.rgb.brief_b64);
        rgb={global:b64ToBytes(q.rgb.global_b64),xy:new Uint16Array(xyb.buffer.slice(xyb.byteOffset,xyb.byteOffset+xyb.byteLength)),brief,count:q.rgb.count||0,width:q.rgb.width||160,height:q.rgb.height||120};
      }else if(obj.version===1&&q.rgb){
        rgb={global:new Uint8Array(q.rgb),xy:new Uint16Array(0),brief:new Uint8Array(0),count:0,width:160,height:120};
      }
      const t265=(obj.version>=3&&q.t265&&validQuat(q.t265.q)&&validVec3(q.t265.p))?{q:normalizeQuat(q.t265.q),p:q.t265.p.map(Number),conf:Number(q.t265.conf)||0}:null;
      loaded.push({frame:Number(q.frame)||0,pose:new Float64Array(q.pose),depth:d,dd:depthDesc(d),rgb,t265});
    }
    if(!loaded.length)throw new Error('Map contains no keyframes.');
    keyframes=loaded;mapMode='localization';mapRevision=Number(obj.mapRevision)||1;mapMatches=0;mapRejects=0;mapRepairs=0;loops=0;relocalized=0;resetTracking();
    lastTrustedFrame=keyframes.length?keyframes[keyframes.length-1].frame:0;lastTrustedAnchorFrame=lastTrustedFrame;needsMapRecovery=true;postMessage({type:'mapImported',keyframes:keyframes.length,mapMode,mapRevision,rgbKeyframes:keyframes.filter(q=>q.rgb?.count>0).length});
  }catch(e){postMessage({type:'mapError',message:'Map import failed: '+e.message});}
}
function bytesToB64(b){let bin='';const step=0x6000;for(let i=0;i<b.length;i+=step)bin+=String.fromCharCode(...b.subarray(i,Math.min(b.length,i+step)));return btoa(bin);}
function b64ToBytes(s){const bin=atob(s),b=new Uint8Array(bin.length);for(let i=0;i<bin.length;i++)b[i]=bin.charCodeAt(i);return b;}
function depthToB64(d){
  const u=new Uint16Array(d.length);for(let i=0;i<d.length;i++)u[i]=d[i]>0?Math.max(1,Math.min(65535,Math.round(d[i]*1000))):0;
  const b=new Uint8Array(u.buffer);let bin='';const step=0x6000;for(let i=0;i<b.length;i+=step)bin+=String.fromCharCode(...b.subarray(i,Math.min(b.length,i+step)));return btoa(bin);
}
function b64ToDepth(s,n){
  const bin=atob(s),b=new Uint8Array(bin.length);for(let i=0;i<bin.length;i++)b[i]=bin.charCodeAt(i);
  const u=new Uint16Array(b.buffer),d=new Float32Array(n);for(let i=0;i<n&&i<u.length;i++)d[i]=u[i]?u[i]/1000:0;return d;
}
function depthDesc(d){const gx=16,gy=12,out=new Uint16Array(gx*gy);for(let by=0;by<gy;by++)for(let bx=0;bx<gx;bx++){let s=0,n=0;const x0=Math.floor(bx*w/gx),x1=Math.floor((bx+1)*w/gx),y0=Math.floor(by*h/gy),y1=Math.floor((by+1)*h/gy);for(let y=y0;y<y1;y+=3)for(let x=x0;x<x1;x+=3){const z=d[y*w+x];if(z>.15&&z<6){s+=z;n++;}}out[by*gx+bx]=n?Math.min(65535,Math.round((s/n)*10000)):0;}return out;}
function descDist(a,b){let s=0,n=0;for(let i=0;i<a.length;i++){if(a[i]&&b[i]){s+=Math.min(1,Math.abs(a[i]-b[i])/12000);n++;}}return n?s/n:1;}
function rgbDist(a,b){let dot=0,aa=0,bb=0;for(let i=0;i<a.length;i++){const x=a[i]-128,y=b[i]-128;dot+=x*y;aa+=x*x;bb+=y*y;}return 1-(dot/(Math.sqrt(aa*bb)+1e-6)+1)*.5;}
function track(curr,ref,initial,maxCorr,steps=[12,7,4],iters=[4,5,6]){let T=initial.slice(),fin=0,rm=Infinity;for(let si=0;si<steps.length;si++){const step=steps[si];for(let it=0;it<iters[si];it++){const H=new Float64Array(36),g=new Float64Array(6);let count=0,sum=0;for(let v=2;v<h-2;v+=step)for(let u=2;u<w-2;u+=step){const z=curr[v*w+u];if(!(z>.12))continue;const p=deproj(u,v,z),tp=tpnt(T,p);if(tp[2]<=.1)continue;const pu=Math.round(K.fx*tp[0]/tp[2]+K.cx),pv=Math.round(K.fy*tp[1]/tp[2]+K.cy);if(pu<2||pv<2||pu>=w-2||pv>=h-2)continue;const qz=ref[pv*w+pu];if(!(qz>.12)||Math.abs(qz-tp[2])>maxCorr)continue;const n=normal(ref,pu,pv);if(!n)continue;const q=deproj(pu,pv,qz),rx=tp[0]-q[0],ry=tp[1]-q[1],rz=tp[2]-q[2],r=n[0]*rx+n[1]*ry+n[2]*rz;if(Math.abs(r)>maxCorr)continue;const wt=Math.abs(r)<.02?1:.02/Math.abs(r),J=[tp[1]*n[2]-tp[2]*n[1],tp[2]*n[0]-tp[0]*n[2],tp[0]*n[1]-tp[1]*n[0],n[0],n[1],n[2]];for(let i=0;i<6;i++){g[i]+=wt*J[i]*r;for(let j=0;j<6;j++)H[i*6+j]+=wt*J[i]*J[j];}count++;sum+=r*r;}fin=count;rm=count?Math.sqrt(sum/count):Infinity;if(count<45)return{ok:false,T,inliers:count,rmse:rm};for(let i=0;i<6;i++)H[i*6+i]+=1e-6;const rhs=new Float64Array(6);for(let i=0;i<6;i++)rhs[i]=-g[i];const dx=solve6(H,rhs);if(!dx)return{ok:false,T,inliers:count,rmse:rm};const mag=Math.hypot(...dx);if(!Number.isFinite(mag)||mag>.45)return{ok:false,T,inliers:count,rmse:rm};T=mul(exp6(dx),T);if(mag<1e-5)break;}}return{ok:fin>=65&&rm<Math.max(.045,maxCorr*.75),T,inliers:fin,rmse:rm};}
function normal(d,u,v){const zl=d[v*w+u-1],zr=d[v*w+u+1],zu=d[(v-1)*w+u],zd=d[(v+1)*w+u];if(!(zl&&zr&&zu&&zd))return null;const l=deproj(u-1,v,zl),r=deproj(u+1,v,zr),a=deproj(u,v-1,zu),b=deproj(u,v+1,zd),ax=r[0]-l[0],ay=r[1]-l[1],az=r[2]-l[2],bx=b[0]-a[0],by=b[1]-a[1],bz=b[2]-a[2];let nx=ay*bz-az*by,ny=az*bx-ax*bz,nz=ax*by-ay*bx,m=Math.hypot(nx,ny,nz);if(m<1e-7)return null;nx/=m;ny/=m;nz/=m;if(nz>0){nx=-nx;ny=-ny;nz=-nz;}return[nx,ny,nz];}
function deproj(u,v,z){return[(u-K.cx)*z/K.fx,(v-K.cy)*z/K.fy,z];}
function validQuat(q){
  return Array.isArray(q)&&q.length===4&&q.every(Number.isFinite)&&Math.hypot(q[0],q[1],q[2],q[3])>.5;
}
function validVec3(v){return Array.isArray(v)&&v.length===3&&v.every(Number.isFinite);}
function normalizeQuat(q){
  const n=Math.hypot(q[0],q[1],q[2],q[3])||1;
  return[q[0]/n,q[1]/n,q[2]/n,q[3]/n];
}
function quatRot(q){
  const x=q[0],y=q[1],z=q[2],wq=q[3],xx=x*x,yy=y*y,zz=z*z,xy=x*y,xz=x*z,yz=y*z,wx=wq*x,wy=wq*y,wz=wq*z;
  return[1-2*(yy+zz),2*(xy-wz),2*(xz+wy),2*(xy+wz),1-2*(xx+zz),2*(yz-wx),2*(xz-wy),2*(yz+wx),1-2*(xx+yy)];
}
function mat3Mul(a,b){
  const o=new Array(9).fill(0);for(let r=0;r<3;r++)for(let c=0;c<3;c++)for(let k=0;k<3;k++)o[r*3+c]+=a[r*3+k]*b[k*3+c];return o;
}
function mat3T(a){return[a[0],a[3],a[6],a[1],a[4],a[7],a[2],a[5],a[8]];}
function t265RelativeOptical(prevQ,prevP,currQ,currP){
  // T265 pose: X-right/Y-up/Z-back. D435 optical: X-right/Y-down/Z-forward.
  // Compute prev_T_curr in the T265 frame, then change basis with S = Rx(pi).
  const Rp=quatRot(prevQ),Rc=quatRot(currQ),Rpt=mat3T(Rp),Rt=mat3Mul(Rpt,Rc);
  const dw=[currP[0]-prevP[0],currP[1]-prevP[1],currP[2]-prevP[2]];
  const tt=[
    Rpt[0]*dw[0]+Rpt[1]*dw[1]+Rpt[2]*dw[2],
    Rpt[3]*dw[0]+Rpt[4]*dw[1]+Rpt[5]*dw[2],
    Rpt[6]*dw[0]+Rpt[7]*dw[1]+Rpt[8]*dw[2]
  ];
  const S=[1,0,0,0,-1,0,0,0,-1],Ro=mat3Mul(mat3Mul(S,Rt),S),to=[tt[0],-tt[1],-tt[2]];
  const T=I();T[0]=Ro[0];T[1]=Ro[1];T[2]=Ro[2];T[4]=Ro[3];T[5]=Ro[4];T[6]=Ro[5];T[8]=Ro[6];T[9]=Ro[7];T[10]=Ro[8];T[3]=to[0];T[7]=to[1];T[11]=to[2];return T;
}
function translationDiff(a,b){return Math.hypot(a[3]-b[3],a[7]-b[7],a[11]-b[11]);}
function rotationOnly(T){
  const o=I();o[0]=T[0];o[1]=T[1];o[2]=T[2];o[4]=T[4];o[5]=T[5];o[6]=T[6];o[8]=T[8];o[9]=T[9];o[10]=T[10];return o;
}
function withRotation(T,R){
  const o=T.slice();o[0]=R[0];o[1]=R[1];o[2]=R[2];o[4]=R[4];o[5]=R[5];o[6]=R[6];o[8]=R[8];o[9]=R[9];o[10]=R[10];return o;
}
function I(){return new Float64Array([1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1]);}
function mul(a,b){const o=new Float64Array(16);for(let r=0;r<4;r++)for(let c=0;c<4;c++){let s=0;for(let k=0;k<4;k++)s+=a[r*4+k]*b[k*4+c];o[r*4+c]=s;}return o;}
function tpnt(T,p){return[T[0]*p[0]+T[1]*p[1]+T[2]*p[2]+T[3],T[4]*p[0]+T[5]*p[1]+T[6]*p[2]+T[7],T[8]*p[0]+T[9]*p[1]+T[10]*p[2]+T[11]];}
function inv(T){return new Float64Array([T[0],T[4],T[8],-(T[0]*T[3]+T[4]*T[7]+T[8]*T[11]),T[1],T[5],T[9],-(T[1]*T[3]+T[5]*T[7]+T[9]*T[11]),T[2],T[6],T[10],-(T[2]*T[3]+T[6]*T[7]+T[10]*T[11]),0,0,0,1]);}
function rel(a,b){return mul(inv(a),b);}
function trans(T){return Math.hypot(T[3],T[7],T[11]);}
function rot(T){return Math.acos(Math.max(-1,Math.min(1,(T[0]+T[5]+T[10]-1)/2)));}
function exp6(x){const wx=x[0],wy=x[1],wz=x[2],th=Math.hypot(wx,wy,wz),T=I();let R;if(th<1e-9)R=[1,-wz,wy,wz,1,-wx,-wy,wx,1];else{const x1=wx/th,y1=wy/th,z1=wz/th,c=Math.cos(th),s=Math.sin(th),C=1-c;R=[c+x1*x1*C,x1*y1*C-z1*s,x1*z1*C+y1*s,y1*x1*C+z1*s,c+y1*y1*C,y1*z1*C-x1*s,z1*x1*C-y1*s,z1*y1*C+x1*s,c+z1*z1*C];}T[0]=R[0];T[1]=R[1];T[2]=R[2];T[4]=R[3];T[5]=R[4];T[6]=R[5];T[8]=R[6];T[9]=R[7];T[10]=R[8];T[3]=x[3];T[7]=x[4];T[11]=x[5];return T;}
function solve6(A,b){const M=new Float64Array(42);for(let r=0;r<6;r++){for(let c=0;c<6;c++)M[r*7+c]=A[r*6+c];M[r*7+6]=b[r];}for(let c=0;c<6;c++){let p=c;for(let r=c+1;r<6;r++)if(Math.abs(M[r*7+c])>Math.abs(M[p*7+c]))p=r;if(Math.abs(M[p*7+c])<1e-10)return null;if(p!==c)for(let j=c;j<7;j++){const t=M[c*7+j];M[c*7+j]=M[p*7+j];M[p*7+j]=t;}const d=M[c*7+c];for(let j=c;j<7;j++)M[c*7+j]/=d;for(let r=0;r<6;r++)if(r!==c){const f=M[r*7+c];for(let j=c;j<7;j++)M[r*7+j]-=f*M[c*7+j];}}const x=new Float64Array(6);for(let i=0;i<6;i++)x[i]=M[i*7+6];return x;}
function euler(T){const pitch=Math.asin(Math.max(-1,Math.min(1,-T[8]))),roll=Math.atan2(T[9],T[10]),yaw=Math.atan2(T[4],T[0]),d=180/Math.PI;return[roll*d,pitch*d,yaw*d];}
function benchmark(){const a=new Float32Array(320*240);for(let y=0;y<240;y++)for(let x=0;x<320;x++)a[y*320+x]=1.2+.001*x+.0007*y+.05*Math.sin(x*.04)*Math.cos(y*.03);w=320;h=240;K={fx:190.951,fy:190.951,cx:159.1145,cy:119.9725};const times=[];for(let i=0;i<18;i++){const t=performance.now();track(a,a,I(),.08);times.push(performance.now()-t);}times.sort((a,b)=>a-b);const median=times[Math.floor(times.length/2)],p95=times[Math.floor(times.length*.95)];postMessage({type:'benchmark',median,p95,recommendedHz:median<20?15:median<35?10:6});}
