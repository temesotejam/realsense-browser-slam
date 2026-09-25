let prev=null,w=320,h=240,K=null,pose=I(),lastRel=I(),frame=0,keyframes=[],loops=0,relocalized=0,lostCount=0,lastTime=0,slamHz=0,path=[],keyframePath=[];
let mapMode='mapping',mapMatches=0,mapRejects=0,lastMapMatchFrame=-999,mapRevision=0,keyframeHoldUntil=0,pendingAnchor=-1,pendingHits=0,pendingLastFrame=-999;
const MAP_CHECK_INTERVAL=12,MAX_KEYFRAMES=120;
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
function reset(){prev=null;pose=I();lastRel=I();frame=0;keyframes=[];loops=0;relocalized=0;lostCount=0;lastTime=0;slamHz=0;path=[];keyframePath=[];mapMode='mapping';mapMatches=0;mapRejects=0;lastMapMatchFrame=-999;mapRevision++;keyframeHoldUntil=0;pendingAnchor=-1;pendingHits=0;pendingLastFrame=-999;}
function resetTracking(){prev=null;pose=I();lastRel=I();frame=keyframes.reduce((n,q)=>Math.max(n,q.frame||0),0)+1;lostCount=0;lastTime=0;slamHz=0;path=[];keyframePath=keyframes.map(q=>({x:q.pose[3],y:q.pose[7],z:q.pose[11]}));lastMapMatchFrame=-999;mapMatches=0;mapRejects=0;keyframeHoldUntil=frame+12;pendingAnchor=-1;pendingHits=0;pendingLastFrame=-999;}

function processFrame(m){
  const t0=performance.now(),depth=new Float32Array(m.depth),maxCorr=m.maxCorr||.08;
  const rgb=m.rgbGlobal?{global:new Uint8Array(m.rgbGlobal),xy:m.rgbXY?new Uint16Array(m.rgbXY):new Uint16Array(0),brief:m.rgbBrief?new Uint8Array(m.rgbBrief):new Uint8Array(0),count:m.rgbCount||0,width:m.rgbW||160,height:m.rgbH||120}:null;
  w=m.w;h=m.h;K=m.K;frame++;
  let state='INITIALIZED',inliers=0,rmse=0,mapLocked=false,mapDriftM=0,mapDriftDeg=0,rgbMatches=0,rgbInliers=0,rgbInlierRatio=0,confirmHits=0;

  if(prev){
    const odom=track(depth,prev,poseGuess(),maxCorr);
    inliers=odom.inliers;rmse=odom.rmse;
    const sane=odom.ok&&trans(odom.T)<.45&&rot(odom.T)<45*Math.PI/180;
    if(sane){
      pose=mul(pose,odom.T);lastRel=odom.T.slice();state='TRACKING';lostCount=0;
      if(keyframes.length&&frame-lastMapMatchFrame>=MAP_CHECK_INTERVAL){
        lastMapMatchFrame=frame;
        const mm=findMapMatch(depth,rgb,maxCorr,false);
        if(mm){
          const before=pose.slice(),dr=rel(before,mm.pose);mapDriftM=trans(dr);mapDriftDeg=rot(dr)*180/Math.PI;
          const verdict=mapCorrectionAccept(mm,mapDriftM,mapDriftDeg,false,!!(rgb&&rgb.count>=12));
          if(verdict.ok){
            const needsConfirm=verdict.confirm;
            confirmHits=needsConfirm?confirmCandidate(mm.anchorFrame):2;
            if(!needsConfirm||confirmHits>=2){
              const correction=mul(mm.pose,inv(before));
              if(mapMode==='mapping')applyMapCorrection(correction,mm.anchorFrame);
              pose=mm.pose;lastRel=I();mapMatches++;mapLocked=true;state='MAP_LOCKED';keyframeHoldUntil=Math.max(keyframeHoldUntil,frame+12);clearPending();
              inliers=mm.inliers;rmse=mm.rmse;rgbMatches=mm.rgbMatches||0;rgbInliers=mm.rgbInliers||0;rgbInlierRatio=mm.rgbInlierRatio||0;
            }else{
              state='MAP_CONFIRM';inliers=mm.inliers;rmse=mm.rmse;rgbMatches=mm.rgbMatches||0;rgbInliers=mm.rgbInliers||0;rgbInlierRatio=mm.rgbInlierRatio||0;
            }
          }else{
            mapRejects++;mm.rejectReason=verdict.reason;clearPending();
          }
        }
      }
    }else{
      lostCount++;
      const r=findMapMatch(depth,rgb,maxCorr,true);
      if(r){
        const recDr=rel(pose,r.pose),recM=trans(recDr),recDeg=rot(recDr)*180/Math.PI,verdict=mapCorrectionAccept(r,recM,recDeg,true,!!(rgb&&rgb.count>=12));
        if(verdict.ok){
          confirmHits=confirmCandidate(r.anchorFrame);
          if(confirmHits>=2){
            pose=r.pose;lastRel=I();inliers=r.inliers;rmse=r.rmse;rgbMatches=r.rgbMatches||0;rgbInliers=r.rgbInliers||0;rgbInlierRatio=r.rgbInlierRatio||0;relocalized++;mapMatches++;lostCount=0;mapLocked=true;state='RELOCALIZED';keyframeHoldUntil=Math.max(keyframeHoldUntil,frame+30);clearPending();
          }else{
            state='RECOVERY_CONFIRM';inliers=r.inliers;rmse=r.rmse;rgbMatches=r.rgbMatches||0;rgbInliers=r.rgbInliers||0;rgbInlierRatio=r.rgbInlierRatio||0;
          }
        }else{mapRejects++;state='LOST';clearPending();}
      }else state='LOST';
    }
  }else if(keyframes.length){
    const r=findMapMatch(depth,rgb,maxCorr,true);
    if(r){
      const verdict=mapCorrectionAccept(r,Infinity,Infinity,true,!!(rgb&&rgb.count>=12));
      if(verdict.ok){
        confirmHits=confirmCandidate(r.anchorFrame);
        if(confirmHits>=2){pose=r.pose;lastRel=I();inliers=r.inliers;rmse=r.rmse;rgbMatches=r.rgbMatches||0;rgbInliers=r.rgbInliers||0;rgbInlierRatio=r.rgbInlierRatio||0;relocalized++;mapMatches++;mapLocked=true;state='RELOCALIZED';keyframeHoldUntil=Math.max(keyframeHoldUntil,frame+30);clearPending();}
        else{state='RECOVERY_CONFIRM';inliers=r.inliers;rmse=r.rmse;rgbMatches=r.rgbMatches||0;rgbInliers=r.rgbInliers||0;rgbInlierRatio=r.rgbInlierRatio||0;}
      }else{mapRejects++;state=mapMode==='localization'?'SEARCHING_MAP':'INITIALIZED';clearPending();}
    }else state=mapMode==='localization'?'SEARCHING_MAP':'INITIALIZED';
  }

  if(mapMode==='mapping'&&state!=='LOST'&&state!=='SEARCHING_MAP'&&frame>=keyframeHoldUntil&&keyframes.length<MAX_KEYFRAMES){
    if(isKeyframe(pose)||keyframes.length===0)addKeyframe(depth,rgb,pose);
  }

  if(state==='TRACKING'||state==='MAP_LOCKED'||state==='RELOCALIZED'||state==='INITIALIZED')prev=depth;
  const e=euler(pose);
  path.push({frame,x:pose[3],y:pose[7],z:pose[11]});if(path.length>3000)path.shift();
  const now=performance.now();if(lastTime){const hz=1000/(now-lastTime);slamHz=slamHz?slamHz*.9+hz*.1:hz;}lastTime=now;
  postMessage({type:'result',state,inliers,rmse,computeMs:performance.now()-t0,slamHz,keyframes:keyframes.length,loops,relocalized,mapMatches,mapRejects,mapMode,mapLocked,mapDriftM,mapDriftDeg,mapRevision,rgbFeatures:rgb?.count||0,rgbMatches,rgbInliers,rgbInlierRatio,confirmHits,pendingAnchor,
    pose:{x:pose[3],y:pose[7],z:pose[11],roll:e[0],pitch:e[1],yaw:e[2]},matrix:Array.from(pose),path:path.slice(-1500),keyframePath:keyframePath.slice(-500)});
}
function poseGuess(){return lastRel.slice();}
function isKeyframe(T){if(!keyframes.length)return true;const q=keyframes[keyframes.length-1],gap=frame-q.frame;if(gap<8)return false;if(gap>=30)return true;const d=rel(q.pose,T);return trans(d)>.10||rot(d)>7*Math.PI/180;}
function cloneRgb(r){return r?{global:r.global.slice(),xy:r.xy.slice(),brief:r.brief.slice(),count:r.count,width:r.width,height:r.height}:null;}
function addKeyframe(depth,rgb,T){
  if(keyframes.length>=MAX_KEYFRAMES)return false;
  const kf={frame,pose:T.slice(),depth:depth.slice(),dd:depthDesc(depth),rgb:cloneRgb(rgb)};
  keyframes.push(kf);keyframePath=keyframes.map(q=>({x:q.pose[3],y:q.pose[7],z:q.pose[11]}));mapRevision++;return true;
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
    const r=track(depth,q.depth,I(),Math.max(recovery?0.14:0.11,maxCorr*(recovery?1.8:1.45)),recovery?[16,10,6,4]:[14,9,5,4],recovery?[5,5,6,6]:[4,4,5,5]);
    const minIn=recovery?120:190,maxRmse=recovery?0.055:0.040;
    if(!r.ok||r.inliers<minIn||r.rmse>maxRmse)continue;
    const back=track(q.depth,depth,inv(r.T),Math.max(.12,maxCorr*1.55),[16,9,5],[3,4,5]);
    if(!back.ok||back.inliers<(recovery?90:130)||back.rmse>(recovery?0.065:0.052))continue;
    const cyc=mul(back.T,r.T);
    if(trans(cyc)>(recovery?0.10:0.065)||rot(cyc)>(recovery?10:6)*Math.PI/180)continue;
    const rgbBoost=e?e.inliers*1.8:0;
    const quality=(r.inliers+rgbBoost)/(1+r.rmse*90+c.rank*4);
    if(quality>bestQuality){
      bestQuality=quality;
      best={pose:mul(q.pose,r.T),inliers:r.inliers,rmse:r.rmse,anchorFrame:q.frame,score:c.rank,
        rgbMatches:e?.matches||0,rgbInliers:e?.inliers||0,rgbInlierRatio:e?.ratio||0,rgbError:e?.error??Infinity,rgbUsable:!!(rgb&&q.rgb&&rgb.count>=12&&q.rgb.count>=12)};
    }
  }
  return best;
}
function mapCorrectionAccept(mm,driftM,driftDeg,recovery,rgbActive){
  const matches=mm.rgbMatches||0,inliers=mm.rgbInliers||0,ratio=mm.rgbInlierRatio||0,err=mm.rgbError??Infinity;
  const depthExcellent=mm.inliers>=2500&&mm.rmse<=.028;
  const rgbStrong=matches>=10&&inliers>=7&&ratio>=.55&&err<=7.0;
  const rgbVeryStrong=matches>=20&&inliers>=16&&ratio>=.72&&err<=5.5;
  if(recovery){
    if(rgbActive){
      if(!rgbStrong)return{ok:false,confirm:false,reason:'recovery requires RGB geometry'};
      if(mm.rmse>.036)return{ok:false,confirm:false,reason:'recovery depth error high'};
      return{ok:true,confirm:true,reason:'RGB+Depth recovery candidate'};
    }
    if(!depthExcellent)return{ok:false,confirm:false,reason:'depth-only recovery not strong enough'};
    return{ok:true,confirm:true,reason:'depth-only recovery candidate'};
  }
  if(driftM>.45||driftDeg>20){
    if(!(rgbVeryStrong&&mm.rmse<=.026&&mm.inliers>=1500))return{ok:false,confirm:false,reason:'very large correction not verified'};
    return{ok:true,confirm:true,reason:'verified large loop closure'};
  }
  if(driftM>.15||driftDeg>5){
    if(!(matches>=16&&inliers>=12&&ratio>=.65&&err<=6.0&&mm.rmse<=.032))return{ok:false,confirm:false,reason:'large correction lacks RGB/depth confidence'};
    return{ok:true,confirm:true,reason:'verified medium loop closure'};
  }
  if(rgbActive&&matches>=6&&(inliers<4||ratio<.40))return{ok:false,confirm:false,reason:'RGB geometry weak'};
  return{ok:true,confirm:false,reason:'small map lock accepted'};
}
function confirmCandidate(anchorFrame){
  if(pendingAnchor===anchorFrame&&frame-pendingLastFrame<=3)pendingHits++;
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
  const test=raw.slice(0,Math.min(28,raw.length));let bestIn=0,bestErr=Infinity;
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
    const avg=inn?err/inn:Infinity;if(inn>bestIn||(inn===bestIn&&avg<bestErr)){bestIn=inn;bestErr=avg;}
  }
  return{matches:raw.length,inliers:bestIn,ratio:raw.length?bestIn/raw.length:0,error:bestErr};
}
function ham128(a,ai,b,bi){let n=0,oa=ai*16,ob=bi*16;for(let k=0;k<16;k++)n+=POP8[a[oa+k]^b[ob+k]];return n;}
function applyMapCorrection(C,anchorFrame){
  for(const q of keyframes)if((q.frame||0)>anchorFrame)q.pose=mul(C,q.pose);
  for(const p of path)if((p.frame||0)>anchorFrame){const v=tpnt(C,[p.x,p.y,p.z]);p.x=v[0];p.y=v[1];p.z=v[2];}
  keyframePath=keyframes.map(q=>({x:q.pose[3],y:q.pose[7],z:q.pose[11]}));loops++;mapRevision++;
}
function exportMap(){
  try{
    const obj={format:'d435-browser-map',version:2,created:new Date().toISOString(),w,h,K,mapRevision,keyframes:keyframes.map(q=>({
      frame:q.frame,pose:Array.from(q.pose),depth_mm_b64:depthToB64(q.depth),
      rgb:q.rgb?{global_b64:bytesToB64(q.rgb.global),xy_b64:bytesToB64(new Uint8Array(q.rgb.xy.buffer,q.rgb.xy.byteOffset,q.rgb.xy.byteLength)),brief_b64:bytesToB64(q.rgb.brief),count:q.rgb.count,width:q.rgb.width,height:q.rgb.height}:null
    }))};
    postMessage({type:'mapExport',map:obj,keyframes:keyframes.length});
  }catch(e){postMessage({type:'mapError',message:'Map export failed: '+e.message});}
}
function importMap(obj){
  try{
    if(!obj||obj.format!=='d435-browser-map'||![1,2].includes(obj.version)||!Array.isArray(obj.keyframes))throw new Error('Unsupported map file.');
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
      loaded.push({frame:Number(q.frame)||0,pose:new Float64Array(q.pose),depth:d,dd:depthDesc(d),rgb});
    }
    if(!loaded.length)throw new Error('Map contains no keyframes.');
    keyframes=loaded;mapMode='localization';mapRevision=Number(obj.mapRevision)||1;mapMatches=0;mapRejects=0;loops=0;relocalized=0;resetTracking();
    postMessage({type:'mapImported',keyframes:keyframes.length,mapMode,mapRevision,rgbKeyframes:keyframes.filter(q=>q.rgb?.count>0).length});
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
