const POP8=new Uint8Array(256);
for(let i=0;i<256;i++){let x=i,c=0;while(x){x&=x-1;c++;}POP8[i]=c;}

export class D435RgbFeatureSource {
  constructor(){
    this.stream=null;
    this.video=null;
    this.label="";
    this.width=0;
    this.height=0;
    this.featureCanvas=null;
    this.logger=null;
    this._briefPairs=null;
  }
  setLogger(fn){this.logger=typeof fn==="function"?fn:null;}
  _log(level,message){try{this.logger?.(level,message);}catch{}}
  async ensurePermission(){
    if(!navigator.mediaDevices?.getUserMedia)throw new Error("getUserMedia is unavailable.");
    const ds=await navigator.mediaDevices.enumerateDevices();
    if(ds.some(d=>d.kind==="videoinput"&&d.label))return;
    const s=await navigator.mediaDevices.getUserMedia({video:true,audio:false});
    s.getTracks().forEach(t=>t.stop());
  }
  async enumerate(){
    await this.ensurePermission();
    return (await navigator.mediaDevices.enumerateDevices()).filter(d=>
      d.kind==="videoinput"&&/realsense/i.test(d.label)&&!/depth/i.test(d.label)&&(/rgb|color|435/i.test(d.label))
    );
  }
  async startAuto(){
    await this.stop();
    const devices=await this.enumerate();
    const selected=devices.find(d=>/rgb/i.test(d.label))||devices[0];
    if(!selected)throw new Error("No browser-visible D435 RGB input found.");
    this._log("INFO",`RGB: opening "${selected.label}"`);
    this.stream=await navigator.mediaDevices.getUserMedia({
      video:{deviceId:{exact:selected.deviceId},width:{ideal:640},height:{ideal:480},frameRate:{ideal:30}},
      audio:false
    });
    this.video=document.createElement("video");
    this.video.autoplay=true;this.video.muted=true;this.video.playsInline=true;this.video.srcObject=this.stream;
    this.video.style.cssText="position:fixed;left:-10000px;top:-10000px;width:320px;height:240px;opacity:0.001;pointer-events:none";
    document.body.appendChild(this.video);
    await waitVideo(this.video,6000);
    await this.video.play();
    const track=this.stream.getVideoTracks()[0],settings=track.getSettings();
    this.label=track.label||selected.label||"RealSense RGB";
    this.width=settings.width||this.video.videoWidth||640;
    this.height=settings.height||this.video.videoHeight||480;
    this._log("INFO",`RGB: OPEN ${this.width}x${this.height} @ ${settings.frameRate||"?"} fps label="${this.label}"`);
    return {label:this.label,width:this.width,height:this.height};
  }
  get active(){return !!(this.video&&this.video.readyState>=2);}
  rgbFeatures(){
    if(!this.active)return null;
    const W=160,H=120;
    if(!this.featureCanvas){
      this.featureCanvas=document.createElement("canvas");
      this.featureCanvas.width=W;this.featureCanvas.height=H;
    }
    const ctx=this.featureCanvas.getContext("2d",{willReadFrequently:true});
    ctx.drawImage(this.video,0,0,W,H);
    const rgba=ctx.getImageData(0,0,W,H).data,gray=new Uint8Array(W*H);
    for(let i=0,j=0;i<gray.length;i++,j+=4)gray[i]=(rgba[j]*77+rgba[j+1]*150+rgba[j+2]*29)>>8;
    const global=new Uint8Array(192);let mean=0;
    for(let by=0;by<12;by++)for(let bx=0;bx<16;bx++){
      let sum=0,n=0,x0=bx*10,y0=by*10;
      for(let y=y0;y<y0+10;y+=2)for(let x=x0;x<x0+10;x+=2){sum+=gray[y*W+x];n++;}
      const v=n?Math.round(sum/n):0;global[by*16+bx]=v;mean+=v;
    }
    mean/=192;
    for(let i=0;i<global.length;i++)global[i]=Math.max(0,Math.min(255,128+(global[i]-mean)));
    const cand=[];
    for(let y=11;y<H-11;y+=2)for(let x=11;x<W-11;x+=2){
      const i=y*W+x,gx=gray[i+1]-gray[i-1],gy=gray[i+W]-gray[i-W],
        d1=gray[i+W+1]-gray[i-W-1],d2=gray[i+W-1]-gray[i-W+1];
      const score=Math.min(Math.abs(gx),Math.abs(gy))*2+Math.min(Math.abs(d1),Math.abs(d2));
      if(score>52)cand.push({x,y,score});
    }
    cand.sort((a,b)=>b.score-a.score);
    const chosen=[];
    for(const c of cand){
      let close=false;
      for(const q of chosen){const dx=c.x-q.x,dy=c.y-q.y;if(dx*dx+dy*dy<49){close=true;break;}}
      if(!close){chosen.push(c);if(chosen.length>=96)break;}
    }
    const xy=new Uint16Array(chosen.length*2),brief=new Uint8Array(chosen.length*16),pairs=this.briefPairs();
    for(let k=0;k<chosen.length;k++){
      const p=chosen[k];xy[k*2]=p.x;xy[k*2+1]=p.y;
      for(let bit=0;bit<128;bit++){
        const o=bit*4,a=gray[(p.y+pairs[o+1])*W+(p.x+pairs[o])],b=gray[(p.y+pairs[o+3])*W+(p.x+pairs[o+2])];
        if(a<b)brief[k*16+(bit>>3)]|=1<<(bit&7);
      }
    }
    return {global,xy,brief,count:chosen.length,width:W,height:H};
  }
  briefPairs(){
    if(this._briefPairs)return this._briefPairs;
    const a=new Int8Array(128*4);let state=0x5f3759df>>>0;
    const rnd=()=>{state^=state<<13;state^=state>>>17;state^=state<<5;return state>>>0;};
    for(let i=0;i<a.length;i++){let v=(rnd()%19)-9;if(v===0)v=(i&1)?1:-1;a[i]=v;}
    this._briefPairs=a;return a;
  }
  async stop(){
    if(this.stream)this.stream.getTracks().forEach(t=>t.stop());
    if(this.video){this.video.srcObject=null;try{this.video.remove();}catch{}}
    this.stream=null;this.video=null;this.featureCanvas=null;this.label="";this.width=0;this.height=0;
  }
}

function waitVideo(v,timeoutMs){
  return new Promise((resolve,reject)=>{
    if(v.readyState>=2&&v.videoWidth)return resolve();
    const t=setTimeout(()=>reject(new Error("Timed out waiting for RGB frame.")),timeoutMs);
    v.addEventListener("loadeddata",()=>{clearTimeout(t);resolve();},{once:true});
    v.addEventListener("error",()=>{clearTimeout(t);reject(new Error("RGB video element error."));},{once:true});
  });
}
