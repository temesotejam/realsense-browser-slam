export class LegacyRealSenseSource {
  constructor(){this.depthStream=null;this.rgbStream=null;this.depthVideo=null;this.rgbVideo=null;this.gl=null;this.srcTex=null;this.smallTex=null;this.fb=null;this.program=null;this.vao=null;this.smallW=320;this.smallH=240;this.rawSmall=new Float32Array(this.smallW*this.smallH);this.cameraFps=0;this.lastMediaTime=null;this.label='';this.width=640;this.height=480;this.rgbCanvas=null;this.logger=null;}
  setLogger(fn){this.logger=typeof fn==='function'?fn:null;}
  _log(level,message){try{this.logger?.(level,message);}catch{}}
  async ensurePermission(){const ds=await navigator.mediaDevices.enumerateDevices();if(ds.some(d=>d.kind==='videoinput'&&d.label))return;const s=await navigator.mediaDevices.getUserMedia({video:true,audio:false});s.getTracks().forEach(t=>t.stop());}
  async enumerate(){if(!navigator.mediaDevices?.getUserMedia)throw new Error('getUserMedia is unavailable.');await this.ensurePermission();return (await navigator.mediaDevices.enumerateDevices()).filter(d=>d.kind==='videoinput'&&/realsense/i.test(d.label));}
  async start(depthId,rgbId=''){
    await this.stop();
    this._log('INFO','LEGACY INPUT: using verified depth-float-probe sequence.');
    this._log('INFO','LEGACY INPUT: priming camera permission with getUserMedia({video:true}).');
    let seed=null;
    try{
      seed=await navigator.mediaDevices.getUserMedia({video:true,audio:false});
      const seedTrack=seed.getVideoTracks()[0],seedSettings=seedTrack?.getSettings?.()||{};
      this._log('INFO',`LEGACY INPUT: seed open success label="${seedTrack?.label||'(unknown)'}" settings=${JSON.stringify(seedSettings)}`);
    }catch(e){
      this._log('WARN',`LEGACY INPUT: seed open failed — ${e.name||'Error'}: ${e.message}`);
      throw e;
    }finally{
      if(seed) seed.getTracks().forEach(t=>t.stop());
      seed=null;
    }

    const inputs=(await navigator.mediaDevices.enumerateDevices())
      .filter(d=>d.kind==='videoinput'&&/realsense/i.test(d.label)&&/depth/i.test(d.label));
    this._log('INFO',`LEGACY INPUT: Depth-labeled RealSense inputs after seed=${inputs.length}`);
    for(const d of inputs)this._log('INFO',`LEGACY INPUT: candidate label="${d.label}" id=${shortId(d.deviceId)}`);
    let selected=inputs.find(d=>d.deviceId===depthId)||inputs[0];
    if(!selected) throw new Error('No Depth-labeled RealSense input after legacy seed.');

    this._log('INFO',`LEGACY INPUT: opening exactly as verified probe: deviceId-only label="${selected.label}"`);
    const t0=performance.now();
    this.depthStream=await navigator.mediaDevices.getUserMedia({
      video:{deviceId:{exact:selected.deviceId}},
      audio:false
    });
    this._log('INFO',`LEGACY INPUT: getUserMedia returned in ${(performance.now()-t0).toFixed(0)} ms`);

    this.depthVideo=makeVideo(this.depthStream,true);
    await waitVideo(this.depthVideo,5000);
    const track=this.depthStream.getVideoTracks()[0],settings=track.getSettings();
    this.width=settings.width||this.depthVideo.videoWidth||640;
    this.height=settings.height||this.depthVideo.videoHeight||480;
    this.label=track.label||selected.label||'RealSense Depth';
    this._log('INFO',`LEGACY INPUT: OPEN success ${this.width}x${this.height} @ ${settings.frameRate||'?'} fps label="${this.label}"`);

    // Same old R32F/Z16 capability check before SLAM starts.
    const probe=legacyFloatProbe(this.depthVideo);
    if(!probe.ok) throw new Error(`Legacy R32F/Z16 probe failed: ${probe.reason}`);
    this._log('INFO',`LEGACY INPUT: R32F FLOAT PATH success; likelyZ16=${probe.likely} nonzero=${probe.stats.nonzero} min=${probe.stats.min} max=${probe.stats.max} residualMean=${probe.stats.integerResidualMean}`);
    if(!probe.likely)this._log('WARN','LEGACY INPUT: float path opened but Z16 signature was not confirmed.');

    if(rgbId){
      try{
        this.rgbStream=await navigator.mediaDevices.getUserMedia({video:{deviceId:{exact:rgbId}},audio:false});
        this.rgbVideo=makeVideo(this.rgbStream,true);
        await waitVideo(this.rgbVideo,5000);
        const rt=this.rgbStream.getVideoTracks()[0],rs=rt.getSettings();
        this._log('INFO',`LEGACY INPUT: RGB open success label="${rt.label||'RGB'}" ${rs.width||this.rgbVideo.videoWidth}x${rs.height||this.rgbVideo.videoHeight} @ ${rs.frameRate||'?'} fps`);
      }catch(e){
        this._log('WARN',`LEGACY INPUT: optional RGB unavailable — ${e.name||'Error'}: ${e.message}`);
        if(this.rgbStream)this.rgbStream.getTracks().forEach(t=>t.stop());
        this.rgbStream=null;this.rgbVideo=null;
      }
    }

    this.initDepthGl();this.lastMediaTime=null;this.cameraFps=0;
    return {label:this.label,width:this.width,height:this.height,settings,rgb:!!this.rgbVideo,openProfile:'legacy-verified-deviceId-only'};
  }
  initDepthGl(){
    const c=document.createElement('canvas');c.width=this.smallW;c.height=this.smallH;const gl=c.getContext('webgl2',{alpha:false,antialias:false,premultipliedAlpha:false,preserveDrawingBuffer:false});if(!gl)throw new Error('WebGL2 unavailable.');if(!gl.getExtension('EXT_color_buffer_float'))throw new Error('EXT_color_buffer_float unavailable.');
    const vs=`#version 300 es
in vec2 p;out vec2 uv;void main(){uv=vec2(p.x,1.0-p.y);gl_Position=vec4(p*2.0-1.0,0,1);}`;
    const fs=`#version 300 es
precision highp float;uniform sampler2D d;in vec2 uv;out float o;void main(){o=texture(d,uv).r;}`;
    const program=link(gl,vs,fs);const vao=gl.createVertexArray();gl.bindVertexArray(vao);const b=gl.createBuffer();gl.bindBuffer(gl.ARRAY_BUFFER,b);gl.bufferData(gl.ARRAY_BUFFER,new Float32Array([0,0,1,0,0,1,0,1,1,0,1,1]),gl.STATIC_DRAW);const loc=gl.getAttribLocation(program,'p');gl.enableVertexAttribArray(loc);gl.vertexAttribPointer(loc,2,gl.FLOAT,false,0,0);
    const src=gl.createTexture();gl.bindTexture(gl.TEXTURE_2D,src);setTex(gl);
    const small=gl.createTexture();gl.bindTexture(gl.TEXTURE_2D,small);setTex(gl);gl.texImage2D(gl.TEXTURE_2D,0,gl.R32F,this.smallW,this.smallH,0,gl.RED,gl.FLOAT,null);
    const fb=gl.createFramebuffer();gl.bindFramebuffer(gl.FRAMEBUFFER,fb);gl.framebufferTexture2D(gl.FRAMEBUFFER,gl.COLOR_ATTACHMENT0,gl.TEXTURE_2D,small,0);if(gl.checkFramebufferStatus(gl.FRAMEBUFFER)!==gl.FRAMEBUFFER_COMPLETE)throw new Error('Depth framebuffer incomplete.');gl.bindFramebuffer(gl.FRAMEBUFFER,null);
    this.gl=gl;this.srcTex=src;this.smallTex=small;this.fb=fb;this.program=program;this.vao=vao;
  }
  grabDepth(scaleM,maxRangeM,metadata=null){
    const gl=this.gl;if(!gl||!this.depthVideo)return null;gl.activeTexture(gl.TEXTURE0);gl.bindTexture(gl.TEXTURE_2D,this.srcTex);clear(gl);gl.texImage2D(gl.TEXTURE_2D,0,gl.R32F,gl.RED,gl.FLOAT,this.depthVideo);let e=gl.getError();if(e!==gl.NO_ERROR)throw new Error(`R32F upload failed 0x${e.toString(16)}`);
    gl.bindFramebuffer(gl.FRAMEBUFFER,this.fb);gl.viewport(0,0,this.smallW,this.smallH);gl.useProgram(this.program);gl.bindVertexArray(this.vao);gl.uniform1i(gl.getUniformLocation(this.program,'d'),0);gl.drawArrays(gl.TRIANGLES,0,6);gl.readPixels(0,0,this.smallW,this.smallH,gl.RED,gl.FLOAT,this.rawSmall);e=gl.getError();gl.bindFramebuffer(gl.FRAMEBUFFER,null);if(e!==gl.NO_ERROR)throw new Error(`R32F readPixels failed 0x${e.toString(16)}`);
    const out=new Float32Array(this.rawSmall.length);for(let y=0;y<this.smallH;y++){const sy=this.smallH-1-y;for(let x=0;x<this.smallW;x++){const n=this.rawSmall[sy*this.smallW+x],z16=Math.max(0,Math.min(65535,Math.round(n*65535))),m=z16*scaleM;out[y*this.smallW+x]=(z16>0&&m<=maxRangeM)?m:0;}}
    if(metadata?.mediaTime!=null){if(this.lastMediaTime!=null){const dt=metadata.mediaTime-this.lastMediaTime;if(dt>0){const hz=1/dt;this.cameraFps=this.cameraFps?this.cameraFps*.9+hz*.1:hz;}}this.lastMediaTime=metadata.mediaTime;}
    return out;
  }
  rgbDescriptor(){
    if(!this.rgbVideo||this.rgbVideo.readyState<2)return null;if(!this.rgbCanvas){this.rgbCanvas=document.createElement('canvas');this.rgbCanvas.width=16;this.rgbCanvas.height=12;}const ctx=this.rgbCanvas.getContext('2d',{willReadFrequently:true});ctx.drawImage(this.rgbVideo,0,0,16,12);const d=ctx.getImageData(0,0,16,12).data,out=new Uint8Array(192);let sum=0;for(let i=0;i<192;i++){const j=i*4,v=(d[j]*77+d[j+1]*150+d[j+2]*29)>>8;out[i]=v;sum+=v;}const mean=sum/192;for(let i=0;i<192;i++)out[i]=Math.max(0,Math.min(255,128+(out[i]-mean)));return out;
  }
  async stop(){for(const s of [this.depthStream,this.rgbStream])if(s)s.getTracks().forEach(t=>t.stop());for(const v of [this.depthVideo,this.rgbVideo])if(v){v.srcObject=null;try{v.remove();}catch{}}if(this.gl){try{this.gl.deleteTexture(this.srcTex);this.gl.deleteTexture(this.smallTex);this.gl.deleteFramebuffer(this.fb);this.gl.deleteProgram(this.program);this.gl.deleteVertexArray(this.vao);}catch{}}this.depthStream=this.rgbStream=this.depthVideo=this.rgbVideo=this.gl=this.srcTex=this.smallTex=this.fb=this.program=this.vao=null;this.rgbCanvas=null;}
}
function makeVideo(stream,attach=false){const v=document.createElement('video');v.autoplay=true;v.muted=true;v.playsInline=true;v.srcObject=stream;if(attach){v.style.cssText='position:fixed;left:-10000px;top:-10000px;width:320px;height:240px;opacity:0.001;pointer-events:none';document.body.appendChild(v);}return v;}
function waitVideo(v,timeoutMs=7000){return new Promise((resolve,reject)=>{if(v.readyState>=2&&v.videoWidth)return resolve();const t=setTimeout(()=>reject(new Error('Timed out waiting for camera frame.')),timeoutMs);v.addEventListener('loadeddata',()=>{clearTimeout(t);resolve();},{once:true});v.addEventListener('error',()=>{clearTimeout(t);reject(new Error('Video element error.'));},{once:true});});}
function setTex(gl){gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MIN_FILTER,gl.NEAREST);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MAG_FILTER,gl.NEAREST);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_WRAP_S,gl.CLAMP_TO_EDGE);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_WRAP_T,gl.CLAMP_TO_EDGE);}
function clear(gl){while(gl.getError()!==gl.NO_ERROR){}}
function compile(gl,type,s){const sh=gl.createShader(type);gl.shaderSource(sh,s);gl.compileShader(sh);if(!gl.getShaderParameter(sh,gl.COMPILE_STATUS))throw new Error(gl.getShaderInfoLog(sh)||'shader compile error');return sh;}
function link(gl,vs,fs){const p=gl.createProgram(),a=compile(gl,gl.VERTEX_SHADER,vs),b=compile(gl,gl.FRAGMENT_SHADER,fs);gl.attachShader(p,a);gl.attachShader(p,b);gl.linkProgram(p);gl.deleteShader(a);gl.deleteShader(b);if(!gl.getProgramParameter(p,gl.LINK_STATUS))throw new Error(gl.getProgramInfoLog(p)||'program link error');return p;}

function delay(ms){return new Promise(resolve=>setTimeout(resolve,ms));}

function legacyFloatProbe(video){
  const w=video.videoWidth,h=video.videoHeight;
  const canvas=document.createElement('canvas');canvas.width=w;canvas.height=h;
  const gl=canvas.getContext('webgl2',{premultipliedAlpha:false,preserveDrawingBuffer:true});
  if(!gl)return {ok:false,reason:'WebGL2 unavailable'};
  const ext=gl.getExtension('EXT_color_buffer_float');
  if(!ext)return {ok:false,reason:'EXT_color_buffer_float unavailable'};
  const tex=gl.createTexture();gl.bindTexture(gl.TEXTURE_2D,tex);
  gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MIN_FILTER,gl.NEAREST);
  gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MAG_FILTER,gl.NEAREST);
  gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_WRAP_S,gl.CLAMP_TO_EDGE);
  gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_WRAP_T,gl.CLAMP_TO_EDGE);
  while(gl.getError()!==gl.NO_ERROR){}
  let thrown=null;
  try{gl.texImage2D(gl.TEXTURE_2D,0,gl.R32F,gl.RED,gl.FLOAT,video);}catch(e){thrown=e;}
  const uploadErr=gl.getError();
  if(thrown||uploadErr!==gl.NO_ERROR)return {ok:false,reason:thrown?`${thrown.name}: ${thrown.message}`:`texImage2D error 0x${uploadErr.toString(16)}`};
  const fb=gl.createFramebuffer();gl.bindFramebuffer(gl.FRAMEBUFFER,fb);
  gl.framebufferTexture2D(gl.FRAMEBUFFER,gl.COLOR_ATTACHMENT0,gl.TEXTURE_2D,tex,0);
  if(gl.checkFramebufferStatus(gl.FRAMEBUFFER)!==gl.FRAMEBUFFER_COMPLETE)return {ok:false,reason:'Framebuffer incomplete'};
  const buf=new Float32Array(w*h);while(gl.getError()!==gl.NO_ERROR){}
  gl.readPixels(0,0,w,h,gl.RED,gl.FLOAT,buf);
  const readErr=gl.getError();if(readErr!==gl.NO_ERROR)return {ok:false,reason:`readPixels error 0x${readErr.toString(16)}`};
  let nonzero=0,min=Infinity,max=-Infinity,resSum=0,n=0;
  for(let i=0;i<buf.length;i++){const v=buf[i];if(!Number.isFinite(v))continue;n++;if(v!==0)nonzero++;if(v<min)min=v;if(v>max)max=v;const x=v*65535;resSum+=Math.abs(x-Math.round(x));}
  const stats={n,nonzero,min,max,integerResidualMean:n?resSum/n:NaN};
  return {ok:true,stats,likely:nonzero>100&&min>=0&&max<=1.0001&&stats.integerResidualMean<0.02};
}
function shortId(id){if(!id)return'(none)';return id.length>18?`${id.slice(0,8)}…${id.slice(-6)}`:id;}
