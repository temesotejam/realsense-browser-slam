export class RealSenseSource {
  constructor(){this.depthStream=null;this.rgbStream=null;this.depthVideo=null;this.rgbVideo=null;this.gl=null;this.srcTex=null;this.smallTex=null;this.fb=null;this.program=null;this.vao=null;this.smallW=320;this.smallH=240;this.rawSmall=new Float32Array(this.smallW*this.smallH);this.cameraFps=0;this.lastMediaTime=null;this.label='';this.width=640;this.height=480;this.rgbCanvas=null;this.logger=null;}
  setLogger(fn){this.logger=typeof fn==='function'?fn:null;}
  _log(level,message){try{this.logger?.(level,message);}catch{}}
  async ensurePermission(){const ds=await navigator.mediaDevices.enumerateDevices();if(ds.some(d=>d.kind==='videoinput'&&d.label))return;const s=await navigator.mediaDevices.getUserMedia({video:true,audio:false});s.getTracks().forEach(t=>t.stop());}
  async enumerate(){if(!navigator.mediaDevices?.getUserMedia)throw new Error('getUserMedia is unavailable.');await this.ensurePermission();return (await navigator.mediaDevices.enumerateDevices()).filter(d=>d.kind==='videoinput'&&/realsense/i.test(d.label));}
  async start(depthId,rgbId=''){
    await this.stop();
    const depthAttempts=[
      {name:'deviceId-only',video:{deviceId:{exact:depthId}}},
      {name:'640x480',video:{deviceId:{exact:depthId},width:{ideal:640},height:{ideal:480}}},
      {name:'848x480',video:{deviceId:{exact:depthId},width:{ideal:848},height:{ideal:480}}},
    ];
    const depthOpen=await this.openVideoWithFallback('Depth',depthAttempts);
    this.depthStream=depthOpen.stream;this.depthVideo=depthOpen.video;
    const track=this.depthStream.getVideoTracks()[0],settings=track.getSettings();this.width=settings.width||this.depthVideo.videoWidth||640;this.height=settings.height||this.depthVideo.videoHeight||480;this.label=track.label||'RealSense Depth';
    this._log('INFO',`Depth active: profile=${depthOpen.name} label="${this.label}" actual=${this.width}x${this.height} @ ${settings.frameRate||'?'} fps`);
    try{const caps=track.getCapabilities?.()||{};this._log('INFO',`Depth capability keys: ${Object.keys(caps).sort().join(', ')||'(none)'}`);}catch{}
    if(rgbId){
      try{
        const rgbOpen=await this.openVideoWithFallback('RGB',[
          {name:'deviceId-only',video:{deviceId:{exact:rgbId}}},
          {name:'640x480',video:{deviceId:{exact:rgbId},width:{ideal:640},height:{ideal:480}}},
        ]);
        this.rgbStream=rgbOpen.stream;this.rgbVideo=rgbOpen.video;
        const rt=this.rgbStream.getVideoTracks()[0],rs=rt.getSettings();this._log('INFO',`RGB active: profile=${rgbOpen.name} label="${rt.label||'RGB'}" actual=${rs.width||this.rgbVideo.videoWidth}x${rs.height||this.rgbVideo.videoHeight} @ ${rs.frameRate||'?'} fps`);
      }catch(e){this._log('WARN',`RGB optional stream unavailable: ${e.name||'Error'}: ${e.message}`);this.rgbStream=null;this.rgbVideo=null;}
    }
    this.initDepthGl();this.lastMediaTime=null;this.cameraFps=0;
    return {label:this.label,width:this.width,height:this.height,settings,rgb:!!this.rgbVideo,openProfile:depthOpen.name};
  }
  async openVideoWithFallback(kind,attempts){
    let lastError=null;
    for(const a of attempts){
      const t0=performance.now();let stream=null,video=null;
      this._log('INFO',`${kind} open attempt: ${a.name} constraints=${JSON.stringify(a.video)}`);
      try{
        stream=await navigator.mediaDevices.getUserMedia({video:a.video,audio:false});
        video=makeVideo(stream);
        await waitVideo(video,8000);
        await video.play();
        const track=stream.getVideoTracks()[0],settings=track.getSettings();
        this._log('INFO',`${kind} open success: ${a.name} in ${(performance.now()-t0).toFixed(0)} ms, settings=${JSON.stringify(settings)}`);
        return {stream,video,name:a.name};
      }catch(e){
        lastError=e;
        this._log('WARN',`${kind} open failed: ${a.name} after ${(performance.now()-t0).toFixed(0)} ms — ${e.name||'Error'}: ${e.message}`);
        if(stream)stream.getTracks().forEach(t=>t.stop());
        if(video)video.srcObject=null;
        await delay(250);
      }
    }
    throw lastError||new Error(`${kind} stream could not be opened.`);
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
  async stop(){for(const s of [this.depthStream,this.rgbStream])if(s)s.getTracks().forEach(t=>t.stop());for(const v of [this.depthVideo,this.rgbVideo])if(v)v.srcObject=null;if(this.gl){try{this.gl.deleteTexture(this.srcTex);this.gl.deleteTexture(this.smallTex);this.gl.deleteFramebuffer(this.fb);this.gl.deleteProgram(this.program);this.gl.deleteVertexArray(this.vao);}catch{}}this.depthStream=this.rgbStream=this.depthVideo=this.rgbVideo=this.gl=this.srcTex=this.smallTex=this.fb=this.program=this.vao=null;this.rgbCanvas=null;}
}
function makeVideo(stream){const v=document.createElement('video');v.autoplay=true;v.muted=true;v.playsInline=true;v.srcObject=stream;return v;}
function waitVideo(v,timeoutMs=7000){return new Promise((resolve,reject)=>{if(v.readyState>=2&&v.videoWidth)return resolve();const t=setTimeout(()=>reject(new Error('Timed out waiting for camera frame.')),timeoutMs);v.addEventListener('loadeddata',()=>{clearTimeout(t);resolve();},{once:true});v.addEventListener('error',()=>{clearTimeout(t);reject(new Error('Video element error.'));},{once:true});});}
function setTex(gl){gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MIN_FILTER,gl.NEAREST);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MAG_FILTER,gl.NEAREST);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_WRAP_S,gl.CLAMP_TO_EDGE);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_WRAP_T,gl.CLAMP_TO_EDGE);}
function clear(gl){while(gl.getError()!==gl.NO_ERROR){}}
function compile(gl,type,s){const sh=gl.createShader(type);gl.shaderSource(sh,s);gl.compileShader(sh);if(!gl.getShaderParameter(sh,gl.COMPILE_STATUS))throw new Error(gl.getShaderInfoLog(sh)||'shader compile error');return sh;}
function link(gl,vs,fs){const p=gl.createProgram(),a=compile(gl,gl.VERTEX_SHADER,vs),b=compile(gl,gl.FRAGMENT_SHADER,fs);gl.attachShader(p,a);gl.attachShader(p,b);gl.linkProgram(p);gl.deleteShader(a);gl.deleteShader(b);if(!gl.getProgramParameter(p,gl.LINK_STATUS))throw new Error(gl.getProgramInfoLog(p)||'program link error');return p;}

function delay(ms){return new Promise(resolve=>setTimeout(resolve,ms));}
