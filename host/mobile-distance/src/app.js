(() => {
  'use strict';
  const $ = id => document.getElementById(id);
  const S = {mm:625, mode:'direct', connected:false, busy:false, port:null, reader:null,
    readTask:null, pending:null, pumping:false, lastAt:0, timer:null, closing:false,
    tx:0, sent:null, applied:null, rx:[], log:[], bridgeSince:0, poll:null,
    session:0, heartbeat:null, sequence:0, inflight:null};
  const bridgeOrigin = location.protocol === 'http:' && !!location.hostname;
  const directAvailable = !!navigator.serial && window.isSecureContext;
  const format = mm => (mm / 10).toFixed(1);
  const sleep = ms => new Promise(r => setTimeout(r, ms));
  function crc8(bytes) {
    let crc=0;
    for(const b of bytes){crc^=b;for(let n=0;n<8;n++)crc=((crc<<1)^((crc&128)?7:0))&255;}
    return crc;
  }
  function command(mm, seq=0) {
    const data=[0xa5,0x5a,seq&255,0x20,mm&255,mm>>8,0];
    return new Uint8Array([...data,crc8(data)]);
  }
  const hex = bytes => Array.from(bytes,b=>b.toString(16).padStart(2,'0').toUpperCase()).join(' ');
  function log(direction,text){S.log.unshift(new Date().toLocaleTimeString('zh-CN',{hour12:false})+' '+direction+' '+text);S.log=S.log.slice(0,18);$('serialLog').textContent=S.log.join('\n');}
  function notice(text,kind=''){$('statusMessage').textContent=text;$('statusMessage').dataset.kind=kind;}
  function updateValue(mm,send=true){
    S.mm=Math.max(500,Math.min(750,Math.round(Number(mm))||500));
    $('distanceSlider').value=S.mm;$('distanceSlider').setAttribute('aria-valuetext',format(S.mm)+'厘米');$('distanceValue').textContent=format(S.mm);
    const fraction=(S.mm-500)/250,shift=-51+fraction*102,x=239+shift;
    $('distanceSlider').style.setProperty('--fill',fraction*100+'%');$('rangeProgress').style.width=fraction*100+'%';
    $('targetBlock').style.transform='translateX('+shift+'px)';$('beamTop').setAttribute('d','M68 27L'+x+' 12');$('beamBottom').setAttribute('d','M68 48L'+x+' 59');$('distanceLine').setAttribute('d','M76 70H'+x+'M76 67V73M'+x+' 67V73');
    $('commandPreview').textContent='SET '+S.mm+' mm';
    document.querySelectorAll('[data-mm]').forEach(b=>b.classList.toggle('selected',Number(b.dataset.mm)===S.mm));
    $('valueCaption').textContent=S.connected?'设定值 · 滑动自动发送':'设定值 · 连接后自动发送';
    if(send&&S.connected)queue(S.mm);
  }
  function renderConnection(){
    $('connectionBadge').dataset.state=S.connected?'connected':'offline';$('badgeText').textContent=S.connected?(S.mode==='direct'?'蓝牙已连接':'桥接串口已连接'):'未连接设备';
    $('connectButton').textContent=S.busy?'连接中…':S.connected?'断开连接':S.mode==='direct'?'连接蓝牙设备':'连接电脑蓝牙串口';
    $('connectButton').disabled=S.busy||(!S.connected&&(S.mode==='direct'?!directAvailable:!bridgeOrigin));
    $('sendButton').disabled=!S.connected;$('sendButton').textContent=S.connected?'发送当前距离':'连接后发送';
    $('baudRate').disabled=S.connected||S.busy;$('directTab').disabled=S.connected||S.busy;$('bridgeTab').disabled=S.connected||S.busy;$('serialPort').disabled=S.connected||S.busy;
    $('valueCaption').textContent=S.connected?'设定值 · 滑动自动发送':'设定值 · 连接后自动发送';
  }
  function setMode(mode){
    if(S.connected||S.busy)return;S.mode=mode;
    $('directTab').setAttribute('aria-selected',mode==='direct');$('bridgeTab').setAttribute('aria-selected',mode==='bridge');$('directPanel').hidden=mode!=='direct';$('bridgePanel').hidden=mode!=='bridge';$('portsField').hidden=!bridgeOrigin;
    if(mode==='direct'&&!directAvailable)notice('当前浏览器无法直连蓝牙串口。安卓请用新版Chrome打开HTTPS地址，或使用电脑桥接。','error');
    else if(mode==='bridge'&&!bridgeOrigin)notice('电脑桥接请使用启动脚本显示的局域网网址打开本页面。此HTTPS页面用于蓝牙直连。');
    else notice('尚未连接。可以先调整滑块。');
    renderConnection();if(mode==='bridge'&&bridgeOrigin)refreshPorts();
  }
  async function api(path,body){
    const result=await fetch('/api/'+path,{method:body===undefined?'GET':'POST',headers:body===undefined?{}:{'Content-Type':'application/json'},body:body===undefined?undefined:JSON.stringify(body),cache:'no-store',signal:AbortSignal.timeout(5000)});
    let data;try{data=await result.json();}catch{throw new Error('未找到电脑桥接服务，请使用启动脚本提供的网址。');}
    if(!result.ok||data.error)throw new Error(data.error||'桥接请求失败');return data;
  }
  async function refreshPorts(){
    try{const data=await api('ports');$('serialPort').replaceChildren();for(const p of data.ports){const o=document.createElement('option');o.value=p.device;o.textContent=p.device+' · '+p.description;$('serialPort').appendChild(o);}if(!data.ports.length)notice('未发现蓝牙串口。请先在电脑配对HC-05，再刷新。','error');}
    catch(e){notice(e.message,'error');}
  }
  function cancelInflight(message){
    if(S.inflight){const job=S.inflight;S.inflight=null;clearTimeout(job.timeout);job.reject(new Error(message));}
  }
  async function connect(){
    if(S.closing||S.busy)return;if(S.connected){await disconnect();return;}
    S.busy=true;S.closing=false;S.rx=[];S.sent=null;S.applied=null;renderConnection();
    try{
      const baud=Number($('baudRate').value);
      if(S.mode==='direct'){
        S.port=await navigator.serial.requestPort();await S.port.open({baudRate:baud,dataBits:8,stopBits:1,parity:'none',flowControl:'none',bufferSize:1024});
        S.connected=true;S.session++;S.readTask=readDirect(S.session);
      }else{
        const port=$('serialPort').value;if(!port)throw new Error('请先选择HC-05对应的电脑蓝牙串口。');
        const data=await api('connect',{port,baud});S.connected=!!data.connected;S.bridgeSince=data.last_rx_id||0;S.poll=setInterval(pollBridge,100);
      }
      $('sentValue').innerHTML='—<span>cm</span>';$('appliedValue').innerHTML='—<span>cm</span>';$('appliedValue').classList.remove('applied');
      notice('串口已连接，使用BP2F二进制协议发送当前设定。');log('LINK','BP2F / 8-byte request / CRC8');
      S.heartbeat=setInterval(()=>{if(S.connected&&!document.hidden)queue(S.mm);},500);queue(S.mm,true);
    }catch(e){S.connected=false;if(S.port){try{await S.port.close();}catch{}S.port=null;}notice(e.name==='NotFoundError'?'已取消设备选择。':e.message,'error');}
    finally{S.busy=false;renderConnection();}
  }
  async function readDirect(session){
    try{S.reader=S.port.readable.getReader();while(S.connected&&session===S.session){const {value,done}=await S.reader.read();if(done)break;if(value)receive(value);}}
    catch(e){if(!S.closing)notice('蓝牙读取中断：'+e.message,'error');}
    finally{
      if(S.reader){try{S.reader.releaseLock();}catch{}S.reader=null;}
      if(!S.closing&&session===S.session){S.connected=false;S.pending=null;clearTimeout(S.timer);clearInterval(S.heartbeat);cancelInflight('蓝牙连接已断开');renderConnection();notice('连接已断开，距离未继续发送。请重新连接。','error');if(S.port){try{await S.port.close();}catch{}S.port=null;}}
    }
  }
  function receive(chunk){
    S.rx.push(...chunk);if(S.rx.length>4096)S.rx=S.rx.slice(-2048);
    while(S.rx.length>=2){
      if(S.rx[0]!==0x5a||S.rx[1]!==0xa5){S.rx.shift();continue;}
      if(S.rx.length<24)return;
      const reply=S.rx.slice(0,24);
      if(crc8(reply.slice(0,23))!==reply[23]){S.rx.shift();log('DROP','回执CRC错误');continue;}
      S.rx.splice(0,24);log('RX',hex(reply));
      const job=S.inflight;if(!job||reply[2]!==job.seq)continue;
      if(reply[3]!==0){finishReply(new Error(reply[3]===3?'当前板卡固件不支持距离更新，需要BP2F新版固件。':'板端拒绝参数，返回码 '+reply[3]));continue;}
      if(reply[4]!==1||reply[5]!==2||reply[6]!==100||(reply[7]&0x20)===0){finishReply(new Error('回执的版本、板角色或距离能力不匹配BP2F。'));continue;}
      const mm=reply[8]|(reply[9]<<8);
      if(reply[10]!==1||mm<500||mm>750||mm!==job.mm){finishReply(new Error('板端距离无效或与本次发送值不一致。'));continue;}
      S.applied=mm;$('appliedValue').innerHTML=format(mm)+'<span>cm</span>';$('appliedValue').classList.add('applied');
      notice((reply[11]&8)?'板端确认 '+format(mm)+' cm · 距离OSD模式已开启。':'板端确认 '+format(mm)+' cm；按B板K2进入距离OSD模式。','ok');
      finishReply(null);
    }
  }
  function finishReply(error){const job=S.inflight;if(!job)return;S.inflight=null;clearTimeout(job.timeout);error?job.reject(error):job.resolve();}
  function queue(mm,immediate=false){S.pending=mm;if(!S.connected)return;clearTimeout(S.timer);if(!S.pumping)S.timer=setTimeout(pump,immediate?0:Math.max(0,100-(performance.now()-S.lastAt)));}
  async function pump(){
    if(S.pumping||!S.connected||S.pending===null)return;
    S.pumping=true;const session=S.session;
    try{
      while(S.connected&&S.pending!==null&&session===S.session){
        const wait=100-(performance.now()-S.lastAt);if(wait>0)await sleep(wait);
        if(!S.connected||session!==S.session)break;
        const mm=S.pending;S.pending=null;const seq=S.sequence++&255,packet=command(mm,seq);
        let job;
        const ack=new Promise((resolve,reject)=>{job={seq,mm,resolve,reject,timeout:null};S.inflight=job;});
        // A reply may arrive immediately; attach a rejection handler before writing.
        ack.catch(()=>{});
        try{
          if(S.mode==='direct'){const writer=S.port.writable.getWriter();try{await writer.write(packet);}finally{writer.releaseLock();}}
          else await api('distance',{mm,seq});
          if(!S.connected||session!==S.session)break;
          S.lastAt=performance.now();S.sent=mm;S.tx++;$('txCount').textContent=S.tx+' PACKETS';$('sentValue').innerHTML=format(mm)+'<span>cm</span>';log('TX',hex(packet));
          if(S.inflight===job){notice('已发送 '+format(mm)+' cm，等待CRC校验回执。');job.timeout=setTimeout(()=>{if(S.inflight===job)finishReply(new Error('尚未收到有效回执。请检查BP2F固件、HC-05接线和波特率。'));},1400);}
          await ack;
        }catch(e){
          if(S.inflight===job)cancelInflight(e.message);
          if(S.connected)notice(e.message,'error');
        }
      }
    }finally{S.pumping=false;if(S.connected&&S.pending!==null)S.timer=setTimeout(pump,100);}
  }
  let polling=false;
  async function pollBridge(){
    if(polling||!S.connected||S.mode!=='bridge')return;polling=true;
    try{const data=await api('status?since='+S.bridgeSince);if(!data.connected){await disconnect(false);notice('电脑蓝牙串口已断开。','error');return;}for(const row of data.rx||[]){S.bridgeSince=Math.max(S.bridgeSince,row.id);receive(row.bytes||[]);}}
    catch(e){await disconnect(false);notice(e.message,'error');}finally{polling=false;}
  }
  async function disconnect(showNotice=true){
    S.closing=true;S.connected=false;S.pending=null;S.session++;clearTimeout(S.timer);clearInterval(S.poll);clearInterval(S.heartbeat);S.poll=null;cancelInflight('连接已断开');
    if(S.reader){try{await S.reader.cancel();}catch{}}if(S.readTask){try{await S.readTask;}catch{}S.readTask=null;}if(S.port){try{await S.port.close();}catch{}S.port=null;}
    if(S.mode==='bridge'){try{await api('disconnect',{});}catch{}}
    S.busy=false;renderConnection();if(showNotice)notice('已断开连接，距离未继续发送。');log('LINK','连接断开');S.closing=false;
  }
  $('distanceSlider').addEventListener('input',e=>updateValue(e.target.value));$('distanceSlider').addEventListener('change',()=>{if(S.connected)queue(S.mm,true);});
  $('decrease').addEventListener('click',()=>updateValue(S.mm-1));$('increase').addEventListener('click',()=>updateValue(S.mm+1));$('sendButton').addEventListener('click',()=>queue(S.mm,true));
  document.querySelectorAll('[data-mm]').forEach(b=>b.addEventListener('click',()=>updateValue(b.dataset.mm)));
  $('connectButton').addEventListener('click',connect);$('directTab').addEventListener('click',()=>setMode('direct'));$('bridgeTab').addEventListener('click',()=>setMode('bridge'));$('refreshPorts').addEventListener('click',refreshPorts);
  if(navigator.serial)navigator.serial.addEventListener('disconnect',event=>{if((event.port||event.target)===S.port&&!S.closing)disconnect(false).then(()=>notice('蓝牙设备已断开。','error'));});
  document.addEventListener('visibilitychange',()=>{if(document.hidden){S.pending=null;clearTimeout(S.timer);}else if(S.connected)queue(S.mm,true);});
  updateValue(625,false);setMode(bridgeOrigin?'bridge':'direct');
  window.DistanceRemote={format,command,crc8,hex,updateValue,receive,state:S};
})();
