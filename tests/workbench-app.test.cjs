// 使用真实HTML与应用脚本检查数据入口；不启动浏览器、不连接硬件。
const fs=require('node:fs'),path=require('node:path'),vm=require('node:vm');
const assert=require('node:assert/strict'),C=require('../host/workbench/src/core.js');
const source=path.resolve(__dirname,'../host/workbench/src');
const html=fs.readFileSync(path.join(source,'index.html'),'utf8');
const app=fs.readFileSync(path.join(source,'app.js'),'utf8');
const actions=['exportPLY','snapshotCloud','exportViews','exportProject','saveSession','reliefButton'];
const ply=new TextEncoder().encode('ply\nformat ascii 1.0\nelement vertex 3\nproperty float x\nproperty float y\nproperty float z\nend_header\n0 0 1\n1 0 2\n0 1 3\n');

// saved可为空或有效用户存档；返回隔离应用、DOM和访问计数，脚本启动错误直接抛出。
function browser(saved=null){
  const elements=new Map(),calls={storage:0,fetch:0},errors=[];
  for(const match of html.matchAll(/<[a-z][a-z0-9-]*\b([^>]*\bid="([^"]+)"[^>]*)>/gi)){
    const [,attrs,id]=match,classes=new Set((/class="([^"]*)"/.exec(attrs)?.[1]||'').split(/\s+/));
    elements.set(id,{
      disabled:/\bdisabled\b/.test(attrs),hidden:/\bhidden\b/.test(attrs),open:false,
      value:id==='serialBaud'?'1000000':'',handlers:{},textContent:'',innerHTML:'',
      classList:{add:name=>classes.add(name),contains:name=>classes.has(name),toggle(name,on){if(on)classes.add(name);else classes.delete(name);}},
      addEventListener(type,handler){this.handlers[type]=handler;},setAttribute(){},replaceChildren(){},
    });
  }
  const context={
    window:{},CloudCore:C,TextEncoder,TextDecoder,Blob,console:{error:error=>errors.push(error)},
    navigator:{},isSecureContext:true,setTimeout:()=>1,clearTimeout(){},
    document:{getElementById:id=>elements.get(id)||null,querySelectorAll:()=>[],addEventListener(){}},
    fetch(){calls.fetch++;throw Error('Unexpected network request');},
    indexedDB:{open(){
      calls.storage++;
      const request={result:{
        transaction:()=>({objectStore:()=>({get(){
          const read={result:saved};queueMicrotask(()=>read.onsuccess());return read;
        }})}),close(){},
      }};
      queueMicrotask(()=>request.onsuccess());return request;
    }},
    CloudViewer:class {
      constructor(canvas,onChange){assert(canvas);this.yaw=0;this.pitch=10;this.pan=[0,0];this.zoom=1;this.size=1.4;this.density=1;this.grid=true;this.changed=onChange;}
      setModel(model){this.model=model;this.changed();}
    },
  };
  vm.runInNewContext(app,context,{filename:'app.js',timeout:1000});
  assert.deepEqual(errors,[]);
  return {W:context.window.Workbench,elements,calls,context,errors};
}

// state为browser()结果且不可为空；断言尚未载入数据，无返回值，失败抛出断言错误。
function empty(state){
  assert.equal(state.W.state.model,null);
  assert.equal(state.elements.get('cloudEmpty').hidden,false);
  assert.equal(state.elements.get('cloudLoading').hidden,true);
  for(const id of actions)assert.equal(state.elements.get(id).disabled,true,id);
}

// corrupt为布尔值；返回分段CLOUD/1假串口和写入记录，不访问任何设备。
function serialFixture(corrupt=false){
  const header=new TextEncoder().encode('CLOUD/1 PLY '+ply.length+' '+C.crc32(ply).toString(16).padStart(8,'0')+'\n');
  const payload=ply.slice();if(corrupt)payload[payload.length-2]^=1;
  const chunks=[header.subarray(0,7),header.subarray(7),payload.subarray(0,31),payload.subarray(31)],writes=[];
  const reader={read:async()=>({done:false,value:chunks.shift()||new Uint8Array()}),cancel:async()=>{},releaseLock(){}};
  const port={open:async options=>assert.equal(options.baudRate,1000000),close:async()=>{},
    readable:{getReader:()=>reader},writable:{getWriter:()=>({write:async bytes=>writes.push(new TextDecoder().decode(bytes)),releaseLock(){}})}};
  return {port,writes};
}

(async()=>{
  const savedModel=C.parsePLY(ply),saved={xyz:savedModel.xyz,rgb:savedModel.rgb,meta:savedModel.meta,name:'user.ply',view:{},captures:[]};
  const fresh=browser(saved);
  await Promise.resolve();empty(fresh);
  assert.equal(fresh.calls.storage,0,'启动时不能自动恢复可能包含旧示例的缓存');
  assert.equal(fresh.calls.fetch,0);assert.equal(fresh.elements.has('sampleSelect'),false);
  assert.equal('loadSample' in fresh.W,false);
  await assert.rejects(fresh.W.saveSession(),/请先导入或接收点云/);
  await fresh.W.openFile({name:'user.ply',size:ply.length,arrayBuffer:async()=>ply.buffer.slice(0)});
  assert.equal(fresh.W.state.model.count,3);assert.equal(fresh.W.state.model.meta.origin,'import');
  assert.equal(fresh.elements.get('cloudEmpty').hidden,true);
  for(const id of actions)assert.equal(fresh.elements.get(id).disabled,false,id);

  const invalid=browser();
  await assert.rejects(invalid.W.openFile({name:'broken.ply',size:4,arrayBuffer:async()=>new Uint8Array(4).buffer}),/PLY/);
  empty(invalid);assert.equal(invalid.calls.fetch,0);

  const restored=browser(saved);
  assert.equal(await restored.W.restoreSession(),true);
  assert.equal(restored.calls.storage,1);assert.equal(restored.W.state.sourceName,'user.ply');
  assert.equal(restored.W.state.model.count,3);

  for(const corrupt of [false,true]){
    const state=browser(),fixture=serialFixture(corrupt);
    state.context.navigator.serial={requestPort:async()=>fixture.port};
    await state.elements.get('connectSerial').handlers.click();
    assert(state.W.state.serial);
    if(corrupt){await assert.rejects(state.W.receiveCloud(),/CRC32/);empty(state);}
    else{await state.W.receiveCloud();assert.equal(state.W.state.model.count,3);assert.equal(state.W.state.model.meta.origin,'serial');}
    assert.deepEqual(fixture.writes,['CLOUD/1 GET PLY\n']);assert.equal(state.calls.fetch,0);
    assert.deepEqual(state.errors,[]);
  }
  console.log(JSON.stringify({passed:true,empty_start:true,no_automatic_cache_restore:true,file_import:true,manual_restore:true,fragmented_serial:true,CRC_failure_no_fallback:true,hardware_accessed:false}));
})().catch(error=>{console.error(error);process.exitCode=1;});
