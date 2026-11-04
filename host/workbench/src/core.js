(function(root){
  'use strict';
  const enc=new TextEncoder(),dec=new TextDecoder();
  const LIMIT=3000000;
  const types={char:['getInt8',1],int8:['getInt8',1],uchar:['getUint8',1],uint8:['getUint8',1],short:['getInt16',2],int16:['getInt16',2],ushort:['getUint16',2],uint16:['getUint16',2],int:['getInt32',4],int32:['getInt32',4],uint:['getUint32',4],uint32:['getUint32',4],float:['getFloat32',4],float32:['getFloat32',4],double:['getFloat64',8],float64:['getFloat64',8]};
  function finish(xyz,rgb,meta={}){
    if(!xyz.length||xyz.length%3||xyz.length/3>LIMIT)throw Error('点数为空或超过300万点限制。');
    const n=xyz.length/3,good=[];for(let i=0;i<n;i++)if(Number.isFinite(Math.fround(xyz[i*3]))&&Number.isFinite(Math.fround(xyz[i*3+1]))&&Number.isFinite(Math.fround(xyz[i*3+2])))good.push(i);
    if(!good.length)throw Error('文件中没有有限三维坐标。');
    const p=new Float32Array(good.length*3),c=new Uint8Array(good.length*3);let min=[Infinity,Infinity,Infinity],max=[-Infinity,-Infinity,-Infinity];
    good.forEach((old,i)=>{for(let k=0;k<3;k++){const v=Number(xyz[old*3+k]);p[i*3+k]=v;c[i*3+k]=rgb?Math.max(0,Math.min(255,Math.round(rgb[old*3+k]))):[157,213,186][k];min[k]=Math.min(min[k],v);max[k]=Math.max(max[k],v);}});
    return {xyz:p,rgb:c,count:p.length/3,min,max,meta:{...meta,dropped:n-good.length}};
  }
  function parsePLY(buffer){
    const bytes=buffer instanceof Uint8Array?buffer:new Uint8Array(buffer),probe=dec.decode(bytes.subarray(0,Math.min(bytes.length,1048576)));
    if(!probe.startsWith('ply'))throw Error('缺少PLY文件头。');
    const end=probe.indexOf('end_header');if(end<0)throw Error('PLY文件头不完整。');
    const nl=probe.indexOf('\n',end);if(nl<0)throw Error('PLY文件头缺少换行。');
    const offset=enc.encode(probe.slice(0,nl+1)).length,lines=probe.slice(0,end).split(/\r?\n/),elements=[];let current=null,format='';const comments=[];
    for(const line of lines){const a=line.trim().split(/\s+/);if(a[0]==='format')format=a[1];else if(a[0]==='comment')comments.push(line.slice(8));else if(a[0]==='element'){current={name:a[1],count:Number(a[2]),props:[]};if(!Number.isSafeInteger(current.count)||current.count<0||current.count>LIMIT*10)throw Error('PLY元素数量无效。');elements.push(current);}else if(a[0]==='property'&&current){const prop=a[1]==='list'?{list:true,countType:a[2],type:a[3],name:a[4]}:{type:a[1],name:a[2]};if(!types[prop.type]||(prop.list&&!types[prop.countType]))throw Error('不支持的PLY属性类型。');current.props.push(prop);}}
    const vertices=elements.find(e=>e.name==='vertex');if(!vertices||!vertices.count||vertices.count>LIMIT)throw Error('PLY顶点数为空或过大。');
    for(const key of ['x','y','z'])if(!vertices.props.some(p=>p.name===key&&!p.list))throw Error('PLY必须包含x、y、z坐标。');
    const xyz=new Float32Array(vertices.count*3),rgb=new Uint8Array(vertices.count*3);let vi=0;
    const colorName={red:0,green:1,blue:2,r:0,g:1,b:2,diffuse_red:0,diffuse_green:1,diffuse_blue:2};
    const colorProps=vertices.props.filter(p=>colorName[p.name]!==undefined);let normalized=colorProps.length>0&&colorProps.every(p=>['float','double','float32','float64'].includes(p.type));
    if(format==='ascii'){
      const tokens=dec.decode(bytes.subarray(offset)).trim().split(/\s+/);let at=0;
      for(const e of elements)for(let i=0;i<e.count;i++){
        if(e.name==='vertex'){rgb.set([157,213,186],vi*3);}
        for(const prop of e.props){if(prop.list){const n=Number(tokens[at++]);if(!Number.isSafeInteger(n)||n<0||n>1000000)throw Error('PLY列表长度无效。');at+=n;}else{if(at>=tokens.length)throw Error('PLY数据被截断。');const v=Number(tokens[at++]);if(e.name==='vertex'){const k=['x','y','z'].indexOf(prop.name);if(k>=0)xyz[vi*3+k]=v;const ck=colorName[prop.name];if(ck!==undefined)rgb[vi*3+ck]=Math.max(0,Math.min(255,Math.round(v*(normalized?255:1))));}}}if(e.name==='vertex')vi++;
      }
    }else if(format==='binary_little_endian'||format==='binary_big_endian'){
      const view=new DataView(bytes.buffer,bytes.byteOffset,bytes.byteLength),little=format==='binary_little_endian';let at=offset;
      const scalar=type=>{const [fn,n]=types[type];if(at+n>view.byteLength)throw Error('PLY二进制数据被截断。');const v=view[fn](at,little);at+=n;return v;};
      for(const e of elements)for(let i=0;i<e.count;i++){
        if(e.name==='vertex')rgb.set([157,213,186],vi*3);
        for(const prop of e.props){if(prop.list){const n=scalar(prop.countType);if(!Number.isSafeInteger(n)||n<0||n>1000000)throw Error('PLY列表长度无效。');at+=n*types[prop.type][1];if(at>view.byteLength)throw Error('PLY列表被截断。');}else{const v=scalar(prop.type);if(e.name==='vertex'){const k=['x','y','z'].indexOf(prop.name);if(k>=0)xyz[vi*3+k]=v;const ck=colorName[prop.name];if(ck!==undefined)rgb[vi*3+ck]=Math.max(0,Math.min(255,Math.round(v*(normalized?255:1))));}}}if(e.name==='vertex')vi++;
      }
    }else throw Error('不支持的PLY编码。');
    return finish(xyz,rgb,{format:'PLY',comments,origin:comments.some(s=>/synthetic|simulation|模拟/i.test(s))?'simulation':'import'});
  }
  function parseText(text,name='points.xyz'){
    if(name.toLowerCase().endsWith('.json')){
      const data=JSON.parse(text),points=Array.isArray(data)?data:data.points;if(!Array.isArray(points))throw Error('JSON需包含points数组。');if(points.length>LIMIT)throw Error('点数超过限制。');const xyz=[],rgb=[];
      for(const p of points){if(Array.isArray(p)){xyz.push(+p[0],+p[1],+p[2]);rgb.push(p[3]??157,p[4]??213,p[5]??186);}else{xyz.push(+p.x,+p.y,+p.z);rgb.push(p.r??p.red??157,p.g??p.green??213,p.b??p.blue??186);}}
      return finish(xyz,rgb,{format:'JSON',origin:'import'});
    }
    const xyz=[],rgb=[];for(const line of text.split(/\r?\n/)){const s=line.trim();if(!s||s.startsWith('#'))continue;const a=s.split(/[\s,;]+/).map(Number);if(a.length<3)continue;if(!a.slice(0,3).every(Number.isFinite))continue;xyz.push(...a.slice(0,3));rgb.push(...(a.length>=6?a.slice(3,6):[157,213,186]));if(xyz.length/3>LIMIT)throw Error('点数超过限制。');}
    return finish(xyz,rgb,{format:'XYZ/CSV',origin:'import'});
  }
  function parse(buffer,name){return /\.ply$/i.test(name)||dec.decode(new Uint8Array(buffer).subarray(0,3))==='ply'?parsePLY(buffer):parseText(dec.decode(buffer),name);}
  function exportPLY(model){
    const comments=['STEREO WORKBENCH point cloud export','Coordinates preserve input units; scale is not independently calibrated',model.meta.origin==='simulation'?'Synthetic noise simulation; not FPGA measurement':'Source '+String(model.meta.label||model.meta.origin||'user import').replace(/[\r\n]/g,' ')];
    const head=enc.encode('ply\nformat binary_little_endian 1.0\n'+comments.map(c=>'comment '+c+'\n').join('')+'element vertex '+model.count+'\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n');
    const body=new Uint8Array(model.count*15),view=new DataView(body.buffer);for(let i=0;i<model.count;i++){const p=i*15;for(let k=0;k<3;k++){view.setFloat32(p+k*4,model.xyz[i*3+k],true);body[p+12+k]=model.rgb[i*3+k];}}return new Blob([head,body],{type:'application/octet-stream'});
  }
  const table=new Uint32Array(256);for(let n=0;n<256;n++){let c=n;for(let k=0;k<8;k++)c=(c&1)?0xedb88320^(c>>>1):c>>>1;table[n]=c>>>0;}
  function crc32(data){let c=0xffffffff;for(const b of data)c=table[(c^b)&255]^(c>>>8);return (c^0xffffffff)>>>0;}
  function zip(files){
    const now=new Date(),date=((Math.max(1980,now.getFullYear())-1980)<<9)|((now.getMonth()+1)<<5)|now.getDate(),time=(now.getHours()<<11)|(now.getMinutes()<<5)|(now.getSeconds()>>1);
    let offset=0;const chunks=[],central=[];for(const file of files){const name=enc.encode(file.name),data=file.data instanceof Uint8Array?file.data:new Uint8Array(file.data),crc=crc32(data),local=new Uint8Array(30+name.length),v=new DataView(local.buffer);v.setUint32(0,0x04034b50,true);v.setUint16(4,20,true);v.setUint16(6,0x800,true);v.setUint16(10,time,true);v.setUint16(12,date,true);v.setUint32(14,crc,true);v.setUint32(18,data.length,true);v.setUint32(22,data.length,true);v.setUint16(26,name.length,true);local.set(name,30);chunks.push(local,data);
      const dir=new Uint8Array(46+name.length),d=new DataView(dir.buffer);d.setUint32(0,0x02014b50,true);d.setUint16(4,20,true);d.setUint16(6,20,true);d.setUint16(8,0x800,true);d.setUint16(12,time,true);d.setUint16(14,date,true);d.setUint32(16,crc,true);d.setUint32(20,data.length,true);d.setUint32(24,data.length,true);d.setUint16(28,name.length,true);d.setUint32(42,offset,true);dir.set(name,46);central.push(dir);offset+=local.length+data.length;}
    const size=central.reduce((s,x)=>s+x.length,0),end=new Uint8Array(22),e=new DataView(end.buffer);e.setUint32(0,0x06054b50,true);e.setUint16(8,files.length,true);e.setUint16(10,files.length,true);e.setUint32(12,size,true);e.setUint32(16,offset,true);return new Blob([...chunks,...central,end],{type:'application/zip'});
  }
  function unzip(buffer){const b=new Uint8Array(buffer),v=new DataView(b.buffer,b.byteOffset,b.byteLength),files=Object.create(null);let p=0;while(p+30<=b.length&&v.getUint32(p,true)===0x04034b50){const flags=v.getUint16(p+6,true),method=v.getUint16(p+8,true),crc=v.getUint32(p+14,true),n=v.getUint32(p+18,true),nl=v.getUint16(p+26,true),xl=v.getUint16(p+28,true);if(method!==0||(flags&8))throw Error('请导入本上位机导出的项目ZIP；不支持其他压缩方式。');const at=p+30+nl+xl;if(at+n>b.length)throw Error('项目包被截断。');const name=dec.decode(b.subarray(p+30,p+30+nl)),data=b.slice(at,at+n);if(crc32(data)!==crc)throw Error('项目包CRC校验失败。');files[name]=data;p=at+n;}if(!Object.keys(files).length)throw Error('项目包中没有文件。');return files;}
  function relief(model,widthMM=80,heightMM=8,resolution=100){
    if(!Number.isFinite(widthMM)||widthMM<10||widthMM>300||heightMM<1||heightMM>30)throw Error('浮雕尺寸参数无效。');
    const dx=model.max[0]-model.min[0],dy=model.max[1]-model.min[1],dz=model.max[2]-model.min[2];if(dx<=0||dy<=0)throw Error('点云在XY平面中没有足够范围。');
    const nx=resolution,ny=Math.max(12,Math.min(150,Math.round(resolution*dy/dx))),height=new Float32Array(nx*ny);height.fill(-Infinity);const span=dz||1;
    for(let i=0;i<model.count;i++){const x=Math.min(nx-1,Math.floor((model.xyz[i*3]-model.min[0])/dx*(nx-1))),y=Math.min(ny-1,Math.floor((model.xyz[i*3+1]-model.min[1])/dy*(ny-1)));const h=1.6+heightMM*(model.max[2]-model.xyz[i*3+2])/span;height[y*nx+x]=Math.max(height[y*nx+x],h);}
    for(let i=0;i<height.length;i++)if(!Number.isFinite(height[i]))height[i]=1.6;
    const top=[],bottom=[],heightPlanar=widthMM*dy/dx;for(let y=0;y<ny;y++)for(let x=0;x<nx;x++){top.push([widthMM*x/(nx-1),heightPlanar*y/(ny-1),height[y*nx+x]]);bottom.push([widthMM*x/(nx-1),heightPlanar*y/(ny-1),0]);}
    const triangles=[];for(let y=0;y<ny-1;y++)for(let x=0;x<nx-1;x++){const a=y*nx+x,b=a+1,c=a+nx,d=c+1;triangles.push([top[a],top[b],top[d]],[top[a],top[d],top[c]],[bottom[a],bottom[d],bottom[b]],[bottom[a],bottom[c],bottom[d]]);}
    const boundary=[];for(let x=0;x<nx;x++)boundary.push(x);for(let y=1;y<ny;y++)boundary.push(y*nx+nx-1);for(let x=nx-2;x>=0;x--)boundary.push((ny-1)*nx+x);for(let y=ny-2;y>0;y--)boundary.push(y*nx);
    for(let i=0;i<boundary.length;i++){const a=boundary[i],b=boundary[(i+1)%boundary.length];triangles.push([top[a],bottom[a],bottom[b]],[top[a],bottom[b],top[b]]);}
    const bytes=new Uint8Array(84+triangles.length*50),view=new DataView(bytes.buffer);bytes.set(enc.encode('STEREO WORKBENCH 2.5D relief; artificial base; not full object scan').subarray(0,80));view.setUint32(80,triangles.length,true);
    triangles.forEach((t,i)=>{const at=84+i*50,u=t[1].map((x,k)=>x-t[0][k]),v=t[2].map((x,k)=>x-t[0][k]),n=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]],len=Math.hypot(...n)||1;for(let k=0;k<3;k++)view.setFloat32(at+k*4,n[k]/len,true);for(let j=0;j<3;j++)for(let k=0;k<3;k++)view.setFloat32(at+12+j*12+k*4,t[j][k],true);});
    return {blob:new Blob([bytes],{type:'application/octet-stream'}),triangles:triangles.length,widthMM,heightMM,baseMM:1.6,kind:'2.5D relief with closed rectangular base',missing:'Missing XY samples remain at base height'};
  }
  const api={parse,parsePLY,parseText,finish,exportPLY,crc32,zip,unzip,relief};root.CloudCore=api;if(typeof module!=='undefined')module.exports=api;
})(typeof window!=='undefined'?window:globalThis);
