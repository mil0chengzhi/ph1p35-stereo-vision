/* Data and integrity checks. No DOM, server, camera or serial port is opened. */
const assert=require('node:assert/strict'),C=require('../host/workbench/src/core.js');
(async()=>{
  assert.equal(C.crc32(new TextEncoder().encode('123456789')),0xcbf43926);
  const ascii='ply\nformat ascii 1.0\nelement vertex 3\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n0 0 1 255 0 0\n1 0 2 0 255 0\n0 1 3 0 0 255\n';
  const expectedXYZ=[0,0,1,1,0,2,0,1,3],expectedRGB=[255,0,0,0,255,0,0,0,255];
  // 独立按PLY的15字节顶点格式构造测试输入，不使用任何预处理场景或硬件测量数据。
  const header=new TextEncoder().encode(ascii.slice(0,ascii.indexOf('end_header\n')+11).replace('format ascii','format binary_little_endian'));
  const binary=new Uint8Array(header.length+45),body=new DataView(binary.buffer);
  binary.set(header);
  for(let i=0;i<3;i++)for(let k=0;k<3;k++){
    body.setFloat32(header.length+i*15+k*4,expectedXYZ[i*3+k],true);
    body.setUint8(header.length+i*15+12+k,expectedRGB[i*3+k]);
  }
  for(const bytes of [new TextEncoder().encode(ascii),binary]){
    const model=C.parsePLY(bytes),again=C.parsePLY(await C.exportPLY(model).arrayBuffer());
    assert.equal(model.count,3);assert.equal(again.count,3);
    for(const parsed of [model,again]){
      assert.deepEqual(Array.from(parsed.xyz),expectedXYZ);
      assert.deepEqual(Array.from(parsed.rgb),expectedRGB);
    }
  }
  assert.throws(()=>C.parsePLY(binary.subarray(0,-1)));
  assert.throws(()=>C.parsePLY(new TextEncoder().encode(ascii.slice(0,-35))));
  assert.equal(C.parseText('x,y,z,r,g,b\n0,0,1,255,0,0\n1,1,2,0,255,0','file.csv').count,2);
  const archive=await C.zip([{name:'model.ply',data:new TextEncoder().encode(ascii)}]).arrayBuffer();
  assert.equal(C.parsePLY(C.unzip(archive)['model.ply']).count,3);
  const broken=new Uint8Array(archive.slice(0)),zv=new DataView(broken.buffer),offset=30+zv.getUint16(26,true)+zv.getUint16(28,true);
  broken[offset+5]^=1;assert.throws(()=>C.unzip(broken));
  const xyz=[],rgb=[];
  for(let y=0;y<12;y++)for(let x=0;x<16;x++){xyz.push(x,y,2+Math.sin(x/3)*Math.cos(y/2));rgb.push(100,200,150);}
  const relief=C.relief(C.finish(xyz,rgb),80,8,32),stl=await relief.blob.arrayBuffer(),dv=new DataView(stl),tri=dv.getUint32(80,true),edges=new Map();
  let volume=0;
  for(let i=0;i<tri;i++){
    const at=84+i*50,pts=[];
    for(let v=0;v<3;v++)pts.push([0,1,2].map(k=>dv.getFloat32(at+12+v*12+k*4,true)));
    const [a,b,c]=pts;
    volume+=(a[0]*(b[1]*c[2]-b[2]*c[1])+a[1]*(b[2]*c[0]-b[0]*c[2])+a[2]*(b[0]*c[1]-b[1]*c[0]))/6;
    for(const [u,v] of [[0,1],[1,2],[2,0]]){const key=[pts[u].join(','),pts[v].join(',')].sort().join('|');edges.set(key,(edges.get(key)||0)+1);}
  }
  assert.equal(stl.byteLength,84+tri*50);assert.equal(tri,relief.triangles);assert(volume>0);
  for(const n of edges.values())assert.equal(n,2,'Relief surface must have two triangles per edge');
  console.log(JSON.stringify({passed:true,fixture_XYZ_RGB_preserved:true,CRC_known_vector:true,truncated_PLY_rejected:true,ZIP_corruption_rejected:true,relief_closed:true,relief_positive_volume:true,hardware_accessed:false}));
})().catch(e=>{console.error(e);process.exitCode=1;});
