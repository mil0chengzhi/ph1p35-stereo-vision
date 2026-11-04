/* Data and integrity checks. No DOM, server, camera or serial port is opened. */
const fs=require('node:fs'),path=require('node:path'),assert=require('node:assert/strict');
const root=path.resolve(__dirname,'..'),C=require('../host/workbench/src/core.js');
(async()=>{
  assert.equal(C.crc32(new TextEncoder().encode('123456789')),0xcbf43926);
  for(const [name,n] of [['scene1_original.ply',373593],['scene2_original.ply',382765]]){
    const m=C.parsePLY(fs.readFileSync(path.join(root,'examples/clouds',name)));
    assert.equal(m.count,n);
    const again=C.parsePLY(await C.exportPLY(m).arrayBuffer());
    assert.equal(again.count,n);
    for(let i=0;i<n;i++)for(let k=0;k<3;k++){
      assert.equal(again.xyz[i*3+k],m.xyz[i*3+k]);assert.equal(again.rgb[i*3+k],m.rgb[i*3+k]);
    }
  }
  const ascii='ply\nformat ascii 1.0\nelement vertex 3\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n0 0 1 255 0 0\n1 0 2 0 255 0\n0 1 3 0 0 255\n';
  assert.equal(C.parsePLY(new TextEncoder().encode(ascii)).count,3);
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
  console.log(JSON.stringify({passed:true,all_sample_XYZ_RGB_preserved:true,CRC_known_vector:true,truncated_PLY_rejected:true,ZIP_corruption_rejected:true,relief_closed:true,relief_positive_volume:true,hardware_accessed:false}));
})().catch(e=>{console.error(e);process.exitCode=1;});
