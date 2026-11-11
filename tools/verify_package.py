"""Read-only publication checks. No network, subprocesses or physical devices."""
from pathlib import Path
from html.parser import HTMLParser
from urllib.parse import urlsplit,unquote
import hashlib,json,re,ast

ROOT=Path(__file__).resolve().parents[1]
TEXT={'.md','.py','.js','.cjs','.v','.sv','.vh','.sdc','.adc','.c','.h','.s','.ld','.txt','.json','.html','.css','.yml','.yaml'}
PROHIBITED={'.git','.openai','node_modules','__pycache__','vendor_originals','al_ip'}
RUNTIME={'live_state.json','live_pid.txt','display_relay.json','local.settings.json'}
SECRET=[re.compile(r'gh[pousr]_[A-Za-z0-9]{30,}'),re.compile(r'github_pat_[A-Za-z0-9_]{40,}'),re.compile(r'-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----')]
PRIVATE=[re.compile(r'(?i)[CD]:[/\\](?:Users|BaiduNetdiskDownload|xwechat_files|Program Files)[/\\]'),re.compile(r'(?i)(?:[0-9a-f]{2}:){5}[0-9a-f]{2}')]

def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def read_source_text(path):
    """读取非空文本路径，归档HDL允许原GB18030编码；返回字符串，解码错误向上传递。"""
    try:return path.read_text(encoding='utf-8-sig')
    except UnicodeDecodeError:
        assert path.is_relative_to(ROOT/'fpga/rtl-archive'),f'Unexpected non-UTF8 source: {path.relative_to(ROOT)}'
        return path.read_text(encoding='gb18030')
def check_local(base,value):
    if not value or value.startswith(('#','data:','blob:')):return
    parsed=urlsplit(value)
    if parsed.scheme or value.startswith('//'):return
    path=unquote(parsed.path)
    if not path:return
    target=(base.parent/path).resolve()
    assert target.is_relative_to(ROOT),f'Outside repository link: {base.relative_to(ROOT)} -> {value}'
    assert target.exists(),f'Missing link: {base.relative_to(ROOT)} -> {value}'

class Links(HTMLParser):
    def __init__(self,path):super().__init__();self.path=path
    def handle_starttag(self,tag,attrs):
        if tag not in {'a','img','link','script','iframe','source'}:return
        for k,v in attrs:
            if k in {'href','src'}:check_local(self.path,v)

def main():
    """校验发布文件与清单；无参数和返回值，失败抛出断言或读取异常，不访问硬件。"""
    files=sorted(p for p in ROOT.rglob('*') if p.is_file() and not any(x in {'.git','__pycache__'} for x in p.relative_to(ROOT).parts))
    for p in files:
        rel=p.relative_to(ROOT)
        assert not PROHIBITED.intersection(rel.parts),f'Excluded directory present: {rel}'
        assert p.name not in RUNTIME and p.suffix.lower() not in {'.bit','.exe','.msi','.dll','.pyc','.log','.logw','.dmp'},f'Unexpected artifact: {rel}'
        assert p.stat().st_size<25*1024*1024,f'File exceeds browser upload budget: {rel}'
        if p.suffix.lower() in TEXT:
            s=read_source_text(p)
            for pattern in SECRET+PRIVATE:assert not pattern.search(s),f'Credential, local path or paired address in {rel}'
            if p.suffix.lower()=='.py':ast.parse(s,filename=str(rel))
            if p.suffix.lower()=='.md':
                for value in re.findall(r'!?\[[^\]]*\]\(([^)]+)\)',s):check_local(p,value.strip('<>'))
            if p.suffix.lower()=='.html' and 'src' not in rel.parts:Links(p).feed(s)
    provenance=json.loads((ROOT/'provenance.json').read_text(encoding='utf-8'))
    for item in provenance['files']:
        p=ROOT/item['file'];assert p.exists() and digest(p)==item['sha256'],f'Copied source changed: {item["file"]}'
    archive_provenance=ROOT/'fpga/rtl-archive/provenance.json'
    if archive_provenance.exists():
        for item in json.loads(archive_provenance.read_text(encoding='utf-8'))['files']:
            p=archive_provenance.parent/item['file']
            assert p.exists() and digest(p)==item['sha256'],f'Archived source changed: {item["file"]}'
    for item in json.loads((ROOT/'fpga/manifest.json').read_text(encoding='utf-8')):
        p=ROOT/item['file']
        assert p.exists() and digest(p)==item['sha256'],f'FPGA manifest mismatch: {item["file"]}'
    removed=('examples/clouds/scene1_original.ply','examples/clouds/scene2_original.ply',
             'fpga/preset-display/rtl/usb_model_staging.v',
             'fpga/preset-display/rtl/preset_view_control.v',
             'fpga/range-demo/rtl/two_cloud_demo_control.v','docs/media/pointcloud-reference.png')
    for name in removed:assert not (ROOT/name).exists(),f'Removed demo file present: {name}'
    html=(ROOT/'docs/index.html').read_text(encoding='utf-8')
    assert all(x not in html for x in ['__INLINE__','__CSS__','__CORE__','__VIEWER__','__APP__'])
    assert all(token not in html for token in ['window.DEMO_PAYLOADS','window.INLINE_DEMO','sampleSelect','loadSample('])
    assert 'id="cloudEmpty"' in html and 'id="cloudLoading" class="loading" hidden' in html
    assert 'bp2f' in (ROOT/'docs/remote/index.html').read_text(encoding='utf-8').lower()
    print(json.dumps({'passed':True,'files':len(files),'total_bytes':sum(p.stat().st_size for p in files),'largest_file_bytes':max(p.stat().st_size for p in files),'copied_sources':len(provenance['files']),'embedded_clouds':0,'local_links_checked':True,'hardware_accessed':False},ensure_ascii=False))
if __name__=='__main__':main()
