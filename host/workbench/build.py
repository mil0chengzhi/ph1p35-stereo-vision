"""Build a self-contained HTML from repository-local sources and original samples."""
from pathlib import Path
import base64,json

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
def build():
    text=(HERE/'src/index.html').read_text(encoding='utf-8')
    for token,name in [('__CSS__','style.css'),('__CORE__','core.js'),('__VIEWER__','viewer.js'),('__APP__','app.js')]:
        source=(HERE/'src'/name).read_text(encoding='utf-8')
        if '</script' in source.lower():raise ValueError('Unsafe embedded script terminator')
        text=text.replace(token,source)
    payloads={f'original{i}':base64.b64encode((ROOT/f'examples/clouds/scene{i}_original.ply').read_bytes()).decode('ascii') for i in [1,2]}
    text=text.replace('window.INLINE_DEMO="__INLINE__";','window.DEMO_PAYLOADS='+json.dumps(payloads,separators=(',',':'))+';')
    if any(x in text for x in ['__CSS__','__CORE__','__VIEWER__','__APP__','__INLINE__']):raise ValueError('Unresolved template')
    target=ROOT/'docs/index.html';target.parent.mkdir(parents=True,exist_ok=True)
    target.write_text(text,encoding='utf-8',newline='\n')
    print(f'Built {target.name}: {target.stat().st_size:,} bytes; 2 computer-preprocessed clouds')
if __name__=='__main__':build()
