"""Build the portable BP2F remote page. Does not start a server or open serial."""
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
def build():
    html=(HERE/'src/template.html').read_text(encoding='utf-8')
    js=(HERE/'src/app.js').read_text(encoding='utf-8')
    if '</script' in js.lower():raise ValueError('Unsafe script terminator')
    target=ROOT/'docs/remote/index.html';target.parent.mkdir(parents=True,exist_ok=True)
    target.write_text(html+'<script>\n'+js+'\n</script>\n</body>\n</html>\n',encoding='utf-8',newline='\n')
    print('Built BP2F remote page; no device accessed')
if __name__=='__main__':build()
