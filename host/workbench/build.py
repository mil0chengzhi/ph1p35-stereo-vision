"""从仓库本地源码生成不含内置点云的离线网页。"""
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def build():
    """内联模板中的 CSS/JS；无参数，返回输出路径，读取及模板错误向上传递。"""
    text = (HERE / 'src/index.html').read_text(encoding='utf-8')
    for token, name in [('__CSS__', 'style.css'), ('__CORE__', 'core.js'),
                        ('__VIEWER__', 'viewer.js'), ('__APP__', 'app.js')]:
        source = (HERE / 'src' / name).read_text(encoding='utf-8')
        if '</script' in source.lower():
            raise ValueError('Unsafe embedded script terminator')
        text = text.replace(token, source)
    if any(token in text for token in ['__CSS__', '__CORE__', '__VIEWER__', '__APP__']):
        raise ValueError('Unresolved template')
    target = ROOT / 'docs/index.html'
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(text, encoding='utf-8', newline='\n')
    print(f'Built {target.name}: {target.stat().st_size:,} bytes; no embedded point clouds')
    return target


if __name__ == '__main__':
    build()
