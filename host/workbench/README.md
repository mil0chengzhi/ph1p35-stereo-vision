# 网页工作台

`src/core.js`处理PLY、XYZ、ZIP/CRC和2.5D浮雕；`viewer.js`绘制点云；`app.js`组织文件、视频、串口及下载；`index.html`和`style.css`提供界面。

在仓库任意位置运行`python host/workbench/build.py`，从本目录源码与examples/clouds的两组原始点云生成`docs/index.html`。Python只需标准库。结果包含CSS、JS和示例数据，没有CDN，可离线打开。

输入和导出边界见快速体验。串口CLOUD/1是数据源接入合同，AP2F当前没有完整回读。摄像头与串口在用户点击后才请求权限。
