# 快速体验

## 1 只看点云

下载并解压仓库，用电脑Chrome或Edge双击`docs/index.html`。页面默认没有点云，先点击“导入点云”选择自己的PLY、XYZ、CSV、JSON或项目ZIP；也可以从兼容数据源串口接收。没有内置示例或自动回放。

已有浏览器存档时，可主动点击页脚“恢复上次会话”。未载入数据前，点云导出和存档按钮禁用。

拖动旋转，右键平移，滚轮缩放；六个视角按钮改变观察方式。PLY导出保存XYZ和颜色，六视图保存截图，项目ZIP保存点云与观察参数。STL生成带底板的2.5D浮雕，打印前仍需在切片软件检查，它不会补出物体未被采集的背面。

若浏览器不便使用本地文件，在仓库根目录执行`python -m http.server 8080 --bind 127.0.0.1`，访问`http://127.0.0.1:8080/docs/`。

## 2 查看手机控制台

`docs/remote/index.html`是完整单文件页面。它调节50.0～75.0cm的手动输入值，精度为界面步进0.1cm。网页发送BP2F 8字节请求，等待24字节CRC正确应答后显示确认，不将串口写入成功当作HDMI已应用。

电脑蓝牙桥接可按以下方式启动：

```console
python -m pip install -r host/mobile-distance/requirements.txt
python host/mobile-distance/bridge.py --host 0.0.0.0 --port 8796
```

手机与电脑处于同一可信局域网，访问程序打印的Phone地址。手机不能用电脑的127.0.0.1。页面选择已配对的蓝牙SPP端口后才打开串口；端口被占用时由使用者核对占用程序，不自动终止其他服务。桥接用于本地操作，不作为公网控制服务。

## 3 HDMI与串口数据

HDMI采集卡提供视频，网页可截图或录制。XYZ点云需从PLY文件或CLOUD/1数据接口取得。本仓库未提供完整板级CLOUD/1回读实现，不会因接上采集卡就自动拥有XYZ导出。

摄像头和Web Serial依赖浏览器及安全上下文，通常在HTTPS或localhost使用，并由用户授权。能力以运行时检测为准；手机系统的经典蓝牙SPP不等于浏览器提供Web Bluetooth。参考[Chrome串口文档](https://developer.chrome.com/docs/capabilities/serial)及[MDN摄像头说明](https://developer.mozilla.org/en-US/docs/Web/API/MediaDevices/getUserMedia)。

## 4 从源码重新生成

```console
python host/workbench/build.py
python host/mobile-distance/build.py
python tools/verify_package.py
node tests/workbench.test.cjs
node tests/workbench-app.test.cjs
```

构建仅使用Python标准库，检查不连接硬件。RTL与MCU用途见各自目录的README，不能使用本仓库摘选替代完整板级工程进行下载。
