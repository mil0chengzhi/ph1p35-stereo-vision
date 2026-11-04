# BP2F手机距离控制台

`src/app.js`与`src/template.html`生成`docs/remote/index.html`。运行`python host/mobile-distance/build.py`即可重建，预览不需要安装依赖。

可选电脑桥接依赖pyserial：`python -m pip install -r host/mobile-distance/requirements.txt`，然后运行`python host/mobile-distance/bridge.py --host 0.0.0.0 --port 8796`。手机与电脑同网，访问程序打印的Phone地址。桥接只用于可信本地网络，不是经过生产安全评估的公网服务。

启动时不打开串口；只有页面明确选择蓝牙SPP端口并点击连接才打开。数值为手动输入50.0～75.0cm；一条请求在途、CRC应答、序号检查及500ms保活由页面实现。协议见docs/guide/protocol-bp2f.md。

直接Web Serial能力由浏览器检测，不承诺所有手机均可直连HC-05。普通Pages页面不会提供电脑端/api接口，需要桥接时使用上述本地服务。
