# FPGA代表性模块与版本归档

这里按来源镜像分组，不合并同名模块。代表性模块用于阅读与研究；[多版本源码归档](rtl-archive/README.md)补充各版本中已有的RTL、头文件及约束。厂商`al_ip`目录、加密IP和部分板级依赖未纳入，**不是完整板级可综合工程**，也不附带可直接下载的位流。

| 目录 | 来源 | 入口与观察点 |
| --- | --- | --- |
| native-depth/rtl | NB94归档 | hd_sgm1_matcher、Census窗口、stereo_dense_engine、深度和模型传输 |
| native-display/rtl | AS95归档 | hd_cloud_projector、模型接收、DDR画布与完整扫描确认 |
| range-demo/rtl | BP2F归档 | bluetooth_distance_control、UART、BCD与距离OSD；不含预存双帧控制 |
| [rtl-archive](rtl-archive/README.md) | 同一工程的PH1P35_FPGA_RTL交付包 | 按版本保留源目录结构、RTL与现有约束；校验和见归档内provenance.json |

NB94匹配器是一条水平路径，配forward/reverse WTA互反，源码默认参数与顶层实参分别理解；不要因保留其他sgm文件而把本镜像标记为四路径。AS95观察器是原生720p正交画布版本，不是理想范文的透视实现。

已移除预存模型上传、预存观察控制和双帧切换模块。距离OSD模块保留原有接口及来源标记，协议中的历史兼容字段不表示保留预存回放功能。

来源、文件SHA及是否仅摘选见[manifest.json](manifest.json)。文件存在与功能启用是不同层级；完整运行条件见[版本表](../docs/guide/versions.md)。
