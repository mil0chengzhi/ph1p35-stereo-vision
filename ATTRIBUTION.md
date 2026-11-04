# 来源与许可说明

本目录是工程材料发布整理稿。未自动授予整个仓库MIT、Apache或其他统一许可；发布者应根据自有代码与素材来源决定适用许可证。这里保留现有源码注释，不将第三方内容改写成原创。

| 材料 | 来源及处理 |
| --- | --- |
| 代表性RTL和固件 | 工程2026-11-04归档快照；不合并不同镜像中的同名模块；原候选状态保留 |
| 多版本RTL与约束归档 | 同一工程的PH1P35_FPGA_RTL交付包；保留原字节及版权头，排除厂商al_ip和加密IP；逐文件校验见fpga/rtl-archive/provenance.json |
| 浏览器程序 | 本工程既有上位机及距离控制台；发布副本调整了相对路径、示例入口及构建脚本 |
| 实物照片 | 用户提供的gctp照片，经既有报告配图整理；不附设备配对记录和原照片元数据 |
| 示例点云 | 本工程场景经电脑离线处理形成，非新FPGA实测；完整记录见examples/clouds/provenance.json |
| 设计海报 | 基于原型参考的AI辅助主视觉和矢量排版，属于设计展示，已保留目标架构标签 |
| 理想完成态报告 | 可选Release附件；示例数据已标明，不用作版本实测记录 |

未纳入：厂商软件安装包、加密DDR/MIPI IP、供应商整套示例、芯片与板卡PDF原文、他人竞赛报告、神经网络模型权重、原工作区Git历史及会话记录。没有给这些材料添加新的授权。

公开文件的来源路径使用工程内相对位置，来源与输出哈希见[provenance.json](provenance.json)。工程中存在模块不代表已在某个实机镜像中启用。

## 技术参考

- Zabih R, Woodfill J. Non-parametric Local Transforms for Computing Visual Correspondence. ECCV 1994. DOI: 10.1007/BFb0028345.
- Hirschmüller H. Stereo Processing by Semiglobal Matching and Mutual Information. TPAMI 2008. [DLR作者机构存档](https://elib.dlr.de/55367/).
- Zhang Z. A Flexible New Technique for Camera Calibration. TPAMI 2000. [Microsoft Research](https://www.microsoft.com/en-us/research/project/a-flexible-new-technique-for-camera-calibration-2/publications/).
- Lipson L, Teed Z, Deng J. RAFT-Stereo Multilevel Recurrent Field Transforms for Stereo Matching. [原论文](https://arxiv.org/abs/2109.07547)与[作者代码](https://github.com/princeton-vl/RAFT-Stereo)。电脑示例曾使用该离线流程，本仓库不分发其代码或权重，也不宣称原创该算法。

PH1P35、SC520CS及HC-05等名称用于识别工程硬件，不表示厂商为本仓库背书。
