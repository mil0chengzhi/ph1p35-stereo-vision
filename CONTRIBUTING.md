# 参与开发

请注明所改模块、来源镜像、输入输出变化和验证方式。网页改动运行`python tools/verify_package.py`及`node tests/workbench.test.cjs`，修改模板后重新执行构建脚本并提交生成的HTML。

来源哈希用于识别归档快照。主动修改已复制的源码后，应更新provenance.json的输出sha256和变化说明，保留source_sha256；RTL还需更新fpga/manifest.json。不要通过忽略校验来掩盖无意的文件变化。

RTL摘选不包含完整厂商IP及板级工程。修改模块时给出时钟域、复位方式、valid/ready规则、位宽和资源影响；不得把“语法通过”替代为“布局布线或实机通过”。带硬件动作的测试只由具有相应装置与操作意图的使用者执行。

功能完成状态统一更新`docs/guide/versions.md`和验证说明。真实计算、预存回放、电脑处理与理想架构分别描述；素材中若新增测试数值，应同时附样本、版本和计时边界。
