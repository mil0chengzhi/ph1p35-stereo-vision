# 上传GitHub与发布网页

## 上传什么

将`ph1p35-stereo-vision`文件夹**内部内容**作为仓库根目录，确保README.md位于最外层。仓库名称可用`ph1p35-stereo-vision`。不要把整个开发工作区或一个ZIP作为唯一仓库内容。

建议Description：`Dual-PH1P35 FPGA vision prototype with native stereo experiments, cloud display and browser tools.`

建议Topics：`fpga`、`verilog`、`risc-v`、`stereo-vision`、`point-cloud`、`web-serial`、`anlogic`。

可以使用GitHub Desktop或Git上传。新建仓库后，在本目录执行以下命令；远程地址使用你自己仓库页面复制的地址：

```console
git init -b main
git add .
git commit -m "Add PH1P35 vision showcase and host tools"
```

随后添加你实际创建的origin并推送。此处不预设用户名或仓库地址，也没有替你创建仓库或推送。

若在网页选择Add file → Upload files，按目录分批上传并保留隐藏配置文件。GitHub官方当前说明为网页单文件不超过25MiB、一次最多100个文件，命令行普通Git单文件不超过100MiB。本包校验脚本使用更保守的25MiB单文件上限。[官方文件上传说明](https://docs.github.com/en/repositories/working-with-files/managing-files/adding-a-file-to-a-repository)。

## GitHub Pages

本仓库已备好`docs/index.html`和`.nojekyll`。进入Settings → Pages，Source选择Deploy from a branch，分支选择`main`，文件夹选择`/docs`，保存后等待GitHub生成实际网址。手机控制台在该网址的`remote/`目录。

Pages静态网页不能替电脑运行Python串口桥接。电脑本地桥接用于同网手机，页面浏览与硬件控制分别说明。[官方Pages设置说明](https://docs.github.com/en/pages/getting-started-with-github-pages/configuring-a-publishing-source-for-your-github-pages-site)。

## Release附件

交付包外层的`release-assets`包含A3海报、高清图片和明确标注的理想完成态报告。它们建议作为Release附件上传，避免反复把大尺寸文档写入Git历史。Release正文可参考`release-notes.md`。

## 发布前核对

- README的作品名称、版本表和联系方式是否符合你的发布意图。
- 选择适用于自有代码与素材的许可；当前未擅自为整个工程添加MIT等授权。
- 实物照片及选中的设计附件是否都愿意公开。
- 没有附上设备密钥、个人目录、蓝牙MAC、实时端口状态、供应商安装包或加密IP。
- 完整硬件工程与本仓库摘选范围的区别已说明；范文示例数据没有被写成当前实测。
