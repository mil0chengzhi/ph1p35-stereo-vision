# CLOUD/1点云接收合同

浏览器在用户选定数据串口并点击接收后发送：

```text
CLOUD/1 GET PLY\n
```

数据源回复以下ASCII头，随后发送指定长度的完整PLY字节：

```text
CLOUD/1 PLY <十进制字节数> <8位十六进制CRC32>\n
```

允许回包头CRLF；长度50字节至100MiB。PLY可为ASCII或binary_little_endian。CRC32使用IEEE/ZIP，反射多项式0xEDB88320、初值0xffffffff、最终异或0xffffffff；标准向量ASCII`123456789`结果CBF43926。

每段等待4秒，总限时3分钟。完整长度、CRC和PLY解析均通过后才载入。错误或超时不会回退到预存文件冒充板端回传。

本仓库未提供完整的板级CLOUD/1回读实现。网页实现接收合同不意味着对应固件已提供接口。HDMI视频不包含此合同中的XYZ字节流。
