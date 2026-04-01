## 使用说明（archlinux）

本文档整理了一些常用命令。实际使用时需要把命令里的串口号（如 `/dev/ttyACM0`、`/dev/ttyUSB0`）替换成自己机器上识别到的设备。

## 1) 加载 ESP-IDF 环境

在当前 shell 会话中加载 ESP-IDF 的环境变量（后续才能使用 `idf.py`）。

```bash
source /opt/esp-idf/export.sh
# 也可以使用以下命令
. /opt/esp-idf/export.sh
```

如果提示虚拟环境未找到，请重新运行安装脚本

```bash
/opt/esp-idf/install.sh
```

## 2) 查看串口设备

插上开发板后，常见串口设备名一般是 `ttyACM*`（CDC ACM）或 `ttyUSB*`（USB 转串口）。

```bash
ls /dev/ttyACM* || ls /dev/ttyUSB*
```

如果提示权限不足，可考虑把当前用户加入串口相关用户组后重新登录（不同发行版组名可能是 `dialout` / `uucp`），或临时用 `sudo` 执行相关命令。

## 3) 配置（menuconfig）

打开交互式配置菜单，用于设置串口、分区、功能开关等。

```bash
idf.py menuconfig
```

## 4) 清理、编译、烧录与串口监视

- `fullclean`：彻底清理构建产物，适合切换配置/分支后避免缓存影响  
- `build`：编译工程  
- `flash monitor`：烧录并进入串口监视（退出监视快捷键为 `Ctrl+]`）

```bash
idf.py fullclean build
idf.py -p /dev/ttyACM0 flash monitor
```

提示：如果你的串口不是 `/dev/ttyACM0`，请替换 `-p` 参数。例如 `/dev/ttyUSB0`。

## 5) 烧录 K210 固件

将固件烧录到 K210。串口与波特率按实际情况调整，固件路径也需要替换成你本地的文件位置。

```bash
sudo kflash -p /dev/ttyUSB0 -b 1500000 ~/Download/maixpy_v0.6.3_2_gd8901fd22.bin
```

上述固件为 K210 原始固件，运行上述命令即可还原 K210 为原始状态。

## 6) 查看/管理 K210 已有文件

通过 `mpremote` 连接设备文件系统，这里示例读取 `/flash/main.py`。

```bash
mpremote connect /dev/ttyUSB0 fs cat /flash/main.py
```

如需删除文件，可使用 `mpremote ... fs rm <path>`（例如 `fs rm /flash/main.py`）。  
建议先 `cat` 确认内容无误再删除，避免误操作。
