小智 AI 聊天机器人百科全书：
https://ccnphfhqs21z.feishu.cn/wiki/F5krwD16viZoF0kKkvDcrZNYnhb
小智 AI 后台配置面板：
https://xiaozhi.me/
小智AI聊天机器人常见问题Q&A
https://rcnv1t9vps13.feishu.cn/wiki/JiQowaSe1itt07kyVvZcHFcQnee
小智AI资料及视频教程:
https://rcnz2dw2xfpt.feishu.cn/wiki/AgDaw0P9liDwpfkUBOjcMbADnec

在线烧录
https://espressif.github.io/esp-launchpad/

环境依赖
source /opt/esp-idf/export.sh

查看串口
ls /dev/ttyACM*
ls /dev/ttyUSB*

idf.py menuconfig         # 改唤醒词
idf.py build
idf.py -p /dev/ttyACM0 erase-flash
idf.py -p /dev/ttyACM0 flash
idf.py -p /dev/ttyACM0 monitor

烧录k210固件
sudo kflash -p /dev/ttyUSB0 -b 1500000 ~/Download/maixpy_v0.6.3_2_gd8901fd22.bin

查看或删除k210已有文件
mpremote connect /dev/ttyUSB0 fs cat /flash/main.py
