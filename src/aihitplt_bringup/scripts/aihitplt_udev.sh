#CP2102 串口号
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="0003", MODE:="0777", GROUP:="dialout", SYMLINK+="aihitplt_controller"' >/etc/udev/rules.d/aihitplt_controller.rules

#CH9102 串口号
echo  'KERNEL=="ttyCH343USB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4",ATTRS{serial}=="0003", MODE:="0777", GROUP:="dialout", SYMLINK+="aihitplt_controller"'>>/etc/udev/rules.d/aihitplt_controller.rules

#CH9102，同时系统安装了对应驱动 串口号0004 设置别名为aihitplt_mic
echo  'KERNEL=="ttyCH343USB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4",ATTRS{serial}=="0004", MODE:="0777", GROUP:="dialout", SYMLINK+="aihitplt_mic"' >/etc/udev/rules.d/aihitplt_mic.rules
echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4",ATTRS{serial}=="0004", MODE:="0777", GROUP:="dialout", SYMLINK+="aihitplt_mic"' >>/etc/udev/rules.d/aihitplt_mic.rules


service udev reload
sleep 2
service udev restart