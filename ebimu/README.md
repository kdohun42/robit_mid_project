# ebimu with ros2


- quaternion -> euler
- gui
- 값 이상할 시 EBTerminal로 e2box imu 설정 다시하기

-symlink 설정

	cd /etc/udev/rules.d
	sudo gedit tty-usb.rules
 
	SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", MODE:="0666",SYMLINK+="ttyUSB-U2D2"
	ACTION=="add", SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",MODE:="0666", SYMLINK+="ttyUSB-EBIMU"

	sudo service udev reload
	sudo service udev restart
