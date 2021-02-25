# Prerequisite

## Set permission UART port (/dev/ttyTHS1) for connecting main board

- Stop use serial console output
```shell
$ systemctl stop nvgetty
$ systemctl disable nvgetty
$ udevadm trigger
```

- Add user to group
```shell
$ sudo usermod -a -G dialout $USER
$ sudo usermod -a -G tty $USER
```

- Reboot and check the permission
```shell
$ ll /dev/ttyTHS*
crw-rw---- 1 root dialout 238, 1  2월 16 14:18 /dev/ttyTHS1
crw-rw---- 1 root dialout 238, 2  2월 16 14:18 /dev/ttyTHS2
```

## Install libserial

```shell
$ wget https://github.com/crayzeewulf/libserial/archive/v1.0.0.tar.gz -O libserial-1.0.0.tar.gz
$ tar zxf libserial-1.0.0.tar.gz
$ sudo apt install doxygen
$ cd libserial-1.0.0
$ mkdir build
$ cd build
$ cmake ..
$ make -j4
$ sudo make install
```
