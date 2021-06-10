#!/bin/bash

src_dir="/usr/gm_cable"
if [ ! -d "$src_dir" ]; then
        echo "创建文件夹$src_dir"
        sudo mkdir $src_dir
else
        echo "文件夹$src_dir已存在"
fi

sh_dir="/usr/gm_cable/scripts"
if [ ! -d "$sh_dir" ]; then
        echo "创建文件夹$sh_dir"
        sudo mkdir $sh_dir
else
        echo "文件夹$sh_dir已存在"
fi

config_dir="/etc/gm_cable"
if [ ! -d "$config_dir" ]; then
        echo "创建文件夹$config_dir"
        sudo mkdir $config_dir
else
        echo "文件夹$config_dir已存在"
fi

sudo chmod -R 777 /usr/gm_cable

log_dir="/home/log"
if [ ! -d "$log_dir" ]; then
        echo "创建文件夹$log_dir"
        sudo mkdir $log_dir
else
        echo "文件夹$log_dir已存在"
fi
usrConfig_dir="/home/config"
if [ ! -d "$usrConfig_dir" ]; then
        echo "创建文件夹$usrConfig_dir"
        sudo mkdir $usrConfig_dir
else
        echo "文件夹$usrConfig_dir已存在"
fi

sudo chmod -R 777 /home/log
tar -xvf ./pasuo_ros.tar.gz -C /usr/gm_cable
sudo chmod 777 /usr/gm_cable/install/share/pasuo/launch/pasuostart.sh
# tar -zxvf ./libapr-1.tar.gz -C /usr/lib/
# tar -zxvf ./libactivemq-cpp.tar.gz -C /usr/lib/
# sudo chmod -R 777 /etc/gm_cable/
sudo chmod -R 777 /home/config
cp /usr/gm_cable/install/share/pasuo/yaml/usr_config.yaml  /home/config
cp ./picNumFile.txt /home/config
# touch /home/log/display_level
# echo 'WARN' >> /home/log/display_level
# sudo apt-get install libgoogle-glog-dev

exit 0




