#!/bin/sh
#実行場所
cd `dirname $0`
cd ..

#rosdep2 でこのリポジトリの依存関係を解決
sudo apt-get update -y
sudo apt install python3-rosdep2 -y
sudo rosdep init
rosdep update
rosdep install -r -y -i --from-paths .

#追加パッケージ
sudo apt install ros-foxy-xacro -y
sudo apt install ros-foxy-gazebo-ros-pkgs -y

sudo pip3 install networkx -y
