#!/bin/bash -e

cd ..
mkdir libs
cd libs

sudo apt install build-essential zlib1g-dev

ubuntu_version=lsb_release --release
echo $ubuntu_version
wget "https://github.com/google/or-tools/releases/download/v9.4/or-tools_amd64_ubuntu-18.04_cpp_v9.4.1874.tar.gz"
#https://github.com/google/or-tools/releases/download/v9.4/or-tools_amd64_ubuntu-20.04_cpp_v9.4.1874.tar.gz
tar -xf or-tools_amd64_ubuntu-18.04_cpp_v9.4.1874.tar.gz or_tools
rm or-tools_amd64_ubuntu-18.04_cpp_v9.4.1874.tar.gz

echo "or tools catkin ready for build!"
