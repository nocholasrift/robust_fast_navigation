#!/bin/bash

# Download
wget https://www.coin-or.org/download/source/Ipopt/Ipopt-3.14.4.tar.gz
tar -xf Ipopt-3.14.4.tar.gz

# Mumps Install
cd Ipopt-releases-3.14.4/
git clone https://github.com/coin-or-tools/ThirdParty-Mumps.git
cd ThirdParty-Mumps
./get.Mumps
mkdir build
cd build
../configure
make
make install

# Ipopt install
cd ../..
mkdir build
cd build
../configure
make
make install

# fix coin-or
cd /usr/local/include
cp -r coin-or coin

