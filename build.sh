cd ThirdParty/MPU6050-Jetson-Orin
make uninstall
make install
make example

cd ../..
rm -rf build
mkdir build
cd build
cmake ..
make