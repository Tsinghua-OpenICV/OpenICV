# cmake_apollo_hdmap
Apollo HDMap SDK compiled with cmake.



## dep
```
sudo apt-get install autoconf automake libtool curl make g++ unzip
```
+ protobuf-3.3.0: compile with source
```
./autogen.sh
./configure --prefix=/home/phy/shared_dir/opt
make -j8 -l8
sudo make install 
sudo ldconfig
```
+ proj.4-4.9.3: compile with source
```
./autogen.sh
./configure --prefix=/home/phy/shared_dir/opt
make -j8 -l8
sudo make install 
sudo ldconfig
```
+ tinyxml2-5.0.1: compile with source
```
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=/home/phy/shared_dir/opt  ..
make 
make install
```
+ abseil-cpp-20200225.2: compile with source or install by apt
install by apt
```
sudo apt install ros-melodic-abseil-cpp
```
install by source
```
cd abseil-cpp-20200225.2
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=/home/phy/shared_dir/opt  ..
make 
make install
```