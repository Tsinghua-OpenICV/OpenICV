# OpenICV

OpenICV is the software framework of Open Intelligent Connected Vehicle.

# Use Cases
OpenICV will initially address the following 2 use case:
1. Autonomous driving of real vehicle
2. Autonomous driving in simulation environment 

# System Requirement
1. [cmake](https://cmake.org/) >=3.15
2. [msgpack-c](http://msgpack.org/releases/cpp/msgpack-0.5.7.tar.gz)  ==0.5.7  
3. [boost](https://www.boost.org/)>=1.66.0
4. eigen
```sh
sudo apt-get install libeigen3-dev
```
5. opencv
```sh
sudo apt-get install libopencv-dev
```
6. pcl
```sh
sudo apt-get install libpcl-dev
```
# Installation
Installation of OpenICV is based on cmake.
```sh
git clone https://github.com/Tsinghua-OpenICV/OpenICV.git
cd OpenICV
mkdir build
cd build
cmake .
make
sudo make install
cd ..
sudo chomod -R 777 ICVOS
```



# Getting Started
- Basic publisher and subscriber demo:

Move OpenICV/Branch/Examples/Basic_Publisher&Subscriber.json to /OpenICV/ICVOS/bin and execute it with icvStarter
```sh
cd Branch/Examples
cp Basic_Publisher\&Subscriber.json ../../ICVOS/bin 
./icvStarter Basic_Publisher\&Subscriber.json
```


# Contributing
Please refer to `STYLE.md` to check the code style.




# Configuration Files

Json Example:
```json
{
    
    "<Node Name1>": {
        "function": {"name":"<Function name>","Parameter_name1":"Parameter_value1","Parameter_name2":"Parameter_value2"}
       
    },
    "<Node Name2>": {
        "function": {"name":"<Function name>","Parameter_name":"Parameter_value"}
       
    }

}
```
***important***
- Only one function in each node is allowed.



# Node Types
- `icvThreadedNode`: Every node has a thread
- `icvQueuedNode`: All node are executed in a queue
- `icvRemoteNode`: The node is in another process or another host
- `icvRosNode`: This node interact with ROS nodes
