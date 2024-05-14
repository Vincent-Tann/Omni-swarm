## Usage

### Docker配置和Repo下载

The Omni-swarm offical support TX2 with Ubuntu 18.04. For those running on other hardware and system setup, converting the models to trt by your own is essential.

[Here](https://www.dropbox.com/s/skq1vgfeawiw151/models.zip?dl=0) to download the CNN models for Omni-swarm and extract it to swarm_loop folder.

[Here](https://www.dropbox.com/sh/w5yagas06a9r14d/AACdKgMfCCg07M6jr6Ipmus1a?dl=0) to get the raw and preprocessed offical omni-directional and pinole dataset.

[swarm_msgs](https://github.com/HKUST-Swarm/swarm_msgs) and [inf_uwb_ros](https://github.com/HKUST-Swarm/inf_uwb_ros) are compulsory.
And [swarm_detector](https://github.com/HKUST-Swarm/swarm_detector) if you want to use detector.

这里原版README省略了很多很多细节，导致配环境要摸索很久。

具体操作起来步骤如下：

首先把作者提供的docker image下载下来（一定不要漏了pc这个tag，否则会下载latest，不是最终版本）:

```zsh
docker pull xuhao1/swarm2020:pc
```

因为要在container中使用rviz的可视化工具，因此需要设置允许其访问本地X服务器的图形界面：

```zsh
xhost +local:root #设置所有root用户（包括container中的root用户）可以访问X服务器
# 也有教程写的是 'xhost +'，也就是允许所有用户访问
# [注意！！] 每次重新运行前要再执行一遍。改动仅在当次有效。
```

构建容器并启动：

```zsh
docker run -it \
       --net=host \
       --gpus all \
       --runtime nvidia \
       --env="DISPLAY=$DISPLAY" \
       --env="QT_X11_NO_MITSHM=1" \
       --env="NVIDIA_VISIBLE_DEVICES=all" \
       --env="NVIDIA_DRIVER_CAPABILITIES=all" \
       --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
       --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
       --name="omni" \
       xuhao1/swarm2020:pc \
       zsh #最后的COMMAND参数一定要设置，zsh等同于/bin/zsh
exit
# 再次使用时：启动容器，并在container中打开终端
docker start -i omni 
# 如果需要额外的终端：
docker exec -it omni /bin/zsh
```

docker run指令的选项说明：
- `--env="DISPLAY=$DISPLAY"`: `设置环境变量-显示器编号
- `--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"`: 挂载和X服务器相关的目录
- `--name="omni"`: 给容器起个名字


进入容器的`~/swarm_ws`，删除所有文件并新建src文件夹在src文件夹下git clone本repo，以及上面作者提到的两个（三个）repo。

下载上面提到的models.zip并解压到swarm_loop文件夹下。

除此之外，还要git clone下面提到的[VINS-Fisheye](https://github.com/HKUST-Aerial-Robotics/VINS-Fisheye) ,以及[FUEL](https://github.com/HKUST-Aerial-Robotics/FUEL)(需要用里面的bspline这个package，这个自己找真的头疼），以及和ros bag相关的[sync_bag_player](https://github.com/HKUST-Swarm/sync_bag_player.git)。

下载好所需要的库之后，打开`FUEL/fuel_planner/bspline/msg/Bspline.msg`，在第一行添加：

`int32 drone_id`

此外还要额外安装一个ROS包：

```zsh
sudo apt update
sudo apt install ros-melodic-nlopt
```

### 编译

接着开始编译，直接`catkin_make`会报错一些包找不到。我们手动一个个编译。首先：

```
catkin_make --pkg inf_uwb_ros -j1
```

`-j1`是用单线程编译，不知道为什么在容器里直接catkin_make，默认的多线程会导致程序可能被kill。本人在ubuntu上用docker desktop的时候只有-j1可行。后来卸载docker desktop，并用命令行安装了docker，就可以多线程了（建议在ubuntu下不要用docker desktop，还容易在打开的时候卡在"Starting the Docker Engine"）。

编译的时候可以先不加`-j1`，如果出现类似`program killed`的报错再加上。毕竟多线程快。

接着按[我Fork的Vins-Fisheye中的说明](https://github.com/Vincent-Tann/VINS-Fisheye/tree/master)配置VIN-Fisheye的OpenCV环境，再编译其中的vins包（文件夹名称是vins_estimator）：

```
catkin_make --pkg vins -j1
```

然后再编译swarm_loop包才不会报错：

```
catkin_make --pkg swarm_loop -j1
```

接着编译该repo下的另外两个包以及sync_bag_player：

```
catkin_make --pkg localization_proxy swarm_localization sync_bag_player -j1
```

### 数据集下载

接下来把下载的数据集（上面给了链接）在本地解压后用`docker cp`复制到容器内的`bags`文件夹：[Here](https://www.dropbox.com/sh/w5yagas06a9r14d/AACdKgMfCCg07M6jr6Ipmus1a?dl=0) to get the raw and preprocessed offical omni-directional and pinole dataset.

```zsh
docker cp ~/Downloads/swarm_raw_parallel_noyaw_2021-11-12 omni:/root/bags/
docker cp ~/Downloads/swarm_raw_parallel_yaw_2021-11-16 omni:/root/bags/
docker cp ~/Downloads/random_fly omni:/root/bags/
```

### 项目运行

First, running the pinhole or fisheye version of [VINS-Fisheye](https://github.com/HKUST-Aerial-Robotics/VINS-Fisheye) (Yes, VINS-Fisheye is pinhole compatiable and is essential for Omni-swarm).

具体做法是(需要先roscore和rosrun nodelet nodelet manager __name:=swarm_manager，或者加到launch文件里）：

```zsh
roscore
```

```zsh
rosrun nodelet nodelet manager __name:=swarm_manager
```

接着：

```zsh
roslaunch vins fisheye.launch config_file:=/root/swarm_ws/src/VINS-Fisheye/config/fisheye_ptgrey_n3/fisheye_cuda.yaml
# /root/SwarmConfig/fisheye_ptgrey_n3/fisheye_cuda.yaml中提供了不同的vins配置，且这个配置在下面的nodelet-sfisheye.launch中被使用。
# 如果为了保持统一性，Vins Fisheye应该也使用这个配置，则指令改为
# roslaunch vins fisheye.launch config_file:=/root/SwarmConfig/fisheye_ptgrey_n3/fisheye_cuda.yaml
roslaunch vins fisheye.launch config_file:=/root/swarm_ws/src/SwarmConfig/fisheye_ptgrey_n3/fisheye_cuda.yaml
```

node版本，不用nodelet:

```
roslaunch vins fisheye_node.launch config_file:=/root/swarm_ws/src/SwarmConfig/fisheye_ptgrey_n3/fisheye_cuda.yaml
```

Start map-based localization with（要先把nodelet-sfisheye.launch文件里第8、9行路径中的/home/dji改为/root;50行开始的tx2相关模型名称改为rtx3080的：superpoint_v1_tx2_fp16.trt改为superpoint_v1_rtx3080_fp16.trt，mobilenetvlad_208x400_tx2_fp16.trt改为mobilenetvlad_208x400_rtx3080_fp16.trt。3060显卡也跑通了）

```
roslaunch swarm_loop nodelet-sfisheye.launch
```

node版本，不用nodelet：

```
roslaunch swarm_loop node-sfisheye-txs.launch
```

or pinhole version

```
roslaunch swarm_loop realsense.launch
```

Start visual object detector by (not compulsory)

```
roslaunch  swarm_detector detector.launch
```

Start UWB communication module with (Support NoopLoop UWB module only)

```
roslaunch localization_proxy uwb_comm.launch start_uwb_node:=true
```

用这个！：

```
roslaunch localization_proxy uwb_comm.launch start_uwb_node:=false self_id:=1
```

If you don't have a UWB module, you may start the communication with a self id(start from 1, different on each drone)

```
roslaunch localization_proxy uwb_comm.launch start_uwb_node:=true enable_uwb:=false self_id:=1
```

Start state estimation with visualizer by

```
roslaunch swarm_localization loop-5-drone.launch bag_replay:=true viz:=true enable_distance:=false cgraph_path:=/root/output/graph.dot
# 在电脑上跑，用这个：
roslaunch swarm_localization loop-5-pc.launch bag_replay:=true viz:=true enable_distance:=false cgraph_path:=/root/output/graph.dot self_id:=1
```

用这个！：

```
roslaunch swarm_localization loop-5-drone.launch enable_distance:=false enable_detection:=false cgraph_path:=/root/output/graph.dot viz:=false
```


You may enable/disable specific measurement by adding

```
enable_distance:=false or enable_detection:=false enable_loop:=true
```

To visualize the real-time estimation result, use __viz:=true__. 
Add __bag_replay:=true__ only when evaluation dataset, when evaluate pre-processed dataset, you may only launch __loop-5-drone.launch__
Some analysis tools is located in [DataAnalysis](swarm_localization/DataAnalysis)
![](./doc/ob-Traj2.png)

## Docker
To evaluate the program, a recommended way is by using a docker. We provide a runnable docker on docker hub, TAG is xuhao1/swarm2020:pc.

Due to the limitation of the TensorRT engine adopted in the frontend to accelerate the CNNs, the docker is restricted to running with an RTX 3080 or similar graphic card. We are working on migrating an onnxruntime version of frontend for CNNs referencing and a buildable docker file, which will be released very soon.
## LICENSE
GPLV3

# Omni-swarm
A Decentralized Omnidirectional Visual-Inertial-UWB State Estimation System for Aerial Swarm
![](./doc/gcs.png)
## Introduction

**Omni-swarm** is a decentralized omnidirectional visual-inertial-UWB state estimation system for the aerial swarm.
In order to solve the issues of observability, complicated initialization, insufficient accuracy and lack of global consistency, we introduce an omnidirectional perception system as the front-end of the **Omni-swarm**, consisting of omnidirectional sensors, which includes stereo fisheye cameras and ultra-wideband (UWB) sensors, and algorithms, which includes fisheye visual inertial odometry (VIO), multi-drone map-based localization and visual object detection.
A graph-based optimization and forward propagation working as the back-end of the **Omni-swarm** to fuse the measurements from the front-end.
According to the experiment result, the proposed decentralized state estimation method on the swarm system achieves centimeter-level relative state estimation accuracy while ensuring global consistency. Moreover, supported by the **Omni-swarm**, inter-drone collision avoidance can be accomplished in a whole decentralized scheme without any external device, demonstrating the potential of **Omni-swarm** to be the foundation of autonomous aerial swarm flights in different scenarios.
       
The is the code for __Omni-swarm: A Decentralized Omnidirectional Visual-Inertial-UWB State Estimation System for Aerial Swarms__. The manuscript has been accepted by IEEE Transactions on Robotics (T-RO), a preprint version is [here](https://arxiv.org/abs/2103.04131).



The structure of Omni-swarm is
![](./doc/structure.PNG)

The fused measurements of Omni-swarm:
![](./doc/measurements.PNG)

The detailed backend structure of state estimation of Omni-swarm:
![](./doc/backend.PNG)
