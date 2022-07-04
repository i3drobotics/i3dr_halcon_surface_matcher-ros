# ROS package for surface matching using Halcon

## Setup

### Halcon
This package uses Halcon for surface matching so a valid installation of Halcon and license is required.

### Create catkin workspace
Create catkin workspace to build and run package
```
mkdir ~/catkin_ws
cd catkin_ws
mkdir ~/catkin_ws/src
```

### ROSINSTALL
For an easy setup, a rosinstall file is provided in 'install' folder of this repo which can be used to get this package and it's dependent ros packages in your workspace. 
In your ROS workspace use the following command:
```
wstool init src https://raw.githubusercontent.com/i3drobotics/i3dr_halcon_surface_matcher-ros/main/install/i3dr_halcon_surface_matcher_https.rosinstall
```
If you already have a wstool workspace setup then use the following command instead:
```
wstool merge -t src https://raw.githubusercontent.com/i3drobotics/i3dr_halcon_surface_matcher-ros/main/install/i3dr_halcon_surface_matcher_https.rosinstall
wstool update -t src
```
To use this package with I3DR's Titania stereo camera also add  the following rosinstall:
```
wstool merge -t src https://raw.githubusercontent.com/i3drobotics/i3dr_titania-ros/main/install/i3dr_titania_https.rosinstall
wstool update -t src
```

### Manual package install
If you do not use wstool, you can download the packages using the following command:
```
cd PATH_TO_ROS_WS/src
git clone https://github.com/i3drobotics/i3dr_halcon_surface_matcher-ros
git clone https://github.com/asr-ros/asr_halcon_bridge.git
git clone https://github.com/wg-perception/object_recognition_msgs.git
git clone https://github.com/i3drobotics/i3dr_titania-ros.git
git clone https://github.com/i3drobotics/i3dr_stereo_camera-ros.git
git clone https://github.com/i3drobotics/camera_control_msgs.git
git clone https://github.com/i3drobotics/pylon_camera.git
```

### Pylon
The pylon_camera package used by the Titania requires the pylonSDK to be installed on your system. Please download and install the pylon debian package for your architecture from [here](https://www.baslerweb.com/en/sales-support/downloads/software-downloads/#type=pylonsoftware;language=all;version=all;os=all)  
Look for 'pylon 6.x.x Camera Software Suite Linux x86 (64 Bit) - Debian Installer Package'

In order to build the package, you need to configure rosdep (i.e. the ROS command-line tool for checking and installing system dependencies for ROS packages) such that
it knows how to resolve this dependency. This can be achieved by executing the following commands:

```
sudo sh -c 'echo "yaml https://raw.githubusercontent.com/i3drobotics/pylon_camera/main/rosdep/pylon_sdk.yaml " > /etc/ros/rosdep/sources.list.d/15-plyon_camera.list'
rosdep update
```

### I3DR Stereo Matcher
Install I3DRSGM support package:
```
curl https://github.com/i3drobotics/phobosIntegration/releases/download/v1.0.54/Phobos-1.0.54-x86_64_reducedTemplates.deb > i3drsgm.deb
sudo dpkg -i i3drsgm.deb
```
To build with the I3DR Stereo Matcher use the following command:
``` bash
catkin_make -DWITH_I3DRSGM=ON
```
As I3DR Stereo Matcher uses CUDA the following enviromental varaiables should be set to avoid JIT recompiling:
```
export CUDA_CACHE_MAXSIZE=2147483648
export CUDA_CACHE_DISABLE=0
```
Contact info@i3drobotics.com for a license to use this matcher. 
We will provide you with a license file that should be placed in the following folder after building the workspace:
```
~/.ros/LICENSE.lic
```

## Build
To install package dependences use rodep:
```
rosdep install --from-paths src --ignore-src -r -y
```
Build with I3DRSGM support:
```
catkin_make -DWITH_I3DRSGM=ON
```
Source workspace build to make package available
```
source devel/setup.bash
```

## Run
This package can be run standalone but is optimised for point clouds from I3DR's Titania stereo camera.  
To test the package with a point cloud (.ply) file use the provided launch file: 
```
roslaunch i3dr_halcon_surface_matcher detect.launch point_cloud_filepath:=/path/to/pointcloud.ply
```

Alternatively to run with live point cloud from Titania. Plug in your Titania camera to your machine and use the following launch file to capture data from the stereo camera (add 'stereo_algorithm:=2' parameter to use the I3DRSGM stereo matcher):
```
roslaunch i3dr_titania titania.launch stereo_algorithm:=2
```

Then in a new terminal launch the Titania primitive detection:
```
roslaunch i3dr_halcon_surface_matcher detect.launch
```