# imgui_ros

View ros images and interact with node topics, services,
and parameters using imgui (https://github.com/ocornut/imgui).

## Instructions


```
sudo apt install libglm-dev libsdl2-dev ros-melodic-pcl-ros libyaml-cpp-dev
pip3 install transforms3d
```

Run a demo:

```
cd ~/catkin_ws/src
git clone https://github.com/lucasw/imgui_ros.git
git clone https://github.com/lucasw/image_manip.git  # required for the demo
cd imgui_ros/imgui_ros
git clone https://github.com/ocornut/imgui.git
cd ~/catkin_ws/src
catkin build imgui_ros image_manip
source install/setup.bash
roslaunch imgui_ros dragndrop.launch
```
