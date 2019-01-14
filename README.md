# imgui_ros

View ros2 images and interact with node topics, services,
and parameters using imgui (https://github.com/ocornut/imgui).

## Instructions

Run a demo:

```
cd colcon_ws/src
git clone https://github.com/lucasw/imgui_ros.git
git clone https://github.com/lucasw/image_manip.git  # required for the demo
cd imgui_ros/imgui_ros
git clone https://github.com/ocornut/imgui.git
cd ../../
colcon build --packages-select imgui_ros image_manip
source install/setup.bash
ros2 launch imgui_ros demo_launch.py
```
