/*
 * Copyright (c) 2019 Lucas Walter
 * October 2019
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <chrono>
// this brings in a boost system dependency,
// will get undefined reference to `boost::system::generic_category()
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
using namespace std::chrono_literals;


class GeneratePointCloud2 : public rclcpp::Node
{
public:
  GeneratePointCloud2() : Node("generate_point_cloud2")
  {
    get_parameter_or("num_points", num_points_, num_points_);
    set_parameter_if_not_set("num_points", num_points_);

    get_parameter_or("frame_id", frame_id_, frame_id_);
    set_parameter_if_not_set("frame_id", frame_id_);

    generatePointCloud2();

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud");
    timer_ = this->create_wall_timer(1000ms,
        std::bind(&GeneratePointCloud2::update, this));
  }

  ~GeneratePointCloud2()
  {
  }
private:
  void generatePointCloud2()
  {
    pcl::PointCloud<pcl::PointXYZRGB> cloud_;
    // TODO(lucasw) generate a cube or sphere instead, more interesting than 2d
    for (int i = 0; i < num_points_; ++i) {
      const float fr = static_cast<float>(i) / static_cast<float>(num_points_);
      pcl::PointXYZRGB pt;
      pt = pcl::PointXYZRGB(fr * 255, 255 - fr * 255, 18 + fr * 20);
      pt.x = cos(fr * M_PI * 2.0) * 1.0;
      pt.y = sin(fr * M_PI * 2.0) * 1.0;
      pt.z = 0.0;
      cloud_.points.push_back(pt);
    }

    // intermediate format - this incurs an extra copy?
    pcl::PCLPointCloud2 pcl_pc2;
    pcl::toPCLPointCloud2(cloud_, pcl_pc2);

    pc2_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl_conversions::fromPCL(pcl_pc2, *pc2_msg_);

    pc2_msg_->header.frame_id = frame_id_;
  }

  void update()
  {
    if (!pc2_msg_) {
      return;
    }

    // TODO(lucasw) regenerate if num_points has changed

    pc2_msg_->header.stamp = now();

    pub_->publish(pc2_msg_);
  }

  int num_points_ = 200;
  std::string frame_id_ = "map";

  sensor_msgs::msg::PointCloud2::SharedPtr pc2_msg_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Force flush of the stdout buffer.
  // This ensures a correct sync of all prints
  // even when executed simultaneously within a launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::spin(std::make_shared<GeneratePointCloud2>());
  rclcpp::shutdown();
  return 0;
}