// Copyright 2021 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BR2_BT_PATROLLING__PATROL_HPP_
#define BR2_BT_PATROLLING__PATROL_HPP_

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"


#include "rclcpp/rclcpp.hpp"

namespace br2_bt_patrolling
{

class SaySomething : public BT::ActionNodeBase
{
public:
  explicit SaySomething(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::string>("message"),
      });
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Time start_time_;
};

}  // namespace br2_bt_patrolling

#endif  // BR2_BT_PATROLLING__PATROL_HPP_
