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

#include <string>
#include <iostream>

#include "br2_bt_patrolling/SaySomething.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"


#include "rclcpp/rclcpp.hpp"

namespace br2_bt_patrolling
{

using namespace std::chrono_literals;

SaySomething::SaySomething(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);

}

void
SaySomething::halt()
{
  std::cout << "SaySomething halt" << std::endl;
}

BT::NodeStatus
SaySomething::tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    start_time_ = node_->now();
  }

  std::string message;
  getInput("message", message);

    std::cout << message << std::endl;
  
  auto elapsed = node_->now() - start_time_;

  if (elapsed < 15s) {
    return BT::NodeStatus::RUNNING;
  } else {
    return BT::NodeStatus::SUCCESS;
  }
}

}  // namespace br2_bt_patrolling

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<br2_bt_patrolling::SaySomething>("SaySomething");
}
