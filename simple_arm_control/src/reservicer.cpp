//BSD 3-Clause License
//
//Copyright (c) 2021, Florent Audonnet
//All rights reserved.
//
//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions are met:
//
//1. Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
//2. Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
//3. Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
//FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
//OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "gazebo_msgs/msg/entity_state.hpp"
#include "gazebo_msgs/srv/get_entity_state.hpp"
#include "gazebo_msgs/srv/get_model_list.hpp"
#include "service_handler.hpp"

#include "add_on_msgs/srv/get_prims.hpp"
#include "add_on_msgs/srv/get_prim_attribute.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include <chrono>
#include <map>
#include <string>
#include <vector>
#include <regex>

std::vector<double> convert_to_vector(std::string str_arr)
{
    std::vector<double> v;

    std::regex float_regex("\\d+\\.\\d+");
    auto numbers_begin = 
        std::sregex_iterator(str_arr.begin(), str_arr.end(), float_regex);
    auto numbers_end = std::sregex_iterator();
 
    std::cout << "Found "
              << std::distance(numbers_begin, numbers_end)
              << " words\n";
    for (std::sregex_iterator i = numbers_begin; i != numbers_end; ++i) {
        std::smatch match = *i;
        double match_float = std::stof(match.str());
        v.push_back(match_float);
    }

    return v;
}

void get_model_state_handler(std::shared_ptr<gazebo_msgs::srv::GetEntityState::Request> request, 
    std::shared_ptr<gazebo_msgs::srv::GetEntityState::Response> response, 
    std::shared_ptr<ServiceClient<add_on_msgs::srv::GetPrimAttribute>> service)
{
    auto position_request = service->create_request_message();
    position_request->path = request->name;
    position_request->attribute = "xformOp:translate"; // position
    auto position_response = service->service_caller(position_request);
    if (position_response->success)
    {
        auto orientation_request = service->create_request_message();
        orientation_request->path = request->name;
        orientation_request->attribute = "xformOp:orient"; // orientation
        auto orientation_response = service->service_caller(orientation_request);
        if (orientation_response->success)
        {

            auto state = gazebo_msgs::msg::EntityState();
            auto pose = geometry_msgs::msg::Pose();
            auto point = geometry_msgs::msg::Point();
            auto quat = geometry_msgs::msg::Quaternion();
            auto position = convert_to_vector(position_response->value);
            auto orientation = convert_to_vector(orientation_response->value);
            point.x = position[0];
            point.y = position[1];
            point.z = position[2];
            pose.position = point;
            quat.w = orientation[0];
            quat.x = orientation[1];
            quat.y = orientation[2];
            quat.z = orientation[3];
            pose.orientation = quat;
            state.name = request->name;
            state.pose = pose;
            response->state = state;
            response->success = true;
        }
    }
    else
    {
        response->success = false;
        RCLCPP_ERROR(rclcpp::get_logger("reservicer"), "Failed to request model state");
    }
}

void get_model_list_handler(std::shared_ptr<gazebo_msgs::srv::GetModelList::Request> request, 
    std::shared_ptr<gazebo_msgs::srv::GetModelList::Response> response, 
    std::shared_ptr<ServiceClient<add_on_msgs::srv::GetPrims>> service)
{
    request->structure_needs_at_least_one_member;
    auto model_request = service->create_request_message();
    model_request->path = "/Spawned";
    auto model_response = service->service_caller(model_request);
    if (model_response->success)
    {
        response->model_names = model_response->paths;
        response->success = true;
    }
    else
    {
        response->success = false;
        RCLCPP_ERROR(rclcpp::get_logger("reservicer"), "Failed to request model list");
    }
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("reservicer");
    auto model_client = std::make_shared<ServiceClient<add_on_msgs::srv::GetPrims>>("get_prims");
    auto state_client = std::make_shared<ServiceClient<add_on_msgs::srv::GetPrimAttribute>>("get_attribute");

    std::function<void(const std::shared_ptr<gazebo_msgs::srv::GetModelList::Request>,
                       std::shared_ptr<gazebo_msgs::srv::GetModelList::Response>)>
        model_fnc = std::bind(get_model_list_handler, std::placeholders::_1,std::placeholders::_2, model_client);

    std::function<void(const std::shared_ptr<gazebo_msgs::srv::GetEntityState::Request>,
                       std::shared_ptr<gazebo_msgs::srv::GetEntityState::Response>)>
        state_fnc = std::bind(get_model_state_handler, std::placeholders::_1,std::placeholders::_2, state_client);

    rclcpp::Service<gazebo_msgs::srv::GetModelList>::SharedPtr model_service =
        node->create_service<gazebo_msgs::srv::GetModelList>("get_model_list", model_fnc);

    rclcpp::Service<gazebo_msgs::srv::GetEntityState>::SharedPtr state_service =
        node->create_service<gazebo_msgs::srv::GetEntityState>("get_entity_state", state_fnc);



    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}