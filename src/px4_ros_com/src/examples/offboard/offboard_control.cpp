/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_ros_com/frame_transforms.h>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <eigen3/Eigen/Eigen>

#include <chrono>
#include <iostream>
#include <vector>
#include <numeric>
#include <cstdlib>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace px4_ros_com;

struct drone_position{
	float x;
	float y;
	float z;
	double yaw;
};

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("offboard_control")
	{
		// publisher
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

		// subscriber
		msg_count =0;
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		subscription_ = this->create_subscription<VehicleOdometry>("/fmu/out/vehicle_odometry", qos, 
		[this](const VehicleOdometry::UniquePtr msg) {

			float x = msg->position[0];
			float y = msg->position[1];
			float z = msg->position[2];
			auto eigenq = px4_ros_com::frame_transforms::utils::quaternion::array_to_eigen_quat(msg->q);
			double yaw = px4_ros_com::frame_transforms::utils::quaternion::quaternion_get_yaw(eigenq);
			
			position_arr.push_back({x, y, z, yaw});

			// if there's more than 10 latest position, remove the first one
			if (position_arr.size() > 10){
				position_arr.erase(position_arr.begin());
			}

			avg_position = this->calc_position_avg(position_arr);

			// send current position and average position
			if (msg_count % 100 == 0){

				std::cout << "Current "
					<< " x = " << x 
					<< "; y = " << y 
					<< "; z = " << z 
					<< "; yaw = " << yaw << std::endl;

				std::cout << "AVG :" 
					<< "  x = " << avg_position.x 
					<< "; y = " << avg_position.y 
					<< "; z = " << avg_position.z 
					<< "; yaw = " << avg_position.yaw 
					<< std::endl << std::endl;
			}
			msg_count++;
		});


		offboard_setpoint_counter_ = 0;
		current_stage = 0;

		auto timer_callback = [this]() -> void {

			if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				// Arm the vehicle
				this->arm();

				// set target for stage 1		
				target_position.x = avg_position.x;
				target_position.y = avg_position.y;
				target_position.z -= 0.7f;
				target_position.yaw = avg_position.yaw;
				current_stage++;
			}

			switch(current_stage){
				case 1:
					this->publish_trajectory_setpoint(
						target_position.x, 
						target_position.y,
						target_position.z,
						target_position.yaw
					);

					if(this->is_trajectory_setpoint_completed(avg_position, target_position)){
						RCLCPP_INFO(this->get_logger(), "Stage %d completed" , current_stage);
						
						// set target for stage 2
						target_position.y += 5.2;
						current_stage++;
					}

					break;
					
				case 2:
					this->publish_trajectory_setpoint(
						target_position.x, 
						target_position.y,
						target_position.z,
						target_position.yaw
					);

					if(this->is_trajectory_setpoint_completed(avg_position, target_position)){
						RCLCPP_INFO(this->get_logger(), "Stage %d completed" , current_stage);
						
						// set target for stage 3
						target_position.yaw = 0.0;
						current_stage++;
					}
					break;

				case 3:
					this->publish_trajectory_setpoint(
						target_position.x, 
						target_position.y,
						target_position.z,
						target_position.yaw
					);

					if(this->is_trajectory_setpoint_completed(avg_position, target_position)){
						RCLCPP_INFO(this->get_logger(), "Stage %d completed" , current_stage);
						
						// set target for stage 4
						target_position.x += 8.0f;
						current_stage++;
					}
					break;

				case 4:
					this->publish_trajectory_setpoint(
						target_position.x, 
						target_position.y,
						target_position.z,
						target_position.yaw
					);

					if(this->is_trajectory_setpoint_completed(avg_position, target_position)){
						RCLCPP_INFO(this->get_logger(), "Stage %d completed" , current_stage);

						// set target for stage 5
						target_position.z -= 0.75f;
						current_stage++;
					}
					break;

				case 5:
					this->publish_trajectory_setpoint(
						target_position.x, 
						target_position.y,
						target_position.z,
						target_position.yaw
					);

					if(this->is_trajectory_setpoint_completed(avg_position, target_position)){
						RCLCPP_INFO(this->get_logger(), "Stage %d completed" , current_stage);
						
						// set target for stage 6
						target_position.x += 4.0f;
						current_stage++;
					}
					break;

				case 6:
					this->publish_trajectory_setpoint(
						target_position.x, 
						target_position.y,
						target_position.z,
						target_position.yaw
					);

					if(this->is_trajectory_setpoint_completed(avg_position, target_position)){
						RCLCPP_INFO(this->get_logger(), "Stage %d completed" , current_stage);
						
						// set target for stage 7
						current_stage++;
					}
					break;
				
				case 7:
					this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
					break;

				default:
					break;
			}
		
			// offboard_control_mode needs to be paired with trajectory_setpoint
			publish_offboard_control_mode();

			// stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
			}
		};
		timer_ = this->create_wall_timer(100ms, timer_callback);
	}

	void arm();
	void disarm();

private:
	rclcpp::TimerBase::SharedPtr timer_;

	int delay =80;
	
	rclcpp::Subscription<VehicleOdometry>::SharedPtr subscription_;
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

	int current_stage;
	int msg_count;

	drone_position target_position;
	drone_position avg_position;
	std::vector<drone_position> position_arr;

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint(float x = 0.0, float y = 0.0, float z = 0.0, float yaw = 0.0);
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

	drone_position calc_position_avg(std::vector<drone_position> arr ) const;
	bool is_trajectory_setpoint_completed(drone_position avg, drone_position target) const;
};

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 * 
 * @param x		move in x axis - positive north, negative south
 * @param y		move in y axis - positive east, negative west
 * @param z		move in z axis - positive down, negative up
 * @param yaw	rotate in z axis - positive CW, negative CCW
 */
void OffboardControl::publish_trajectory_setpoint(float x , float y, float z, float yaw)
{
	TrajectorySetpoint msg{};
	msg.position = {x, y, z};
	msg.yaw = yaw; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

/**
 * @brief Calculate average position
 * @param arr   vector of drone_position struct
 */
drone_position OffboardControl::calc_position_avg(std::vector<drone_position> arr ) const
{
	int len = arr.size();

	std::vector<float> x, y, z;
	std::vector<double> yaw;

	for (auto element : arr){
		x.push_back(element.x);
		y.push_back(element.y);
		z.push_back(element.z);
		yaw.push_back(element.yaw);
	}

	// calculate each average by sum it first, then divide it by len
	float avg_x = std::accumulate(x.begin(), x.end(), 0.0f) / len;
	float avg_y = std::accumulate(y.begin(), y.end(), 0.0f) / len;
	float avg_z = std::accumulate(z.begin(), z.end(), 0.0f) / len;
	double avg_yaw = std::accumulate(yaw.begin(), yaw.end(), 0.0) / len;

	drone_position avg_position { avg_x, avg_y, avg_z, avg_yaw};

	return avg_position;
}

/**
 * @brief Check if the trajectory is achived
 * @param avg   	average position
 * @param target	target position   
 */
bool OffboardControl::is_trajectory_setpoint_completed(drone_position avg, drone_position target) const
{
	if (std::abs(avg.x - target.x) < 0.1
		&& std::abs(avg.y - target.y) < 0.1
		&& std::abs(avg.z - target.z) < 0.1
		&& std::abs(avg.yaw - target.yaw) < 0.05
	) {
		return true;

	}else{
		return false;
	}
}


int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}
