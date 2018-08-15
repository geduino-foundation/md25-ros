/*
 md25_simple_controller.cpp

 Copyright (C) 2018 Alessandro Francescon

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "md25_simple_controller.h"

#include <md25_msgs/StampedSpeeds.h>

MD25SimpleController::MD25SimpleController(ros::NodeHandle &_nodeHandle, ros::NodeHandle &_privateNodeHandle) :
    nodeHandle(_nodeHandle), privateNodeHandle(_privateNodeHandle) {
}

int MD25SimpleController::init() {

    // Get params
    privateNodeHandle.param("speed_sensitivity", speedSensitivity, 12.4620);
    privateNodeHandle.param("wheel_base", wheelBase, 0.275);
    privateNodeHandle.param("wheel_diameter", wheelDiameter, 0.1);
    privateNodeHandle.param<std::string>("base_frame", baseFrame, "base_link");

    // Init speeds publisher
    ros::Publisher speedsPublisher = nodeHandle.advertise<md25_msgs::StampedSpeeds>("speeds", 20);
    speedsPublisherPtr = boost::make_shared<ros::Publisher>(speedsPublisher);

    // Init cmd_vel subscriber
    ros::Subscriber cmdVelSubscriber = nodeHandle.subscribe("cmd_vel", 1, & MD25SimpleController::cmdVelCallback, this);
    cmdVelSubscriberPtr = boost::make_shared<ros::Subscriber>(cmdVelSubscriber);

    return 0;

}

void MD25SimpleController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr & cmdVel) {

    // Calculate cmd_speeds
    uint8_t speed1 = 128 + (cmdVel->linear.x - cmdVel->angular.z * wheelBase / 2) * speedSensitivity / wheelDiameter;
    uint8_t speed2 = 128 + (cmdVel->linear.x + cmdVel->angular.z * wheelBase / 2) * speedSensitivity / wheelDiameter;

    if (speed1 == 128 && speed2 == 128 && (cmdVel->linear.x != 0 || cmdVel->angular.z != 0)) {

        // Log
        ROS_WARN("Received velocities are too low for current MD25 configuration (%g, %g), check navigation stack setup for minimum speeds",
                 cmdVel->linear.x, cmdVel->angular.z);

    }

    // Create speeds message
    md25_msgs::StampedSpeeds speedsMessage;
    speedsMessage.header.stamp = ros::Time::now();
    speedsMessage.header.frame_id = baseFrame;
    speedsMessage.speeds.speed1 = speed1;
    speedsMessage.speeds.speed2 = speed2;

    // Publish speed message
    speedsPublisherPtr->publish(speedsMessage);

}
