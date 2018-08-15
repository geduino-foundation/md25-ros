/*
 md25_controller.cpp

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

#include "md25_controller.h"

#include <md25_msgs/StampedSpeeds.h>

MD25Controller::MD25Controller(ros::NodeHandle &_nodeHandle, ros::NodeHandle &_privateNodeHandle) :
    nodeHandle(_nodeHandle), privateNodeHandle(_privateNodeHandle) {
}

int MD25Controller::init() {

    double controllerLinearGain;
    double controllerLinearDerivativeGain;
    double controllerAngularGain;
    double controllerAngularDerivativeGain;

    // Get params
    privateNodeHandle.param("controller_linear_gain", controllerLinearGain, 10.0);
    privateNodeHandle.param("controller_linear_derivative_gain", controllerLinearDerivativeGain, 10.0);
    privateNodeHandle.param("controller_angular_gain", controllerAngularGain, 10.0);
    privateNodeHandle.param("controller_angular_derivative_gain", controllerAngularDerivativeGain, 10.0);
    privateNodeHandle.param<std::string>("base_frame", baseFrame, "base_link");

    // Create PID
    pidPtr.reset(new PID(controllerLinearGain, controllerLinearDerivativeGain,
                         controllerAngularGain, controllerAngularDerivativeGain));

    // Init speeds publisher
    ros::Publisher speedsPublisher = nodeHandle.advertise<md25_msgs::StampedSpeeds>("speeds", 20);
    speedsPublisherPtr = boost::make_shared<ros::Publisher>(speedsPublisher);

    // Init cmd_vel subscriber
    ros::Subscriber cmdVelSubscriber = nodeHandle.subscribe("cmd_vel", 1, & MD25Controller::cmdVelCallback, this);
    cmdVelSubscriberPtr = boost::make_shared<ros::Subscriber>(cmdVelSubscriber);

    // Init odometry subscriber
    ros::Subscriber odometrySubscriber = nodeHandle.subscribe("odom", 1, & MD25Controller::odometryCallback, this);
    odomSubscriberPtr = boost::make_shared<ros::Subscriber>(odometrySubscriber);

    // Create dynamic reconfigure server
    dynamicReconfigureServer = new dynamic_reconfigure::Server<md25::MD25ControllerConfig>(privateNodeHandle);
    dynamic_reconfigure::Server<md25::MD25ControllerConfig>::CallbackType callback = boost::bind(& MD25Controller::reconfigureCallback, this, _1, _2);
    dynamicReconfigureServer->setCallback(callback);

    // Set target and actual velocity
    targetVelocityLinearX = 0;
    targetVelocityAngularZ = 0;

    // Set last update time
    lastUpdateTime = 0;

    return 0;

}

void MD25Controller::cmdVelCallback(const geometry_msgs::Twist::ConstPtr & cmdVelMessage) {

    // Update target velocity
    targetVelocityLinearX = cmdVelMessage->linear.x;
    targetVelocityAngularZ = cmdVelMessage->angular.z;

}

void MD25Controller::odometryCallback(const nav_msgs::Odometry::ConstPtr & odometryMessage) {

    if (lastUpdateTime > 0) {

        // Compute deltas
        double deltaLinearX = targetVelocityLinearX - odometryMessage->twist.twist.linear.x;
        double deltaAngularZ = targetVelocityAngularZ - odometryMessage->twist.twist.angular.z;
        double deltaTime = odometryMessage->header.stamp.toSec() - lastUpdateTime;

        // Update PID
        pidPtr->update(deltaLinearX, deltaAngularZ, deltaTime);

        // Get speeds
        int speed1;
        int speed2;
        pidPtr->getSpeeds(& speed1, & speed2);

        // Publish speeds
        publishSpeeds(odometryMessage->header.stamp, speed1, speed2);

    }

    // Update last update time
    lastUpdateTime = odometryMessage->header.stamp.toSec();

}

void MD25Controller::reconfigureCallback(md25::MD25ControllerConfig & config, uint32_t level) {

    // Apply configuration
    pidPtr->set(config.controller_linear_gain, config.controller_linear_derivative_gain,
                config.controller_angular_gain, config.controller_angular_derivative_gain);

    // Log
    ROS_INFO("Reconfigured");

}

void MD25Controller::publishSpeeds(ros::Time stamp, uint8_t speed1, uint8_t speed2) {

    // Create speeds message
    md25_msgs::StampedSpeeds speedsMessage;
    speedsMessage.header.stamp = stamp;
    speedsMessage.header.frame_id = baseFrame;
    speedsMessage.speeds.speed1 = speed1;
    speedsMessage.speeds.speed2 = speed2;

    // Publish speed message
    speedsPublisherPtr->publish(speedsMessage);

}
