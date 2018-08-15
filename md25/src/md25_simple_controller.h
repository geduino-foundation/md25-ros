/*
 md25_simple_controller.h

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

#ifndef MD25SIMPLECONTROLLER_H
#define MD25SIMPLECONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class MD25SimpleController {

    public:

        // Create MD25 simple controller
        MD25SimpleController(ros::NodeHandle & _nodeHandle, ros::NodeHandle & _privateNodeHandle);

        // Init MD25 simple controller
        int init();

    private:

        // The cmd_vel callback
        void cmdVelCallback(const geometry_msgs::Twist::ConstPtr & cmdVel);

        // Publish speeds message
        void publishSpeeds(ros::Time stamp, uint8_t speed1, uint8_t speed2);

        // The ROS node handles
        ros::NodeHandle nodeHandle, privateNodeHandle;

        // The speed senitivity
        double speedSensitivity;

        // The wheel base
        double wheelBase;

        // The wheel diameter
        double wheelDiameter;

        // The base frame speeds message will be stamped to
        std::string baseFrame;

        // The speeds publisher
        boost::shared_ptr<ros::Publisher> speedsPublisherPtr;

        // The cmd_vel subscriber
        boost::shared_ptr<ros::Subscriber> cmdVelSubscriberPtr;

};

#endif
