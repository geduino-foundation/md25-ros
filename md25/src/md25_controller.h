/*
 md25_controller.h

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

#ifndef MD25CONTROLLER_H
#define MD25CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <dynamic_reconfigure/server.h>
#include <md25/MD25ControllerConfig.h>

#include "pid.h"

class MD25Controller {

    public:

        // Create new MD25 controller
        MD25Controller(ros::NodeHandle & _nodeHandle, ros::NodeHandle & _privateNodeHandle);

        // Init MD25 controller
        int init();

    private:

        // The cmd_vel callback
        void cmdVelCallback(const geometry_msgs::Twist::ConstPtr & cmdVelMessage);

        // The odometry callback
        void odometryCallback(const nav_msgs::Odometry::ConstPtr & odometryMessage);

        // Publish speeds message
        void publishSpeeds(ros::Time stamp, uint8_t speed1, uint8_t speed2);

        // The reconfigure callback
        void reconfigureCallback(md25::MD25ControllerConfig & config, uint32_t level);

        // The ROS node handles
        ros::NodeHandle nodeHandle, privateNodeHandle;

        // The dynamic reconfigure server
        dynamic_reconfigure::Server<md25::MD25ControllerConfig> * dynamicReconfigureServer;

        // The base frame speeds message will be stamped to
        std::string baseFrame;

        // The target velocity
        double targetVelocityLinearX, targetVelocityAngularZ;

        // The last update time
        double lastUpdateTime;

        // The pointer to PID
        boost::shared_ptr<PID> pidPtr;

        // The speeds publisher
        boost::shared_ptr<ros::Publisher> speedsPublisherPtr;

        // The cmd_vel subscriber
        boost::shared_ptr<ros::Subscriber> cmdVelSubscriberPtr;

        // The odometry subscriber
        boost::shared_ptr<ros::Subscriber> odomSubscriberPtr;

};

#endif
