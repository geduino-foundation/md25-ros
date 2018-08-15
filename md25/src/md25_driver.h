/*
 md25_driver.h

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

#ifndef MD25_DRIVER_H
#define MD25_DRIVER_H

#include <ros/ros.h>
#include <md25_msgs/StampedSpeeds.h>

#include "md25.h"
#include "asleep_rate.h"

class MD25Driver {

    public:

        // Create MD25 driver wrapper
        MD25Driver(ros::NodeHandle & _nodeHandle, ros::NodeHandle & _privateNodeHandle);

        // Destroy MD25 driver wrapper
        ~MD25Driver();

        // Init MD25 driver
        int init();

        // Execute driver main cycle
        void run();

        // Stop motors
        void stop();

    private:

        // The speeds callback
        void speedsCallback(const md25_msgs::StampedSpeedsConstPtr & stampedSpeedsMessage);

        // Publish MD25 diagnostics
        void publishDiagnostics(uint8_t level, std::string message);

        // The ROS node handles
        ros::NodeHandle nodeHandle, privateNodeHandle;

        // The base frame encoders message will be stamped to
        std::string baseFrame;

        // The encoders publisher
        boost::shared_ptr<ros::Publisher> encodersPublisherPtr;

        // The diagnostic publisher
        boost::shared_ptr<ros::Publisher> diagnosticPublisherPtr;

        // The speeds subscriber
        boost::shared_ptr<ros::Subscriber> speedsSubscriberPtr;

        // The pointer to MD25
        boost::shared_ptr<MD25> md25Ptr;

        // The pointer to encoders publish rate
        boost::shared_ptr<ros::Rate> encodersPublishRatePtr;

        // The pointer to diagnostic publish rate
        boost::shared_ptr<AsleepRate> diagnosticPublishRatePtr;

};

#endif
