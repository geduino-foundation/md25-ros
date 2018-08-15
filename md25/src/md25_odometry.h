/*
 md25_odometry.h

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

#ifndef MD25ODOMETRY_H
#define MD25ODOMETRY_H

#include <ros/ros.h>
#include <md25_msgs/StampedEncoders.h>
#include <sensor_msgs/Imu.h>
#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include <md25/MD25OdometryConfig.h>

#include "ddr_odometry.h"
#include "yaw_filter.h"
#include "md25_odometry.h"

class MD25Odometry {

    public:

        // Create MD25 odometry
        MD25Odometry(ros::NodeHandle & _nodeHandle, ros::NodeHandle & _privateNodeHandle);

        // Init MD25 odometry
        int init();

    private:

        // The encoders callback
        void encodersCallback(const md25_msgs::StampedEncodersConstPtr & stampedEncodersMessage);

        // The reconfigure callback
        void reconfigureCallback(md25::MD25OdometryConfig & config, uint32_t level);

        // Update odometry
        void updateOdometry(double deltaSpace, double deltaTheta, double deltaTime);

        // Update filtered odometry
        void updateOdometryFiltered(double deltaSpace, double deltaTheta, double deltaTime, std_msgs::Header header);

        // Publish odometry message
        void publishOdometry(std_msgs::Header header, boost::shared_ptr<ros::Publisher> _odometryPublisherPtr,
                                     boost::shared_ptr<DDROdometry> _ddrOdometryPtr, bool _publishOdometryTransformation);

        // The ROS node handles
        ros::NodeHandle nodeHandle, privateNodeHandle;

        // The dynamic reconfigure server
        dynamic_reconfigure::Server<md25::MD25OdometryConfig> * dynamicReconfigureServer;

        // The odom frame
        std::string odomFrame;

        // The wheel base
        double wheelBase;

        // The encoder sensitivity
        double encodersSensitivity;

        // The positin and velocity covariance matrix diagonal
        std::vector<double> positionCovarianceDiagonal, velocityCovarianceDiagonal;

        // The flag to publish odometry transformation
        bool publishOdometryTransformation;

        // The flag to enable odometry fusing with IMU data
        bool filterEnabled;

        // The last encoder values
        uint32_t lastEncoder1, lastEncoder2;

        // The last update time
        double lastUpdateTime;

        // The encoders initialized flag
        bool encodersInitialized;

        // The odometry publisher
        boost::shared_ptr<ros::Publisher> odometryPublisherPtr;
        boost::shared_ptr<ros::Publisher> odometryFilteredPublisherPtr;

        // The odometry transform broadcaster
        tf::TransformBroadcaster odometryTransformBroadcaster;

        // The transform listener
        boost::shared_ptr<tf::TransformListener> transformListenerPtr;

        // The encoders subscriber
        boost::shared_ptr<ros::Subscriber> encodersSubscriberPtr;

        // The encoders subscriber
        boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Imu> > imuSubscriberPtr;

        // The imu message cache
        boost::shared_ptr<message_filters::Cache<sensor_msgs::Imu> > imuCachePtr;

        // The pointer to DDR odometry
        boost::shared_ptr<DDROdometry> ddrOdometryPtr;
        boost::shared_ptr<DDROdometry> ddrOdometryFilteredPtr;

        // The pointer to yaw filter
        boost::shared_ptr<YawFilter> yawFilterPtr;

};

#endif
