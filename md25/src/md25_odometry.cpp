/*
 md25_odometry.cpp

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

#include "md25_odometry.h"

#include <message_filters/subscriber.h>
#include <nav_msgs/Odometry.h>

#define PI 3.1415926535

MD25Odometry::MD25Odometry(ros::NodeHandle &_nodeHandle, ros::NodeHandle &_privateNodeHandle) :
    nodeHandle(_nodeHandle), privateNodeHandle(_privateNodeHandle) {
}

int MD25Odometry::init() {

    int clickPerRevolution;
    double wheelDiameter;
    int rollingWindowSize;
    int imuCacheSize;
    double filterTau;

    // Get parameters
    privateNodeHandle.param("wheel_base", wheelBase, 0.275);
    privateNodeHandle.param("click_per_revolution", clickPerRevolution, 360);
    privateNodeHandle.param("wheel_diameter", wheelDiameter, 0.1);
    privateNodeHandle.param("rolling_window_size", rollingWindowSize, 3);
    privateNodeHandle.getParam("position_covariance_diagonal", positionCovarianceDiagonal);
    privateNodeHandle.getParam("velocity_covariance_diagonal", velocityCovarianceDiagonal);
    privateNodeHandle.param("publish_odometry_transformation", publishOdometryTransformation, true);
    privateNodeHandle.param<std::string>("odom_frame", odomFrame, "odom");
    privateNodeHandle.param("imu_cache_size", imuCacheSize, 10);
    privateNodeHandle.param("filter_enabled", filterEnabled, false);
    privateNodeHandle.param("filter_tau", filterTau, 0.075);

    if (positionCovarianceDiagonal.size() != 6) {

        // Log
        ROS_WARN("Position_covariance_diagonal size must be 6, actual: %lu. Use zeroes.", positionCovarianceDiagonal.size());

        // Assign a six sized vector fill with zeroes
        positionCovarianceDiagonal.assign(6, 0.0f);

    }

    if (velocityCovarianceDiagonal.size() != 6) {

        // Log
        ROS_WARN("Velocity_covariance_diagonal size must be 6, actual: %lu. Use zeroes.", velocityCovarianceDiagonal.size());

        // Assign a six sized vector fill with zeroes
        velocityCovarianceDiagonal.assign(6, 0.0f);

    }

    // Create DDR odometry
    ddrOdometryPtr.reset(new DDROdometry(rollingWindowSize));
    ddrOdometryFilteredPtr.reset(new DDROdometry(rollingWindowSize));

    // Create yaw filter
    yawFilterPtr.reset(new YawFilter(filterTau));

    // Init odometry publisher
    ros::Publisher odometryPublisher = nodeHandle.advertise<nav_msgs::Odometry>("odom", 20);
    odometryPublisherPtr = boost::make_shared<ros::Publisher>(odometryPublisher);
    ros::Publisher odometryFilteredPublisher = nodeHandle.advertise<nav_msgs::Odometry>("odom_filtered", 20);
    odometryFilteredPublisherPtr = boost::make_shared<ros::Publisher>(odometryFilteredPublisher);

    // Init encoders subscriber
    ros::Subscriber encodersSubscriber = nodeHandle.subscribe("encoders", 20, & MD25Odometry::encodersCallback, this);
    encodersSubscriberPtr = boost::make_shared<ros::Subscriber>(encodersSubscriber);

    // Init imu message subscriber and cache
    message_filters::Subscriber<sensor_msgs::Imu> imuSubscriber(nodeHandle, "imu", 1);
    imuCachePtr.reset(new message_filters::Cache<sensor_msgs::Imu>(imuSubscriber, imuCacheSize));

    // Init transform listener
    transformListenerPtr.reset(new tf::TransformListener(nodeHandle));

    // Create dynamic reconfigure server
    dynamicReconfigureServer = new dynamic_reconfigure::Server<md25::MD25OdometryConfig>(privateNodeHandle);
    dynamic_reconfigure::Server<md25::MD25OdometryConfig>::CallbackType callback = boost::bind(& MD25Odometry::reconfigureCallback, this, _1, _2);
    dynamicReconfigureServer->setCallback(callback);

    // Compute encoder sensitivity
    encodersSensitivity = wheelDiameter * PI / clickPerRevolution;

    // Set encoders as not initialized
    encodersInitialized = false;

    return 0;

}

void MD25Odometry::encodersCallback(const md25_msgs::StampedEncodersConstPtr & stampedEncodersMessage) {

    if (encodersInitialized) {

        // Compute deltas
        int32_t deltaEncoder1 = stampedEncodersMessage->encoders.encoder1 - lastEncoder1;
        int32_t deltaEncoder2 = stampedEncodersMessage->encoders.encoder2 - lastEncoder2;
        double deltaTime = stampedEncodersMessage->header.stamp.toSec() - lastUpdateTime;
        double deltaSpace = (deltaEncoder1 + deltaEncoder2) * encodersSensitivity / 2;
        double deltaTheta = (deltaEncoder2 - deltaEncoder1) * encodersSensitivity / wheelBase;

        // Update odometry
        updateOdometry(deltaSpace, deltaTheta, deltaTime);

        // Update odometry
        updateOdometryFiltered(deltaSpace, deltaTheta, deltaTime, stampedEncodersMessage->header);

        // Publish odometry
        publishOdometry(stampedEncodersMessage->header, odometryPublisherPtr, ddrOdometryPtr, false);

        // Publish filtered odometry
        publishOdometry(stampedEncodersMessage->header, odometryFilteredPublisherPtr, ddrOdometryFilteredPtr, publishOdometryTransformation);

    } else {

        // Log
        ROS_INFO("Received first encoders");

        // Set encoders as initialized
        encodersInitialized = true;

    }

    // Set last encoders and time
    lastEncoder1 = stampedEncodersMessage->encoders.encoder1;
    lastEncoder2 = stampedEncodersMessage->encoders.encoder2;
    lastUpdateTime = stampedEncodersMessage->header.stamp.toSec();

}

void MD25Odometry::reconfigureCallback(md25::MD25OdometryConfig & config, uint32_t level) {

    // Apply configuration
    yawFilterPtr->set(config.filter_tau);
    filterEnabled = config.filter_enabled;

    // Log
    ROS_INFO("Reconfigured");

}

void MD25Odometry::updateOdometry(double deltaSpace, double deltaTheta, double deltaTime) {

    // Update odometry
    ddrOdometryPtr->update(deltaSpace, deltaTheta, deltaTime);

}


void MD25Odometry::updateOdometryFiltered(double deltaSpace, double deltaTheta, double deltaTime, std_msgs::Header header) {

    if (!filterEnabled) {

        // Update odometry filtered with only encoder information
        ddrOdometryFilteredPtr->update(deltaSpace, deltaTheta, deltaTime);

        return;

    }

    // Get imu message
    sensor_msgs::ImuConstPtr imuPtr = imuCachePtr->getElemBeforeTime(header.stamp);

    if (imuPtr == NULL) {

        // Log
        ROS_ERROR_THROTTLE(1.0, "Imu message not received at %f. Update odometry with encoder only.", header.stamp.toSec());

        // Update odometry filtered with only encoder information
        ddrOdometryFilteredPtr->update(deltaSpace, deltaTheta, deltaTime);

        return;

    }

    // Get angular velocity in base frame
    geometry_msgs::Vector3Stamped angularVelocityInBaseFrame;

    // Get angular velocity in IMU frame
    geometry_msgs::Vector3Stamped angularVelocityInImuFrame;
    angularVelocityInImuFrame.header = imuPtr->header;
    angularVelocityInImuFrame.vector = imuPtr->angular_velocity;

    std::string errorMsg;

    if (transformListenerPtr->canTransform(header.frame_id, imuPtr->header.frame_id, header.stamp, & errorMsg)) {

        // Transform angular velocity in base frame
        transformListenerPtr->transformVector(header.frame_id, angularVelocityInImuFrame, angularVelocityInBaseFrame);

    } else {

        // Log
        ROS_ERROR_THROTTLE(1.0, "Cannot transform from %s -> %s at %f: %s. Update odometry with encoder only.", imuPtr->header.frame_id.c_str(),
                           header.frame_id.c_str(), header.stamp.toSec(), errorMsg.c_str());

        // Update odometry filtered with only encoder information
        ddrOdometryFilteredPtr->update(deltaSpace, deltaTheta, deltaTime);

        return;

    }

    // Get position from unfiltered odometry
    Vector3 position;
    ddrOdometryPtr->getPosition(position);

    // Update yaw filter
    yawFilterPtr->filter(position(2), angularVelocityInBaseFrame.vector.z, deltaTime);

    // Get filtered delta theta
    double filteredDeltaTheta;
    yawFilterPtr->getDelta(filteredDeltaTheta);

    // Update filtered odometry
    ddrOdometryFilteredPtr->update(deltaSpace, filteredDeltaTheta, deltaTime);

}

void MD25Odometry::publishOdometry(std_msgs::Header header, boost::shared_ptr<ros::Publisher> _odometryPublisherPtr,
                                   boost::shared_ptr<DDROdometry> _ddrOdometryPtr, bool _publishOdometryTransformation) {

    // Get position
    Vector3 position;
    _ddrOdometryPtr->getPosition(position);

    // Get velocity
    Vector3 velocity;
    _ddrOdometryPtr->getVelocity(velocity);

    // Create odometry quaternion from th
    geometry_msgs::Quaternion odometryQuaternion = tf::createQuaternionMsgFromYaw(position(2));

    // Create odometry message
    nav_msgs::Odometry odometryMessage;
    odometryMessage.header.stamp = header.stamp;
    odometryMessage.header.frame_id = odomFrame;
    odometryMessage.child_frame_id = header.frame_id;
    odometryMessage.pose.pose.position.x = position(0);
    odometryMessage.pose.pose.position.y = position(1);
    odometryMessage.pose.pose.position.z = 0.0;
    odometryMessage.pose.pose.orientation = odometryQuaternion;
    odometryMessage.twist.twist.linear.x = velocity(0);
    odometryMessage.twist.twist.linear.y = velocity(1);
    odometryMessage.twist.twist.angular.z = velocity(2);
    odometryMessage.pose.covariance[0] = positionCovarianceDiagonal[0];
    odometryMessage.pose.covariance[7] = positionCovarianceDiagonal[1];
    odometryMessage.pose.covariance[14] = positionCovarianceDiagonal[2];
    odometryMessage.pose.covariance[21] = positionCovarianceDiagonal[3];
    odometryMessage.pose.covariance[28] = positionCovarianceDiagonal[4];
    odometryMessage.pose.covariance[35] = positionCovarianceDiagonal[5];
    odometryMessage.twist.covariance[0] = velocityCovarianceDiagonal[0];
    odometryMessage.twist.covariance[7] = velocityCovarianceDiagonal[1];
    odometryMessage.twist.covariance[14] = velocityCovarianceDiagonal[2];
    odometryMessage.twist.covariance[21] = velocityCovarianceDiagonal[3];
    odometryMessage.twist.covariance[28] = velocityCovarianceDiagonal[4];
    odometryMessage.twist.covariance[35] = velocityCovarianceDiagonal[5];

    // Publish odometry message
    _odometryPublisherPtr->publish(odometryMessage);

    if (_publishOdometryTransformation) {

        // Send odometry transformation
        geometry_msgs::TransformStamped odometryTransformation;
        odometryTransformation.header.stamp = header.stamp;
        odometryTransformation.header.frame_id = odomFrame;
        odometryTransformation.child_frame_id = header.frame_id;
        odometryTransformation.transform.translation.x = position(0);
        odometryTransformation.transform.translation.y = position(1);
        odometryTransformation.transform.translation.z = 0.0;
        odometryTransformation.transform.rotation = odometryQuaternion;

        // Broadcast odometry transformation
        odometryTransformBroadcaster.sendTransform(odometryTransformation);

    }

}


