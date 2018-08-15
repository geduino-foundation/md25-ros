/*
 md25_driver.cpp

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

#include "md25_driver.h"

#include <diagnostic_msgs/DiagnosticArray.h>
#include <md25_msgs/StampedEncoders.h>

#define STATUS_OK diagnostic_msgs::DiagnosticStatus::OK
#define STATUS_WARN diagnostic_msgs::DiagnosticStatus::WARN
#define STATUS_ERROR diagnostic_msgs::DiagnosticStatus::ERROR
#define STATUS_STALE diagnostic_msgs::DiagnosticStatus::STALE

MD25Driver::MD25Driver(ros::NodeHandle &_nodeHandle, ros::NodeHandle &_privateNodeHandle) :
    nodeHandle(_nodeHandle), privateNodeHandle(_privateNodeHandle) {
}

MD25Driver::~MD25Driver() {

    if (md25Ptr != NULL) {

        // Log
        ROS_INFO("Disposing MD25...");

        // Dispose MD25
        int result = md25Ptr->dispose();

        if (result != MD25_RESPONSE_OK) {

            // Log
            ROS_ERROR("Failed to dispose MD25");

        }

    }

}

int MD25Driver::init() {

    std::string serialPort;
    int serialBaudrate;
    int serialTimeout;
    int md25Mode;
    int md25Acceleration;
    bool md25EnableTimeout;
    bool md25EnableRegulator;
    double encodersPublishFrequency;
    double diagnosticPublishFrequency;

    // Get parameters
    privateNodeHandle.param<std::string>("serial_port", serialPort, "/dev/ttymxc2");
    privateNodeHandle.param("serial_baudrate", serialBaudrate, 38400);
    privateNodeHandle.param("serial_timeout", serialTimeout, 250);
    privateNodeHandle.param("md25_mode", md25Mode, 0);
    privateNodeHandle.param("md25_acceleration", md25Acceleration, 1);
    privateNodeHandle.param("md25_enable_timeout", md25EnableTimeout, false);
    privateNodeHandle.param("md25_enable_regulator", md25EnableRegulator, true);
    privateNodeHandle.param<std::string>("base_frame", baseFrame, "base_link");
    privateNodeHandle.param("encoders_publish_frequency", encodersPublishFrequency, 10.0);
    privateNodeHandle.param("diagnostic_publish_frequency", diagnosticPublishFrequency, 1.0);

    // Log
    ROS_INFO("Connecting to: %s port with baudrate: %d ...", serialPort.c_str(), serialBaudrate);

    // Create MD25 and set the pointer
    md25Ptr.reset(new MD25(serialPort, serialBaudrate, serialTimeout));

    // Log
    ROS_INFO("Initializing MD25...");

    // Init MD25
    if (md25Ptr->init() != MD25_RESPONSE_OK) {

        // Log
        ROS_FATAL("MD25 initialization failed");

        return -1;

    }

    // Log
    ROS_INFO("Setting MD25 mode to: %d ...", md25Mode);

    // Setting MD25 mode
    if (md25Ptr->setMode(md25Mode) != MD25_RESPONSE_OK) {

        // Log
        ROS_FATAL("MD25 set mode failed");

        return -1;

    }

    // Log
    ROS_INFO("Setting MD25 acceleration to: %d ...", md25Acceleration);

    // Setting MD25 acceleration
    if (md25Ptr->setAcceleration(md25Acceleration) != MD25_RESPONSE_OK) {

        // Log
        ROS_FATAL("MD25 set acceleration failed");

        return -1;

    }

    if (md25EnableTimeout) {

        // Log
        ROS_INFO("Enabling MD25 timeout...");

        // Enabling MD25 timeout
        if (md25Ptr->enableTimeout() != MD25_RESPONSE_OK) {

            // Log
            ROS_FATAL("MD25 enabling timeout failed");

            return -1;

        }

    } else {

        // Log
        ROS_INFO("Disabling MD25 timeout...");

        // Disabling MD25 timeout
        if (md25Ptr->disableTimeout() != MD25_RESPONSE_OK) {

            // Log
            ROS_FATAL("MD25 disabling timeout failed");

            return -1;

        }

    }

    if (md25EnableRegulator) {

        // Log
        ROS_INFO("Enabling MD25 regulator...");

        // Enabling MD25 regulator
        if (md25Ptr->enableRegulator() != MD25_RESPONSE_OK) {

            // Log
            ROS_FATAL("MD25 enabling regulator failed");

            return -1;

        }

    } else {

        // Log
        ROS_INFO("Disabling MD25 regulator...");

        // Disabling MD25 regulator
        if (md25Ptr->disableRegulator() != MD25_RESPONSE_OK) {

            // Log
            ROS_FATAL("MD25 disabling regulator failed");

            return -1;

        }

    }

    // Init encoders publisher
    ros::Publisher encodersPublisher = nodeHandle.advertise<md25_msgs::StampedEncoders>("encoders", 20);
    encodersPublisherPtr = boost::make_shared<ros::Publisher>(encodersPublisher);

    // Init diagnostic publisher
    ros::Publisher diagnosticPublisher = nodeHandle.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 20);
    diagnosticPublisherPtr = boost::make_shared<ros::Publisher>(diagnosticPublisher);

    // Init speeds subscriber
    ros::Subscriber speedsSubscriber = nodeHandle.subscribe("speeds", 20, & MD25Driver::speedsCallback, this);
    speedsSubscriberPtr = boost::make_shared<ros::Subscriber>(speedsSubscriber);

    // Create encoders publish rate
    encodersPublishRatePtr.reset(new ros::Rate(encodersPublishFrequency));

    // Create diagnostic publish rate
    diagnosticPublishRatePtr.reset(new AsleepRate(diagnosticPublishFrequency));

    return 0;

}

void MD25Driver::run() {

    if (md25Ptr == NULL) {

        // Log
        ROS_WARN("Cannot run MD25 driver: driver not initialized");

        return;

    }

    // Sleep
    encodersPublishRatePtr->sleep();

    // Get now
    ros::Time now = ros::Time::now();

    // Read encoders
    uint32_t encoder1, encoder2;

    if (md25Ptr->getEncoders(& encoder1, & encoder2) != MD25_RESPONSE_OK) {

        // Log
        ROS_WARN_THROTTLE(1.0, "Error reading encoders");

        // Publish diagnostic
        publishDiagnostics(STATUS_ERROR, "Error reading encoders");

        return;

    }

    // Create encoders message
    md25_msgs::StampedEncoders stampedEncodersMessage;
    stampedEncodersMessage.header.stamp = now;
    stampedEncodersMessage.header.frame_id = baseFrame;
    stampedEncodersMessage.encoders.encoder1 = encoder1;
    stampedEncodersMessage.encoders.encoder2 = encoder2;

    // Publish encoders message
    encodersPublisherPtr->publish(stampedEncodersMessage);

    if (diagnosticPublishRatePtr->ellapsed(now.toSec())) {

        // Publish diagnostic
        publishDiagnostics(STATUS_OK, "OK");

    }

}

void MD25Driver::stop() {

    if (md25Ptr != NULL) {

        // Log
        ROS_INFO("Stopping MD25 motors...");

        // Stop MD25 motors
        int result = md25Ptr->stop();

        if (result != MD25_RESPONSE_OK) {

            // Log
            ROS_ERROR("Failed to stop motors");

            // Publish diagnostic
            publishDiagnostics(STATUS_ERROR, "Failed to stop motors");

        }

    } else {

        // Log
        ROS_WARN("Cannot stop MD25 motors: driver not initialized");

    }

}

void MD25Driver::speedsCallback(const md25_msgs::StampedSpeedsConstPtr & stampedSpeedsMessage) {

    // Get speeds
    uint8_t speed1 = stampedSpeedsMessage->speeds.speed1;
    uint8_t speed2 = stampedSpeedsMessage->speeds.speed2;

    // Set sppeds on MD25
    if (md25Ptr->setSpeed1(speed1) == MD25_RESPONSE_OK &&
         md25Ptr->setSpeed2(speed2) == MD25_RESPONSE_OK) {

        // Log
        ROS_ERROR("Failed to set speeds");

        // Publish diagnostic
        publishDiagnostics(STATUS_ERROR, "Failed to set speeds");

    }

}

void MD25Driver::publishDiagnostics(uint8_t level, std::string message) {

    // Create diagnostics message
    diagnostic_msgs::DiagnosticArray diagnosticMessage;
    diagnosticMessage.header.stamp = ros::Time::now();
    diagnosticMessage.status.resize(1);
    diagnosticMessage.status[0].level = level;
    diagnosticMessage.status[0].name = "MD25";
    diagnosticMessage.status[0].message = message.c_str();
    diagnosticMessage.status[0].hardware_id = "md25";

    // Read md25 diagnostics
    uint8_t version, volts, current1, current2;
    uint8_t result = (md25Ptr->getVersion(& version) == MD25_RESPONSE_OK &&
            md25Ptr->getVI(& volts, & current1, & current2) == MD25_RESPONSE_OK)
            ? MD25_RESPONSE_OK : MD25_RESPONSE_ERROR;

    // Convert volts to float
    float voltsFloat = 0.1 * volts;

    // Covert current to float
    float current1Float = 0.1 * current1;
    float current2Float = 0.1 * current2;

    if (result == MD25_RESPONSE_OK) {

        // Get values as char array
        char versionChars[3];
        sprintf(versionChars, "%d", version);
        char voltsChars[6];
        sprintf(voltsChars, "%g V", voltsFloat);
        char current1Chars[6];
        sprintf(current1Chars, "%g A", current1Float);
        char current2Chars[6];
        sprintf(current2Chars, "%g A", current2Float);

        diagnosticMessage.status[0].values.resize(4);
        diagnosticMessage.status[0].values[0].key = "Version";
        diagnosticMessage.status[0].values[0].value = versionChars;
        diagnosticMessage.status[0].values[1].key = "Volts";
        diagnosticMessage.status[0].values[1].value = voltsChars;
        diagnosticMessage.status[0].values[2].key = "Motor 1 current";
        diagnosticMessage.status[0].values[2].value = current1Chars;
        diagnosticMessage.status[0].values[3].key = "Motor 2 current";
        diagnosticMessage.status[0].values[3].value = current2Chars;

    } else {

        // Log
            ROS_WARN("Error reading diagnostics data: %d", result);

        // No values
        diagnosticMessage.status[0].values.resize(0);

    }

    // Publish diagnostic message
    diagnosticPublisherPtr->publish(diagnosticMessage);

}
