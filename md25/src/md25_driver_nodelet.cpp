/*
 md25_driver_nodelet.cpp

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

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <boost/thread.hpp>

#include "md25_driver.h"

namespace md25 {

    class MD25DriverNodelet : public nodelet::Nodelet {

        public:

            MD25DriverNodelet() : running(false) {};

            ~MD25DriverNodelet() {

                if (running) {

                    // Set running to false
                    running = false;

                    // Join thread
                    thread->join();

                }

            }

        private:

            virtual void onInit() {

                // Get node handle
                ros::NodeHandle nodeHandle = getNodeHandle();
                ros::NodeHandle privateNodeHanlde = getPrivateNodeHandle();

                // Create md25 driver
                md25Driver.reset(new MD25Driver(nodeHandle, privateNodeHanlde));

                // Init md25 driver
                md25Driver->init();

                // Set running to true
                running = true;

                // Create nodelet thread
                thread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(& MD25DriverNodelet::main, this)));

            }

            void main() {

                while (running) {

                    // Process
                    md25Driver->run();

                }

                // Stop driver
                md25Driver->stop();

            }

            bool running;

            boost::shared_ptr<MD25Driver> md25Driver;

            boost::shared_ptr<boost::thread> thread;

    };

}

PLUGINLIB_DECLARE_CLASS(md25, MD25DriverNodelet, md25::MD25DriverNodelet, nodelet::Nodelet);
