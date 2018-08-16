/*
 md25_simple_controller_nodelet.cpp

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

#include "md25_simple_controller.h"

namespace md25 {

    class MD25SimpleControllerNodelet : public nodelet::Nodelet {

        public:

            MD25SimpleControllerNodelet() {};

        private:

            virtual void onInit() {

                // Get node handle
                ros::NodeHandle nodeHandle = getNodeHandle();
                ros::NodeHandle privateNodeHanlde = getPrivateNodeHandle();

                // Create md25 simple controller
                md25SimpleController.reset(new MD25SimpleController(nodeHandle, privateNodeHanlde));

                // Init md25 simple controller
                md25SimpleController->init();

                // Set running to true
                running = true;

                // Create nodelet thread
                thread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(& MD25SimpleControllerNodelet::main, this)));

            }

            void main() {

                while (running) {

                    // Run
                    md25SimpleController->run();

                }

            }

            bool running;

            boost::shared_ptr<MD25SimpleController> md25SimpleController;

            boost::shared_ptr<boost::thread> thread;

    };

}

PLUGINLIB_DECLARE_CLASS(md25, MD25SimpleControllerNodelet, md25::MD25SimpleControllerNodelet, nodelet::Nodelet);


