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

            }

            boost::shared_ptr<MD25SimpleController> md25SimpleController;

    };

}

PLUGINLIB_DECLARE_CLASS(md25, MD25SimpleControllerNodelet, md25::MD25SimpleControllerNodelet, nodelet::Nodelet);

