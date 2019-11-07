/*
This is an extremely simple demo application to showcase the
basic structure, features and use of cvui.

Code licensed under the MIT license, check LICENSE file.
*/

#include <opencv2/opencv.hpp>

// One (and only one) of your C++ files must define CVUI_IMPLEMENTATION
// before the inclusion of cvui.h to ensure its implementaiton is compiled.
#define CVUI_IMPLEMENTATION
#include "cvui.h"
#include <iostream>
#include <cstdlib>
#include <unistd.h>

#define WINDOW_NAME "Attack & Defense Platform"

int main(int argc, const char *argv[])
{
	cv::Mat frame = cv::Mat(200, 500, CV_8UC3);
	int count = 0;

	// Init cvui and tell it to create a OpenCV window, i.e. cv::namedWindow(WINDOW_NAME).
	cvui::init(WINDOW_NAME);
	std::string start;
	std::string run; 
	std::string reset;
	std::string kill;
	while (true) {
		// Fill the frame with a nice color
		frame = cv::Scalar(49, 52, 49);

		// Buttons will return true if they were clicked, which makes
		// handling clicks a breeze.
		start = "No reading...";
		run = "if ready, you can click...";
		reset = "Do you need reset? ";
		kill = "Do you need kill all? ";
		if (cvui::button(frame, 50, 40, "Start: bash start_platform_6vs6.sh")) {
			// The button was clicked, so let's increment our counter.
			std::stringstream fly_cmd;
			fly_cmd << "bash start_platform_6vs6.sh"<< std::endl;
			// ROS_INFO_STREAM(fly_cmd.str());
			system(fly_cmd.str().c_str());
			start = "waiting for ready...";
		}
		cvui::printf(frame, 330, 45, 0.6, 0xff0000, start.c_str());

		if (cvui::button(frame, 50, 80, "Running: ./reset.sh")) {
			// The button was clicked, so let's increment our counter.
			std::stringstream fly_cmd;
			fly_cmd << "./reset.sh"<< std::endl;
			// ROS_INFO_STREAM(fly_cmd.str());
			system(fly_cmd.str().c_str());
			reset = "Reset all environment... ";
		}
		cvui::printf(frame, 240, 85, 0.6, 0xff0000, reset.c_str());

		if (cvui::button(frame, 50, 120, "Running: ./battle.sh")) {
			// The button was clicked, so let's increment our counter.
			std::stringstream fly_cmd;
			fly_cmd << "./battle.sh"<< std::endl;
			// ROS_INFO_STREAM(fly_cmd.str());
			system(fly_cmd.str().c_str());
			run = "click to run...";
		}
		cvui::printf(frame, 240, 125, 0.6, 0xff0000, run.c_str());

		if (cvui::button(frame, 50, 160, "Running: ./kill.sh")) {
			// The button was clicked, so let's increment our counter.
			std::stringstream fly_cmd;
			fly_cmd << "./kill.sh"<< std::endl;
			// ROS_INFO_STREAM(fly_cmd.str());
			system(fly_cmd.str().c_str());
			kill = "killing all process... ";
		}
		cvui::printf(frame, 240, 165, 0.6, 0xff0000, kill.c_str());

		// Update cvui stuff and show everything on the screen
		cvui::imshow(WINDOW_NAME, frame);

		// Check if ESC key was pressed
		if (cv::waitKey(20) == 27) {
			break;
		}
		sleep(0.1);
	}

	return 0;
}
