///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2017, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////


#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <cstring>
#include <fstream>
#include <fstream>
#include <conio.h> 
//#include "utils.h"
using namespace sl;
using namespace std;
enum APP_TYPE {
	LEFT_AND_RIGHT,
	LEFT_AND_DEPTH,
	LEFT_AND_DEPTH_16
};

cv::Mat slMat2cvMat(sl::Mat &input);

int main(int argc, char **argv) {

	// Create a ZED camera object
	Camera zed;

	// Set configuration parameters
	InitParameters init_params;
	init_params.camera_resolution = RESOLUTION_HD1080; // Use HD1080 video mode
	init_params.camera_fps = 30; // Set fps at 30
	init_params.depth_mode = DEPTH_MODE_ULTRA;
	init_params.coordinate_system = COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP;
	init_params.coordinate_units = UNIT_MILLIMETER;
	// Open the camera
	ERROR_CODE err = zed.open(init_params);
	if (err != SUCCESS)
		exit(-1);
	TrackingParameters tracking_parameters;
	err = zed.enableTracking(tracking_parameters);
	if (err != SUCCESS)
		exit(-1);
	APP_TYPE app_type = LEFT_AND_DEPTH;
	Resolution image_size = zed.getResolution();
	int width = image_size.width;
	int height = image_size.height;

	Mat left_image(width, height, MAT_TYPE_8U_C4);
	cv::Mat left_image_ocv;

	Mat  right_image(width, height, MAT_TYPE_8U_C4);
	cv::Mat right_image_ocv;

	Mat  depth_image(width, height, MAT_TYPE_32F_C1);
	cv::Mat depth_image_ocv;

	// Green detection
	string output_path("C:\\Users\\Yucheng Chen\\Documents\\ZED\\ZED_IMAGE_CAP\\Images");
	//float depth_val = -1;
	string filename0 = "C:\\Users\\Yucheng Chen\\Documents\\ZED\\ZED_IMAGE_CAP\\Images\\pose_data.txt";
	Pose zed_pose;
	ofstream posedata;
	posedata.open(filename0);
	while (zed.grab() == SUCCESS && !kbhit()) {
		//Track pose
		TRACKING_STATE state = zed.getPosition(zed_pose, REFERENCE_FRAME_WORLD);
		/**
		printf("Translation: tx: %.3f, ty:  %.3f, tz:  %.3f, timestamp: %llu\r",
			zed_pose.getTranslation().tx, zed_pose.getTranslation().ty, zed_pose.getTranslation().tz, zed_pose.timestamp);
		// Display orientation quaternion
		printf("Orientation: ox: %.3f, oy:  %.3f, oz:  %.3f, ow: %.3f\r",
			zed_pose.getOrientation().ox, zed_pose.getOrientation().oy, zed_pose.getOrientation().oz, zed_pose.getOrientation().ow);
			*/
			//Save pose data
		posedata << zed_pose.timestamp << "," << zed_pose.getTranslation().tx << "," << zed_pose.getTranslation().ty << "," << zed_pose.getTranslation().tz << ","
			<< zed_pose.getOrientation().ox << "," << zed_pose.getOrientation().oy << "," << zed_pose.getOrientation().oz << "," << zed_pose.getOrientation().ow << '\n';
		// A new image is available if grab() returns SUCCESS
		unsigned long long timestamp = zed.getCameraTimestamp(); // Get the timestamp at the time the image was captured
		ostringstream filename1;
		filename1 << output_path << "\\left" << timestamp << ".png";
		ostringstream filename2;
		filename2 << output_path << (app_type == LEFT_AND_RIGHT ? "\\right" : "\\depth") << timestamp << ".png";
		char str_name[86];
		string filename_depth = filename2.str();
		strcpy(str_name, filename_depth.c_str());
		String filename_depth_sl(str_name);
		zed.retrieveImage(left_image, VIEW_LEFT); // Get the left image
		left_image_ocv = slMat2cvMat(left_image);
		/*
		cv::Point pt1(1284, 859);
		cv::Point pt2(1419, 462);
		cv::Point center;
		center = (pt1 + pt2) / 2.0;
		cv::rectangle(left_image_ocv, pt1, pt2, (0, 0, 255), 5);
		cv:circle(left_image_ocv, center, 0.2, (0, 0, 255), 5);
		cv::imshow("hello", left_image_ocv);
		cv::waitKey(10);
		*/
		cv::imwrite(filename1.str(), left_image_ocv);
		switch (app_type) {
		case LEFT_AND_RIGHT:
			zed.retrieveImage(right_image, VIEW_RIGHT);
			right_image_ocv = slMat2cvMat(right_image);
			break;
		case  LEFT_AND_DEPTH:
			zed.retrieveMeasure(depth_image, MEASURE_DEPTH);
			//depth_image.getValue(center.x, center.y, &depth_val);
			//printf("%f\n", depth_val);
			saveDepthAs(depth_image, DEPTH_FORMAT_PNG, filename_depth_sl);
			//saveDepthAs(depth_image, DEPTH_FORMAT_PNG,filename_depth,0.01);
			break;
		case  LEFT_AND_DEPTH_16:
			zed.retrieveMeasure(depth_image, MEASURE_DEPTH);
			//saveDepthAs(depth_image, DEPTH_FORMAT_PNG, filename_depth_sl, 0.01f);
			break;
		default:
			break;
		}
	}
	// Close the camera
	zed.close();
	posedata.close();
	return 0;
}

cv::Mat slMat2cvMat(sl::Mat &input) {
	int cv_type = -1;
	switch (input.getDataType()) {
	case sl::MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
	case sl::MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
	case sl::MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
	case sl::MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
	case sl::MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
	case sl::MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
	case sl::MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
	case sl::MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
	default: break;
	}
	// Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
	// cv::Mat and sl::Mat will share a single memory structure
	return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM_CPU));
}
