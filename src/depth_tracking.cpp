#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include "libfreenect.hpp"
#include <kinect_device.h>
#include <vector>

// Kinect variables
Freenect::Freenect freenect;
MyFreenectDevice* device;
freenect_video_format requested_format;
std::vector<cv::Mat> depth_buffer;
std::vector<cv::Mat> rgb_buffer;
double freenect_angle;
int n_buffers, buffer_id, width, height, rgb_channels, depth_channels;
bool show_depth, show_rgb, stop_program;

void convertRGB(cv::Mat &image) {
	static std::vector<uint8_t> rgb_raw(width*height*rgb_channels);
	try {
		device->getRGB(rgb_raw);
	} catch (...) {
		printf("couldn't grab RGB\n");
	}
	int k = 0;
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			image.at<cv::Vec3b>(i,j)[0] = rgb_raw[k];
			image.at<cv::Vec3b>(i,j)[1] = rgb_raw[k+1];
			image.at<cv::Vec3b>(i,j)[2] = rgb_raw[k+2];
			k += 3;
		}
	}
	cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
}

void convertDepth(cv::Mat &depth_image) {
	static std::vector<uint8_t> depth_raw(width*height*depth_channels);
	try {
		device->getDepth(depth_raw);
	} catch (...) {
		printf("couldn't grab RGB\n");
	}
	int k = 0;
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			depth_image.at<cv::Vec3b>(i,j)[2] = depth_raw[k];
			depth_image.at<cv::Vec3b>(i,j)[1] = depth_raw[k+1];
			depth_image.at<cv::Vec3b>(i,j)[0] = depth_raw[k+2];
			k += 3;
		}
	}	
}

void userMenu ()
{
  printf("\nOPTIONS:\n"
  			"w:\tINCREASE CAMERA ANGLE\n"
  			"s:\tCENTER CAMERA ANGLE\n"
  			"x:\tDECREASE CAMERA ANGLE\n"
  			"f:\tCHANGE FORMAT\n"
  			"a:\tPAUSE RGB VIDEO\n"
  			"d:\tPAUSE DEPTH VIDEO\n"
  			"0-5:\tCHANGE LED\n"
  			"esc:\tEXIT\n");
}

void keyPressed(Freenect::FreenectDevice* kinect, int key)
{
	if (key == 27) {
		stop_program = true;
	} else if (key == '1') {
		kinect->setLed(LED_GREEN);
	} else if (key == '2') {
		kinect->setLed(LED_RED);
	} else if (key == '3') {
		kinect->setLed(LED_YELLOW);
	} else if (key == '4') {
		kinect->setLed(LED_BLINK_GREEN);
	} else if (key == '5') {
		kinect->setLed(LED_BLINK_RED_YELLOW);
	} else if (key == '0') {
		kinect->setLed(LED_OFF);
	} else if (key == 'f') {
		if (requested_format == FREENECT_VIDEO_IR_8BIT)
			requested_format = FREENECT_VIDEO_RGB;
		else if (requested_format == FREENECT_VIDEO_RGB)
			requested_format = FREENECT_VIDEO_YUV_RGB;
		else
			requested_format = FREENECT_VIDEO_IR_8BIT;
		kinect->setVideoFormat(requested_format);
	} else if (key == 'w') {
		freenect_angle++;
		if (freenect_angle > 30) {
			freenect_angle = 30;
		}
	} else if (key == 's') {
		freenect_angle = 0;
	} else if (key == 'x') {
		freenect_angle--;
		if (freenect_angle < -30) {
			freenect_angle = -30;
		}
	} else if (key == 'e') {
		freenect_angle = 10;
	} else if (key == 'c') {
		freenect_angle = -10;
	} else if (key == 'd') {
		show_depth = !show_depth;
	} else if (key == 'a') {
		show_rgb = !show_rgb;
	}
	kinect->setTiltDegrees(freenect_angle);
}

void loop()
{
	userMenu();
	while (!stop_program) {
		int key = cv::waitKey(5);
		if (key != -1) {
			keyPressed(device, key);
		}
		device->updateState();
		if(show_rgb) {
			cv::Mat image(height, width, CV_8UC3, cv::Scalar(0,0,0));
			convertRGB(image);
			imshow("RGBD", image);
		}
		if(show_depth) {
			cv::Mat depth(height, width, CV_8UC3, cv::Scalar(0,0,0));
			convertDepth(depth);
			imshow("DEPTH", depth);
		}
	}
}

int main()
{
	// initialize
	freenect_angle = 0;
	buffer_id = 0;
	n_buffers = 10;
	width = 640;
	height = 480;
	depth_channels = 4;
	rgb_channels = 3;
	show_depth = true;
	show_rgb = true;
	stop_program = false;
	requested_format = FREENECT_VIDEO_RGB;
	// depth_buffer.resize(n_buffers);
	// rgb_buffer.resize(n_buffers);
	// for(int i = 0; i < n_buffers; i++)
	// {
	// 	rgb_buffer[i].create(height, width, CV_8UC3, cv::Scalar(0,0,0));
	// 	depth_buffer[i].create(height, width, CV_8UC3, cv::Scalar(0,0,0));
	// }

	printf("Opening Kinect...\n");
	device = &freenect.createDevice<MyFreenectDevice>(0);
	device->startVideo();
	device->startDepth();
	loop();
	device->stopVideo();
	device->stopDepth();
}