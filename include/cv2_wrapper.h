#ifndef CV2_WRAPPER_H
#define CV2_WRAPPER_H

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include "libfreenect.hpp"
#include <kinect_device.h>
#include <vector>


class CV2Wrapper {
public:
	CV2Wrapper()
	{
		freenect_angle = 0;
		buffer_id = 0;
		idx = 0;
		n_buffers = 10;
		width = 640;
		height = 480;
		depth_channels = 4;
		image_channels = 3;
		show_depth = true;
		show_image = true;
		stop_program = false;
		requested_format = FREENECT_VIDEO_RGB;
	}
	virtual ~CV2Wrapper()
	{
		device_->stopVideo();
		device_->stopDepth();
	}

	void initializeDevice() 
	{
		depth_buffer.resize(n_buffers);
		image_buffer.resize(n_buffers);
		for(int i = 0; i < n_buffers; i++)
		{
			image_buffer[i].create(height, width, CV_8UC3);
			depth_buffer[i].create(height, width, CV_8UC3);
		}
		device_ = &freenect_.createDevice<MyFreenectDevice>(0);
		device_->startVideo();
		device_->startDepth();
		device_->updateState();
	}

	void updateDevice() 
	{
		device_->updateState();
		convertImageMat(image_buffer[buffer_id]);
		convertDepthMat(depth_buffer[buffer_id]);
		idx = buffer_id;
		buffer_id = (buffer_id + 1) % n_buffers;
	}

	void convertImageMat(cv::Mat &image) 
	{
		static std::vector<uint8_t> rgb_raw(width*height*image_channels);
		try {
			device_->getRGB(rgb_raw);
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

	void convertDepthMat(cv::Mat &depth_image) 
	{
		static std::vector<uint8_t> depth_raw(width*height*depth_channels);
		try {
			device_->getDepth(depth_raw);
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

	cv::Mat getImageMat() 
	{
		return image_buffer[idx];
	}

	cv::Mat getDepthMat() 
	{
		return depth_buffer[idx];
	}

	bool showImage()
	{
		return show_image;
	}

	bool showDepth()
	{
		return show_depth;
	}

	bool checkState()
	{
		return stop_program;
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

	void checkKey(int key)
	{
		if (key == 27) {
			stop_program = true;
		} else if (key == '1') {
			device_->setLed(LED_GREEN);
		} else if (key == '2') {
			device_->setLed(LED_RED);
		} else if (key == '3') {
			device_->setLed(LED_YELLOW);
		} else if (key == '4') {
			device_->setLed(LED_BLINK_GREEN);
		} else if (key == '5') {
			device_->setLed(LED_BLINK_RED_YELLOW);
		} else if (key == '0') {
			device_->setLed(LED_OFF);
		} else if (key == 'f') {
			if (requested_format == FREENECT_VIDEO_IR_8BIT)
				requested_format = FREENECT_VIDEO_RGB;
			else if (requested_format == FREENECT_VIDEO_RGB)
				requested_format = FREENECT_VIDEO_YUV_RGB;
			else
				requested_format = FREENECT_VIDEO_IR_8BIT;
			device_->setVideoFormat(requested_format);
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
			show_image = !show_image;
		}
		device_->setTiltDegrees(freenect_angle);
	}

private:
	Freenect::Freenect freenect_;
	MyFreenectDevice* device_;
	freenect_video_format requested_format;
	std::vector<cv::Mat> depth_buffer;
	std::vector<cv::Mat> image_buffer;
	double freenect_angle;
	int n_buffers;
	int buffer_id;
	int idx;
	int width;
	int height;
	int image_channels;
	int depth_channels;
	bool show_depth;
	bool show_image;
	bool stop_program;
};

#endif /* CV2_WRAPPER_H */