/* lpetrich 22/04/18  /**/

#include "cv2_wrapper.h"
#include "camera_intrinsics.h"
#include "macros.h"
#include <dvo/dense_tracking.h>
#include <dvo/dense_tracking_impl.h>
#include <dvo/core/rgbd_image.h>
#include <dvo/core/surface_pyramid.h>
#include <dvo/util/id_generator.h>
#include <boost/thread.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

/* global variables /**/
CV2Wrapper wrapper_;
boost::mutex tracker_mutex_;
uint32_t width;
uint32_t height;
dvo::core::RgbdImagePyramidPtr current, reference;
dvo::DenseTracker::Config cfg;
dvo::core::RgbdCameraPyramidPtr camera;
Eigen::Affine3d accumulated_transform, latest_absolute_transform_;
dvo::core::IntrinsicMatrix intrinsics;
boost::shared_ptr<dvo::DenseTracker> tracker;
size_t frames_since_last_success;
cv::Mat current_image;
cv::Mat current_depth;
std::string window_image, window_depth;
std::vector<double> U, V;
std::vector<Eigen::Vector3d> uv;
cv::Point2d centroid;
std::vector<Eigen::Vector4d> point3D;
double fx, fy, cx, cy;
cv::Mat intensity, depth;

void shutdown()
{
	wrapper_.shutdownDevice();
	cv::destroyWindow(window_image);
	cv::destroyWindow(window_depth);
}

void handleImages(cv::Mat rgb_in, cv::Mat depth_in)
{
	boost::mutex::scoped_lock lock(tracker_mutex_);
	// cv::Mat intensity, depth;
	if(rgb_in.channels() == 3)
	{
		cv::Mat tmp;
		cv::cvtColor(rgb_in, tmp, CV_BGR2GRAY, 1);
		tmp.convertTo(intensity, CV_32F);
	}
	else
	{
		rgb_in.convertTo(intensity, CV_32F);
	}
	dvo::core::SurfacePyramid::convertRawDepthImageSse(depth_in, depth, 0.001);
	reference.swap(current);
	current = camera->create(intensity, depth);
	static Eigen::Affine3d first;
	if(!reference)
	{
		accumulated_transform = latest_absolute_transform_;
		first = accumulated_transform;
		return;
	}
	Eigen::Affine3d transform;
	bool success = tracker->match(*reference, *current, transform);
	if(success)
	{
		frames_since_last_success = 0;
		accumulated_transform = accumulated_transform * transform;
	}
	else
	{
		frames_since_last_success++;
		reference.swap(current);
		std::cout << "matching failed";
	}
	std::cout << "Current transform: \n" << transform.matrix() << "\n";
}

void drawPatch(cv::Mat& frame) 
{
    line(frame, cv::Point(uv[0][0], uv[0][1]), cv::Point(uv[1][0], uv[1][1]), (255,255,255));
    line(frame, cv::Point(uv[1][0], uv[1][1]), cv::Point(uv[2][0], uv[2][1]), (255,255,255));
    line(frame, cv::Point(uv[2][0], uv[2][1]), cv::Point(uv[3][0], uv[3][1]), (255,255,255));
    line(frame, cv::Point(uv[3][0], uv[3][1]), cv::Point(uv[0][0], uv[0][1]), (255,255,255));
}

void calculateUV()
{ // (7) calculate warped pixel coordinates
	// for (int i = 0; i < uv.size(); i++)
	// {
	// 	previous_uv[i] = uv[i];
	// 	uv[i][0] = ((fx * ps[i][0]) / ps[i][2]) + cx;
	// 	uv[i][1] = ((fy * ps[i][1]) / ps[i][2]) + cy;
	// 	std::cout << "fx: " << fx << "\n";
	// 	std::cout << "fy: " << fy << "\n";
	// 	std::cout << "cx: " << cx << "\n";
	// 	std::cout << "cy: " << cy << "\n";
	// 	std::cout << "ps: " << ps[i] << "\n";
	// 	std::cout << "new uv point (" << uv[i][0] << ", " << uv[i][1] << ")\n";
	// }
}

void calculateNewpoint3Ds()
{ // (4) 	T(g, p) = Rp + t 
	// for (int i = 0; i < p.size(); i++)
	// {
	// 	Eigen::Vector4d tmp, P;
	// 	P = p[i];
	// 	p[i] = ps[i];

	// 	// tmp = R*P;
	// 	// std::cout << "\nP: \n" << P << "\n";
	// 	// std::cout << "\nR*P: \n" << tmp << "\n";
	// 	ps[i][0] = P[0] + T[0];
	// 	ps[i][1] = P[1] + T[1];
	// 	ps[i][2] = P[2] + T[2];
	// 	std::cout << "\nNew 3D Point: \n" << ps[i] << "\n";
	// }
	// calculateUV();
}

void initializepoint3Ds()
{
	double px, py, u, v;
	int Z;
	for (int i = 0; i < uv.size(); i++)
	{	
		u = uv[i][0];
		v = uv[i][1];
		Z = depth.at<double>(u, v);
		if (!std::isfinite(Z))
		{	
			std::cout << "z is nan, setting to 1...\n";
			Z = 1.0;
		}
		point3D[i][0] = Z * ((u - cx) / fx);
		point3D[i][1] = Z * ((v - cy) / fy);
		point3D[i][2] = Z;
		point3D[i][3] = 1;
		// std::cout << "fx: " << fx << "\n";
		// std::cout << "fy: " << fy << "\n";
		// std::cout << "cx: " << cx << "\n";
		// std::cout << "cy: " << cy << "\n";
		std::cout << "point3D: " << point3D[i] << "\n";
		std::cout << "u: " << u << "\n";
		std::cout << "v: " << v << "\n";
		std::cout << "Z: " << Z << "\n";
	}
}

void initializeCorners()
{
	Eigen::Vector3d v1, v2, v3, v4, l1, l2, intersection;
	double minu, maxu, minv, maxv;
	if (U[0] < U[1]) 
	{
		minu = U[0];
		maxu = U[1];
	}
	else
	{
		minu = U[1];
		maxu = U[0];
	}
	if (V[0] < V[1])
	{
		minv = V[0];
		maxv = V[1];
	}
	else
	{
		minv = V[1];
		maxv = V[0];
	}
  	v1 << minu, minv, 1;
  	v2 << maxu, minv, 1;
  	v3 << maxu, maxv, 1;
  	v4 << minu, maxv, 1;
  	uv.push_back(v1);
	uv.push_back(v2);
	uv.push_back(v3);
	uv.push_back(v4);
  	l1 = v1.cross(v3);
  	l2 = v2.cross(v4);
  	intersection = l1.cross(l2);
  	centroid.x = intersection(0) / intersection(2);
  	centroid.y = intersection(1) / intersection(2);
  	// initializepoint3Ds();
}
void mouseHandler(int mouse_event, int x, int y, int flags, void* param) 
{
    if (mouse_event == CV_EVENT_RBUTTONUP) {
        // resetVariables();
        return;
    }
    if (mouse_event == CV_EVENT_LBUTTONUP && U.size() < 2) 
    {
        std::cout << "Click at x: " << x << ", " << y << "\n";
        U.push_back(x);
        V.push_back(y);
        if (U.size() == 2) 
        {	
        	initializeCorners();
        }
        return;
    }
}

void mainLoop()
{
	while (!wrapper_.checkState()) {
		int key = cv::waitKey(5);
		if (key != -1) {
			wrapper_.checkKey(key);
		}
		wrapper_.updateDevice();
		cv::Mat image(wrapper_.getImageMat());
		cv::Mat depth(wrapper_.getDepthMat());
		current_image = image.clone();
		current_depth = depth.clone();
		if (wrapper_.showImage())
		{
			if (uv.size() == 4)
			{
				drawPatch(image);
			}
			imshow(window_image, image);
		}
		if (wrapper_.showDepth())
		{
			if (uv.size() == 4)
			{
				drawPatch(depth);
			}
			imshow(window_depth, depth);
		}
		handleImages(image, depth);
	}
}

void initialize()
{
	wrapper_.initializeDevice();
	wrapper_.userMenu();
	window_image = "IMAGE";
	window_depth = "DEPTH";
	cv::namedWindow(window_image);
	cv::namedWindow(window_depth);
	cv::setMouseCallback(window_image, mouseHandler);
	cv::startWindowThread();
	fx = 521.72;
	fy = 523.37;
	cx = 309.75;
	cy = 256.13;
	intrinsics = dvo::core::IntrinsicMatrix::create(fx, fy, cx, cy);
	cfg = dvo::DenseTracker::getDefaultConfig();
	cfg.UseWeighting = false;
	cfg.UseInitialEstimate = true;
	cfg.FirstLevel = 3;
	cfg.LastLevel = 1;
	cfg.MaxIterationsPerLevel = 50;
	width = 640;
	height = 480;
	camera.reset(new dvo::core::RgbdCameraPyramid(width, height, intrinsics));
	camera->build(cfg.getNumLevels());
	tracker.reset(new dvo::DenseTracker(cfg));
	static dvo::core::RgbdImagePyramid* const __null__ = 0;
	reference.reset(__null__);
	current.reset(__null__);
}

int main()
{
	initialize();
	mainLoop();
	shutdown();
}