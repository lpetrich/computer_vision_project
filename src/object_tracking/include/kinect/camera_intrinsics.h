/* lpetrich 22/04/18  /**/

#ifndef CAMERA_INTRINSICS_H_
#define CAMERA_INTRINSICS_H_

#include <Eigen/Core>
#include <stdio.h>
#include <string.h>
#include "macros.h"

class CameraIntrinsics
{
public:
	CameraIntrinsics() {}
	virtual ~CameraIntrinsics() {}

	void initializeIntrinsics(int device_id)
	{
			camera_matrix.resize(3,3);
			rectification_matrix.resize(3,3);
			projection_matrix.resize(3,4);
			distortion_coefficients.resize(1,5);
			width = 640;
			height = 480;
		if (device_id == 0) //A00363A00089102A -- home
		{
			distortion_model = "plumb_bob";
			camera_matrix << 521.7290146470901, 0, 309.7566864103458, 0, 523.3786277281491, 256.1347096994411, 0, 0, 1;
			rectification_matrix << 1, 0, 0, 0, 1, 0, 0, 0, 1;
			projection_matrix << 533.4115600585938, 0, 308.1548251421373, 0, 0, 536.0866088867188, 256.83125984039, 0, 0, 0, 1, 0;
			distortion_coefficients << 0.1937523284103504, -0.3514710479081101, 0.00352149223038772, -0.001091591833277532, 0;
		} 
		else if (device_id == 2) //A00366A17003143A -- lab
		{
			distortion_model = "plumb_bob";
			camera_matrix << 538.7668543670886, 0, 331.1718099583036, 0, 539.5321820327622, 274.4248396400299, 0, 0, 1;
			rectification_matrix << 1, 0, 0, 0, 1, 0, 0, 0, 1;
			projection_matrix << 556.3593139648438, 0, 334.3652883131508, 0, 0, 558.5757446289062, 276.5663113791379, 0, 0, 0, 1, 0;
			distortion_coefficients << 0.2656207385181749, -0.4852135138255346, 0.006902239396962818, 0.005696615035588637, 0;		
		}		
	}

	Eigen::MatrixXf getCameraMatrix()
	{
		return camera_matrix;
	}

	Eigen::MatrixXf getRectificationMatrix()
	{
		return rectification_matrix;
	}

	Eigen::MatrixXf getProjectionMatrix()
	{
		return projection_matrix;
	}

	Eigen::MatrixXf getDistortionCoefficients()
	{
		return distortion_coefficients;
	}

	float getfx()
	{
		return projection_matrix(0,0);
	}

	float getfy()
	{
		return projection_matrix(1,1);
	}

	float getox()
	{
		return projection_matrix(0,2);
	}

	float getoy()
	{
		return projection_matrix(1,2);
	}

	int getWidth()
	{
		return width;
	}

	int getHeight()
	{
		return height;
	}

private:
	int width;
	int height;
	std::string distortion_model;
	Eigen::MatrixXf camera_matrix;
	Eigen::MatrixXf rectification_matrix;
	Eigen::MatrixXf projection_matrix;
	Eigen::MatrixXf distortion_coefficients;
};

#endif /* CAMERA_INTRINSICS_H_ */


