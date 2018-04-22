#include "cv2_wrapper.h"

int main()
{
	CV2Wrapper wr;
	wr.initializeDevice();
	wr.userMenu();
	while (!wr.checkState()) {
		int key = cv::waitKey(5);
		if (key != -1) {
			wr.checkKey(key);
		}
		wr.updateDevice();
		if (wr.showImage())
		{
			cv::Mat image(wr.getImageMat());
			imshow("RGBD", image);
		}
		if (wr.showDepth())
		{
			cv::Mat depth(wr.getDepthMat());
			imshow("DEPTH", depth);
		}
	}
}