// Adapted from runMTF.cc by Abhineet Singh 
// CMPUT 428 final project by Laura Petrich
// 
//! main header that provides functions for creating trackers
#include "mtf/mtf.h"
//! tools for reading in images from various sources like image sequences, 
//! videos and cameras, pre processing these images and getting
//! objects to track either from ground truth or interactively from the user
#include "mtf/pipeline.h"
//! parameters for different modules
#include "mtf/Config/parameters.h"
//! general utilities for image drawing, etc.
#include "mtf/Utilities/miscUtils.h"
//! MTF specific exceptions
#include "mtf/Utilities/excpUtils.h"
//! Kinect specific headers
#include "mtf/Utilities/kinectUtils.h"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/path.hpp"

#include <vector>
#include <memory>

#define MAX_FPS 1e6

using namespace std;
using namespace mtf::params;
namespace fs = boost::filesystem;

typedef unique_ptr<mtf::TrackerBase> Tracker_;
mtf::utils::ObjUtils obj_utils(obj_cols, img_resize_factor);
Input_ input;
FILE *multi_fid = nullptr;
vector<Tracker_> trackers(n_trackers);
vector<PreProc_> pre_procs(n_trackers);
string cv_win_name = "MTF :: IMAGE";
string depth_win_name = "MTF :: DEPTH";
string roi_win_name = "MTF :: ROI";
bool depth_enabled = false;
double fps = 0, fps_win = 0;
double tracking_time, tracking_time_with_input;
double avg_fps = 0, avg_fps_win = 0;
int fps_count = 0;
cv::Point fps_origin(10, 20);
double fps_font_size = 0.50;
cv::Scalar fps_col_rgb;
int valid_frame_count = 0;
bool invalid_tracker_state = false, tracker_failed = false;
bool resized_images = img_resize_factor != 1;

// Kinect variables
bool show_depth = true;
bool show_rgb = true;
bool stop_program = false;
Freenect::Freenect freenect;
MyFreenectDevice* device;
freenect_video_format requested_format(FREENECT_VIDEO_RGB);
double freenect_angle(0);

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

bool trackingLoop() {

	// *************************************** start tracking ! ************************************* //
	while(true) {
		cv::Mat tracker_corners = trackers[0]->getRegion().clone();
		if(resized_images){ tracker_corners /= img_resize_factor; }
		//! non finite entries in the tracker region indicate invalid tracker state
		if(invalid_state_check){
			if(mtf::utils::hasNaN<double>(tracker_corners) || mtf::utils::hasInf<double>(tracker_corners)){
				printf("Tracker corners in frame %d have non finite values\n", input->getFrameID() + 1);
				invalid_tracker_state = true;
			}
		}
		if(print_corners){
			cout << tracker_corners << "\n";
		}
		if(invalid_tracker_state){
			printf("Unrecoverable tracking loss detected. Exiting...\n");
			break;
		}
		// *************************** display tracking output *************************** //

		if(mtf_visualize) {
			// ************************** draw tracker positions on OpenCV window ******************************* //
			for(unsigned int tracker_id = 0; tracker_id < n_trackers; ++tracker_id) {
				cv::Mat drawn_corners = tracker_id == 0 ? tracker_corners :
					resized_images ? trackers[tracker_id]->getRegion() / img_resize_factor :
					trackers[tracker_id]->getRegion();
				mtf::utils::drawRegion(input->getFrame(mtf::utils::MUTABLE), drawn_corners,
					obj_utils.getCol(tracker_id), line_thickness, tracker_labels[tracker_id].c_str(),
					fps_font_size, show_corner_ids, 1 - show_corner_ids);
			}
			// std::string fps_text = cv::format("frame: %d c: %9.3f a: %9.3f cw: %9.3f aw: %9.3f fps",
				// input->getFrameID() + 1, fps, avg_fps, fps_win, avg_fps_win);
			// putText(input->getFrame(mtf::utils::MUTABLE), fps_text, fps_origin, cv::FONT_HERSHEY_SIMPLEX, fps_font_size, fps_col_rgb);
			imshow(cv_win_name, input->getFrame());
			if(depth_enabled){
				for(unsigned int tracker_id = 0; tracker_id < n_trackers; ++tracker_id) {
					cv::Mat drawn_corners = tracker_id == 0 ? tracker_corners :
						resized_images ? trackers[tracker_id]->getRegion() / img_resize_factor :
						trackers[tracker_id]->getRegion();
					mtf::utils::drawRegion(input->getDepthFrame(mtf::utils::MUTABLE), drawn_corners,
						obj_utils.getCol(tracker_id), line_thickness, tracker_labels[tracker_id].c_str(),
						fps_font_size, show_corner_ids, 1 - show_corner_ids);
				}
				imshow(depth_win_name, input->getDepthFrame());
				// imshow(roi_win_name, input->getFrame());
			}
			int pressed_key = cv::waitKey(1 - pause_after_frame);
			if(pressed_key % 256 == 27){
				break;
			}
			if(pressed_key % 256 == 32){
				pause_after_frame = 1 - pause_after_frame;
			}
		}
		if(!mtf_visualize && (input->getFrameID() + 1) % 50 == 0){
			printf("frame_id: %5d avg_fps: %15.9f avg_fps_win: %15.9f\n",
				input->getFrameID() + 1, avg_fps, avg_fps_win);
		}

		if(input->getNFrames() > 0 && input->getFrameID() >= input->getNFrames() - 1){
			printf("==========End of input stream reached==========\n");
			break;
		}
		// ******************************* update pipeline and trackers ******************************* //
		mtf_clock_get(start_time_with_input);
		//! update pipeline
		for(int skip_id = 0; skip_id < frame_gap; ++skip_id) {
			if(!input->update()){
				printf("Frame %d could not be read from the input pipeline\n", input->getFrameID() + 1);
				break;
			}
		}
		//! update trackers       
		for(unsigned int tracker_id = 0; tracker_id < n_trackers; ++tracker_id) {
			//! update pre processor
			pre_procs[tracker_id]->update(input->getFrame(), input->getFrameID());
			try{
				mtf_clock_get(start_time);
				trackers[tracker_id]->update();
				mtf_clock_get(end_time);
				mtf_clock_measure(start_time, end_time, tracking_time);
				mtf_clock_measure(start_time_with_input, end_time, tracking_time_with_input);
			} catch(const mtf::utils::InvalidTrackerState &err){
				//! exception thrown by MTF modsules when the tracker ends up in an invalid state 
				//! due to NaNs or Infs in the result of some numerical computation
				printf("Invalid tracker state encountered in frame %d: %s\n", input->getFrameID() + 1, err.what());
				invalid_tracker_state = true;
				//! allow the tracker to be reinitialized if this option is enabled otherwise exit
				continue;
			} catch(const mtf::utils::Exception &err){
				printf("Exception of type %s encountered while updating the tracker: %s\n", 
					err.type(), err.what());
				return EXIT_FAILURE;
			}
			fps = 1.0 / tracking_time;
			fps_win = 1.0 / tracking_time_with_input;
			if(reset_template && (input->getFrameID() - init_frame_id) % reset_template == 0){
				trackers[tracker_id]->initialize(trackers[tracker_id]->getRegion());
			}
		}
		if(!std::isinf(fps) && fps < MAX_FPS){
			++fps_count;
			avg_fps += (fps - avg_fps) / fps_count;
			avg_fps_win += (fps_win - avg_fps_win) / fps_count;
		}
	}
	return true;
}

bool setupOutput() {
	try{
		if(mtf_visualize) {
			cv::namedWindow(cv_win_name, cv::WINDOW_AUTOSIZE);
			if(img_source == SRC_KINECT_CAM) {
				depth_enabled = true;
				printf("Creating windows for depth information.");
				cv::namedWindow(depth_win_name, cv::WINDOW_AUTOSIZE);
				// cv::namedWindow(roi_win_name, cv::WINDOW_AUTOSIZE);
			}
		}
	} catch(...){
		printf("OpenCV window could not be created to show tracking output. Turning this option off...");
		mtf_visualize = 0;
	}
	if(mtf_visualize){
		for(unsigned int tracker_id = 0; tracker_id < n_trackers; ++tracker_id) {
			if(static_cast<int>(tracker_labels.size()) < tracker_id + 1){
				tracker_labels.push_back(trackers[tracker_id]->name);
			}
		}
	}
	try{
		fps_col_rgb = obj_utils.getCol(fps_col);
	} catch(std::out_of_range e){
		printf("Invalid FPS provided: %s", e.what());
		return EXIT_FAILURE;
	}
	if(reset_template){ printf("Template resetting is enabled\n"); }
	valid_frame_count = 0;
	//! set to true if a call to the tracker update function results in a runtime error gettimng thrown
	invalid_tracker_state = false; 
	tracker_failed = false;
	if(frame_gap < 1){
		frame_gap = 1;
	} else if(frame_gap > 1){
		printf("Using a gap of %d between consecutive tracked frames\n", frame_gap);
	}
	resized_images = img_resize_factor != 1;
	return true;
}
bool initializeTrackersAndPreProcs() {
	// ***************************** initialize trackers and pre processors ***************************** //
	if(n_trackers > 1){
		printf("Multi tracker setup enabled\n");
		read_obj_from_gt = write_tracking_data = reinit_on_failure = show_ground_truth = 0;
	}
	if(res_from_size){
		printf("Getting sampling resolution from object size...\n");
	}
	for(unsigned int tracker_id = 0; tracker_id < n_trackers; ++tracker_id) {
		if(n_trackers > 1){ multi_fid = readTrackerParams(multi_fid); }
		try {
			if(res_from_size){
				resx = static_cast<unsigned int>(obj_utils.getObj(tracker_id).size_x / res_from_size);
				resy = static_cast<unsigned int>(obj_utils.getObj(tracker_id).size_y / res_from_size);
			}
			trackers[tracker_id].reset(mtf::getTracker(mtf_sm, mtf_am, mtf_ssm, mtf_ilm));
			if(!trackers[tracker_id]){
				printf("Tracker could not be created successfully\n");
				return false;
			}
		} catch(const mtf::utils::Exception &err){
			printf("Exception of type %s encountered while creating the tracker: %s\n", err.type(), err.what());
			return false;
		}
		try{
			pre_procs[tracker_id] = mtf::getPreProc(pre_procs, trackers[tracker_id]->inputType(), pre_proc_type);
			pre_procs[tracker_id]->initialize(input->getFrame(), input->getFrameID());
		} catch(const mtf::utils::Exception &err){
			printf("Exception of type %s encountered while initializing the pre processor: %s\n", err.type(), err.what());
			return false;
		}
		try{
			for(PreProc_ curr_obj = pre_procs[tracker_id]; curr_obj; curr_obj = curr_obj->next){
				trackers[tracker_id]->setImage(curr_obj->getFrame());
			}
			printf("Initializing tracker %d with object of size %f x %f\n", tracker_id, obj_utils.getObj(tracker_id).size_x, obj_utils.getObj(tracker_id).size_y);
			trackers[tracker_id]->initialize(obj_utils.getObj(tracker_id).corners);
		} catch(const mtf::utils::Exception &err){
			printf("Exception of type %s encountered while initializing the tracker: %s\n", err.type(), err.what());
			return false;
		}
	}
	if(input->getNFrames() == 0){
		start_frame_id = init_frame_id;
	} 
	if(start_frame_id > init_frame_id){
		if(input->getNFrames() > 0 && start_frame_id >= input->getNFrames()){
			printf("start_frame_id: %d is larger than the maximum frame ID in the sequence: %d\n", start_frame_id, input->getNFrames() - 1);
			return false;
		}
		printf("Skipping to frame %d before starting tracking...\n", start_frame_id + 1);
		while(input->getFrameID() < start_frame_id){
			if(!input->update()){
				printf("Frame %d could not be read from the input pipeline\n", input->getFrameID() + 1);
				return false;
			}
		}
		try{
			for(unsigned int tracker_id = 0; tracker_id < n_trackers; ++tracker_id) {
				pre_procs[tracker_id]->update(input->getFrame(), input->getFrameID());
				trackers[tracker_id]->setRegion(obj_utils.getGT(input->getFrameID()));
			}
		} catch(const mtf::utils::Exception &err){
			printf("Exception of type %s encountered while resetting the tracker location: %s\n", err.type(), err.what());
			return false;
		}
	}
	return true;
}

bool initializeObjects() {
	// ******************* get objects to be tracked  ******************* //
	try {
		if(!mtf::getObjectsToTrack(obj_utils, input.get())){
			printf("Object(s) to be tracked could not be obtained.\n");
			return false;
		}
	} catch(const mtf::utils::Exception &err){
		printf("Exception of type %s encountered while obtaining the objects to track: %s\n", err.type(), err.what());
		return false;
	}
	return true;
}

bool initializePipeline() {
	// ********************************** initialize input pipeline ********************************** //
	try{
		input.reset(mtf::getInput(pipeline));
		if(!input->initialize()){
			printf("Pipeline could not be initialized successfully. Exiting...\n");
			return false;
		} else {
			printf("Pipeline initialized.\n");
		}
	} catch(const mtf::utils::Exception &err){
		printf("Exception of type %s encountered while initializing the input pipeline: %s\n",
			err.type(), err.what());
		return false;
	}
	if(init_frame_id > 0){
		if(input->getNFrames() > 0 && init_frame_id >= input->getNFrames()){
			printf("init_frame_id: %d is larger than the maximum frame ID in the sequence: %d\n",
				init_frame_id, input->getNFrames() - 1);
			return false;
		}
		printf("Skipping to frame %d before initializing trackers...\n", init_frame_id + 1);
		for(int frame_id = 0; frame_id < init_frame_id; ++frame_id){
			if(!input->update()){
				printf("Frame %d could not be read from the input pipeline\n", input->getFrameID() + 1);
				return false;
			}
		}
	}
	if(start_frame_id < init_frame_id){
		start_frame_id = init_frame_id;
	}
	return true;
}

void showParams() {
	printf("*******************************\n");
	printf("Using parameters:\n");
	printf("n_trackers: %u\n", n_trackers);
	printf("actor_id: %d\n", actor_id);
	printf("source_id: %d\n", seq_id);
	printf("source_name: %s\n", seq_name.c_str());
	printf("actor: %s\n", actor.c_str());
	printf("pipeline: %c\n", pipeline);
	printf("img_source: %c\n", img_source);
	printf("mtf_visualize: %d\n", mtf_visualize);
	printf("read_obj_from_gt: %d\n", read_obj_from_gt);
	printf("write_tracking_data: %d\n", write_tracking_data);
	printf("mtf_sm: %s\n", mtf_sm);
	printf("mtf_am: %s\n", mtf_am);
	printf("mtf_ssm: %s\n", mtf_ssm);
	printf("*******************************\n");
}

int main(int argc, char * argv[]) {
	printf("\nStarting MTFD...\n");

	if(!readParams(argc, argv)){ return EXIT_FAILURE; }
	showParams();

	if (!initializePipeline()) { return EXIT_FAILURE; }
	if (!initializeObjects()) { return EXIT_FAILURE; }
	if (!initializeTrackersAndPreProcs()) { return EXIT_FAILURE; }
	if (!setupOutput()) { return EXIT_FAILURE; }
	if (!trackingLoop()) { return EXIT_FAILURE; }

	cv::destroyAllWindows();			
	printf("Average FPS: %15.10f\n", avg_fps);
	printf("Average FPS with Input: %15.10f\n", avg_fps_win);
	pre_procs.clear();
	trackers.clear();
	return EXIT_SUCCESS;
}