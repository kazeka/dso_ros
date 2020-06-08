/**
* This file is part of DSO.
*
* Copyright 2016 Technical University of Munich and Intel.
* Developed by Jakob Engel <engelj at in dot tum dot de>,
* for more information see <http://vision.in.tum.de/dso>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DSO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DSO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DSO. If not, see <http://www.gnu.org/licenses/>.
*/





#include <locale.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "util/settings.h"
#include "FullSystem/FullSystem.h"
#include "util/Undistort.h"
#include "IOWrapper/Pangolin/PangolinDSOViewer.h"
#include "OutputWrapper.h"


#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include "cv_bridge/cv_bridge.h"


std::string calib = "";
std::string vignetteFile = "";
std::string gammaFile = "";
std::string saveFile = "";
bool useSampleOutput = false;

int rosQueueSize = 1;

using namespace dso;

void parseArgument(char* arg)
{
	int option;
	char buf[1000];
	if(1==sscanf(arg,"savefile=%s",buf))
	{
		saveFile = buf;
		printf("saving to %s on finish!\n", saveFile.c_str());
		return;
	}

	if(1==sscanf(arg,"sampleoutput=%d",&option))
	{
		if(option==1)
		{
			useSampleOutput = true;
			printf("USING SAMPLE OUTPUT WRAPPER!\n");
		}
		return;
	}

	if(1==sscanf(arg,"quiet=%d",&option))
	{
		if(option==1)
		{
			setting_debugout_runquiet = true;
			printf("QUIET MODE, I'll shut up!\n");
		}
		return;
	}


	if(1==sscanf(arg,"nolog=%d",&option))
	{
		if(option==1)
		{
			setting_logStuff = false;
			printf("DISABLE LOGGING!\n");
		}
		return;
	}

	if(1==sscanf(arg,"nogui=%d",&option))
	{
		if(option==1)
		{
			disableAllDisplay = true;
			printf("NO GUI!\n");
		}
		return;
	}
	if(1==sscanf(arg,"nomt=%d",&option))
	{
		if(option==1)
		{
			multiThreading = false;
			printf("NO MultiThreading!\n");
		}
		return;
	}
	if(1==sscanf(arg,"calib=%s",buf))
	{
		calib = buf;
		printf("loading calibration from %s!\n", calib.c_str());
		return;
	}
	if(1==sscanf(arg,"vignette=%s",buf))
	{
		vignetteFile = buf;
		printf("loading vignette from %s!\n", vignetteFile.c_str());
		return;
	}

	if(1==sscanf(arg,"gamma=%s",buf))
	{
		gammaFile = buf;
		printf("loading gammaCalib from %s!\n", gammaFile.c_str());
		return;
	}

	printf("could not parse argument \"%s\"!!\n", arg);
}




FullSystem* fullSystem = 0;
Undistort* undistorter = 0;
int frameID = 0;

void vidCb(const sensor_msgs::ImageConstPtr img)
{
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
	assert(cv_ptr->image.type() == CV_8U);
	assert(cv_ptr->image.channels() == 1);

	if(setting_fullResetRequested)
	{
		std::vector<IOWrap::Output3DWrapper*> wraps = fullSystem->outputWrapper;
		delete fullSystem;
		for(IOWrap::Output3DWrapper* ow : wraps) ow->reset();
		fullSystem = new FullSystem();
		fullSystem->linearizeOperation=false;
		fullSystem->outputWrapper = wraps;
	    if(undistorter->photometricUndist != 0)
	    	fullSystem->setGammaFunction(undistorter->photometricUndist->getG());
		setting_fullResetRequested=false;
	}

	MinimalImageB minImg((int)cv_ptr->image.cols, (int)cv_ptr->image.rows,(unsigned char*)cv_ptr->image.data);
	ImageAndExposure* undistImg = undistorter->undistort<unsigned char>(&minImg, 1, img->header.stamp.toSec(), 1.0f);
	fullSystem->addActiveFrame(undistImg, frameID);
	frameID++;
	delete undistImg;

}


int main( int argc, char** argv )
{
	ros::init(argc, argv, "dso_live");

	for(int i=1; i<argc;i++) parseArgument(argv[i]);

	ros::NodeHandle param_nh("~");
	param_nh.param<float>("point_candidates", setting_desiredImmatureDensity, 1500.);
	param_nh.param<float>("active_points", setting_desiredPointDensity, 2000.);
	param_nh.param<int>("min_frames", setting_minFrames, 5);
	param_nh.param<int>("max_frames", setting_maxFrames, 9);
	param_nh.param<int>("min_iterations", setting_minOptIterations, 1);
	param_nh.param<int>("max_iterations", setting_maxOptIterations, 5);
	param_nh.param<float>("kf_frequency", setting_kfGlobalWeight, 1.0);    // general weight on threshold, the larger the more KF's are taken (e.g., 2 = double the amount of KF's).
																		   // original is 1.0. 0.3 is a balance between speed and accuracy. if tracking lost, set this para higher
	param_nh.param<bool>("kf_realtime", setting_realTimeMaxKF, false);     // if true, takes as many KF's as possible (will break the system if the camera stays stationary)
	param_nh.param<float>("max_shift_t", setting_maxShiftWeightT, 0.04f * (720+540));
	param_nh.param<float>("max_shift_r", setting_maxShiftWeightR, 0.04f * (720+540));
	param_nh.param<float>("max_shift_rt", setting_maxShiftWeightRT, 0.02f * (720+540));

	param_nh.param<int>("queue_size", rosQueueSize, 1);

	setting_logStuff = true;

	// printf("MODE WITH CALIBRATION, but without exposure times!\n");
	// setting_photometricCalibration = 2;
	printf("MODE WITHOUT CALIBRATION, without exposure times!\n");
	setting_photometricCalibration = 0;
	setting_affineOptModeA = 0;
	setting_affineOptModeB = 0;

    undistorter = Undistort::getUndistorterForFile(calib, gammaFile, vignetteFile);

    setGlobalCalib(
            (int)undistorter->getSize()[0],
            (int)undistorter->getSize()[1],
            undistorter->getK().cast<float>());


    fullSystem = new FullSystem();
    fullSystem->linearizeOperation=false;


    if(!disableAllDisplay)
	    fullSystem->outputWrapper.push_back(new IOWrap::PangolinDSOViewer(
	    		 (int)undistorter->getSize()[0],
	    		 (int)undistorter->getSize()[1]));


    if(undistorter->photometricUndist != 0)
    	fullSystem->setGammaFunction(undistorter->photometricUndist->getG());

    ros::NodeHandle nh;
	ros::Subscriber imgSub = nh.subscribe("image", rosQueueSize, &vidCb);

    fullSystem->outputWrapper.push_back(new IOWrap::OutputWrapper(nh));

    ros::spin();
    fullSystem->printResult(saveFile);
    for(IOWrap::Output3DWrapper* ow : fullSystem->outputWrapper)
    {
        ow->join();
        delete ow;
    }

    delete undistorter;
    delete fullSystem;

	return 0;
}
