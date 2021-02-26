#ifndef _INCL_GUARD
#define _INCL_GUARD

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <bitset>
#include <chrono>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/videoio.hpp>

#include "dirent.h"

#include "spinnaker/Spinnaker.h"
#include "spinnaker/SpinGenApi/SpinnakerGenApi.h"
using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;

#include <modbus.h>

const int module_n = 3;
const char servo_address[] = "192.168.1.33";

struct Images{
    cv::Mat current_image;
    cv::Mat previous_image;
    cv::Mat pattern_image;
    std::chrono::time_point<std::chrono::system_clock> c_stamp;
    std::chrono::time_point<std::chrono::system_clock> p_stamp;

    cv::Mat c1, c2, c3;

    int shift;
    int travel;
    int p_travel;
};

bool getMovement(Images *local_set);
void startVision();
void reloadVision();

#endif

//wut
