#pragma once
#ifndef __UTILS_H__
#define __UTILS_H__


#include <iostream>
#include <string>
#include <direct.h>
#include <io.h>
#include <time.h>
#include <vector>
#include "thread_safe_queue.h"
#include "detection.h"
#include "sigma_controller.h"
#include "hv_cam_dahua.h"
using namespace std;

// DA Contorller Setting.
#define COM_PORT			3
#define DA_STEP_PCT			1		// %
#define DA_STEP_TIME		0.25		// ms
#define DA_STEP_DLY			0.1		// us
#define DA_EXP				1000	// us    should be same with HC_EXP
#define DA_CYCLE			2500	// us
#define DA_PULSE_W			100		// us

// Camera Setting.
#define MID_CAM_SERIAL_NUMBER "6G03CFBPAK00001"
#define MID_OFFSET_X 228   // 00   228         outdoor 132
#define MID_OFFSET_Y 160   // 00   160		   outdoor 16
#define MID_WIDTH 264   //720    indoor  264    outdoor 400  
#define MID_HEIGHT 224  //540    indoor  224    outdoor 340

//#define MID_OFFSET_X 0   // 00
//#define MID_OFFSET_Y 0   // 00
//#define MID_WIDTH 720  //720
//#define MID_HEIGHT 540  //540

#define MID_EXPOSURE_MODE HV_CAM_DAHUA::EXPOSURE_AUTO_MODE_OFF
#define MID_BALANCEWHITE_MODE HV_CAM_DAHUA::BALANCEWHITE_AUTO_OFF
#define MID_EXPOSURE_TIME 1000  //1000  daytime 300   outdoor  1100 
#define MID_REVERSE_X false
#define MID_REVERSE_Y false
#define MID_GAIN_RAW 4.8 //9  5.2  daytime 2.5     outdoor 2.8 
#define MID_BRIGHTNESS 95 //85     daytime 80       outdoor 92
#define MID_TRIGGE_MODE HV_CAM_DAHUA::TRIGGER_MODE_LINE1
#define MID_PIXEL_FORMAT HV_CAM_DAHUA::BayerRG8
#define MID_ACQUISITION_FRAME_RATE 800

extern CSigmaController m_Sigmal;
extern HV_CAM_DAHUA midcamera;

void CreateFolder(int scan_samples);
void SendSave(vector<ResampleCenters>& final_objs);
void InitializeComPort();
bool ConnectSettingCamera();
bool CloseCamera();

#endif


