#include "utils.h"


extern string dir_name;
extern threadsafe_queue<cv::Mat> img_mat_list;

// Create folder to save image if no exist.
void CreateFolder(int scan_samples)
{
	time_t current_time;
	time(&current_time);
	tm* p = localtime(&current_time);
	dir_name = to_string(p->tm_mon + 1) + to_string(p->tm_mday) + "_" + to_string(p->tm_hour) + to_string(p->tm_min) + "_samples_" + to_string(scan_samples);
	string folderPath = "C:\\CODEREPO\\DahuaGal\\Image\\" + dir_name;
	if (_access(folderPath.c_str(), 0) == -1)	//If file is not exist.
	{
		cout << "Dir is not exist, make dir: " << dir_name << endl;
		int no_use = _mkdir(folderPath.c_str());
	}
	else
	{
		cout << "Please see detection results in dir: " << dir_name << endl;
	}
}

void SendSave(vector<ResampleCenters>& final_objs)
{
	cout << ">>>> Begin the send and save thread" << endl;
	cv::Mat save_img;
	int i = 0;
	int img_save = 0;
	while (1)
	{
		while (!img_mat_list.empty())
		{
			save_img = img_mat_list.wait_and_pop();
			//save_img = save_img(cv::Rect(20, 0, 224, 224));
			save_img = save_img(cv::Rect(0, 0, MID_WIDTH, MID_HEIGHT));
			//save_img = save_img(cv::Rect(0, 0, 264, 224));
			if (!save_img.empty())
			{
				std::stringstream filename2;
				time_t currenttime = time(0);
				char tmp[64];
				strftime(tmp, sizeof(tmp), "%Y%m%d_%H%M%S", localtime(&currenttime));
				string time_str = tmp;
				filename2 << "./Image/" << dir_name << "/" << "result" << "_" << time_str << "_" << final_objs[i].center_x << "_" << final_objs[i].center_y << ".jpg";
				cv::imwrite(filename2.str(), save_img);
				img_save += 1;
				i += 1;
			}
		}
		this_thread::sleep_for(chrono::microseconds(5000));
		//cout << "img list in the queue: " << img_list1.size() << endl;
		if (img_mat_list.empty() && img_save == final_objs.size())
		{
			break;
		}
	}
	cout << ">>>> End save img!!!" << endl;
}

// Initilize and connect the COM port.
void InitializeComPort()
{
	bool sigma1_status = m_Sigmal.OpenController(COM_PORT);//COM1对应第一个控制器
	Sleep(50);
	if (sigma1_status == TRUE)
	{
		// 1%,0.1ms,0.1us
		m_Sigmal.InitialSystem_STEP(DA_STEP_PCT, DA_STEP_TIME, DA_STEP_DLY);//步长初始化
																			// 5000 us
		m_Sigmal.InitialSystem_EXP(DA_EXP);//曝光初始化
										   // 10000 us
		m_Sigmal.InitialSystem_CYT(DA_CYCLE);//周期初始化
											 // 100 us
		m_Sigmal.InitialSystem_PW(DA_PULSE_W);//脉宽初始化

		m_Sigmal.on_rotate1_usb(0, 0);
		std::cout << "====>>>> 2. Initialize the COM port succeed!" << endl;
	}
	else
	{
		std::cout << "====>>>> 2. Initialize the COM port failed !!!" << endl;
	}
	Sleep(50);
	//m_Sigmal.on_rotate1_usb(-4.51, -4.51);
	m_Sigmal.on_rotate1_usb(0, 1.6);
	//m_Sigmal.on_rotate1_usb(-0.7, 1.5);
	Sleep(50);
}

// Connecting and setting the camera.
bool ConnectSettingCamera()
{
	bool is_ok = midcamera.scanCameraDevice();
	is_ok = midcamera.linkCamera(MID_CAM_SERIAL_NUMBER);
	is_ok = midcamera.openCamera();
	is_ok = midcamera.setCameraAcquisitionFrameRate(MID_ACQUISITION_FRAME_RATE);
	is_ok = midcamera.setCameraROI(MID_OFFSET_X, MID_OFFSET_Y, MID_WIDTH, MID_HEIGHT);
	is_ok = midcamera.setCameraExposureMode(MID_EXPOSURE_MODE);
	is_ok = midcamera.setCameraExposureTime(MID_EXPOSURE_TIME);
	is_ok = midcamera.setCameraBalanceWihteAuto(MID_BALANCEWHITE_MODE);
	is_ok = midcamera.setCameraReverseXY(MID_REVERSE_X, MID_REVERSE_Y);
	is_ok = midcamera.setCameraExposureGain(MID_GAIN_RAW);
	is_ok = midcamera.setCameraBrightness(MID_BRIGHTNESS);
	is_ok = midcamera.setCameraTriggerMode(MID_TRIGGE_MODE);
	is_ok = midcamera.setCameraImageType(MID_PIXEL_FORMAT);
	// create stream -> register callback -> start grabbing;
	is_ok = midcamera.createStream();
	is_ok = midcamera.registerCallback();
	is_ok = midcamera.cameraStartGrabbing();
	if (is_ok)
	{
		std::cout << "====>>>> 3. Connecting and setting the camera succeed." << endl;
	}
	else
	{
		std::cout << "====>>>> 3. Connecting and setting the camera failed !!!" << endl;
	}
	return is_ok;
}

// Close the camera.
bool CloseCamera()
{
	bool is_ok;
	is_ok = midcamera.cameraStopGrabbing();
	is_ok = midcamera.unregisterCallback();
	is_ok = midcamera.closeCamera();
	if (is_ok)
	{
		std::cout << "====>>>> 6. Close the camera succeed." << endl;
	}
	else
	{
		std::cout << "====>>>> 6. Close the camera failed !!!" << endl;
	}
	return is_ok;
}