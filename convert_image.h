#pragma once
#ifndef __CONVERT_IMAGE_H__
#define __CONVERT_IMAGE_H__

#include "hv_cam_dahua.h"
#include "thread_safe_queue.h"

class ConvertImage {
public:
	ConvertImage();
	~ConvertImage();

public:
	void ProcessImage();
	int num_samples = 0;
	bool is_frame_ok;

private:
	int nBGRBufferSize1;
	const void* pImage1;
	queue<uchar*> _imgBuffer1;
	queue<int> _imgIDs1;
	cv::Mat image1;
	IMGCNV_SOpenParam openParam1;
	IMGCNV_EErr status1;
};

extern vector<cv::Mat> img_list1;
extern threadsafe_queue<cv::Mat> img_mat_list;

#endif // !CONVERTIMAGE_H