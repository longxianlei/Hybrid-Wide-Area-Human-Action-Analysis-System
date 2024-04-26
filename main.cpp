

#include "main.h"


#define _CRT_SECURE_NO_WARNINGS

using namespace std;
//using namespace cv;

// Multithread mutex and condition variable.
mutex detect_nms_mutex;
condition_variable detect_nms_cv;
bool is_detect_done = false;
bool is_nms_over = false;

// Global shared data.
CSigmaController m_Sigmal;
HV_CAM_DAHUA midcamera = HV_CAM_DAHUA();
//extern HV_CAM_DAHUA midcamera;
vector<vector<float>> gen_scan_routes;
ObjectDetector detector = ObjectDetector();
int scan_samples;
vector<vector<float>> detected_objs_voltages;
string dir_name;
bool is_save_img = true;
bool is_save_results = true;
//  The nms threshold for filter nearest samples
float pixel_error = 0.12; //  before is 0.20,100 pixels as error.   10-07 outdorr, 0.50; indoor, 0.2.    0.14v <-> 70 pixels in the indoor. 07-14, however, 24 pixel in outdoor. We scale it to 0.4v, about 68 pixels.
int scan_time = 1;

// The detected objs;
vector<ResampleCenters> total_objs;

// Thread functions for processing and helper functions for nms.
vector<vector<float>> SolveScanRoutes(int sample_nums, int max_range, int min_range);
//void SolveScanRoutes(int sample_nums, int max_range, int min_range, vector<vector<float>>& gen_scan_routes);
void SendXYSignal();
//bool SendSolvedXYSignal(vector<vector<float>>& solved_scan_voltages);
void SendSolvedXYSignal();
//void InitializeComPort();
//bool ConnectSettingCamera();
//bool CloseCamera();
void InitializeDetector(cv::String cfg_file, cv::String weights_file);
//int GenerateResample(vector<vector<float>>& detected_objs_voltages, DetectedResults& detected_results, vector<ResampleCenters>& resample_centers);
//void NMSResamples(vector<ResampleCenters>& resample_centers, vector<ResampleCenters>& filtered_centers, int total_samples);
//void CreateFolder(int particle_num);
void ThreadProcessImage();
void ThreadFilterResamples();
//vector<vector<float>> RemoveNearstPoints(vector<vector<float>>& gen_scan_routes);
//void SendSave(vector<ResampleCenters>& final_objs);

// prior probability region_and_object relationship.
float prob_road = 0.847;;
float prob_sidewalk = 0.129;
float prob_tree = 0.022;
float prob_building = 0.002;

// count the region area, as prior.
int road_area = 0;
int sidewalk_area = 0;
int tree_area = 0;
int building_area = 0;


int main()
{

	auto beg_t = chrono::system_clock::now();
	auto sys_time_begin = chrono::high_resolution_clock::now();
	cv::String cfg_file = "C:\\CODEREPO\\DahuaGal\\model\\yolov3.cfg";
	cv::String weights_file = "C:\\CODEREPO\\DahuaGal\\model\\yolov3.weights";
	scan_samples = 145;
	

	// Semantic segmentation
	//Mat pano_img = imread("panoramic_img.jpg");
	//Mat seg_results = semantic_seg(pano_img);
	// get the region probability. prob = area% *region_object_prior
	//int image_area = pano_img.rows * pano_img.cols;
	//float prob_car_on_road = prob_road * road_area / image_area;
	//float prob_car_on_sidewalk = prob_sidewalk * sidewalk_area / image_area;
	//float prob_car_on_tree = prob_tree * tree_area / image_area;
	//float prob_car_on_building = prob_building * building_area / image_area;

	//int sampling_road = scan_samples * prob_car_on_road;
	//int sampling_sidewalk = scan_samples * prob_car_on_sidewalk;
	//int sampling_tree = scan_samples * prob_car_on_tree;
	//int sampling_building = scan_samples * prob_car_on_building;


	/* 0. Create folder to store the detection results.*/

	CreateFolder(scan_samples);

	/* 1. Initialize and warm up the detector.*/
	InitializeDetector(cfg_file, weights_file);


	/* 2. Initialize the COM_DA contorller.*/
	InitializeComPort();

	/* 3. Connecting and setting the camera.*/
	bool is_camera_open = ConnectSettingCamera();

	/* 4. Compute the scanning path given the samples.Using the Route Planning algorithms.*/
	//gen_scan_routes = SolveScanRoutes(scan_samples, 500, -500);

	/**************************
	Note, here is the added new sampling strategy. Chen proposed to sampling particles under the region constrained. 
	So, the new method is just add a position shift to the center of high_probability region. (The extracted constraint region).
	The region center is (0.2, 1.6). So, every sampling particle is shift around (0.2, 1.6).
	Total region is (10v x 10v) in voltage, and (40 x 40) in degree. 
	The sampling region is (3 x 3). Just predefine the region center (x, y), and the sampling area, e.g., (2v x 2v), (3v x 3v).
	So, max range is 1.5, min range is -1.5.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
	***********************/
	gen_scan_routes = SolveScanRoutes(scan_samples, 150, -150);
	for (int i = 0; i < gen_scan_routes.size(); i++)
	{
		gen_scan_routes[i][0] += 0.2;
		gen_scan_routes[i][1] += 1.6;
		//cout << gen_scan_routes[i][0] << ", " << gen_scan_routes[i][1] << endl;
	}

	/* 5. Processing here. Create ConvertImage object. Grab and convert the image.
		Create 1) send (x,y) thread and 2) image convert therad. */
	ConvertImage image_convertor = ConvertImage();
	image_convertor.num_samples = scan_samples;
	chrono::steady_clock::time_point begin_time2 = chrono::steady_clock::now();


	//thread send_com_thread(SendXYSignal);
	thread send_com_thread(SendSolvedXYSignal);
	thread convert_image_thread(&ConvertImage::ProcessImage, image_convertor);
	//thread send_com_thread(SendSolvedXYSignal, ref(gen_scan_routes));
	
	thread nms_filter_thread(ThreadFilterResamples);
	thread img_process_thread(ThreadProcessImage);

	while (img_list1.size() < scan_samples)
	{
		this_thread::sleep_for(chrono::microseconds(2000));
	}

	chrono::steady_clock::time_point send_end_time3 = chrono::steady_clock::now();
	cout << "The whole process is  get clock (ms): !!!!!!!!!!!!!" << chrono::duration_cast<chrono::microseconds>(send_end_time3 - begin_time2).count() / 1000 << endl;

	send_com_thread.join();
	convert_image_thread.join();
	nms_filter_thread.join();
	img_process_thread.join();

	while (is_nms_over == false)
	{
		this_thread::sleep_for(chrono::milliseconds(2));
	}

	cout << ">>>>>>>>>>>>>The first time detected object:<<<<<<<<<<< " << endl;
	for (auto i : total_objs)
	{
		cout << "before filtered: " << i.num_samples << ",  " << i.center_x << ", " << i.center_y << ", " << i.confidence << endl;
	}
	cout << ">>>>>>>>>>>End the output object.<<<<<<<<<<" << endl;
	//pixel_error = 0.12;
	scan_time += 1;
	cout << ">>>>>>>>>>>>>>begin second part."<<scan_samples << endl;
	cout << "The total img list: " << img_list1.size() << " total nums:" << scan_samples << " , " << img_mat_list.empty() << endl;
	image_convertor.num_samples = scan_samples;
	//thread send_com_thread(SendXYSignal);
	thread send_com_thread1(SendSolvedXYSignal);
	thread convert_image_thread1(&ConvertImage::ProcessImage, image_convertor);
	//this_thread::sleep_for(chrono::milliseconds(5));
	//thread send_com_thread(SendSolvedXYSignal, ref(gen_scan_routes));
	
	//this_thread::sleep_for(chrono::milliseconds(5));
	thread nms_filter_thread1(ThreadFilterResamples);
	//this_thread::sleep_for(chrono::milliseconds(5));
	thread img_process_thread1(ThreadProcessImage);

	while (img_list1.size() < scan_samples)
	{
		this_thread::sleep_for(chrono::microseconds(2000));
	}
	send_com_thread1.detach();
	convert_image_thread1.detach();
	nms_filter_thread1.join();
	img_process_thread1.join();

	while (is_nms_over == false)
	{
		this_thread::sleep_for(chrono::milliseconds(10));
	}

	//pixel_error = 0.20;
	scan_time += 1;
	cout << ">>>>>>>>>>>>>>begin second part." << scan_samples << endl;
	cout << "The total img list: " << img_list1.size() << " total nums:" << scan_samples << " , " << img_mat_list.empty() << endl;
	image_convertor.num_samples = scan_samples;
	//thread send_com_thread(SendXYSignal);
	thread send_com_thread2(SendSolvedXYSignal);
	thread convert_image_thread2(&ConvertImage::ProcessImage, image_convertor);
	//this_thread::sleep_for(chrono::milliseconds(5));
	//thread send_com_thread(SendSolvedXYSignal, ref(gen_scan_routes));
	//thread send_com_thread2(SendSolvedXYSignal);
	//this_thread::sleep_for(chrono::milliseconds(5));
	thread nms_filter_thread2(ThreadFilterResamples);
	//this_thread::sleep_for(chrono::milliseconds(5));
	thread img_process_thread2(ThreadProcessImage);

	while (img_list1.size() < scan_samples)
	{
		this_thread::sleep_for(chrono::microseconds(2000));
	}
	send_com_thread2.detach();
	convert_image_thread2.detach();
	nms_filter_thread2.join();
	img_process_thread2.join();

	auto end_t = chrono::system_clock::now();
	chrono::steady_clock::time_point end_detect = chrono::steady_clock::now();
	auto sys_time_end = chrono::high_resolution_clock::now();

	std::cout << "Total time (ms): !!!!!!!!!!!!!" << chrono::duration_cast<chrono::microseconds>(end_detect - begin_time2).count() / 1000.0 << endl;
	std::cout << "Sys Total time (ms): !!!!!!!!!!!!!" << chrono::duration_cast<chrono::microseconds>(end_t - beg_t).count() / 1000.0 << endl;
	std::cout << "Highresol Total time (ms): !!!!!!!!!!!!!" << chrono::duration_cast<chrono::microseconds>(sys_time_end - sys_time_begin).count() / 1000.0 << endl;

	cout << img_list1.size() << img_mat_list.size() << endl;

	cout << "Before filtered: " << endl;

	for (auto i : total_objs)
	{
		cout << "before filtered: " << i.num_samples << ",  " << i.center_x << ", " << i.center_y << ", " << i.confidence << endl;
	}
	vector<ResampleCenters> final_objs;
	NMSResamples(total_objs, final_objs, pixel_error, 100);
	for (auto j : final_objs)
	{
		cout << "after filtered: " << j.num_samples << ",  " << j.center_x << ", " << j.center_y << ", " << j.confidence << endl;
	}

	if (is_save_results)
	{
		vector<vector<float>> results_voltage(final_objs.size(), vector<float>(2, 0));
		for (int index = 0; index < results_voltage.size(); index++)
		{
			//std::cout << "[" << scan_samples[scan_index[index]][0] << ", " << scan_samples[scan_index[index]][1] << "] ";
			results_voltage[index][0] = final_objs[index].center_x;
			results_voltage[index][1] = final_objs[index].center_y;
		}

		image_convertor.num_samples = final_objs.size();
		thread convert_image_thread3(&ConvertImage::ProcessImage, image_convertor);

		gen_scan_routes = results_voltage;
		this_thread::sleep_for(chrono::milliseconds(5));
		//thread send_com_thread(SendSolvedXYSignal, ref(gen_scan_routes));
		thread send_com_thread3(SendSolvedXYSignal);
		thread send_save_thread3(SendSave, ref(final_objs));
		convert_image_thread3.join();
		send_com_thread3.join();
		send_save_thread3.join();
	}

	/* 6. Disconnect and close the camera.*/
	img_list1.clear();
	bool is_close = CloseCamera();
	return 0;
}

// Send the simualtion (x, y) signals to COM port.
void SendXYSignal()
{
	std::cout << "!!!!!!!!! begin to send com thread" << endl;
	int count = 0;
	chrono::steady_clock::time_point begin_time_thre = chrono::steady_clock::now();
	while (1)
	{
		if (count == 100)
		{
			std::cout << "break the send thread" << endl;
			break;
		}
		if (count % 4 == 0)
		{
			m_Sigmal.on_rotate1_usb(-2.6, 1.7);
		}
		else if (count % 4 == 1)
		{
			m_Sigmal.on_rotate1_usb(-2.8, 2.0);
		}
		else if (count % 4 == 2)
		{
			m_Sigmal.on_rotate1_usb(-2.93, 1.8);
		}
		else
		{
			m_Sigmal.on_rotate1_usb(-3.25, 1.71);
		}

		count++;
		cout << "send signals: " << count << endl;
		this_thread::sleep_for(chrono::microseconds(1000));
	}
	std::cout << "cout the x y is overed!" << endl;
	chrono::steady_clock::time_point send_end_time2_thre = chrono::steady_clock::now();
	std::cout << "Send clock (ms): " << chrono::duration_cast<chrono::microseconds>(send_end_time2_thre - begin_time_thre).count() / 1000 << endl;
}

// Send the given solved (x, y) signals to COM port.
void SendSolvedXYSignal()
{
	std::cout << "!!!!!!!!! begin to send com thread" << endl;
	int count = 0;
	chrono::steady_clock::time_point begin_time_thre = chrono::steady_clock::now();
	for (int i = 0; i < gen_scan_routes.size(); i++)
	{
		count++;
		//cout << "sent " << count << " :[" << solved_scan_voltages[i][0] << "," << solved_scan_voltages[i][1] << "]" << endl;
		m_Sigmal.on_rotate1_usb(gen_scan_routes[i][0], gen_scan_routes[i][1]);
		this_thread::sleep_for(chrono::microseconds(4000));
		//while (!is_callback_ok)
		//{
		//	this_thread::sleep_for(chrono::microseconds(100));
		//}
		//is_callback_ok = false;
	}
	std::cout << ">>>> End the send com thread" << endl;
	chrono::steady_clock::time_point send_end_time2_thre = chrono::steady_clock::now();
	std::cout << "Send clock (ms): " << chrono::duration_cast<chrono::microseconds>(send_end_time2_thre - begin_time_thre).count() / 1000 << endl;
	//return true;
	std::cout << ">>>>> Sent total samples: " << count << " end!" << endl;
}

// Given the smaples data, solve the scanning routes.
vector<vector<float>> SolveScanRoutes(int sample_nums, int max_range, int min_range)
{
	//chrono::steady_clock::time_point begin_time_ortools = chrono::steady_clock::now();
	int num_samples = sample_nums;
	//// 1. Generate samples;
	vector<vector<int>> scan_samples = operations_research::GenerateSamples(num_samples, max_range, min_range);
	// compute the start and end points of each scan.
	int* assign_points = operations_research::CalculateStartEndPoints(scan_samples);
	//std::cout << "original samples" << endl;
	//for (auto temp_sample : scan_samples)
	//{

	//	std::cout << "[" << temp_sample[0] << ", " << temp_sample[1] << "] ";
	//}
	//std::cout << "Scan begin points: " << scan_samples[assign_points[0]][0] << ", " << scan_samples[assign_points[0]][1] << endl;
	//std::cout << "Scan end points: " << scan_samples[assign_points[1]][0] << ", " << scan_samples[assign_points[1]][1] << endl;
	//// 2. Compute distance matrix;
	vector<vector<int64_t>> chebyshev_dist = operations_research::ComputeChebyshevDistanceMatrix(num_samples, scan_samples);
	//// 3. Initial the Datamodel;

	operations_research::DefineStartDataModel StartEndData;
	StartEndData.distance_matrix = chebyshev_dist;
	StartEndData.num_vehicles = 1;
	StartEndData.starts.emplace_back(operations_research::RoutingIndexManager::NodeIndex{ assign_points[0] });
	StartEndData.ends.emplace_back(operations_research::RoutingIndexManager::NodeIndex{ assign_points[1] });
	vector<int> scan_index = operations_research::Tsp(StartEndData);

	vector<vector<float>> scan_voltages(num_samples, vector<float>(2, 0));
	for (int index = 0; index < scan_index.size(); index++)
	{
		//std::cout << "[" << scan_samples[scan_index[index]][0] << ", " << scan_samples[scan_index[index]][1] << "] ";
		scan_voltages[index][0] = scan_samples[scan_index[index]][0] / 100.0;
		scan_voltages[index][1] = scan_samples[scan_index[index]][1] / 100.0;
	}
	//for (auto i : scan_index)
	//{
	//	std::cout << i << "-> ";
	//}
	//std::cout << " " << endl;
	//chrono::steady_clock::time_point end_time_ortools = chrono::steady_clock::now();
	//std::cout << "Waitting for save (ms): " << chrono::duration_cast<chrono::microseconds>(end_time_ortools - begin_time_ortools).count() / 1000.0 << endl;
	//std::cout << "====>>>> 4. Solve the scanning routes succeed." << endl;
	return scan_voltages;
}

// Initilize the detector.
void InitializeDetector(cv::String cfg_file, cv::String weights_file)
{
	detector.is_save_img = is_save_img;
	detector.Initialization(cfg_file, weights_file, 224, 224); //224,224
	// Warm up the detector.
	cv::Mat temp_img = cv::imread("1122all_new44.jpg");
	bool temp_result;
	for (int i = 0; i < 2; i++)
	{
		temp_result = detector.Inference(temp_img, 0);
	}
	std::cout << "====>>>> 1. Warm up the detector end!" << endl;

	// When warm up the detector, we need to clear the output of the confidence vector.
	detector.detected_results.detected_box.clear();
	detector.detected_results.detected_conf.clear();
	detector.detected_results.detected_ids.clear();
}


void ThreadProcessImage()
{
	lock_guard<mutex> lck(detect_nms_mutex);
	auto sys_time_begin = chrono::high_resolution_clock::now();
	cout << "!!!!!!!! begin the Image processing thread!!!!!!!!" << endl;
	cv::Mat grab_img2;
	int real_img_count = 0;
	//chrono::steady_clock::time_point thread_begin = chrono::steady_clock::now();
	while (1)
	{
		while (!img_mat_list.empty())
		{	// for debug image detection
			cout << " beggin to detect: " << real_img_count << " The image is remain: " << img_mat_list.size() << endl;
			grab_img2 = img_mat_list.wait_and_pop();

			//imshow("captrued img", grab_img2);
			//waitKey(300);
			//vector<vector<float>> detected_objs_voltages;
			//grab_img2 = grab_img2(cv::Rect(20, 0, 224, 224));
			//grab_img2 = grab_img2(cv::Rect(20, 0, 224, 224));
			cv::resize(grab_img2, grab_img2, cv::Size(224, 224), 0, 0, cv::INTER_AREA); //224,224
			if (!grab_img2.empty())
			{
				bool is_detected = detector.Inference(grab_img2, real_img_count);
				if (is_detected)
				{
					cout << "!!!!!!!!!!!!!detected !!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
					detected_objs_voltages.emplace_back(gen_scan_routes[real_img_count]);
					if (is_save_img)
					{
						std::stringstream filename2;
						time_t currenttime = time(0);
						char tmp[64];

						strftime(tmp, sizeof(tmp), "%Y%m%d_%H%M%S", localtime(&currenttime));
						string time_str = tmp;

						filename2 << "./Image/" << dir_name << "/"<< scan_time <<"_" << time_str << "_" << real_img_count << "_" << gen_scan_routes[real_img_count][0] << "_" << gen_scan_routes[real_img_count][1] << ".jpg";
						cv::imwrite(filename2.str(), grab_img2);
						cout <<"frame id: "<<real_img_count<<"x, y: " << gen_scan_routes[real_img_count][0] <<", " << gen_scan_routes[real_img_count][1] << endl;
					}
				}
				real_img_count += 1;
			}
		}
		this_thread::sleep_for(chrono::microseconds(1000));
		//cout << "img list in the queue: " << img_list1.size() << endl;
		if (img_mat_list.empty() && real_img_count == scan_samples)
		{
			cout << "Image queue is empty" << endl;
			break;
		}
		else
		{
			cout << "Image mat queue is: " << img_mat_list.size() << ", total processing: " << real_img_count<<" total list: "<<scan_samples << endl;
		}
	}

	cout << "End detected thread!!!!!!" << endl;
	auto sys_time_end = chrono::high_resolution_clock::now();
	std::cout << "Thread Image process cost: Total time (ms): !!!!!!!!!!!!!" << chrono::duration_cast<chrono::microseconds>(sys_time_end - sys_time_begin).count() / 1000.0 << endl;
	cout << "Total processing Img: " << scan_samples << ", FPS is: " << (scan_samples) * 1000000.0 / float(chrono::duration_cast<chrono::microseconds>(sys_time_end - sys_time_begin).count()) << endl;
	
	is_detect_done = true;
	detect_nms_cv.notify_one();
	//cout << "Processing speed: " << real_img * 1000000.0 / float(chrono::duration_cast<chrono::microseconds>(end_detect - begin_detect).count()) << " FPS!" << endl;
}

void ThreadFilterResamples()
{
	cout << ">>>>>>>>>>>>Begin filter thread!!!!" << endl;
	unique_lock<mutex> lck(detect_nms_mutex);
	detect_nms_cv.wait(lck, [] {return is_detect_done; });
	auto sys_time_begin = chrono::high_resolution_clock::now();
	is_detect_done = false;
	vector<ResampleCenters> resample_centers;
	vector<ResampleCenters> filtered_centers;
	int total_sample_nums = 0;

	if (detected_objs_voltages.size() > 0)
	{
		total_sample_nums = GenerateResample(detected_objs_voltages, detector.detected_results, resample_centers);
		NMSResamples(resample_centers, filtered_centers, pixel_error, total_sample_nums);
		// Add the filtered sample into the final results.
		total_objs.insert(total_objs.end(), filtered_centers.begin(), filtered_centers.end());
	}

	//for (auto i : resample_centers)
	//{
	//	cout << "before filtered: " << i.num_samples << ",  " << i.center_x << ", " << i.center_y << ", " << i.confidence << endl;
	//}
	//for (auto j : filtered_centers)
	//{
	//	cout << "after filtered: " << j.num_samples << ",  " << j.center_x << ", " << j.center_y << ", " << j.confidence << endl;
	//}
	lck.unlock();

	detector.detected_results.detected_box.clear();
	detector.detected_results.detected_conf.clear();
	detector.detected_results.detected_ids.clear();
	detected_objs_voltages.clear();
	img_list1.clear();
	gen_scan_routes.clear();

	int temp_i = 0;
	for (auto j : filtered_centers)
	{
		//cout << " begin new scan: " << endl;
		//cout << "after filtered: " << j.num_samples << ",  " << j.center_x << ", " << j.center_y << ", " << j.confidence << endl;
		// 07-14, in the indoor setting, the sample radius is (-50, 50), which means (-0.5, 0.50)---> (-250,+250 pixels). We should change it accordingly. 
		vector<vector<float>> scan_routes_j = SolveScanRoutes(j.num_samples, int(50.0 * 3.0 / j.confidence), int(-50.0 * 3.0 / j.confidence));
		//SolveScanRoutes(j.num_samples, int(50.0 * 1.0 / j.confidence), int(-50.0 * 1.0 / j.confidence), gen_scan_routes);
		for (int i = 0; i < scan_routes_j.size(); i++)
		{
			scan_routes_j[i][0] += j.center_x;
			scan_routes_j[i][1] += j.center_y;
			//cout << gen_scan_routes[i][0] << ", " << gen_scan_routes[i][1] << endl;
		}
		gen_scan_routes.insert(gen_scan_routes.end(), scan_routes_j.begin(), scan_routes_j.end());
		temp_i += scan_routes_j.size();
		//cout <<"scan routes: " << scan_routes_j[scan_routes_j.size() - 1][0] << ", total_list: " << gen_scan_routes[temp_i - 1][0] << endl;
		//cout << "Gen scan routes: " << gen_scan_routes.size() << endl;
	}
	//cout << "End the NMS thread!!!!!!!!" << endl;

	//cout << ">>>>>>Before remove nearst points: " << endl;
	//for (auto i : gen_scan_routes)
	//{
	//	cout << i[0] << " " << i[1] << endl;
	//}

	vector<vector<float>> sample_removed = RemoveNearstPoints(gen_scan_routes, pixel_error);
	//cout << ">>>>>>After remove nearst points: " << endl;
	//for (auto j : sample_removed)
	//{
	//	cout << j[0] << " " << j[1] << endl;
	//}

	gen_scan_routes.clear();
	//copy(sample_removed.begin(), sample_removed.end(), gen_scan_routes.begin());
	//gen_scan_routes.insert(gen_scan_routes.end(), sample_removed.begin(), sample_removed.end());
	gen_scan_routes = sample_removed;
	//cout << "Copy scan after: " << gen_scan_routes.size() << sample_removed.size() << endl;
	scan_samples = gen_scan_routes.size();
	is_nms_over = true;

	resample_centers.clear();
	filtered_centers.clear();

	auto sys_time_end = chrono::high_resolution_clock::now();
	std::cout << "Thread NMS and filter cost: Total time (ms): !!!!!!!!!!!!!" << chrono::duration_cast<chrono::microseconds>(sys_time_end - sys_time_begin).count() / 1000.0 << endl;
}
