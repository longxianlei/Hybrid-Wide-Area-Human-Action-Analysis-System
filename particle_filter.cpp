#include "particle_filter.h"

// Generate Resample points (x,y) around the center of detected objs.
// num_resample_i = total_samples* conf_i / total_conf.
// This step is the particle calibration setp. Get the sample center, and adjust to the center of each detected object.
// calib_x = (object_x - img_center_x)*aplha + particle_x;
// calib_y = (object_y - img_center_y)*aplha + particle_y;
// For more details, refer to the  Equation (7) in paper.
int GenerateResample(vector<vector<float>>& detected_objs_voltages, DetectedResults& detected_results, vector<ResampleCenters>& resample_centers)
{
	//cout << ">>>>>>>>In the Generate resamples center " << endl;
	int nums_detected = detected_results.detected_box.size();
	int newly_scan_samples = 0.8 * scan_samples;
	int global_scan_samples = scan_samples - newly_scan_samples;
	float total_confs = 0.001;
	for (auto i : detected_results.detected_conf)
	{
		total_confs += i;
	}

	int previous_id = detected_results.detected_ids[0];
	int voltage_index = 0;
	ResampleCenters temp_centers;
	for (int i = 0; i < nums_detected; i++)
	{
		int scan_id = detected_results.detected_ids[i];
		if (scan_id != previous_id)
		{
			voltage_index += 1;
			previous_id = scan_id;
		}
		float samples_i_x = detected_objs_voltages[voltage_index][0], samples_i_y = detected_objs_voltages[voltage_index][1];
		float sample_conf_i = detected_results.detected_conf[i];
		cv::Rect sample_box_i = detected_results.detected_box[i];
		// original is 224*224 in the previous setting. In the new test, we use 400x340 image as the captured image.
		// 07-14, * 0.002, we change it to new settting.
		float complement_i_x = ((sample_box_i.x + sample_box_i.width / 2.0) - 112) * (0.45 / (224)); // indoor experiment: 224 pixels <--> 0.45v   0.002 voltage is 1 pixel.
		float complement_i_y = ((sample_box_i.y + sample_box_i.height / 2.0) - 112) * (0.45 / (224)); // outdoor experiment: 76 pixels <-> 0.45v.    0.45/224*Img_width  (outdoor is not the parameter)
		float real_x = samples_i_x + complement_i_x, real_y = samples_i_y + complement_i_y;
		int newly_samples_i = newly_scan_samples * sample_conf_i / total_confs;
		temp_centers.center_x = real_x, temp_centers.center_y = real_y;
		temp_centers.confidence = sample_conf_i, temp_centers.num_samples = newly_samples_i;
		//cout << "box information: " << sample_box_i.x << " " << sample_box_i.y << endl;
		//cout << "id: " << scan_id << "sample i x: " << samples_i_x << ", y: " << samples_i_y << ", complement_i_x: " << complement_i_x << ", complement_i_y:" << complement_i_y << endl;

		resample_centers.emplace_back(temp_centers);
	}
	return newly_scan_samples;
}

// NMS filter out the same samples in different frames, using abs error. (Chebyshv distance.)
// filter_out abs_error(sample_i, sample_j)<0.06.
// This step is NMS step. We only retain the largest confidence particle if there are two closing particles.
void NMSResamples(vector<ResampleCenters>& resample_centers, vector<ResampleCenters>& filtered_centers, float pixel_error, int total_samples)
{
	int len = resample_centers.size();
	vector<bool> is_checked(len, false);
	int remain_nums = 0;
	for (int i = 0; i < len; i++)
	{
		bool is_filtered = false;
		if (i < len - 1 && is_checked[i] == false) // i=len-1, j=len, the vector will access failed. Segment fault.
		{
			for (int j = i + 1; j < len; j++)
			{
				// Calculate the distance of two objects, i and j. If the distance abs_error is smaller than the predefined threshold t,
				// we only retian the object with a largest confidence. Otherwise, we retain both particles.
				float abs_x = abs(resample_centers[i].center_x - resample_centers[j].center_x);
				float abs_y = abs(resample_centers[i].center_y - resample_centers[j].center_y);
				double abs_error = double(abs_x) + double(abs_y);
				if (abs_error < pixel_error)
				{
					//if (resample_centers[i].num_samples > resample_centers[j].num_samples)
					if (resample_centers[i].confidence >= resample_centers[j].confidence) // object_conf i >= j, retain i, remove j.
					{
						is_checked[j] = true;
					}
					else // otherwise, remain j, remove i.
					{
						is_checked[i] = true;
						is_filtered = true;
						break;
					}
				}
			}
		}
		if (is_checked[i] == false && !is_filtered)
		{
			filtered_centers.push_back(resample_centers[i]);
			remain_nums += resample_centers[i].num_samples;
		}
	}
	int len_filtered = filtered_centers.size();
	int avg_sample = (total_samples - remain_nums) / len_filtered;
	int last_samples = (total_samples - remain_nums) - avg_sample * (len_filtered - 1);
	for (int i = 0; i < len_filtered; i++)
	{
		if (i < len_filtered - 1)
		{
			filtered_centers[i].num_samples += avg_sample;
		}
		else
		{
			filtered_centers[i].num_samples += last_samples;
		}
	}
}


vector<vector<float>> RemoveNearstPoints(vector<vector<float>>& gen_scan_routes, float pixel_error)
{
	int len = gen_scan_routes.size();
	vector<bool> is_checked(len, false);
	vector<vector<float>> sparse_samples;
	//cout << "Begin to remove the samples: " << len << endl;
	for (int i = 0; i < len; i++)
	{
		bool is_filtered = false;
		if (i < len - 1 && is_checked[i] == false) // i=len-1, j=len, the vector will access failed. Segment fault.
		{
			for (int j = i + 1; j < len; j++)
			{
				float abs_x = abs(gen_scan_routes[i][0] - gen_scan_routes[j][0]);
				float abs_y = abs(gen_scan_routes[i][1] - gen_scan_routes[j][1]);
				double abs_error = double(abs_x) + double(abs_y);
				if (abs_error < pixel_error)   // 0.20, 07-14 0.20 <-> 100 pixel in indoor, we change to 0.60v for 100 pixels in outdoor.
				{
					is_checked[j] = true;
				}
			}
		}
		if (is_checked[i] == false && !is_filtered)
		{
			sparse_samples.emplace_back(gen_scan_routes[i]);
		}
	}
	//cout << "After to remove the samples: " << sparse_samples.size() << endl;
	return sparse_samples;
}
