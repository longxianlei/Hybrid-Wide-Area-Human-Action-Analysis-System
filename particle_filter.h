#pragma once
#ifndef __PARTICLE_FILTER_H__
#define __PARTICLE_FILTER_H__

#include <vector>
#include "detection.h"
using namespace std;
extern int scan_samples;


int GenerateResample(vector<vector<float>>& detected_objs_voltages, DetectedResults& detected_results, vector<ResampleCenters>& resample_centers);
void NMSResamples(vector<ResampleCenters>& resample_centers, vector<ResampleCenters>& filtered_centers, float pixel_error, int total_samples);
vector<vector<float>> RemoveNearstPoints(vector<vector<float>>& gen_scan_routes, float pixel_error);
#endif