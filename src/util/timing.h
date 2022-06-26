/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#ifndef ORTHO_SFM_TIMING_H
#define ORTHO_SFM_TIMING_H

#include <chrono>
#include <string>

namespace orthosfm
{

	struct TimeMeasurements
	{
		double initTime = 0;
		double trackBuildingTime = 0;
		double poseEstimationTime = 0;
		double totalTime = 0;
	};

	double
	elapsedSeconds(const std::chrono::steady_clock::time_point& start, const std::chrono::steady_clock::time_point& end);

	void saveRuntimesToTxt(const std::string& path, double init, double track, double pose, double total);
	TimeMeasurements runtimesFromTxt(const std::string& path);

}



#endif //ORTHO_SFM_TIMING_H
