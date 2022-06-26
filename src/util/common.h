/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#ifndef ORTHO_SFM_COMMON_H
#define ORTHO_SFM_COMMON_H

#include <vector>
#include <string>
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>
#include <iostream>
#include "Eigen/Dense"

#include <data_structures/Camera.h>
#include <data_structures/track.h>
#include <algorithms/ReconstructionAlgorithm.h>

namespace orthosfm
{

#define M_PI 3.1415926535897932384626433832795028841971693993751058209749445923
#define M_PI_2 1.5707963267948966192313216916397514420985846996875529104874722961

	std::vector<std::string> getImagesInFolder(const std::string& folder);

	std::string zfill(const int& value, const int& zeros);

	std::string type2str(int type);

	std::string vec2str(const Eigen::Vector3d& vec);

	std::pair<double, double> meanAndStd(const std::vector<double>& values);

	std::vector<Track>
	filterTracksToAvailableCameras(const std::vector<std::shared_ptr<Camera>>& cameras, const std::vector<Track>& tracks, bool onlyFullSizeTracks, bool keepAdditionalCamera);
	std::vector<Track>
	filterTracksToAvailableCameras(const std::vector<unsigned int>& ids, const std::vector<Track>& tracks, bool onlyFullSizeTracks, bool keepAdditionalCamera);

	void propagateColorsToTracks(std::vector<Track>& tracks, const std::vector<View>& views);

// Save tracks to ply
	void
	savePointsToPLY(const std::string& outPath, std::vector<std::shared_ptr<Camera>>& cameras, const std::vector<Track>& tracks, const std::shared_ptr<
		ReconstructionAlgorithm>& algorithm);

	void printCameras(const std::vector<std::shared_ptr<Camera>>& cameras, bool ordered);

	std::string solverTypeToString(const SFM_SOLVER_TYPE& solver);
	unsigned int solverTypeToIndex(const SFM_SOLVER_TYPE& solver);
	SFM_SOLVER_TYPE solverTypeFromIndex(const int& index);

}

#endif //ORTHO_SFM_COMMON_H