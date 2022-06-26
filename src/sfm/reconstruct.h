/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#pragma once

#include <string>
#include <vector>

#include <data_structures/view.h>
#include <data_structures/track.h>
#include <data_structures/solver_type.h>
#include <util/common.h>

#ifndef ORTHO_SFM_RECONSTRUCT_H
#define ORTHO_SFM_RECONSTRUCT_H

namespace orthosfm
{
	// Define the setting struct that is input into the reconstruction method
	struct reconstruction_config
	{
		std::string projectFolder = "";
		std::string imageFolder = "";
		std::string maskFolder = "";
		std::string trackFile = "";
		int downscaleFactor = 1;
		SFM_SOLVER_TYPE solver = SFM_SOLVER_TYPE::ORTHO_QUATERNION;
		bool useMveForMatching = true;
		bool exportPairwiseTracks = true;
	};

	std::vector<std::shared_ptr<Camera>> reconstruct(const reconstruction_config& config);

	std::vector<std::shared_ptr<Camera>>
	runPoseEstimation(const std::vector<View>& views, const std::shared_ptr<ReconstructionAlgorithm>& algorithm, std::vector<Track>& tracks, const reconstruction_config& config);
}

#endif //ORTHO_SFM_RECONSTRUCT_H