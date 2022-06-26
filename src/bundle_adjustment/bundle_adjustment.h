/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#ifndef ORTHO_SFM_BUNDLEADJUSTMENT_H
#define ORTHO_SFM_BUNDLEADJUSTMENT_H

#include <data_structures/Camera.h>
#include <algorithms/ReconstructionAlgorithm.h>


namespace orthosfm
{
	void runBundleAdjustment(std::vector<std::shared_ptr<Camera>>& cameras, std::vector<Track>& tracks,
		const std::shared_ptr<ReconstructionAlgorithm>& algorithm,
		const bool& optimizePoints, const bool& retriangulatePoints);

	void runEvenOddBundleAdjustment(std::vector<std::shared_ptr<Camera>>& cameras, std::vector<Track>& tracks,
		const std::shared_ptr<ReconstructionAlgorithm>& algorithm,
		const bool& optimizePoints, const bool& retriangulatePoints,
		const bool& fixFirstTwoCameras);
}

#endif //ORTHO_SFM_BUNDLEADJUSTMENT_H