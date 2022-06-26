/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#ifndef ORTHO_SFM_OUTLIERFILTERING_H
#define ORTHO_SFM_OUTLIERFILTERING_H

#include <algorithms/ReconstructionAlgorithm.h>
#include <data_structures/track.h>

namespace orthosfm
{

	std::vector<Track>
	filterOutlierTracks(const std::vector<Track>& tracks, const std::vector<std::shared_ptr<Camera>>& cameras);

	std::vector<Track>
	filterTracksWithReprojectionError(const std::vector<Track>& tracks, const std::vector<std::shared_ptr<Camera>>& cameras, const std::shared_ptr<
		ReconstructionAlgorithm>& algorithm);

	std::vector<std::pair<double, int>>
	getNearestNeighbourDistance(const std::vector<std::pair<Eigen::Vector4d, int>>& points);
}

#endif //ORTHO_SFM_OUTLIERFILTERING_H