/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#ifndef ORTHO_SFM_TRIANGULATION_H
#define ORTHO_SFM_TRIANGULATION_H

#include <Eigen/Dense>
#include <vector>

#include <data_structures/colored_point_cloud.h>
#include <data_structures/Camera.h>
#include <data_structures/track.h>

namespace orthosfm
{

	Eigen::Vector3d
	intersectRays(const std::vector<Eigen::Vector3d>& origins, const std::vector<Eigen::Vector3d>& directions);

	void
	triangulateOrthographicTracks(const std::vector<std::shared_ptr<Camera>>& cameras, std::vector<Track>& tracks, bool resetExistingPoints);
}

#endif //ORTHO_SFM_TRIANGULATION_H