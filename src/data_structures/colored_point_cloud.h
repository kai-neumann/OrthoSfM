/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#ifndef ORTHO_SFM_COLORED_POINT_CLOUD_H
#define ORTHO_SFM_COLORED_POINT_CLOUD_H

#include <Eigen/Dense>
#include <utility>

namespace orthosfm
{
	struct ColoredPointCloud
	{
		std::vector<Eigen::Vector3d> points;
		std::vector<Eigen::Vector3i> colors;

		ColoredPointCloud(std::vector<Eigen::Vector3d> points, std::vector<Eigen::Vector3i> colors)
		{
			this->points = std::move(points);
			this->colors = std::move(colors);
		}
	};
}

#endif //ORTHO_SFM_COLORED_POINT_CLOUD_H
