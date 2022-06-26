/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/


#ifndef ORTHO_SFM_DATASETGENERATION_H
#define ORTHO_SFM_DATASETGENERATION_H

#include <data_structures/track.h>
#include <data_structures/view.h>
#include <algorithms/orthographic/OrthographicCamera.h>

#include <Eigen/Dense>

namespace orthosfm
{

	#define PointCloud std::vector<Eigen::Vector3d>

	struct TestDataset
	{
		std::vector<Track> tracks;
		std::vector<View> views;
		std::vector<std::shared_ptr<OrthographicCamera>> groundTruthCameras;
	};

	std::vector<std::shared_ptr<OrthographicCamera>> generateGroundTruthCameras(const std::vector<View>& views);

	std::vector<TestDataset> generateDatasets(const std::vector<std::string>& modelPaths);

	std::vector<PointCloud > loadPointClouds(const std::vector<std::string>& modelPaths);
}

#endif //ORTHO_SFM_DATASETGENERATION_H