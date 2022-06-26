/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#ifndef ORTHO_SFM_FULLPIPELINETESTS_H
#define ORTHO_SFM_FULLPIPELINETESTS_H

#include <string>
#include <Eigen/Dense>
#include <utility>
#include <sfm/reconstruct.h>

namespace orthosfm
{

	struct FullPipelineResult
	{
		double meanAngularError = 0;
		double stdAngularError = 0;
		double meanPositionError = 0;
		double stdPositionError = 0;
		double runtime = 0;
		double poseEstimationTime = 0;
		int configurationID = 0;
	};

	struct ReferenceCamera
	{
		Eigen::Quaternion<double> rotation;
		Eigen::Vector3d position;
		std::string name;
	};

	struct RunConfiguration
	{
		SFM_SOLVER_TYPE solverType;
		std::string name;

		RunConfiguration(SFM_SOLVER_TYPE solverType, std::string name)
		{
			this->solverType = solverType;
			this->name = std::move(name);
		}

		bool operator==(const RunConfiguration& other) const
		{
			return solverType == other.solverType && name == other.name;
		}
	};

	void runFullPipelineTests(const std::string& datasetFolder, const std::string& projectFolderRoot, const std::string& executablePath);
}

#endif //ORTHO_SFM_FULLPIPELINETESTS_H