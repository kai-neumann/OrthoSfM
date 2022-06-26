/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#ifndef ORTHO_SFM_ORTHOQUATERNIONRECOALGORITM_H
#define ORTHO_SFM_ORTHOQUATERNIONRECOALGORITM_H

#include <algorithms/ReconstructionAlgorithm.h>
#include "OrthoQuaternionCamera.h"

namespace orthosfm
{
	class OrthoQuaternionReconstructionAlgorithm : public orthosfm::ReconstructionAlgorithm
	{
	 public:
		// Set the solver type
		bool setSolverType(SFM_SOLVER_TYPE solver) override;

		std::vector<std::shared_ptr<Camera>>
		calculateInitialAlignment(const ViewGroup& group, const std::vector<View>& views, const std::vector<Track>& tracks, const std::vector<
			std::shared_ptr<Camera>>& globalCameras) override;

		void
		triangulateTracks(const std::vector<std::shared_ptr<Camera>>& cameras, std::vector<Track>& tracks, bool resetExistingData) override;

		// Setup the parameters for optimization
		void
		SetupParameterBlocks(const std::vector<std::shared_ptr<Camera>>& cameras, ceres::Problem& problem) override;

		// Add a resiudal block to the optimization problem
		void
		AddResidualBlock(const std::shared_ptr<Camera>& camera, const Feature& observation, double* point4d, ceres::Problem& problem, ceres::LossFunction* lossFunction) override;

		// Place a specific camera at the origin
		void
		normalizeSceneToCamera(std::vector<std::shared_ptr<Camera>>& cameras, const std::shared_ptr<Camera>& target) const override;

		void normalizeScene(std::vector<std::shared_ptr<Camera>>& cameras) override;

		void alignToGlobalCameras(std::vector<std::shared_ptr<Camera>>& localCameras, const std::vector<std::shared_ptr<
			Camera>>& globalCameras) override;

		// Evaluate the reprojection error for a specific camera, feature and point
		double
		evaluateReprojectionError(const std::shared_ptr<Camera>& camera, const Feature& observation, double* point3d) override;

		std::vector<std::shared_ptr<Camera>>
		copyCameraArray(const std::vector<std::shared_ptr<Camera>>& cameras) const override;

		int getGroupSize() override
		{
			return 3;
		};

		std::string getName() override;
	};
}

#endif //ORTHO_SFM_ORTHOQUATERNIONRECOALGORITM_H