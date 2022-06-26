/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#ifndef ORTHO_SFM_ORTHOGRAPHICRECONSTRUCTIONALGORITHM_H
#define ORTHO_SFM_ORTHOGRAPHICRECONSTRUCTIONALGORITHM_H

#include <algorithms/ReconstructionAlgorithm.h>
#include <data_structures/Camera.h>

namespace orthosfm
{
	class OrthographicReconstructionAlgorithm : public ReconstructionAlgorithm
	{
	 public:
		// Set the solver type
		bool setSolverType(SFM_SOLVER_TYPE solver) override;

		// Calculate rough relative poses for the given view group
		std::vector<std::shared_ptr<Camera>>
		calculateInitialAlignment(const ViewGroup& group, const std::vector<View>& views, const std::vector<Track>& tracks, const std::vector<
			std::shared_ptr<Camera>>& globalCameras) override;

		// Triangulate tracks
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

		// Place the first camera at the origin
		void normalizeScene(std::vector<std::shared_ptr<Camera>>& cameras) override;

		// Move / Rotate the local cameras to match their corresponding cameras in the global coordiante system
		void alignToGlobalCameras(std::vector<std::shared_ptr<Camera>>& localCameras, const std::vector<std::shared_ptr<
			Camera>>& globalCameras) override;

		// Evaluate the reprojection error for a specific camera, feature and point
		double
		evaluateReprojectionError(const std::shared_ptr<Camera>& camera, const Feature& observation, double* point3d) override;

		std::vector<std::shared_ptr<Camera>>
		copyCameraArray(const std::vector<std::shared_ptr<Camera>>& cameras) const override;

		// Define a custom group size
		int getGroupSize() override;

		std::string getName() override;

	 private:
		double m_degreesOfFreedom = -1;
	};
}


#endif //ORTHO_SFM_ORTHOGRAPHICRECONSTRUCTIONALGORITHM_H
