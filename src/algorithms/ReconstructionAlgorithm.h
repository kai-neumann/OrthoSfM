/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#ifndef ORTHO_SFM_RECONSTRUCTIONALGORITHM_H
#define ORTHO_SFM_RECONSTRUCTIONALGORITHM_H

#include <data_structures/track.h>
#include <data_structures/Camera.h>
#include <data_structures/group.h>
#include <vector>
#include <ceres/ceres.h>
#include <data_structures/colored_point_cloud.h>
#include <data_structures/solver_type.h>

namespace orthosfm
{
	class ReconstructionAlgorithm
	{
	 public:
		// Set the solver type
		virtual bool setSolverType(SFM_SOLVER_TYPE solver) = 0;

		// Calculate rough relative poses for the given view group
		virtual std::vector<std::shared_ptr<Camera>>
		calculateInitialAlignment(const ViewGroup& group, const std::vector<View>& views, const std::vector<Track>& tracks, const std::vector<
			std::shared_ptr<Camera>>& globalCameras) = 0;

		// Triangulate tracks
		virtual void
		triangulateTracks(const std::vector<std::shared_ptr<Camera>>& cameras, std::vector<Track>& tracks, bool resetExistingData) = 0;

		// Setup the parameters for optimization
		virtual void
		SetupParameterBlocks(const std::vector<std::shared_ptr<Camera>>& cameras, ceres::Problem& problem) = 0;

		// Add a resiudal block to the optimization problem
		virtual void
		AddResidualBlock(const std::shared_ptr<Camera>& camera, const Feature& observation, double* point4d, ceres::Problem& problem, ceres::LossFunction* lossFunction) = 0;

		// Place a specific camera at the origin
		virtual void
		normalizeSceneToCamera(std::vector<std::shared_ptr<Camera>>& cameras, const std::shared_ptr<Camera>& target) const = 0;

		// Place the first camera at the origin
		virtual void normalizeScene(std::vector<std::shared_ptr<Camera>>& cameras) = 0;

		// Move / Rotate the local cameras to match their corresponding cameras in the global coordiante system
		virtual void
		alignToGlobalCameras(std::vector<std::shared_ptr<Camera>>& localCameras, const std::vector<std::shared_ptr<
			Camera>>& globalCameras) = 0;

		// Evaluate the reprojection error for a specific camera, feature and point
		virtual double
		evaluateReprojectionError(const std::shared_ptr<Camera>& camera, const Feature& observation, double* point3d) = 0;

		// Add any new cameras into the global camera array
		void
		mergeIntoGlobal(const std::vector<std::shared_ptr<Camera>>& localCameras, std::vector<std::shared_ptr<Camera>>& globalCameras);

		// Create a copy of the cameras using the correct class
		virtual std::vector<std::shared_ptr<Camera>>
		copyCameraArray(const std::vector<std::shared_ptr<Camera>>& cameras) const = 0;

		// Define a custom group size
		virtual int getGroupSize() = 0;

		// Get a name
		virtual std::string getName() = 0;
	};
}


#endif //ORTHO_SFM_RECONSTRUCTIONALGORITHM_H
