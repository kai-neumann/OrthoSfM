/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#include "bundle_adjustment.h"
#include <vector>
#include <set>
#include <map>

#include <Eigen/Dense>
#include <ceres/ceres.h>

#include <util/common.h>
#include <data_structures/colored_point_cloud.h>
#include <triangulation/outlier_filtering.h>


class TriangulationCallback : public ceres::IterationCallback
{
 public:
	explicit TriangulationCallback(std::vector<std::shared_ptr<orthosfm::Camera>>& cameras, std::vector<orthosfm::Track>& tracks, const std::shared_ptr<
		orthosfm::ReconstructionAlgorithm>& algorithm)
		:
		m_cameras(cameras), m_tracks(tracks), m_algorithm(algorithm)
	{
	}

	~TriangulationCallback()
	{
	}

	ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary)
	{
		m_algorithm->triangulateTracks(m_cameras, m_tracks, true);
		return ceres::SOLVER_CONTINUE;
	}

 private:
	std::vector<std::shared_ptr<orthosfm::Camera>>& m_cameras;
	std::vector<orthosfm::Track>& m_tracks;
	const std::shared_ptr<orthosfm::ReconstructionAlgorithm>& m_algorithm;
};


void orthosfm::runBundleAdjustment(std::vector<std::shared_ptr<Camera>>& cameras, std::vector<Track>& tracks,
                         const std::shared_ptr<ReconstructionAlgorithm>& algorithm,
                         const bool& optimizePoints, const bool& retriangulatePoints) {

    // Create a map for looking up cameras by their viewID
    std::map<unsigned int, std::shared_ptr<Camera>> viewIDToCamera;
    for(const auto& cam : cameras) {
        viewIDToCamera.insert(std::make_pair(cam->getView()->getID(), cam));
    }


    // Create an optimization problem
    ceres::Problem problem;

    // Create a loss function
    ceres::LossFunction* loss = new ceres::HuberLoss(1.0);
    //ceres::LossFunction* loss = new ceres::SoftLOneLoss(10);

    // Setup the parameter blocks (and local parametrization) of all cameras
    algorithm->SetupParameterBlocks(cameras, problem);

    // get a reference to the tracks that we want to optimized
    std::vector<Track>* tracksForOptimization = &tracks;

    // Make a filtered copy of the input tracks, so we do not modify the original tracks
    std::vector<Track> localTracks = filterTracksToAvailableCameras(cameras, tracks, false, false);

    // triangulate if necessary
    if(retriangulatePoints) {
        // Triangulate the tracks
        algorithm->triangulateTracks(cameras, localTracks, true);

        // Use the local tracks for optimization
        tracksForOptimization = &localTracks;
    }

    // Setup local parametrization of all points
    for (int i = 0; i < tracksForOptimization->size(); i++) {
        if(tracksForOptimization->at(i).hasPoint()) {
            problem.AddParameterBlock(tracksForOptimization->at(i).getPoint().data(), 4);
            problem.SetParameterization(tracksForOptimization->at(i).getPoint().data(),
                                        new ceres::HomogeneousVectorParameterization(4));

            // Set to constant if wished
            if (!optimizePoints) {
                problem.SetParameterBlockConstant(tracksForOptimization->at(i).getPoint().data());
            }
        }
    }

    // Make a backup copy of the tracks for later comparison purposes
    std::vector<Track> tracksBackup = *tracksForOptimization;

    // Loop over all (filtered) tracks
    for (int trackID = 0; trackID < tracksForOptimization->size(); trackID++) {
        // Skip if no point is available
        if(!tracksForOptimization->at(trackID).hasPoint()) {
            continue;
        }

        // Loop over all features of the track
        for(int featureID=0; featureID < tracksForOptimization->at(trackID).size(); featureID++) {
            // Check if the camera corresponding to that view ID is available
            if(viewIDToCamera.find(tracksForOptimization->at(trackID).get(featureID).viewID) == viewIDToCamera.end()) {
                // Skip if not available
                continue;
            }

            // Get the camera with the correct ID
            std::shared_ptr<Camera> cam = viewIDToCamera.at(tracksForOptimization->at(trackID).get(featureID).viewID);

            // Add The algorithm specific residual
            algorithm->AddResidualBlock(cam, tracksForOptimization->at(trackID).get(featureID), tracksForOptimization->at(trackID).getPoint().data(), problem, loss);
        }
    }

    // Set solver options
    ceres::Solver::Options options;
    options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.minimizer_progress_to_stdout = false;
    options.function_tolerance = 1e-6; // Default is 1e-6
    options.gradient_tolerance = 1e-10; // Default is 1e-10
    options.parameter_tolerance = 1e-10; // Default is 1e-10
    options.max_num_iterations = 100;

    // Retriangulate at every iteration
    /*if(retriangulatePoints) {
        options.update_state_every_iteration = true;
        options.callbacks.push_back(new TriangulationCallback(cameras, *tracksForOptimization, algorithm));
    }*/

    // Create a summary for outputting optimization information
    ceres::Solver::Summary summary;

    // Solve the problem
    ceres::Solve(options, &problem, &summary);

    // Output the summary
    std::cout << summary.BriefReport() << "\n";

    // Calculate how much the points have been moved
    double maxDistance = 0;
    double sum = 0;
    for(int i=0; i<tracksBackup.size(); i++) {
        double dist = (tracksBackup[i].getPoint() - tracksForOptimization->at(i).getPoint()).norm();
        if(dist > maxDistance) {
            maxDistance = dist;
        }
        sum += dist;
    }
    std::cout << "Average point change: " << sum / ((double) tracksBackup.size()) << " (maximum change: " << maxDistance << ")" << std::endl;
}

void orthosfm::runEvenOddBundleAdjustment(std::vector<std::shared_ptr<Camera>> &cameras, std::vector<Track> &tracks,
                                const std::shared_ptr<ReconstructionAlgorithm> &algorithm, const bool &optimizePoints,
                                const bool &retriangulatePoints, const bool& fixFirstTwoCameras) {

    // First store a backup, which cameras are currently fixed
    std::vector<bool> fixed;
    fixed.resize(cameras.size());
    for(int i=0; i<cameras.size(); i++) {
        fixed[i] = cameras[i]->isFixed();
    }

    // Fix all even cameras (0, 2, 4, 6, ..)
    for(int i=0; i<cameras.size(); i++) {
        cameras[i]->setFixed(i % 2 == 0 || (i < 2 && fixFirstTwoCameras));
    }
    // Run bundle adjustment
    runBundleAdjustment(cameras, tracks, algorithm, optimizePoints, retriangulatePoints);

    // Fix all odd cameras (1, 3, 5, 7, ..)
    for(int i=0; i<cameras.size(); i++) {
        cameras[i]->setFixed(i % 2 != 0 || (i < 2 && fixFirstTwoCameras));
    }
    // Run bundle adjustment
    runBundleAdjustment(cameras, tracks, algorithm, optimizePoints, retriangulatePoints);

    // Reset fixed state
    for(int i=0; i<cameras.size(); i++) {
        cameras[i]->setFixed(fixed[i]);
        if(i < 2 && fixFirstTwoCameras) {
            cameras[i]->setFixed(true);
        }
    }

    // Run bundle adjustment one more time
    runBundleAdjustment(cameras, tracks, algorithm, optimizePoints, retriangulatePoints);
}
