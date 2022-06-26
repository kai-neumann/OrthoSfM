/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#include "reconstruct.h"

#include <omp.h>
#include "opencv2/opencv.hpp"

#include <matching/matching_io.h>
#include <matching/matching_mve.h>
#include <bundle_adjustment/bundle_adjustment.h>
#include <algorithms/ReconstructionAlgorithm.h>
#include <algorithms/orthographic/OrthographicReconstructionAlgorithm.h>
#include <algorithms/orthographic_quaternion/OrthoQuaternionRecoAlgorithm.h>
#include <data_structures/group.h>
#include <data_structures/Camera.h>
#include <data_structures/camera_io.h>
#include <triangulation/outlier_filtering.h>
#include <util/timing.h>
#include <matching/matching.h>

// Only include code for matching calls if matching is enabled inside CMake
#ifdef ENABLE_SIFT
#include "matching.h"
#endif

std::vector<std::shared_ptr<orthosfm::Camera>> orthosfm::reconstruct(const reconstruction_config &config) {
    // Start the timer
    auto startAll = std::chrono::steady_clock::now();

    // First load all images inside the image folder into view objects
    std::vector<View> views;
    {
        // Get all image files inside the specified image folder
        std::vector<std::string> images = getImagesInFolder(config.imageFolder);

        // Convert them to views
        for(int i=0; i<images.size(); i++) {
            views.emplace_back(i, images[i]);
        }

        // If there is a mask folder specified
        if(!config.maskFolder.empty()) {
            // For each view: Find and assign a mask with the pattern "{imageName}_mask.png"
            for(auto & view : views) {
                view.findCorrespondingMask(config.maskFolder);
            }
        }

        // Asynchronously load all pixel data (Only in release mode. Causes problems in debug builds)
        #pragma omp parallel for
        for(int i=0; i<views.size(); i++) {
            views[i].loadPixelData(config.downscaleFactor);
        }

        std::cout << "Initialized project with " << views.size() << " views" << std::endl;
    }

    auto endInit = std::chrono::steady_clock::now();

    // Match all images and put the matches into Track objects
    std::vector<Track> tracks;
    {
        // If there is a track File specified: Try to load the track data from that file
        if(!config.trackFile.empty()) {
            std::cout << "Loading tracks from " << config.trackFile << std::endl;

            // Load
            loadTracksFromFile(tracks, config.trackFile);

            // Print how many tracks of what length we have
            printTrackOverview(tracks);
        }
        else {
            if(config.useMveForMatching) {
                // Calculate Tracks using MVE
                tracks = calculateTracksUsingMVE(views, config.projectFolder, config.downscaleFactor);

                // Filter the tracks using masks
                tracks = filterTracksWithMasks(tracks, views);

                // Propagate pixel colors to the tracks
                propagateColorsToTracks(tracks, views);
            }
            else {
#ifdef ENABLE_SIFT
                // First extract all features
                extractFeaturesForAllImages(views);

                // Then match them pairwise
                std::vector<Track> pairwiseTracks = runExhaustivePairwiseMatching(views, true);
                //std::vector<Track> pairwiseTracks = runCustomExhaustivePairwiseMatching(views);

                // Remove all tracks that are too similar to any other tracks
                //std::vector<Track> uniqueTracks = filterDuplicateTracks(pairwiseTracks);
                std::vector<Track> uniqueTracks = pairwiseTracks;

                // Remove any tracks, which contains features that are masked out. This is done before merging, to provide a
                // more stable base for merged tracks
                std::vector<Track> filteredTracks = filterTracksWithMasks(uniqueTracks, views);

                // Merge pairwise tracks that contain the same features
                tracks = mergePairwiseTracks(filteredTracks);
#else
                std::cerr << "Failed to build tracks through feature matching, because SIFT was disabled in cmake." << std::endl;
#endif
            }

            // Visualize two matches (For debug purposes)
            /*for(int i=0; i<views.size(); i++) {
                for(int j=i+1; j<views.size(); j++) {
                    visualizeMatches(tracks, views[i], views[j]);
                }
            }*/

            // Print how many tracks of what length we have
            printTrackOverview(tracks);

            // Save to disk
            saveTracksToFile(tracks, config.projectFolder + "/tracks.txt");
        }
    }

    auto endTrack = std::chrono::steady_clock::now();

    if(config.exportPairwiseTracks) {
        saveTracksToPairwiseFiles(tracks, views, config.projectFolder);
    }

    // Initialize a ReconstructionAlgorithm object (for the correct backend)
    std::shared_ptr<ReconstructionAlgorithm> algorithm;
    if(config.solver == SFM_SOLVER_TYPE::ORTHO_QUATERNION) {
        algorithm = std::make_shared<OrthoQuaternionReconstructionAlgorithm>();
    }
    // Angle based representations
    else {
        algorithm = std::make_shared<OrthographicReconstructionAlgorithm>();
    }
    // Apply the solver type
    if(!algorithm->setSolverType(config.solver)) {
        std::cout << "Error: Failed to initialize solver!" << std::endl;
        return std::vector<std::shared_ptr<Camera>>();
    }

    // Timer
    auto startPose = std::chrono::steady_clock::now();

    // Execute the pose estimation
    std::vector<std::shared_ptr<Camera>> cameras = runPoseEstimation(views, algorithm, tracks, config);

    // timer
    auto endPose = std::chrono::steady_clock::now();

    // Export the point cloud
	orthosfm::savePointsToPLY(config.projectFolder + "/sparse_cloud.ply", cameras, tracks, algorithm);

    // Export time measurements
    std::chrono::steady_clock::time_point endAll = std::chrono::steady_clock::now();
    double elapsedInit = elapsedSeconds(startAll, endInit);
    double elapsedTrack = elapsedSeconds(endInit, endTrack);
    double elapsedPose = elapsedSeconds(startPose, endPose);
    double elapsedAll = elapsedSeconds(startAll, endAll);
    saveRuntimesToTxt(config.projectFolder + "/time_measurements.txt", elapsedInit, elapsedTrack, elapsedPose, elapsedAll);

    // Return
    return cameras;
}

std::vector<std::shared_ptr<orthosfm::Camera>> orthosfm::runPoseEstimation(const std::vector<orthosfm::View> &views,
	const std::shared_ptr<orthosfm::ReconstructionAlgorithm> &algorithm, std::vector<orthosfm::Track>& tracks, const orthosfm::reconstruction_config& config) {
    // Initialize the list of aligned cameras
    std::vector<std::shared_ptr<orthosfm::Camera>> alignedCameras;

    // Determine an ordered list of groups in which to add the views (always starting with view 0)
    std::deque<ViewGroup> groups;
    {
        buildGroups(groups, views, tracks, algorithm->getGroupSize());
    }

    bool optimizePoints = true;
    int globalBundleAdjustmentInterval = 3; // Use 1 to always run BA

    // Count the number or processed groups
    int processedGroups = 0;
    int groupCount = groups.size();

    // Iteratively add all other groups one by one
    while (!groups.empty()) {
        processedGroups++;
        std::cout << std::endl << "===== Reconstructing group " << groups.front().toString() << " (" << processedGroups << "/" <<  groupCount <<") =====" << std::endl;

        // Get the next group
        ViewGroup group = groups.front();
        groups.pop_front();

        // Create a copy of the tracks for local alignment
        std::vector<Track> localTracks = tracks;

        // Calculate the initial local alignment of all cameras that are part of the group
        std::vector<std::shared_ptr<Camera>> localCameras = algorithm->calculateInitialAlignment(group, views, localTracks, alignedCameras);

        // Print the local alignment
        std::cout << "\nCalculated local alignment:" << std::endl;
		printCameras(localCameras, false);

        // Filter the tracks based on their reprojection error to remove outliers that would negatively affect the bundle adjustment
        localTracks = filterTracksWithReprojectionError(localTracks, localCameras, algorithm);

        // If this is the first group: Fix the first camera for bundle adjustment
        if(alignedCameras.empty()) localCameras[0]->setFixed(true);

        // Run bundle adjustment on the local alignment. Separately triangulate the tracks for local bundle adjustment.
        std::cout << "Running local bundle adjustment" << std::endl;
        runBundleAdjustment(localCameras, localTracks, algorithm, true, true);

        // Print the local alignment
        std::cout << "\nOptimized local alignment:" << std::endl;
        printCameras(localCameras, false);

        // if this is the first group that is reconstructed: Add all three cameras
        if(alignedCameras.empty()) {
            // Normalize the coordinate system
            algorithm->normalizeScene(localCameras);

            // Add local cameras to global cameras
            for(const auto& cam : localCameras) {
                alignedCameras.push_back(cam);
            }

            // Triangulate 3d points of the tracks used for the first three cameras
            algorithm->triangulateTracks(alignedCameras, tracks, true);
        }
            // Else: Align the new cameras to the previous cameras and run global bundle adjustment
        else {
            // Transform the local alignment into the global coordinate system
            algorithm->alignToGlobalCameras(localCameras, alignedCameras);

            // Print
            std::cout << "Aligned local cameras to scene: " << std::endl;
            printCameras(localCameras, false);

            // Add new camera(s) to the alignment
            algorithm->mergeIntoGlobal(localCameras, alignedCameras);

            // Triangulate 3d points of NEW tracks
            algorithm->triangulateTracks(alignedCameras, tracks, true);

            // Run global BA every few steps
            if(processedGroups % globalBundleAdjustmentInterval == 0) {
                std::cout << "\nCameras before global bundle Adjustment:" << std::endl;
                printCameras(alignedCameras, true);

                // Run global bundle adjustment on even and then on odd cameras
                std::cout << "Running global bundle adjustment" << std::endl;
                //runEvenOddBundleAdjustment(alignedCameras, tracks, algorithm, optimizePoints, false, false);
                runBundleAdjustment(alignedCameras, tracks, algorithm, optimizePoints, false);

                // Filter outlier tracks, so they are not used in the next iteration
                tracks = filterOutlierTracks(tracks, alignedCameras);
                tracks = filterTracksWithReprojectionError(tracks, alignedCameras, algorithm);
            }

            // Normalize the scene
            algorithm->normalizeScene(alignedCameras);

            // Print the current alignment
            std::cout << "\nCurrent Cameras:" << std::endl;
            printCameras(alignedCameras, true);
        }
    }

    std::cout << "Finished iterative alignment. Running final bundle adjustment" << std::endl;

    // Run bundle adjustment one last time (so the outlier filtering in the last iteration also has an effect)
    //runEvenOddBundleAdjustment(alignedCameras, tracks, algorithm, optimizePoints, false, false);
    runBundleAdjustment(alignedCameras, tracks, algorithm, optimizePoints, false);
    algorithm->normalizeScene(alignedCameras);

    // Print the cameras one more time
    std::cout << "\nFinal Alignment:" << std::endl;
    printCameras(alignedCameras, true);

    // Export the alignment in the project folder
    if(!config.projectFolder.empty()) {
        exportCamerasToFile(alignedCameras, config.projectFolder + "/cameras.txt");
    }


    return alignedCameras;
}
