/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#include "matching.h"


#include <opencv2/opencv.hpp>
#include <chrono>
#include <omp.h>

#include <algorithms/ReconstructionAlgorithm.h>
#include <cuda_sift/cudaSift.h>

//#ifdef ENABLE_SIFT

// Define the function here, because CudaSift does not supply a header for geomFuncs.cpp in which this is implemented
// TODO: Create header
int ImproveHomography(SiftData &data, float *homography, int numLoops, float minScore, float maxAmbiguity, float thresh);

void orthosfm::extractFeaturesForAllImages(std::vector<View> &views) {
    std::cout << "\nRunning feature extraction" << std::endl;

#ifdef ENABLE_CUDA
    // Skip if empty
    if(views.empty()) {
        return;
    }

    // Check if all images have the same dimensions. If yes, this allows us to use shared allocated memory for feature extraction
    int referenceWidth = views[0].getWidth();
    int referenceHeight = views[0].getHeight();
    bool sharedDimensions = true;

    // Compare to reference
    for(auto & view : views) {
        if(view.getWidth() != referenceWidth || view.getHeight() != referenceHeight) {
            sharedDimensions = false;
            break;
        }
    }

    // Settings
    int octaves = 7;
    float initBlur = 1.5f;
    float thresh = 2.0; // Setting this lower drastically increases the amount of features
    int deviceNumber = 0; // Which GPU to use
    bool upscale = true;

    // Allocate temporary memory for feature extraction
    float *memoryTmp;
    if(sharedDimensions) {
        memoryTmp = AllocSiftTempMemory(referenceWidth, referenceHeight, octaves, upscale);
    }

    // Initialize cuda
    InitCuda(deviceNumber);

    // Run feature extraction for each image
    for(auto & view : views) {
        // Allocate memory temp if necessary
        if(!sharedDimensions) {
            memoryTmp = AllocSiftTempMemory(referenceWidth, referenceHeight, octaves, upscale);
        }

        // Allocate memory for the extracted features
        InitSiftData(view.getSiftData(), MAX_FEATURES, true, true);

        // Convert the image to greyscale
        cv::Mat gray;
        cv::cvtColor(view.getPixels(), gray, cv::COLOR_BGR2GRAY);

        // Convert the image to 32 bit float data
        cv::Mat img_float;
        gray.convertTo(img_float, CV_32FC1);

        CudaImage img;
        img.Allocate(view.getWidth(), view.getHeight(), iAlignUp(view.getWidth(), 128), false, NULL, (float*)img_float.data);
        img.Download();

        ExtractSift(view.getSiftData(), img, octaves, initBlur, thresh, 0.0f, upscale, memoryTmp);

        // Print number of features
        std::cout << view.getDisplayName() << ": Extracted " << view.getSiftData().numPts << " keypoints" << std::endl;

        // Deallocate if necessary
        if(!sharedDimensions) {
            FreeSiftTempMemory(memoryTmp);
        }
    }

    // Deallocate if necessary
    if(sharedDimensions) {
        FreeSiftTempMemory(memoryTmp);
    }
#endif
}

std::vector<orthosfm::Track> orthosfm::runExhaustivePairwiseMatching(std::vector<View> &views, bool refineWithHomography) {
#ifdef ENABLE_CUDA

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    // Put possible combinations into a vector (for better parallelization through openmp)
    std::vector<std::pair<int, int>> pairs;
    for(int i=0; i<views.size(); i++) {
        for (int j = i + 1; j < views.size(); j++) {
            pairs.emplace_back(i, j);
        }
    }

    // Init track list
    std::vector<Track> tracks;

    // Keep track how many pairs have been matched
    int done = 0;

    // For each combination of views
    #pragma omp parallel for
    for(int i=0; i<pairs.size(); i++) {
        //std::cout << "Matching " << i << " and " << j << std::endl;
        std::vector<Track> localTracks = calculatePairwiseMatchesThreadSafe(views[pairs[i].first], views[pairs[i].second], refineWithHomography);

        // Push local tracks into global track list
        #pragma omp critical
        {
            for(const auto & localTrack : localTracks) {
                tracks.push_back(localTrack);
            }

            done++;
            if(done % 25 == 0) {
                std::cout << round((((double) done) / ((double) pairs.size())) * 1000) / 10.0 << "% Matched. (" << done
                          << " out of " << pairs.size() << " pairs)" << std::endl;
            }
        };
    }

    std::cout << "Found " << tracks.size() << " pairwise tracks" << std::endl;

    // Free all sift data
    for(int i=0; i<views.size(); i++) {
        FreeSiftData(views[i].getSiftData());
    }

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Matching took = " << std::chrono::duration_cast<std::chrono::milliseconds> (end - begin).count() << " ms" << std::endl;

    return tracks;
#else
	return {};
#endif
}

std::vector<orthosfm::Track> orthosfm::calculatePairwiseMatchesThreadSafe(const View &view1, const View &view2, bool refineWithHomography) {
#ifdef ENABLE_CUDA

    // Create a copy of both SiftData Objects so they can be modified and filled with matching data
    SiftData siftData1, siftData2;

    // Initialize datastructures with the correct size
    InitSiftData(siftData1, view1.getSiftData().maxPts, true, true);
    InitSiftData(siftData2, view2.getSiftData().maxPts, true, true);

    CopySiftData(view1.getSiftData(), siftData1, true, true);
    CopySiftData(view2.getSiftData(), siftData2, true, true);

    MatchSiftData(siftData1, siftData2);
    float homography[9];
    int numMatches;

    // Initialize with a default value, that lets the features pass if refineWithHomography is disabled
    int inlierCount = 100;


    // TODO: Make parameters configurable
    // maxAmbiguity: [0.0 - 1.0] -> More features, the higher the value
    // thresh: Upper threshold for matching error. The higher it is set, the more values come through
    double homography_thresh = 30;

    if(refineWithHomography) {
        FindHomography(siftData1, homography, &numMatches, 10000, 0.0f, 0.99f, 60.0);
        inlierCount = ImproveHomography(siftData1, homography, 50, 0.0f, 0.975f, homography_thresh);
    }

    //std::cout << "Num matches: " << numMatches << " (" << inlierCount << " inliers)" << std::endl;

    //std::cout << "Matching: " << view1.getID() << " and " << view2.getID() << " returned " << numMatches << " matches and " << inlierCount << " inliers!" << std::endl;

    // Init the output tracks
    std::vector<Track> tracks;

    // Only put matches into track data structure when there are at least 50 inliers
    // TODO make limit configurable
    if(inlierCount > 50) {
        // Convert all matches to tracks. For this first loop over all points in siftData1
        for (int i = 0; i < siftData1.numPts; i++) {
            // If the error of a point is smaller than homography_thresh, this is an inlier (see implementation of ImproveHomography)
            if (siftData1.h_data[i].match_error < homography_thresh) {
                // Create a track object
                Track track;

                track.add(Feature(view1.getID(), i, view1.getID()*MAX_FEATURES + i , siftData1.h_data[i].xpos, siftData1.h_data[i].ypos, view1.getPixels()));
                track.add(Feature(view2.getID(),  siftData1.h_data[i].match, view2.getID()*MAX_FEATURES + siftData1.h_data[i].match, siftData1.h_data[i].match_xpos,
                                  siftData1.h_data[i].match_ypos, view2.getPixels()));

                // Add to list
                tracks.push_back(track);
            }
        }
    }

    for(int i=0; i<tracks.size(); i++) {
        //std::cout << tracks[i] << std::endl;
    }

    // Clean up memory after use
    FreeSiftData(siftData1);
    FreeSiftData(siftData2);

    // Return the output tracks
    return tracks;
#else
	return {};
#endif

}

std::vector<orthosfm::Track> orthosfm::mergePairwiseTracks(const std::vector<Track>& pairwiseTracks) {
    std::cout << "Merging pairwise tracks" << std::endl;

    // Create a lookup table that stores which global feature ID has already been used in which track
    std::unordered_map<unsigned int, unsigned int> featureToTrack;

    // Merge pairwise tracks
    std::vector<Track> mergedTracks;
    // Add the tracks one by one
    for(int i=0; i<pairwiseTracks.size(); i++) {
        if(i%2500 == 0) {
            std::cout << round((((double) i) / ((double) pairwiseTracks.size())) * 1000) / 10.0 << "% Merged. (" << i << " out of " << pairwiseTracks.size() << " tracks)" << std::endl;
        }

        // Check if the track contains exactly two features
        if(pairwiseTracks[i].size() != 2) {
            throw std::runtime_error("Invalid pairwise track!");
        }

        // Check if there already exists a track with the same features in the merged tracks
        auto searchResult = featureToTrack.find(pairwiseTracks[i].get(0).globalFeatureID);
        if(searchResult != featureToTrack.end()) {
            // If this contains ONLY the first feature of the pairwise track and there is no feature of this camera present in the track: Add the second one
            if(mergedTracks[searchResult->second].contains(pairwiseTracks[i].get(0)) && !mergedTracks[searchResult->second].contains(pairwiseTracks[i].get(1)) && !mergedTracks[searchResult->second].containsCamera(pairwiseTracks[i].get(1).viewID)) {
                // Add feature to track
                mergedTracks[searchResult->second].add(pairwiseTracks[i].get(1));

                // Also add to hashmap
                featureToTrack.insert(std::make_pair(pairwiseTracks[i].get(1).globalFeatureID, searchResult->second));
            }
            // Else: if this contains ONLY the second feature of the pairwise track and there is no feature of this camera present in the track: Add the first one
            else if(mergedTracks[searchResult->second].contains(pairwiseTracks[i].get(1)) && !mergedTracks[searchResult->second].contains(pairwiseTracks[i].get(0)) && !mergedTracks[searchResult->second].containsCamera(pairwiseTracks[i].get(0).viewID)) {
                // Add feature to track
                mergedTracks[searchResult->second].add(pairwiseTracks[i].get(0));

                // Also add to hashmap
                featureToTrack.insert(std::make_pair(pairwiseTracks[i].get(0).globalFeatureID, searchResult->second));
            }
        }
        // If there is no existing track: Add it to the merged track list
        else {
            mergedTracks.push_back(pairwiseTracks[i]);
            featureToTrack.insert(std::make_pair(pairwiseTracks[i].get(0).globalFeatureID, mergedTracks.size() - 1));
            featureToTrack.insert(std::make_pair(pairwiseTracks[i].get(1).globalFeatureID, mergedTracks.size() - 1));
        }
    }

    std::cout << mergedTracks.size() << " merged tracks remaining" << std::endl;

    return mergedTracks;
}

void orthosfm::visualizeMatches(const std::vector<Track> &tracks, const View &view1, const View &view2) {
    cv::Mat stacked;
    cv::hconcat(view1.getPixels(), view2.getPixels(), stacked);

    // Count number of matches
    int matchesCount = 0;

    // Loop over all tracks
    for(int i=0; i<tracks.size(); i++) {
        // Check if the track contains both views
        if(tracks[i].containsCamera(view1.getID()) && tracks[i].containsCamera(view2.getID())) {
            // Get the feature that contains the first (second) camera
            const Feature* f1;
            const Feature* f2;
            for(int j=0; j<tracks[i].size(); j++) {
                if(tracks[i].get(j).viewID == view1.getID()) {
                    f1 = &tracks[i].get(j);
                }
                if(tracks[i].get(j).viewID == view2.getID()) {
                    f2 = &tracks[i].get(j);
                }
            }

            cv::Scalar color = cv::Scalar( rand() % 255, rand() % 255, rand() % 255 );
            //cv::Scalar color = cv::Scalar( f1->b, f1->g, f1->r);

            // Draw lines for matches
            cv::line(stacked, cv::Point(f1->x, f1->y), cv::Point(view1.getWidth()+f2->x, f2->y), color, 1, cv::LINE_8);

            matchesCount++;
        }
    }

    std::cout << "Visualizing " << matchesCount << " matches" << std::endl;

    // Display
    cv::imshow("Matches", stacked);
    cv::waitKey(0);
}

std::vector<orthosfm::Track> orthosfm::filterTracksWithMasks(const std::vector<Track> &pairwiseTracks, const std::vector<View> &views) {
    // Check if any of the views has a provided mask
    bool masksAvailable = false;
    for(int i=0; i<views.size(); i++) {
        if(views[i].hasMask()) {
            masksAvailable = true;
            break;
        }
    }

    // If there are no masks available: Return unfiltered tracks
    if(!masksAvailable) {
        std::cout << "No masks available. Continuing without masking." << std::endl;
        return pairwiseTracks;
    }

    // Initialize the output structure
    std::vector<Track> filtered;
    // Loop over all tracks
    for(int i=0; i<pairwiseTracks.size(); i++) {
        // Flag if the track is valid
        bool valid = true;

        // Loop over all features of this track
        for(int j=0; j<pairwiseTracks[i].size(); j++) {
            // Check if the image corresponding to that feature has a mask
            if(views[pairwiseTracks[i].get(j).viewID].hasMask()) {
                // Check if the point should be masked out
                if(!views[pairwiseTracks[i].get(j).viewID].isPixelMaskedIn(pairwiseTracks[i].get(j).x, pairwiseTracks[i].get(j).y)) {
                    valid = false;
                    break;
                }
            }
        }

        if(valid) {
            filtered.push_back(pairwiseTracks[i]);
        }
    }

    std::cout << filtered.size() << " tracks out of " << pairwiseTracks.size() << " remaining after filtering" << std::endl;

    return filtered;
}

std::vector<orthosfm::Track> orthosfm::filterDuplicateTracks(const std::vector<Track> &pairwiseTracks) {
    // FIXME: Reimplement using custom hashing strategy
    std::cout << "Filtering for duplicated tracks.." << std::endl;
    return pairwiseTracks;

    /*std::vector<Track> filtered;

    // Build a database of previously added points (as pairs of the point position and their trackID)
    hnswlib::L2Space space(2);
    hnswlib::AlgorithmInterface<float>* algorithm = new hnswlib::HierarchicalNSW<float>(&space, pairwiseTracks.size());

    // Define a minimum distance two points can have (in pixels)
    double minimumDistance = 1;

    // For each track
    for(const auto& track : pairwiseTracks) {
        // Check if the first point has a duplicate
        const auto& firstSearchResult = algorithm->searchKnnCloserFirst(track.getFeaturePosition(0).data(), 1);

        // If the search is empty: We can directly add the point (only happens in the first iteration)
        if(firstSearchResult.empty()) {
            filtered.push_back(track);

            // Add both points to the search tree
            algorithm->addPoint(track.getFeaturePosition(0).data(), filtered.size()-1);
            algorithm->addPoint(track.getFeaturePosition(1).data(), filtered.size()-1);

            continue;
        }

        // If the distance to the first point is too similar
        if(firstSearchResult[0].first < minimumDistance) {
            // Check if the second point is also too similar
            if((track.getFeaturePosition(1) - filtered[firstSearchResult[0].second].getFeaturePosition(0)).norm() < minimumDistance) {
                continue;
            }
            else if((track.getFeaturePosition(1) - filtered[firstSearchResult[0].second].getFeaturePosition(1)).norm() < minimumDistance) {
                continue;
            }
        }

        // If we did not find a duplicated track based on the first point: Repeat the previous steps with the second point
        const auto& secondSearchResult = algorithm->searchKnnCloserFirst(track.getFeaturePosition(1).data(), 1);
        if(secondSearchResult[0].first < minimumDistance) {
            // Check if the first point is also too similar
            if((track.getFeaturePosition(0) - filtered[secondSearchResult[0].second].getFeaturePosition(0)).norm() < minimumDistance) {
                continue;
            }
            else if((track.getFeaturePosition(0) - filtered[secondSearchResult[0].second].getFeaturePosition(1)).norm() < minimumDistance) {
                continue;
            }
        }

        // If no duplicate track was found until now -> Add this track to the output
        filtered.push_back(track);

        // Add both points to the search tree
        algorithm->addPoint(track.getFeaturePosition(0).data(), filtered.size()-1);
        algorithm->addPoint(track.getFeaturePosition(1).data(), filtered.size()-1);
    }

    // Print
    std::cout << filtered.size() << " unique tracks out of " << pairwiseTracks.size() << " tracks remaining after filtering." << std::endl;

    // Return the filtered tracks
    return filtered;*/
}

//#endif
