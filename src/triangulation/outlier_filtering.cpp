/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#include "outlier_filtering.h"
#include <data_structures/colored_point_cloud.h>
#include <util/common.h>

// Get the nearest neighbour distance of each point
std::vector<std::pair<double, int>> orthosfm::getNearestNeighbourDistance(const std::vector<std::pair<Eigen::Vector4d, int>>& points) {
    std::vector<std::pair<double, int>> minDistance;
    minDistance.resize(points.size());

    // Calculate distance from each point to each other point
    // Also store the minimum distance of each point to any other point
    #pragma omp parallel for schedule(dynamic,1)
    for (int i = 0; i < points.size(); i++) {
        double minDist = 1000000;

        for (int j = 0; j < points.size(); j++) {
            if (i != j) {
                double dist = (points[i].first - points[j].first).norm();

                if (dist < minDist) {
                    minDist = dist;
                }
            }
        }

        minDistance[i] = std::make_pair(minDist, points[i].second);
    }

    return minDistance;
}

std::vector<orthosfm::Track> orthosfm::filterOutlierTracks(const std::vector<Track> &tracks, const std::vector<std::shared_ptr<Camera>>& cameras) {
    std::cout << "Removing outlier tracks." << std::endl;

    std::vector<Track> outTracks;

    // Create a reduced array of points (for faster distance calculation)
    std::vector<std::pair<Eigen::Vector4d, int>> reduced;
    for(int trackID=0; trackID<tracks.size(); trackID++) {
        if(!tracks[trackID].hasPoint()) {
            continue;
        }
        reduced.emplace_back(tracks[trackID].getPoint(), trackID);
    }

    // Calculate the distance of each point to the nearest other point
    std::vector<std::pair<double, int>> distancePairs = orthosfm::getNearestNeighbourDistance(reduced);

    // Put into full double vector
    std::vector<double> distances;
    distances.resize(tracks.size());
    for(const auto& pair : distancePairs) {
        distances[pair.second] = pair.first;
    }

    // First calculate the mean
    double sum = 0;
    int counter = 0;
    for(int trackID=0; trackID<tracks.size(); trackID++) {
        // Do not include it in the statistics that have not been computed
        if(!tracks[trackID].hasPoint()) {
            continue;
        }

        // Else: Add to mean sum
        sum += distances[trackID];
        counter++;
    }

    // Calculate the mean
    double mean = sum / ((double) counter);

    // Then calculate the standard deviation
    double squaredSum = 0;
    for(int trackID=0; trackID<tracks.size(); trackID++) {
        // Do not include it in the statistics that have not been computed
        if(!tracks[trackID].hasPoint()) {
            continue;
        }

        // Else: Add to squared sum
        squaredSum += pow(distances[trackID] - mean, 2);
        counter++;
    }

    double sigma = sqrt(squaredSum / ((double) counter));

    // Use a minimum value for sigma to also support "noise free" images
    sigma = fmax(sigma, 1e-3);

    // Then filter the tracks based on a sigma environment
    double sigmaThreshold = 1.6;

    for(int trackID=0; trackID<tracks.size(); trackID++) {
        // If the point is zero (because it could not be calculated) -> We always want to keep it
        if(!tracks[trackID].hasPoint()) {
            outTracks.push_back(tracks[trackID]);
            continue;
        }

        // If the point is outside a reasonable bounding box of 10 units: Skip it
        if(tracks[trackID].getPoint().norm() > 10) {
            continue;
        }

        // If the point is inside the sigma environment: Also keep it
        if(distances[trackID] < mean + sigmaThreshold * sigma) {
            outTracks.push_back(tracks[trackID]);
        }
    }

    // Print how many tracks were discarded
    std::cout << tracks.size() - outTracks.size() << " outlier tracks discarded (" << outTracks.size() << " tracks remaining)" << std::endl;


    return outTracks;
}

std::vector<orthosfm::Track> orthosfm::filterTracksWithReprojectionError(const std::vector<orthosfm::Track> &tracks,
		const std::vector<std::shared_ptr<orthosfm::Camera>> &cameras, const std::shared_ptr<orthosfm::ReconstructionAlgorithm>& algorithm) {

    // Filter tracks to only include full sized tracks (all other tracks are kept)
    std::vector<Track> fullSizeTracks = orthosfm::filterTracksToAvailableCameras(cameras, tracks, true, true);

    // Triangulate the full size tracks
    algorithm->triangulateTracks(cameras, fullSizeTracks, true);

    // Build a list of filtered tracks
    std::vector<Track> filteredTracks;

    // The threshold in pixels at which tracks should be regarded as inliers / outliers
    double maxAllowedReprojectionError = 1.5;

    // Loop over all tracks
    for(const auto& track : tracks) {
        // If the track is inside fullSizeTracks
        auto search = std::find(fullSizeTracks.begin(), fullSizeTracks.end(), track);
        if(search != fullSizeTracks.end()) {
            // Create a new track and incrementally fill it with features, if they are valid
            Track newTrack;
            if(track.hasPoint()) {
                newTrack.setPoint(track.getPoint());
            }

            // Calculate the reprojection error for each Feature
            for(int featureID=0; featureID<track.size(); featureID++) {
                // Get the camera for that feature
                bool cameraFound = false;
                for(const auto& cam : cameras) {
                    if(cam->getView()->getID() == track.get(featureID).viewID) {
                        // Calculate the reprojection error
                        double localError = algorithm->evaluateReprojectionError(cam, track.get(featureID), search->getPoint().data());

                        // if the error is smaller than the threshold: Add the feature to the output
                        if(localError < maxAllowedReprojectionError) {
                            newTrack.add(track.get(featureID));
                        }

                        cameraFound = true;
                        break;
                    }
                }

                // If the camera was not found: Keep the feature anyway, as not judgement regarding its validity can be made
                if(!cameraFound) {
                    newTrack.add(track.get(featureID));
                }
            }

            // If there are at least two valid features in the filtered track: Add it to the output
            if(newTrack.size() > 1) {
                filteredTracks.push_back(newTrack);
            }
        }
        // Else: Simply add it to the output
        else {
            filteredTracks.push_back(track);
        }
    }

    std::cout << filteredTracks.size() << " out of " << tracks.size() << " tracks remaining after outlier filtering" << std::endl;

    return filteredTracks;
}
