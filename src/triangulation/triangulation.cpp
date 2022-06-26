/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#include "triangulation.h"

Eigen::Vector3d orthosfm::intersectRays(const std::vector<Eigen::Vector3d>& origins, const std::vector<Eigen::Vector3d>& directions) {
    // Perform line intersection based on the equation inside Eric's answer at:
    // https://stackoverflow.com/questions/52088966/nearest-intersection-point-to-many-lines-in-python


    // Initialize the left and right side (System: R*p = q)
    Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
    Eigen::Vector3d q = Eigen::Vector3d::Zero();

    // Create a identity matrix (for reusing)
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

    // Sum over all directions and origins
    for (int i = 0; i < origins.size(); i++) {
        // Calculate v * vT
        Eigen::Matrix3d dirMat = (directions[i].normalized()) * (directions[i].normalized().transpose());

        // Calculate I - v*vT
        Eigen::Matrix3d diff = Eigen::Matrix3d::Identity() - dirMat;

        // Add to left side
        R += diff;

        // Add to right side
        q += diff * origins[i];
    }

    // Solve using svd
    Eigen::Vector3d point = R.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(q);

    return point;
}

void orthosfm::triangulateOrthographicTracks(const std::vector<std::shared_ptr<Camera>> &cameras, std::vector<Track> &tracks, bool resetExistingPoints) {
    // Create a lookup map that maps the view id to the local index inside the camera array
    std::map<unsigned int, int> cameraMap;
    for(int i=0; i<cameras.size(); i++) {
        cameraMap.insert(std::make_pair(cameras[i]->getView()->getID(), i));
    }

    // Loop over all tracks that should be triangulated
    #pragma omp parallel for schedule(dynamic,1)
    for (int trackID = 0; trackID < tracks.size(); trackID++) {
        // Assemble the directions and origins of all rays
        std::vector<Eigen::Vector3d> origins;
        std::vector<Eigen::Vector3d> directions;

        // Loop over all features of the track
        for (int featureID = 0; featureID < tracks[trackID].size(); featureID++) {
            // Check if the camera of the (view, feature) pair corresponds to a known camera
            unsigned int viewID = tracks[trackID].get(featureID).viewID;

            if (cameraMap.count(viewID) > 0) {

                // Get the pixel position of the corresponding feature
                float x = tracks[trackID].get(featureID).x;
                float y = tracks[trackID].get(featureID).y;

                // Get a shared pointer to the camera
                //std::shared_ptr<OrthographicCamera> camera = std::dynamic_pointer_cast<OrthographicCamera>(cameras[cameraMap.find(viewID)->second]);

                origins.push_back(cameras[cameraMap.find(viewID)->second]->getPointOnCameraPlane(x, y));
                directions.push_back(cameras[cameraMap.find(viewID)->second]->getLookDirection());
            }
        }

        // If there are two or more cameras for this track
        if(origins.size() > 1) {
            // If the track does not contain a point or all points should be retriangualted: Triangulate it based on the rays
            if(!tracks[trackID].hasPoint() || resetExistingPoints) {
                Eigen::Vector3d triangulatedPoint = intersectRays(origins, directions);

                // Convert to a homogenous point and assign to track
                Eigen::Vector4d homogenousPoint = Eigen::Vector4d(triangulatedPoint.x(), triangulatedPoint.y(), triangulatedPoint.z(), 1);
                tracks[trackID].setPoint(homogenousPoint);
            }
        }
        // If there are less than 2 valid rays: Reset the old point data
        else if(resetExistingPoints) {
            tracks[trackID].invalidatePoint();
        }
    }
}
