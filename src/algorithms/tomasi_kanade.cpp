/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#include "tomasi_kanade.h"

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <stdlib.h>
#include <random>

#include <util/common.h>
#include <algorithms/orthographic/OrthographicCamera.h>
#include <triangulation/triangulation.h>

std::pair<std::vector<Eigen::Matrix3d>, std::vector<Eigen::Matrix3d>> orthosfm::tomasiKanadeFactorization(const ViewGroup &group, const std::vector<View> &views, const std::vector<Track> &tracks) {

    // Initialize the matrix
    Eigen::MatrixXd D(2 * group.ids.size(), tracks.size());

    // Fill the matrix
    for (int trackID = 0; trackID < tracks.size(); trackID++) {
        for(int camID = 0; camID < group.ids.size(); camID++) {
            // Check which feature of the current track corresponds to the current id
            const Feature* f;
            for(int i=0; i < tracks[trackID].size(); i++) {
                if(tracks[trackID].get(i).viewID == group.ids[camID]) {
                    f = &tracks[trackID].get(i);
                }
            }

            D(camID, trackID) = -f->x;
            D(camID + group.ids.size(), trackID) = -f->y;
        }
    }

    // Subtract the row mean from each row
    for (int rowID = 0; rowID < D.rows(); rowID++) {
        double mean = D.row(rowID).mean();

        for (int colID = 0; colID < D.cols(); colID++) {
            D(rowID, colID) = D(rowID, colID) - mean;
        }
    }

    // We only need U for calculating camera positions
    Eigen::MatrixXd U = D.bdcSvd(Eigen::ComputeFullU).matrixU();

    Eigen::MatrixXd RStar = U.block(0, 0, 2*group.ids.size(), 3);

    // Initialize parameters
    Eigen::Matrix3d rnd = Eigen::Matrix3d::Random();
    std::vector<double> initialQ = { rnd(0), rnd(1), rnd(2), rnd(3), rnd(4), rnd(5), rnd(6), rnd(7), rnd(8) };
    //std::vector<double> initialQ = { 1, 0, 0, 0, 1, 0, 0, 0, 1};
    std::vector<double> Q = initialQ;

    // Build the problem.
    ceres::Problem problem;

    // Set up the only cost function (also known as residual). This uses
    // auto-differentiation to obtain the derivative (jacobian).
    ceres::CostFunction* cost_function =
            new ceres::AutoDiffCostFunction<TomasiKanadeCostFunctor, 15, 9>(new TomasiKanadeCostFunctor(RStar));
    problem.AddResidualBlock(cost_function, nullptr, Q.data());

    // Run the solver!
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    Solve(options, &problem, &summary);

    Eigen::Matrix3d initialQMat;
    initialQMat << initialQ[0], initialQ[1], initialQ[2], initialQ[3], initialQ[4], initialQ[5], initialQ[6], initialQ[7], initialQ[8];

    Eigen::Matrix3d finalQ;
    finalQ << Q[0], Q[1], Q[2], Q[3], Q[4], Q[5], Q[6], Q[7], Q[8];

    /*if(!summary.IsSolutionUsable()) {
        std::cout << summary.BriefReport() << "\n";
    }*/

    // Transform the Matrix
    Eigen::MatrixXd RFinal = RStar * finalQ;

    // Because of depth ambiguity, there are two possible solutions. Transfom RFinal into the second possible solution
    Eigen::Matrix3d transformToAlt;
    transformToAlt << 1, 0, 0, 0, 1, 0, 0, 0, -1;

    // Intitialize the two output vectors
    std::vector<Eigen::Matrix3d> solutions1;
    std::vector<Eigen::Matrix3d> solutions2;

    // Create rotation matrices to transform the cameras into normalized coordinates
    Eigen::Matrix3d rot1;
    Eigen::Matrix3d rot2;

    // Extract the cameras from their Axes
    for(int camID = 0; camID < group.ids.size(); camID++) {
        // First solution
        Eigen::Vector3d xAxis(RFinal(camID, 0), RFinal(camID, 1), RFinal(camID, 2));
        Eigen::Vector3d yAxis(RFinal(camID + group.ids.size(), 0), RFinal(camID + group.ids.size(), 1), RFinal(camID + group.ids.size(), 2));
        Eigen::Vector3d zAxis = xAxis.cross(yAxis);

        // Normalize Axis
        xAxis.normalize();
        yAxis.normalize();
        zAxis.normalize();

        // Put the axes into a matrix
        Eigen::Matrix3d combined1;
        combined1 << xAxis(0), yAxis(0), zAxis(0), xAxis(1), yAxis(1), zAxis(1), xAxis(2), yAxis(2), zAxis(2);

        // If this is the first camera: Create the rotation matrix for transforming
        if(camID == 0) {
            rot1 = combined1;
        }

        // Transform into normalized coordinates
        Eigen::Matrix3d transformed1 = rot1.transpose() * combined1;

        // Push to output
        solutions1.push_back(transformed1);

        // -------------------------------------------------------------------------------------------------------------

        // Put the axes into a matrix
        Eigen::Matrix3d combined2;
        combined2 << xAxis(0), yAxis(0), zAxis(0), xAxis(1), yAxis(1), zAxis(1), xAxis(2), yAxis(2), zAxis(2);

        // Transform into alternate solution (due to depth ambiguity)
        combined2 = transformToAlt * combined2 * transformToAlt;

        // If this is the first camera: Create the rotation matrix for transforming
        if(camID == 0) {
            rot2 = combined2;
        }

        // Transform into normalized coordinates
        Eigen::Matrix3d transformed2 = rot2.transpose() * combined2;

        // Push to output
        solutions2.push_back(transformed2);
    }

    return std::make_pair(solutions1, solutions2);
}

std::vector<std::shared_ptr<orthosfm::Camera>> orthosfm::getOrderedCameraVector(const ViewGroup &group, const std::vector<View> &views) {
    // Generates a list of cameras in the same order as the ids inside the group
    std::vector<std::shared_ptr<Camera>> tempCameras;

    for (unsigned int id : group.ids) {
        // Find the view with the matching id
        for (const auto &view: views) {
            if (view.getID() == id) {
                tempCameras.push_back(std::make_shared<OrthographicCamera>(view));
            }
        }
    }

    return tempCameras;
}

std::vector<std::shared_ptr<orthosfm::Camera>> orthosfm::basisVectorToCameraVector(const ViewGroup &group, const std::vector<View> &views, std::vector<Eigen::Matrix3d> model) {
    // Convert the model to a list of cameras (for easier calculation of reprojection errors)
    std::vector<std::shared_ptr<Camera>> modelCameras;
    for(int i=0; i<group.ids.size(); i++) {
        // Get the view with the corresponding id
        const View* view = nullptr;
        for(const auto& v : views) {
            if(v.getID() == group.ids[i]) {
                view = &v;
                break;
            }
        }

        if(view != nullptr) {
            // Create a camera
            std::shared_ptr<OrthographicCamera> cam = std::make_shared<OrthographicCamera>(*view);
            cam->convertFromAxis(model[i].col(0), model[i].col(1), model[i].col(2));
            modelCameras.push_back(cam);
        }
    }

    return modelCameras;
}

std::pair<std::vector<Eigen::Matrix3d>, std::vector<Eigen::Matrix3d>> orthosfm::robustlyEstimateTomasiKanadeFactorization(const ViewGroup &group, const std::vector<View> &views, const std::vector<Track> &tracks) {
    std::cout << "Calculating Tomasi-Kanade Factorization" << std::endl;

    // Only keep the tracks that contain all cameras of the group
    std::vector<Track> filteredTracks = filterTracksToAvailableCameras(group.ids, tracks, true, false);

    std::cout << "Using " << filteredTracks.size() << " filtered tracks" << std::endl;

    // If there are less than 10 Track the factorization is unlikely to succeed and will be stopped
    if(filteredTracks.size() < 10) {
        std::cout << "Error: To few tracks for tomasi kanade." << std::endl;
        throw std::runtime_error("To few tracks for tomasi kanade.");
    }

    // Ransac settings
    int sampleSize = 10;   // The number of tracks that are sampled at each iteration
    double p = 0.999; // Probability that the correct model is found during the iterations
    double omega = 0.7; // Pessimistic estimate of the percentage of inliers inside the given data

    int maxIterations = (int)(log(1 - p)/log(1 - pow(omega, sampleSize)));

    std::cout << "Ransac needs " << maxIterations << " iterations to achieve a " << p*100 << "% probability of finding a correct model" << std::endl;

    // Initialize the model
    bool modelFound = false;
    std::pair<std::vector<Eigen::Matrix3d>, std::vector<Eigen::Matrix3d>> bestFit;
    double bestErr = 1e32;
    int numberOfInliers = 0; // The number of inliers that have been found
    int minimumConsensusSize = 25; // The minimum number of inliers that support a model
    double maximumInlierReprojectionError = 3.0; // In pixels

    // Ransac iterations
    #pragma omp parallel for
    for(int iter=0; iter<maxIterations; iter++) {
        // Create a working copy of the tracks
        std::vector<Track> tracksWorkingCopy = filteredTracks;

        // Randomly select sampleSize tracks
        std::vector<Track> sampleTracks;
        std::sample(tracksWorkingCopy.begin(), tracksWorkingCopy.end(), std::back_inserter(sampleTracks), sampleSize, std::mt19937{std::random_device{}()});

        // Calculate an initial alignment from the sample
        std::pair<std::vector<Eigen::Matrix3d>, std::vector<Eigen::Matrix3d>> models = tomasiKanadeFactorization(group, views, sampleTracks);

        // For now: Always take the first one
        std::vector<Eigen::Matrix3d> model = models.first;

        // Check validity using a heuristic
        if(!isTomasiKanadeResultUsable(model)) {
            continue;
        }

        std::vector<std::shared_ptr<Camera>> modelCameras = orthosfm::basisVectorToCameraVector(group, views, model);

        // Triangulate all tracks (in place on the filtered tracks)
		orthosfm::triangulateOrthographicTracks(modelCameras, tracksWorkingCopy, true);

        // Collect any other tracks that would support these inliers (called "consensus set")
        std::vector<Track> consensusSet;
        double errSum = 0;
        int errCounter = 0;
        for(int i=0; i<tracksWorkingCopy.size(); i++) {
            // Check if this track is not inside the sample
            if(std::find(sampleTracks.begin(), sampleTracks.end(), tracksWorkingCopy[i]) != sampleTracks.end()) {
                continue;
            }

            // Check if the reprojection error for any camera is bigger than the threshold
            bool valid = true;
            for(int camID=0; camID<modelCameras.size(); camID++) {
                std::shared_ptr<OrthographicCamera> cam = std::dynamic_pointer_cast<OrthographicCamera>(modelCameras[camID]);

                // Calculate the reprojected point in pixels
                Eigen::Vector2d pixels = cam->projectPointOntoImagePlane(tracksWorkingCopy[i].getPoint());

                // Get the corresponding feature
                for(int featureID=0; featureID<tracksWorkingCopy[i].size(); featureID++) {
                    const Feature& f = tracksWorkingCopy[i].get(featureID);

                    // Check if this is the correct view
                    if(f.viewID == modelCameras[camID]->getView()->getID()) {
                        // Calculate the reprojection error
                        double error = sqrt(pow(f.x - pixels.x(), 2) + pow(f.y - pixels.y(), 2));
                        errSum += error;
                        errCounter++;

                        // Check if the error is bigger than a set threshold
                        if(error > maximumInlierReprojectionError) {
                            valid = false;
                        }

                        // We can stop the search here
                        break;
                    }
                }

                // If the track is already invalid, skip checking any further features
                if(valid == false) {
                    break;
                }
            }

            // If this is an inlier: Add to consensus set
            if(valid) {
                consensusSet.push_back(tracksWorkingCopy[i]);
            }
        }

        //std::cout << "Consensus set size: " << consensusSet.size() << " (of " << tracksWorkingCopy.size() << ") with average error " << errSum / ((double)errCounter) << " [px]" << std::endl;

        // Check if there are more inliers than specified in the threshold
        if(consensusSet.size() >= minimumConsensusSize) {
            // Calculate a model based on the inliers as well as the consensus set
            std::vector<Track> inlierTracks;
            inlierTracks.insert(inlierTracks.end(), sampleTracks.begin(), sampleTracks.end());
            inlierTracks.insert(inlierTracks.end(), consensusSet.begin(), consensusSet.end());
            std::pair<std::vector<Eigen::Matrix3d>, std::vector<Eigen::Matrix3d>> recalculatedModels = tomasiKanadeFactorization(group, views, sampleTracks);

            // Always use the first for now
            std::vector<Eigen::Matrix3d> recalculatedModel = recalculatedModels.first;
            std::vector<std::shared_ptr<Camera>> recalculatedCameras = basisVectorToCameraVector(group, views, recalculatedModel);

            // Triangulate the inlier tracks
            triangulateOrthographicTracks(recalculatedCameras, inlierTracks, true);

            // Measure the error of this model (based on the selected points)
            double recalculatedErrSum = 0;
            for(int i=0; i<inlierTracks.size(); i++) {
                // Calculate the reprojection error per camera
                for(int camID=0; camID<recalculatedCameras.size(); camID++) {
                    std::shared_ptr<OrthographicCamera> cam = std::dynamic_pointer_cast<OrthographicCamera>(recalculatedCameras[camID]);

                    // Calculate the reprojected point in pixels
                    Eigen::Vector2d pixels = cam->projectPointOntoImagePlane(inlierTracks[i].getPoint());

                    // Get the corresponding feature
                    for(int featureID=0; featureID<inlierTracks[i].size(); featureID++) {
                        const Feature& f = inlierTracks[i].get(featureID);

                        // Check if this is the correct view
                        if(f.viewID == recalculatedCameras[camID]->getView()->getID()) {
                            // Calculate the reprojection error
                            double error = sqrt(pow(f.x - pixels.x(), 2) + pow(f.y - pixels.y(), 2));
                            recalculatedErrSum += error;

                            // We can stop the search here
                            break;
                        }
                    }
                }
            }

            #pragma omp critical
            {
                // If the error is better than current best error, and the number of inliers is higher than the previously found set of inliers
                if (recalculatedErrSum < bestErr && consensusSet.size() > numberOfInliers) {
                    // Keep the model
                    bestErr = recalculatedErrSum;
                    numberOfInliers = consensusSet.size();
                    bestFit = recalculatedModels;
                    modelFound = true;

                    //std::cout << "New best model with " << numberOfInliers + sampleSize << " inliers and an error of " << bestErr << std::endl;
                }
            }
        }
    }

    if(!modelFound) {
        // If no solution was found until now: Throw an error
        std::cout << "WARNING: Failed to calculate robust initial alignment using tomasi kanade. Using backup solution." << std::endl;
        return tomasiKanadeFactorization(group, views, tracks);
    }
    else {
        std::cout << "Successfully calculated Tomasi-Kanade Factorization with " << numberOfInliers + sampleSize << " inliers (out of " << tracks.size() << " tracks)" << std::endl;
        return bestFit;
    }
}

std::vector<Eigen::Matrix3d> orthosfm::resolveAmbiguity(const std::pair<std::vector<Eigen::Matrix3d>, std::vector<Eigen::Matrix3d>>& models, const ViewGroup &group,
                                              const std::vector<std::shared_ptr<Camera>> &globalCameras, const std::vector<View> &views, const ReconstructionAlgorithm* algorithm) {
    // If there are no global cameras: Simply return the first model
    if(globalCameras.empty()) {
        return models.first;
    }

    // Convert both models into cameras
    std::vector<std::shared_ptr<Camera>> localCamerasModel1 = basisVectorToCameraVector(group, views, models.first);
    std::vector<std::shared_ptr<Camera>> localCamerasModel2 = basisVectorToCameraVector(group, views, models.second);

    // Create a copy of the global cameras
    std::vector<std::shared_ptr<Camera>> realignedGlobalCameras = algorithm->copyCameraArray(globalCameras);

    // Get the two global cameras that are part of the group
    std::vector<std::shared_ptr<Camera>> cams;
    std::vector<std::shared_ptr<Camera>> camsModel1;
    std::vector<std::shared_ptr<Camera>> camsModel2;
    for(const auto& id : group.ids) {
        // Global cameras
        for(const auto & globalCamera : realignedGlobalCameras) {
            if(globalCamera->getView()->getID() == id) {
                cams.push_back(globalCamera);
            }
        }
        // Local cameras 1
        for(auto & cam : localCamerasModel1) {
            if(cam->getView()->getID() == id) {
                camsModel1.push_back(cam);
            }
        }
        // Local cameras 2
        for(auto & cam : localCamerasModel2) {
            if(cam->getView()->getID() == id) {
                camsModel2.push_back(cam);
            }
        }
    }

    // Normalize them, so that the first camera of the local group is set to zero! -> This allows us to directly compare the vectors between the cameras origins!
    algorithm->normalizeSceneToCamera(realignedGlobalCameras, cams[0]);

    // Print for debugging
    //std::cout << "Realigned global cameras:" << std::endl;
    //printCameras(realignedGlobalCameras, true);

    // Calculate the vector between both global cameras
    Eigen::Vector3d globalCameraVector = cams[1]->getOrigin().normalized() - cams[0]->getOrigin().normalized();

    //std::cout << "Global Vector: \n" << globalCameraVector << std::endl;

    // For both models
    // Calculate the angle-axis between the same two cameras
    Eigen::Vector3d localCameraVector1 = camsModel1[1]->getOrigin().normalized() - camsModel1[0]->getOrigin().normalized();
    Eigen::Vector3d localCameraVector2 = camsModel2[1]->getOrigin().normalized() - camsModel2[0]->getOrigin().normalized();

    //std::cout << "Local Vector 1: \n" << localCameraVector1 << std::endl;
    //std::cout << "Local Vector 2: \n" << localCameraVector2 << std::endl;

    // Calculate the distance to the global angle-axis
    double similarityModel1 = globalCameraVector.dot(localCameraVector1);
    double similarityModel2 = globalCameraVector.dot(localCameraVector2);

    // Keep the model with the smaller error
    if(similarityModel1 > similarityModel2) {
        std::cout << "Selected ambiguous model 1 with similarity " << similarityModel1 << "(compared to similarity " << similarityModel2 << ")" << std::endl;
        return models.first;
    }
    else {
        std::cout << "Selected ambiguous model 2 with similarity " << similarityModel2 << "(compared to similarity " << similarityModel1 << ")" << std::endl;
        return models.second;
    }
}

bool orthosfm::isTomasiKanadeResultUsable(const std::vector<Eigen::Matrix3d>& cameras) {
    // FixMe: This always evaluates to false, if the same image is used twice in the same reconstruction. Usually not a problem.

    // Check if any of the cameras are too close to eachother
    for(int i=0; i<cameras.size(); i++) {
        for(int j=0; j<cameras.size(); j++) {
            if(i != j) {
                // Convert to angles
                Eigen::Vector3d phiThetaRoll1 = OrthographicCamera::basisToPhiThetaRho(cameras[i], true);
                Eigen::Vector3d phiThetaRoll2 = OrthographicCamera::basisToPhiThetaRho(cameras[j], true);

                // If phi or theta are too similar: This factorization is not usable!
                if(abs(phiThetaRoll1.x() - phiThetaRoll2.x()) < 0.1 && abs(phiThetaRoll1.y() - phiThetaRoll2.y()) < 0.1) {
                    return false;
                }

                // If the matrices are too similar: This factorization is probably not usable!
                if((cameras[i] - cameras[j]).norm() < 0.1) {
                    return false;
                }
            }
        }
    }
    return true;
}
