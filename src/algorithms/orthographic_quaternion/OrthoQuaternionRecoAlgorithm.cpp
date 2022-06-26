/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#include "OrthoQuaternionRecoAlgorithm.h"

#include <algorithms/tomasi_kanade.h>
#include <triangulation/triangulation.h>
#include <algorithms/orthographic_quaternion/OrthographicQuaternionReprojectorError.h>

bool orthosfm::OrthoQuaternionReconstructionAlgorithm::setSolverType(SFM_SOLVER_TYPE solver) {
    if(solver != SFM_SOLVER_TYPE::ORTHO_QUATERNION) {
        std::cout << "Error: Invalid solver type for quaternion based algorithm." << std::endl;
        return false;
    }
    return true;
}

std::vector<std::shared_ptr<orthosfm::Camera>>
orthosfm::OrthoQuaternionReconstructionAlgorithm::calculateInitialAlignment(const ViewGroup &group, const std::vector<View> &views,
                                                                  const std::vector<Track> &tracks, const std::vector<std::shared_ptr<Camera>>& globalCameras) {
    // Get initial alignment as groups of three basis vectors inside a 3x3 matrix
    std::pair<std::vector<Eigen::Matrix3d>, std::vector<Eigen::Matrix3d>> models = robustlyEstimateTomasiKanadeFactorization(group, views, tracks);
    std::vector<Eigen::Matrix3d> basisVectors = resolveAmbiguity(models, group, globalCameras, views, this);

    // Convert the returned basis vectors into cameras
    std::vector<std::shared_ptr<Camera>> outCameras;
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
            auto* cam = new OrthoQuaternionCamera(basisVectors[i], *view);
            std::shared_ptr<OrthoQuaternionCamera> camPtr(cam);
            outCameras.push_back(camPtr);
        }
    }
    return outCameras;
}

void orthosfm::OrthoQuaternionReconstructionAlgorithm::triangulateTracks(const std::vector<std::shared_ptr<Camera>>& cameras, std::vector<Track>& tracks, bool resetExistingData) {
    triangulateOrthographicTracks(cameras, tracks, resetExistingData);
}

void orthosfm::OrthoQuaternionReconstructionAlgorithm::normalizeSceneToCamera(std::vector<std::shared_ptr<Camera>> &cameras, const std::shared_ptr<Camera> &target) const {
    // Calculate the rotation from the target camera to the identitiy quaternion
    const std::shared_ptr<OrthoQuaternionCamera> cam0 = std::static_pointer_cast<OrthoQuaternionCamera>(target);
    Eigen::Quaternion<double> fromTo = OrthoQuaternionCamera::fromToRotation(cam0->getRotationQuaternion(), Eigen::Quaternion<double>::Identity());

    // Apply the rotation to all cameras
    for(auto & camera : cameras) {
        std::shared_ptr<OrthoQuaternionCamera> cam = std::static_pointer_cast<OrthoQuaternionCamera>(camera);
        cam->applyRotation(fromTo);
    }
}

void orthosfm::OrthoQuaternionReconstructionAlgorithm::normalizeScene(std::vector<std::shared_ptr<Camera>> &cameras) {
    normalizeSceneToCamera(cameras, cameras[0]);
}

void orthosfm::OrthoQuaternionReconstructionAlgorithm::alignToGlobalCameras(std::vector<std::shared_ptr<Camera>> &localCameras, const std::vector<std::shared_ptr<Camera>> &globalCameras) {
    std::vector<Eigen::Quaternion<double>> transformations;

    // For each local camera
    for(const auto& cam : localCameras) {
        // Check if there is an existing global camera with the same id
        bool found = false;
        for (const auto &globalCam: globalCameras) {
            if (cam->getView()->getID() == globalCam->getView()->getID()) {
                // Cast both cameras to orthographic cameras
                std::shared_ptr<OrthoQuaternionCamera> orthoLocal = std::dynamic_pointer_cast<OrthoQuaternionCamera>(cam);
                std::shared_ptr<OrthoQuaternionCamera> orthoGlobal = std::dynamic_pointer_cast<OrthoQuaternionCamera>(globalCam);

                // Calculate the rotation from the local to the global camera
                transformations.push_back(OrthoQuaternionCamera::fromToRotation(orthoLocal->getRotationQuaternion(), orthoGlobal->getRotationQuaternion()));

                break;
            }
        }
    }

    // There should be exactly two correspondences in a group of three!
    if(transformations.size() != 2) {
        std::cerr << "Align to global cameras failed. UnExpected number of corresponding cameras in group!";
        return;
    }

    // Interpolate between both rotation using q1.slerp(0.5, q2)
    Eigen::Quaternion<double> smoothed = transformations[0].slerp(0.5, transformations[1]);
    //Eigen::Quaternion<double> smoothed = coordinateTransform * transformations[0] * coordinateTransform.inverse();

    // Simply use the transformation given by the first camera, as there seems to be some fault in the above slerp based calculation
    //Eigen::Quaternion<double> smoothed = transformations[0];

    double distance_to_identity = sqrt(pow((1.0-transformations[0].w()), 2) + pow(transformations[0].x(), 2) + pow(transformations[0].y(), 2) + pow(transformations[0].z(), 2));
    if(distance_to_identity < 0.05) {
        std::cout << "The transformation to the first global camera is too small. Using the second global camera instead" << std::endl;
        smoothed = transformations[1];
    }

    // Apply to local cameras
    for(const auto& cam : localCameras) {
        // Cast to camera
        std::shared_ptr<OrthoQuaternionCamera> orthoLocal = std::dynamic_pointer_cast<OrthoQuaternionCamera>(cam);
        orthoLocal->applyRotation(smoothed);
    }

}

void orthosfm::OrthoQuaternionReconstructionAlgorithm::SetupParameterBlocks(const std::vector<std::shared_ptr<Camera>> &cameras,
                                                                  ceres::Problem &problem) {

    // Add the camera parameter blocks one by one
    for(int i=0; i<cameras.size(); i++) {
        // Cast to the correct type
        std::shared_ptr<OrthoQuaternionCamera> cam = std::dynamic_pointer_cast<OrthoQuaternionCamera>(cameras[i]);

        // Add the parameters one by one
        problem.AddParameterBlock(cam->getRotation(), 4);
        problem.AddParameterBlock(cam->getOffsetX(), 1);
        problem.AddParameterBlock(cam->getOffsetY(), 1);
        problem.AddParameterBlock(cam->getScale(), 1);

        // Set the local parametrization
        problem.SetParameterization(cam->getRotation(), new ceres::EigenQuaternionParameterization());
        problem.SetParameterization(cam->getOffsetX(), new ceres::IdentityParameterization(1));
        problem.SetParameterization(cam->getOffsetY(), new ceres::IdentityParameterization(1));
        problem.SetParameterization(cam->getScale(), new ceres::IdentityParameterization(1));

        // Set parameters fixed (if specified)
        if(cam->isFixed() || cam->getRotationFixed()) problem.SetParameterBlockConstant(cam->getRotation());
        if(cam->isFixed() || cam->getOffsetFixed()) problem.SetParameterBlockConstant(cam->getOffsetX());
        if(cam->isFixed() || cam->getOffsetFixed()) problem.SetParameterBlockConstant(cam->getOffsetY());
        if(cam->isFixed() || cam->getScaleFixed()) problem.SetParameterBlockConstant(cam->getScale());
    }

}

void orthosfm::OrthoQuaternionReconstructionAlgorithm::AddResidualBlock(const std::shared_ptr<Camera> &camera,
                                                              const Feature &observation, double *point4d,
                                                              ceres::Problem &problem,
                                                              ceres::LossFunction *lossFunction) {
    // Cast to the correct type
    std::shared_ptr<OrthoQuaternionCamera> cam = std::dynamic_pointer_cast<OrthoQuaternionCamera>(camera);

    // Create the cost function
    ceres::CostFunction* cost_function = OrthographicQuaternionReprojectionError::Create(observation.x, observation.y, cam->getView()->getWidth(), cam->getView()->getHeight());

    // Add it as residual
    problem.AddResidualBlock(cost_function,
                             lossFunction, // Nullpointer is squared loss
                             cam->getRotation(),
                             cam->getOffsetX(),
                             cam->getOffsetY(),
                             cam->getScale(),
                             point4d);

}

std::string orthosfm::OrthoQuaternionReconstructionAlgorithm::getName() {
    return "Ortho Quaternion Reconstruction";
}

double orthosfm::OrthoQuaternionReconstructionAlgorithm::evaluateReprojectionError(const std::shared_ptr<Camera> &camera, const Feature &observation, double *point3d) {
    // Cast to the correct type
    std::shared_ptr<OrthoQuaternionCamera> cam = std::dynamic_pointer_cast<OrthoQuaternionCamera>(camera);

    // Create the cost function
    ceres::CostFunction* cost_function = OrthographicQuaternionReprojectionError::Create(observation.x, observation.y, cam->getView()->getWidth(), cam->getView()->getHeight());

    // Create the parameter array
    const double* parameters[] = {cam->getRotation(), cam->getOffsetX(), cam->getOffsetY(), cam->getScale(), point3d};

    // Create double array for the residuals
    double residuals[] = {0.0, 0.0};

    // Evaluate
    cost_function->Evaluate(parameters, residuals, nullptr);

    // Calculate the euclidean error
    double err = sqrt(residuals[0]*residuals[0] + residuals[1]*residuals[1]);
    return err;
}

std::vector<std::shared_ptr<orthosfm::Camera>> orthosfm::OrthoQuaternionReconstructionAlgorithm::copyCameraArray(const std::vector<std::shared_ptr<Camera>> &cameras) const {
    std::vector<std::shared_ptr<Camera>> copy;
    for(const auto& camera : cameras) {
        std::shared_ptr<OrthoQuaternionCamera> cam = std::dynamic_pointer_cast<OrthoQuaternionCamera>(camera);
        auto* camCpy = new OrthoQuaternionCamera(*cam);
        std::shared_ptr<OrthoQuaternionCamera> camCpyPtr(camCpy);
        copy.push_back(camCpyPtr);
    }
    return copy;
}
