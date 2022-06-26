/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#include "OrthographicReconstructionAlgorithm.h"

#include <algorithms/tomasi_kanade.h>
#include <algorithms/orthographic/OrthographicReprojectionError.h>
#include <triangulation/triangulation.h>

bool orthosfm::OrthographicReconstructionAlgorithm::setSolverType(SFM_SOLVER_TYPE solver) {
    if(solver == SFM_SOLVER_TYPE::ORTHO_EULER_HORIZONTAL) {
        m_degreesOfFreedom = 1;
        std::cout << "Parameters that are optimized: Phi" << std::endl;
        return true;
    }
    else if(solver == SFM_SOLVER_TYPE::ORTHO_EULER_HORIZONTAL_VERTICAL) {
        m_degreesOfFreedom = 2;
        std::cout << "Parameters that are optimized: Phi, Theta" << std::endl;
        return true;
    }
    else if(solver == SFM_SOLVER_TYPE::ORTHO_EULER_ALL_DOF) {
        m_degreesOfFreedom = 4;
        std::cout << "Parameters that are optimized: Phi, Theta, Roll, Offset" << std::endl;
        return true;
    }

    std::cout << "Error: Invalid solver type for euler angle based algorithm." << std::endl;
    return false;
}

std::vector<std::shared_ptr<orthosfm::Camera>> orthosfm::OrthographicReconstructionAlgorithm::calculateInitialAlignment(const ViewGroup &group, const std::vector<View> &views,
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
            std::shared_ptr<OrthographicCamera> cam = std::make_shared<OrthographicCamera>(*view);
            cam->setDegreesOfFreedom(m_degreesOfFreedom);
            cam->convertFromAxis(basisVectors[i].col(0), basisVectors[i].col(1), basisVectors[i].col(2));
            outCameras.push_back(cam);
        }
    }
    return outCameras;
}

void orthosfm::OrthographicReconstructionAlgorithm::triangulateTracks(const std::vector<std::shared_ptr<Camera>>& cameras, std::vector<Track>& tracks, bool resetExistingData) {
    triangulateOrthographicTracks(cameras, tracks, resetExistingData);
}

void orthosfm::OrthographicReconstructionAlgorithm::normalizeSceneToCamera(std::vector<std::shared_ptr<Camera>> &cameras, const std::shared_ptr<Camera> &target) const {
    Eigen::Matrix3d summedTransformation = Eigen::Matrix3d::Identity();

    for (int i = 0; i < 1; i++) {
        // Create a pointer
        std::shared_ptr<OrthographicCamera> camera = std::dynamic_pointer_cast<OrthographicCamera>(target);

        Eigen::Matrix3d trans = camera->calculateTransformationToAxes(Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 1, 0), Eigen::Vector3d(0, 0, 1));
        camera->applyTransformation(trans);
        summedTransformation = summedTransformation * trans;
    }

    /*std::cout << "Transformation to axes: " << std::endl;
    std::cout << summedTransformation;
    std::cout << std::endl;*/

    for (int cam = 0; cam < cameras.size(); cam++) {
        // Skip the target camera
        if(cameras[cam]->getView()->getID() == target->getView()->getID()) {
            continue;
        }

        // Else align
        std::shared_ptr<OrthographicCamera> camera = std::dynamic_pointer_cast<OrthographicCamera>(cameras[cam]);
        camera->applyTransformation(summedTransformation);
    }
}

void orthosfm::OrthographicReconstructionAlgorithm::normalizeScene(std::vector<std::shared_ptr<Camera>> &cameras) {
    normalizeSceneToCamera(cameras, cameras[0]);
}

void orthosfm::OrthographicReconstructionAlgorithm::alignToGlobalCameras(std::vector<std::shared_ptr<Camera>> &localCameras, const std::vector<std::shared_ptr<Camera>> &globalCameras) {
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> correspondences;

    // For each local camera
    for(const auto& cam : localCameras) {
        // Check if there is an existing global camera with the same id
        bool found = false;
        for (const auto &globalCam: globalCameras) {
            if (cam->getView()->getID() == globalCam->getView()->getID()) {
                // Cast both cameras to orthographic cameras
                std::shared_ptr<OrthographicCamera> orthoLocal = std::dynamic_pointer_cast<OrthographicCamera>(cam);
                std::shared_ptr<OrthographicCamera> orthoGlobal = std::dynamic_pointer_cast<OrthographicCamera>(globalCam);

                // Add the corresponding axes to the matrix for calculation
                correspondences.emplace_back(orthoLocal->getOrigin(), orthoGlobal->getOrigin());
                correspondences.emplace_back(orthoLocal->getXAxis(), orthoGlobal->getXAxis());
                correspondences.emplace_back(orthoLocal->getYAxis(), orthoGlobal->getYAxis());
                correspondences.emplace_back(orthoLocal->getZAxis(), orthoGlobal->getZAxis());

                break;
            }
        }
    }

    // Build two matrices of the correspondences
    Eigen::Matrix3Xd src(3, correspondences.size());
    Eigen::Matrix3Xd dst(3, correspondences.size());
    for(int i=0; i<correspondences.size(); i++) {
        src.col(i) = correspondences[i].first;
        dst.col(i) = correspondences[i].second;
    }

    // Use umeyama to calculate the transformation between the pairs
    Eigen::Matrix4d result = Eigen::umeyama(src, dst, false);
    Eigen::Matrix3d transformation = result.block(0, 0, 3, 3);

    // Apply the transformation to all local cameras
    for(int i=0; i<localCameras.size(); i++) {
        std::shared_ptr<OrthographicCamera> cam = std::dynamic_pointer_cast<OrthographicCamera>(localCameras[i]);
        cam->applyTransformation(transformation);
    }
}

int orthosfm::OrthographicReconstructionAlgorithm::getGroupSize() {
    return 3;
}

void orthosfm::OrthographicReconstructionAlgorithm::SetupParameterBlocks(const std::vector<std::shared_ptr<Camera>> &cameras, ceres::Problem &problem) {
    // Add the camera parameter blocks one by one
    for(int i=0; i<cameras.size(); i++) {
        // Cast to the correct type
        std::shared_ptr<OrthographicCamera> cam = std::dynamic_pointer_cast<OrthographicCamera>(cameras[i]);

        // Add the parameters one by one
        problem.AddParameterBlock(cam->getPhi(), 1);
        problem.AddParameterBlock(cam->getTheta(), 1);
        problem.AddParameterBlock(cam->getRoll(), 1);
        problem.AddParameterBlock(cam->getOffsetX(), 1);
        problem.AddParameterBlock(cam->getOffsetY(), 1);
        problem.AddParameterBlock(cam->getScale(), 1);

        // Set the local parametrization
        problem.SetParameterization(cam->getPhi(), new ceres::IdentityParameterization(1));
        problem.SetParameterization(cam->getTheta(), new ceres::IdentityParameterization(1));
        problem.SetParameterization(cam->getRoll(), new ceres::IdentityParameterization(1));
        problem.SetParameterization(cam->getOffsetX(), new ceres::IdentityParameterization(1));
        problem.SetParameterization(cam->getOffsetY(), new ceres::IdentityParameterization(1));
        problem.SetParameterization(cam->getScale(), new ceres::IdentityParameterization(1));

        // Set parameters fixed (if specified)
        if(cam->isFixed() || cam->getPhiFixed()) problem.SetParameterBlockConstant(cam->getPhi());
        if(cam->isFixed() || cam->getThetaFixed()) problem.SetParameterBlockConstant(cam->getTheta());
        if(cam->isFixed() || cam->getRollFixed()) problem.SetParameterBlockConstant(cam->getRoll());
        if(cam->isFixed() || cam->getOffsetFixed()) problem.SetParameterBlockConstant(cam->getOffsetX());
        if(cam->isFixed() || cam->getOffsetFixed()) problem.SetParameterBlockConstant(cam->getOffsetY());
        if(cam->isFixed() || cam->getScaleFixed()) problem.SetParameterBlockConstant(cam->getScale());
    }
}

void
orthosfm::OrthographicReconstructionAlgorithm::AddResidualBlock(const std::shared_ptr<Camera> &camera, const Feature &observation, double *point4d, ceres::Problem &problem, ceres::LossFunction* lossFunction) {
    // Cast to the correct type
    std::shared_ptr<OrthographicCamera> cam = std::dynamic_pointer_cast<OrthographicCamera>(camera);

    // Create the cost function
    ceres::CostFunction* cost_function = OrthographicReprojectionError::Create(observation.x, observation.y, cam->getView()->getWidth(), cam->getView()->getHeight());

    // Add it as residual
    problem.AddResidualBlock(cost_function,
                             lossFunction, // Nullpointer is squared loss
                             cam->getPhi(),
                             cam->getTheta(),
                             cam->getRoll(),
                             cam->getOffsetX(),
                             cam->getOffsetY(),
                             cam->getScale(),
                             point4d);
}

std::string orthosfm::OrthographicReconstructionAlgorithm::getName() {
    return "Orthographic Reconstruction";
}

double orthosfm::OrthographicReconstructionAlgorithm::evaluateReprojectionError(const std::shared_ptr<Camera> &camera, const Feature &observation, double *point3d) {
    // Cast to the correct type
    std::shared_ptr<OrthographicCamera> cam = std::dynamic_pointer_cast<OrthographicCamera>(camera);

    // Create the cost function
    ceres::CostFunction* cost_function = OrthographicReprojectionError::Create(observation.x, observation.y, cam->getView()->getWidth(), cam->getView()->getHeight());

    // Create the parameter array
    const double* parameters[] = {cam->getPhi(), cam->getTheta(), cam->getRoll(), cam->getOffsetX(), cam->getOffsetY(), cam->getScale(), point3d};

    // Create double array for the residuals
    double residuals[] = {0.0, 0.0};

    // Evaluate
    cost_function->Evaluate(parameters, residuals, nullptr);

    // Calculate the euclidean error
    double err = sqrt(residuals[0]*residuals[0] + residuals[1]*residuals[1]);
    return err;
}

std::vector<std::shared_ptr<orthosfm::Camera>> orthosfm::OrthographicReconstructionAlgorithm::copyCameraArray(const std::vector<std::shared_ptr<Camera>> &cameras) const {
    std::vector<std::shared_ptr<Camera>> copy;
    for(const auto& camera : cameras) {
        std::shared_ptr<OrthographicCamera> cam = std::dynamic_pointer_cast<OrthographicCamera>(camera);
        auto* camCpy = new OrthographicCamera(*cam);
        std::shared_ptr<OrthographicCamera> camCpyPtr(camCpy);
        copy.push_back(camCpyPtr);
    }
    return copy;
}


