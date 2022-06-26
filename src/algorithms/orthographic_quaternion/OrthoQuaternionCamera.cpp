/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#include "OrthoQuaternionCamera.h"
#include <iostream>
#include <util/common.h>
#include <algorithms/orthographic/OrthographicCamera.h>

orthosfm::OrthoQuaternionCamera::OrthoQuaternionCamera(const Eigen::Matrix3d &basis, const View &view) {
    // Assign the view to the member field
    m_view = std::make_shared<View>(view);

    // Extract a quaternion from the basis vectors to represent the rotation
    m_rotation = Eigen::Quaternion<double>(basis);
}


void orthosfm::OrthoQuaternionCamera::print() const {
    // For printing: Convert the rotation back into an 3d rotation matrix and convert that to the known angle format
    Eigen::Matrix3d basis = m_rotation.toRotationMatrix();

    // Convert to angles
    Eigen::Vector3d angles = OrthographicCamera::basisToPhiThetaRho(basis, true);

    std::cout << "Quaternion Camera " << m_view->getID() << " [phi: " << (angles[0]*180)/M_PI << "; theta: " << (angles[1] * 180) / M_PI <<
              "; roll: " << (angles[2] * 180) / M_PI << "; offset (" << m_offsetX << "; " << m_offsetY << "); scale: " << m_scale << "]"<< std::endl;
}

const Eigen::Quaternion<double> orthosfm::OrthoQuaternionCamera::fromToRotation(const Eigen::Quaternion<double> &from, const Eigen::Quaternion<double> &to) {
    // Make sure that both quaternions are normalized
    Eigen::Quaternion<double> fromNormalized = from.normalized();
    Eigen::Quaternion<double> toNormalized = to.normalized();

    // Use the equation (4) from ["Evaluating the Performance of Structure from Motion Pipelines" by Bianco et al.]
    Eigen::Quaternion<double> fromTo = fromNormalized.conjugate() * toNormalized;

    return fromTo.normalized();
}

Eigen::Vector3d orthosfm::OrthoQuaternionCamera::getLookDirection() const {
    return m_rotation * Eigen::Vector3d(0, 0, 1);
}

Eigen::Vector3d orthosfm::OrthoQuaternionCamera::getPointOnCameraPlane(double xPixel, double yPixel) const {
    // Convert pixel into range [-1, 1]
    double x_norm = -2 * ((xPixel / m_view->getWidth()) - 0.5) + m_offsetX;
    double y_norm = -2 * ((yPixel / m_view->getHeight()) - 0.5) + m_offsetY;

    // Create the point in local coordinates
    Eigen::Vector3d localPoint(m_scale*x_norm, m_scale*y_norm, -10);

    // Return the rotated point
    return m_rotation*localPoint;
}

const Eigen::Quaternion<double> &orthosfm::OrthoQuaternionCamera::getRotationQuaternion() const {
    return m_rotation;
}

void orthosfm::OrthoQuaternionCamera::applyRotation(const Eigen::Quaternion<double> &rot) {
    m_rotation = rot * m_rotation;
}

Eigen::Vector3d orthosfm::OrthoQuaternionCamera::getOrigin() const {
    return m_rotation*Eigen::Vector3d(0, 0, -10);
}

Eigen::Vector3d orthosfm::OrthoQuaternionCamera::getXAxis() const {
    return m_rotation * Eigen::Vector3d(1, 0, 0);
}

Eigen::Vector3d orthosfm::OrthoQuaternionCamera::getYAxis() const {
    return m_rotation * Eigen::Vector3d(0, 1, 0);
}

Eigen::Vector3d orthosfm::OrthoQuaternionCamera::getZAxis() const {
    return m_rotation * Eigen::Vector3d(0, 0, 1);
}


