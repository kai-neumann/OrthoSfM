/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#include "OrthographicCamera.h"
#include <iostream>
#include <util/common.h>

orthosfm::OrthographicCamera::OrthographicCamera(const View &view) {
    // Assign the view to the member field
    m_view = std::make_shared<View>(view);
}

orthosfm::OrthographicCamera::OrthographicCamera(const View &view, double phi, double theta, double roll) {
    // Assign the view to the member field
    m_view = std::make_shared<View>(view);

    // Set angles
    this->phi = phi*M_PI/180.0;
    this->theta = theta*M_PI/180.0;
    this->roll = roll*M_PI/180.0;
}

void orthosfm::OrthographicCamera::convertFromAxis(const Eigen::Vector3d &xAxis, const Eigen::Vector3d &yAxis, const Eigen::Vector3d &zAxis) {
    // Convert to matrix
    Eigen::Matrix3d basis;
    basis.col(0) = xAxis;
    basis.col(1) = yAxis;
    basis.col(2) = zAxis;

    // Convert to angles
    Eigen::Vector3d phiThetaRho = orthosfm::OrthographicCamera::basisToPhiThetaRho(basis, true);

    if(!fixPhi) phi = phiThetaRho[0];
    if(!fixTheta) theta = phiThetaRho[1];
    if(!fixRoll) roll = phiThetaRho[2];
}

Eigen::Vector3d orthosfm::OrthographicCamera::getXAxis() const
{
    return toCameraSpace(Eigen::Vector3d(1, 0, 0));
}

Eigen::Vector3d orthosfm::OrthographicCamera::getYAxis() const
{
    return toCameraSpace(Eigen::Vector3d(0, 1, 0));
}

Eigen::Vector3d orthosfm::OrthographicCamera::getZAxis() const
{
    return toCameraSpace(Eigen::Vector3d(0, 0, 1));
}

Eigen::Vector3d orthosfm::OrthographicCamera::getOrigin() const
{
    return toCameraSpace(Eigen::Vector3d(0, 0, -fixedCameraDistance));
}

Eigen::Vector2d orthosfm::OrthographicCamera::projectPointOntoImagePlane(const Eigen::Vector4d& point) const
{
    // Convert to non homogenous 3d point
    Eigen::Vector3d nonHomogenousPoint = Eigen::Vector3d(point.x()/point.w(), point.y()/point.w(), point.z()/point.w());

    // Project
    Eigen::Vector3d proj = toWorldSpace(nonHomogenousPoint) / scaleFactor;

    // Convert from [-1, 1] to pixel coordinates
    double x_pixel = m_view->getWidth() * (((proj[0] - offsetX)/ -2) + 0.5) ;
    double y_pixel = m_view->getHeight() * (((proj[1] - offsetY) / -2) + 0.5);

    return Eigen::Vector2d(x_pixel, y_pixel);
}

Eigen::Matrix3d orthosfm::OrthographicCamera::getSphericalProjectionMatrix() const
{
    // Define theta to be at the horizon
    double omega = theta + 0.5 * M_PI;


    Eigen::Matrix3d Ry;
    Ry << cos(roll), -sin(roll), 0, sin(roll), cos(roll), 0, 0, 0, 1;

    Eigen::Matrix3d Rx;
    Rx << 1, 0, 0, 0, cos(omega), -sin(omega), 0, sin(omega), cos(omega);

    Eigen::Matrix3d Rz;
    Rz << cos(phi), -sin(phi), 0, sin(phi), cos(phi), 0, 0, 0, 1;


    return (Rz * Rx) * Ry;
}

void orthosfm::OrthographicCamera::applyTransformation(const Eigen::Matrix3d& transformation)
{
    // Convert cameras from transformed axis
    convertFromAxis(transformation * getXAxis(), transformation * getYAxis(), transformation * getZAxis());
}

Eigen::Matrix3d orthosfm::OrthographicCamera::calculateTransformationToAxes(Eigen::Vector3d x_axis, Eigen::Vector3d y_axis, Eigen::Vector3d z_axis)
{
    // Use Umeyama
    // Transfrom into 3xN Point vectors
    Eigen::Matrix3d src;
    src.col(0) = getXAxis();
    src.col(1) = getYAxis();
    src.col(2) = getZAxis();

    Eigen::Matrix3d dest;
    dest.col(0) = x_axis;
    dest.col(1) = y_axis;
    dest.col(2) = z_axis;

    Eigen::Matrix4d transformation = Eigen::umeyama(src, dest, false);

    if (transformation.block(0, 3, 3, 1).norm() > 1e-4) {
        std::cout << "Umeyama returned translation: " << transformation.block(0, 3, 3, 1).transpose() << std::endl;

        throw std::runtime_error("Error: The umeyama translation was not 0!");
    }

    return transformation.block(0, 0, 3, 3);
}

Eigen::Matrix3d orthosfm::OrthographicCamera::getCoordinateSystemTransform() const
{
    Eigen::Matrix3d R;
    R << 1, 0, 0, 0, 0, -1, 0, 1, 0;

    return R;
}

Eigen::Vector3d orthosfm::OrthographicCamera::toCameraSpace(const Eigen::Vector3d& point) const
{
    return getCoordinateSystemTransform().transpose() * getSphericalProjectionMatrix() * point;
}

Eigen::Vector3d orthosfm::OrthographicCamera::toWorldSpace(const Eigen::Vector3d& point) const
{
    return getSphericalProjectionMatrix().transpose() * getCoordinateSystemTransform() * point;
}

void orthosfm::OrthographicCamera::print() const {
    std::cout << "Camera " << m_view->getID() << " [phi: " << (phi*180)/M_PI << "; theta: " << (theta * 180) / M_PI <<
              "; roll: " << (roll * 180) / M_PI << "; offset (" << offsetX << "; " << offsetY << "); scale: " << scaleFactor << "]" << std::endl;
}

const Eigen::Vector3d orthosfm::OrthographicCamera::basisToPhiThetaRho(Eigen::Matrix3d basis, bool applyCoordinateTransform) {
    /*
     * Even though this way of defining angles is specific to the original version of the algorithm / paper, it can
     * be used in other algorithms to print the camera rotation in a uniform way.
     */

    // Transform to standardized coordinate system
    if(applyCoordinateTransform) {
        // Rotate the coordinate system, so that our up axis (y) matches the spherical coordinate systems axis (z)
        Eigen::Matrix3d R;
        R << 1, 0, 0, 0, 0, -1, 0, 1, 0;
        basis = R * basis;
    }

    // Calculate phi and theta
    double phi = atan2(-basis(1,2), -basis(0, 2)) - M_PI_2;
    double theta = acos(basis(2, 2)/basis.col(2).norm()) - M_PI_2;

    // Define theta to be at the horizon
    double omega = theta + 0.5 * M_PI;
    Eigen::Matrix3d Rx;
    Rx << 1, 0, 0, 0, cos(omega), -sin(omega), 0, sin(omega), cos(omega);
    Eigen::Matrix3d Rz;
    Rz << cos(phi), -sin(phi), 0, sin(phi), cos(phi), 0, 0, 0, 1;

    Eigen::Vector3d testAxis = (Rz * Rx).transpose() * basis.col(0);
    double roll = atan2(testAxis(1), testAxis(0));

    Eigen::Vector3d phiThetaRho(phi, theta, roll);
    return phiThetaRho;
}

Eigen::Vector3d orthosfm::OrthographicCamera::getLookDirection() const {
    return toCameraSpace(Eigen::Vector3d(0, 0, 1));
}

Eigen::Vector3d orthosfm::OrthographicCamera::getPointOnCameraPlane(double xPixel, double yPixel) const {
    // Convert pixel into range [-1, 1]
    double x_norm = -2 * ((xPixel / m_view->getWidth()) - 0.5) + offsetX;
    double y_norm = -2 * ((yPixel / m_view->getHeight()) - 0.5) + offsetY;

    return getOrigin() + x_norm * getXAxis() * scaleFactor + y_norm * getYAxis() * scaleFactor;
}

void orthosfm::OrthographicCamera::setDegreesOfFreedom(int dof) {
    // The value should be in between 0 and 5
    if(dof < 0 || dof > 5) {
        std::cout << "Error: Invalid number of degrees of freedom for orthographic camera";
    }

    // Convert the number of dof to the actual fixed state
    fixPhi = dof < 1;
    fixTheta = dof < 2;
    fixRoll = dof < 3;
    fixOffset = dof < 4;
    fixScale = dof < 5;
}

