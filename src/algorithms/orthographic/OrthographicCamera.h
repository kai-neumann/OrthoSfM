/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#ifndef ORTHO_SFM_ORTHOGRAPHICCAMERA_H
#define ORTHO_SFM_ORTHOGRAPHICCAMERA_H

#include <data_structures/Camera.h>

namespace orthosfm
{

// Define the number of parameters
	const int PARAMETER_COUNT = 6;

	class OrthographicCamera : public Camera
	{
	 public:
		explicit OrthographicCamera(const View& view);
		OrthographicCamera(const View& view, double phi, double theta, double roll);

		// Necessary interface
		const int getParameterCount() override
		{
			return m_parameterCount;
		};

		// Conversion from axis
		void convertFromAxis(const Eigen::Vector3d& xAxis, const Eigen::Vector3d& yAxis, const Eigen::Vector3d& zAxis);

		// Axes
		Eigen::Vector3d getXAxis() const override;
		Eigen::Vector3d getYAxis() const override;
		Eigen::Vector3d getZAxis() const override;

		// Origin
		Eigen::Vector3d getOrigin() const override;

		// Projection functions
		Eigen::Vector3d getPointOnCameraPlane(double xPixel, double yPixel) const override;
		Eigen::Vector2d projectPointOntoImagePlane(const Eigen::Vector4d& point) const;

		// Main transformation matrix
		Eigen::Matrix3d getSphericalProjectionMatrix() const;

		// Apply Transformation matrix to camera
		void applyTransformation(const Eigen::Matrix3d& transformation);

		// Calculate rotation from one matrix to another
		Eigen::Matrix3d
		calculateTransformationToAxes(Eigen::Vector3d x_axis, Eigen::Vector3d y_axis, Eigen::Vector3d z_axis);

		void print() const override;

		// Methods for triangulation
		Eigen::Vector3d getLookDirection() const override;

		// Convert a basis of three vectors (as columns of a 3x3 matrix) into spherical coordinates
		static const Eigen::Vector3d basisToPhiThetaRho(Eigen::Matrix3d basis, bool applyCoordinateTransform);

		// Getter for the parameters
		double* getPhi()
		{
			return &phi;
		};
		double* getTheta()
		{
			return &theta;
		};
		double* getRoll()
		{
			return &roll;
		};
		double* getOffsetX()
		{
			return &offsetX;
		};
		double* getOffsetY()
		{
			return &offsetY;
		};
		double* getScale()
		{
			return &scaleFactor;
		};

		// Getter for the fixed states
		bool getPhiFixed() const
		{
			return fixPhi;
		};
		bool getThetaFixed() const
		{
			return fixTheta;
		};
		bool getRollFixed() const
		{
			return fixRoll;
		};
		bool getOffsetFixed() const
		{
			return fixOffset;
		};
		bool getScaleFixed() const
		{
			return fixScale;
		};

		// Easy method to specify the number of free / fixed parameters
		void setDegreesOfFreedom(int dof);

	 private:
		// Constant settings
		const int m_parameterCount = 6;
		const double fixedCameraDistance = 10;

		// Member variables that define the pose of the camera
		double phi = 0;
		double theta = 0;
		double roll = 0;
		double offsetX = 0;
		double offsetY = 0;
		double scaleFactor = 1;

		// Specify which parameters are supposed to be fixed
		bool fixPhi = false;
		bool fixTheta = false;
		bool fixRoll = false;
		bool fixOffset = true;
		bool fixScale = true;

		// Transformation functions
		Eigen::Matrix3d getCoordinateSystemTransform() const;
		Eigen::Vector3d toCameraSpace(const Eigen::Vector3d& point) const;
		Eigen::Vector3d toWorldSpace(const Eigen::Vector3d& point) const;
	};
}

#endif //ORTHO_SFM_ORTHOGRAPHICCAMERA_H
