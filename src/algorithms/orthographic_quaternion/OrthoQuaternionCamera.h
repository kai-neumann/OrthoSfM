/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#ifndef ORTHO_SFM_ORTHOQUATERNIONCAMERA_H
#define ORTHO_SFM_ORTHOQUATERNIONCAMERA_H

#include <data_structures/Camera.h>
#include <Eigen/Dense>
#include <data_structures/view.h>

namespace orthosfm
{
	class OrthoQuaternionCamera : public Camera
	{
	 public:
		OrthoQuaternionCamera(const Eigen::Matrix3d& basis, const View& view);

		const int getParameterCount() override
		{
			return m_paramCount;
		};

		void print() const override;

		Eigen::Vector3d getPointOnCameraPlane(double xPixel, double yPixel) const override;
		Eigen::Vector3d getLookDirection() const override;

		// Axes
		Eigen::Vector3d getXAxis() const override;
		Eigen::Vector3d getYAxis() const override;
		Eigen::Vector3d getZAxis() const override;

		Eigen::Vector3d getOrigin() const override;

		const Eigen::Quaternion<double>& getRotationQuaternion() const;
		void applyRotation(const Eigen::Quaternion<double>& rot);

		static const Eigen::Quaternion<double>
		fromToRotation(const Eigen::Quaternion<double>& from, const Eigen::Quaternion<double>& to);

		// Getter for parameters
		double* getRotation()
		{
			return m_rotation.coeffs().data();
		};
		double* getOffsetX()
		{
			return &m_offsetX;
		};
		double* getOffsetY()
		{
			return &m_offsetY;
		};
		double* getScale()
		{
			return &m_scale;
		};

		// Getter for fixed state
		bool getRotationFixed()
		{
			return m_fixRotation;
		};
		bool getOffsetFixed()
		{
			return m_fixOffset;
		};
		bool getScaleFixed()
		{
			return m_fixScale;
		};

	 private:
		// Define the number of parameters
		const int m_paramCount = 7;

		// Define the parameters of the camera
		Eigen::Quaternion<double> m_rotation; //Store the camera rotation as a quaternion
		double m_offsetX = 0.0; // Also store the position
		double m_offsetY = 0.0; // Also store the position
		double m_scale = 1.0; // Scaling factor of the orthographic view

		// Flags that specify if the parameters are fixed
		bool m_fixRotation = false;
		bool m_fixOffset = false;
		bool m_fixScale = true;

	};
}

#endif //ORTHO_SFM_ORTHOQUATERNIONCAMERA_H