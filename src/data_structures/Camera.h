/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#ifndef ORTHO_SFM_CAMERA_H
#define ORTHO_SFM_CAMERA_H

#include <Eigen/Dense>

#include <data_structures/view.h>

namespace orthosfm
{
	class Camera
	{
	 public:
		virtual const int getParameterCount() = 0;

		// Necessary functionality for triangulation
		virtual Eigen::Vector3d getLookDirection() const = 0;
		virtual Eigen::Vector3d getPointOnCameraPlane(double xPixel, double yPixel) const = 0;
		virtual Eigen::Vector3d getXAxis() const = 0;
		virtual Eigen::Vector3d getYAxis() const = 0;
		virtual Eigen::Vector3d getZAxis() const = 0;
		virtual Eigen::Vector3d getOrigin() const = 0;

		// Getter and setter for fixed variable
		const bool& isFixed() const
		{
			return m_fixed;
		}

		void setFixed(const bool& fixed)
		{
			m_fixed = fixed;
		}

		// Getter for the view
		const std::shared_ptr<View>& getView() const
		{
			return m_view;
		}

		virtual void print() const = 0;

	 protected:
		// Reference to the view that this has been generated from
		std::shared_ptr<View> m_view;

		// Specify if this view should stay fixed during bundle adjustment
		bool m_fixed = false;
	};
}


#endif //ORTHO_SFM_CAMERA_H
