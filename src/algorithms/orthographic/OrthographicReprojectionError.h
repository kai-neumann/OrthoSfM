/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#ifndef ORTHO_SFM_ORTHOGRAPHICREPROJECTIONERROR_H
#define ORTHO_SFM_ORTHOGRAPHICREPROJECTIONERROR_H

#include <ceres/ceres.h>
#include "OrthographicCamera.h"
#include <util/common.h>

namespace orthosfm
{
	struct OrthographicReprojectionError
	{
		OrthographicReprojectionError(double observed_x, double observed_y, const int& imgWidth, const int& imgHeight)
			: observed_x(observed_x), observed_y(observed_y), imgWidth(imgWidth), imgHeight(imgHeight)
		{

		}

		template<typename T>
		bool
		operator()(const T* const phi, const T* const theta, const T* const rho, const T* const offsetX, const T* const offsetY,
			const T* const scale, const T* const point, T* residuals) const
		{

			// Build the helper angle omega
			T omega = *theta + M_PI_2;

			// Extract the input point
			Eigen::Matrix<T, 3, 1> point3d;
			point3d << ((T)point[0] / ((T)point[3])), ((T)point[1] / ((T)point[3])), ((T)point[2] / ((T)point[3]));

			// Build the rotation matrices
			Eigen::Matrix<T, 3, 3> Rx;
			Rx << (T)1.0, (T)0.0, (T)0.0,
				(T)0.0, (T)cos(omega), (T)(-sin(omega)),
				(T)0.0, (T)sin(omega), (T)cos(omega);

			Eigen::Matrix<T, 3, 3> Ry;
			Ry << (T)cos(*rho), (T)(-sin(*rho)), (T)0.0,
				(T)sin(*rho), (T)cos(*rho), (T)0.0,
				(T)0.0, (T)0.0, (T)1.0;

			Eigen::Matrix<T, 3, 3> Rz;
			Rz << (T)cos(*phi), (T)(-sin(*phi)), (T)0.0,
				(T)sin(*phi), (T)cos(*phi), (T)0.0,
				(T)0.0, (T)0.0, (T)1.0;

			Eigen::Matrix<T, 3, 3> sphericalMatrix = Rz * Rx * Ry;

			// Build the coordinate system transform
			Eigen::Matrix<T, 3, 3> coordinateSystemTransform;
			coordinateSystemTransform << (T)1.0, (T)0.0, (T)0.0, (T)0.0, (T)0.0, (T)-1.0, (T)0.0, (T)1.0, (T)0.0;

			// Transform the world point into local coordinates
			Eigen::Matrix<T, 3, 1> localPoint = sphericalMatrix.transpose() * coordinateSystemTransform * point3d;

			// Convert width and height to type T
			T width = (T)imgWidth;
			T height = (T)imgHeight;

			// Convert from [-1, 1] to pixel coordinates
			T x_pixel = (T)width * ((T)((((localPoint[0] / (*scale)) - (*offsetX)) / ((T)(-2))) + (T)0.5));
			T y_pixel = (T)height * ((T)((((localPoint[1] / (*scale)) - (*offsetY)) / ((T)(-2))) + (T)0.5));

			// Calculate the distance from the observed point
			residuals[0] = x_pixel - ((T)observed_x);
			residuals[1] = y_pixel - ((T)observed_y);

			return true;
		}

		// Factory to hide the construction of the CostFunction object from the client code.
		static ceres::CostFunction*
		Create(const double observed_x, const double observed_y, const int& imgWidth, const int& imgHeight)
		{
			return (new ceres::AutoDiffCostFunction<OrthographicReprojectionError,
													2, // 2 residuals
													1, // Phi
													1, // Theta
													1, // Roll
													1, // OffsetX
													1, // OffsetY
													1, // scale
													4> // point
				(new OrthographicReprojectionError(observed_x, observed_y, imgWidth, imgHeight)));
		}

		double observed_x;
		double observed_y;
		const int imgWidth;
		const int imgHeight;
	};
}

#endif //ORTHO_SFM_ORTHOGRAPHICREPROJECTIONERROR_H
