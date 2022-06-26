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
#include <util/common.h>

namespace orthosfm
{
	struct OrthographicQuaternionReprojectionError
	{
		OrthographicQuaternionReprojectionError(double observed_x, double observed_y, const int& imgWidth, const int& imgHeight)
			: observed_x(observed_x), observed_y(observed_y), imgWidth(imgWidth), imgHeight(imgHeight)
		{
		}

		template<typename T>
		bool operator()(const T* const rotation, const T* const offsetX, const T* const offsetY, const T* const scale,
			const T* const point, T* residuals) const
		{
			// Extract the input point
			Eigen::Matrix<T, 3, 1> point3d;
			point3d << ((T)point[0] / ((T)point[3])), ((T)point[1] / ((T)point[3])), ((T)point[2] / ((T)point[3]));

			// Build a Quaternion out of the input data
			Eigen::Quaternion<T> quaternion(rotation[3], rotation[0], rotation[1], rotation[2]);
			//quaternion.normalize();
			//Eigen::Quaternion<T> dq(dW, dX, dY, dZ);

			//Eigen::Quaternion<T> combined = dq * quaternion;
			Eigen::Quaternion<T> combined = quaternion;

			// Print
			/*std::cout << "Quaternion: [w= ";
			std::cout << ((ceres::Jet<double, 10>)combined.w()).a << ", x=";
			std::cout << ((ceres::Jet<double, 10>)combined.x()).a << ", y=";
			std::cout << ((ceres::Jet<double, 10>)combined.y()).a << ", z=";
			std::cout << ((ceres::Jet<double, 10>)combined.z()).a << std::endl;*/
			//std::cout << "Quaternion: [w= " << qd.w() << ", x=" << qd.x() << ", y=" << qd.y() << ", z=" << qd.z() << "]" << std::endl;

			// Transform the input point from world coordiantes to local coordinates
			Eigen::Matrix<T, 3, 1> localPoint = combined.inverse() * point3d;

			// Convert width and height to type T
			T width = (T)imgWidth;
			T height = (T)imgHeight;

			// Convert from [-1, 1] to pixel coordinates
			T x_pixel = (T)width * ((T)((((localPoint[0] / (*scale)) - (*offsetX)) / ((T)(-2))) + (T)0.5));
			T y_pixel = (T)height * ((T)((((localPoint[1] / (*scale)) - (*offsetY)) / ((T)(-2))) + (T)0.5));

			// Calculate the distance from the observed point
			residuals[0] = x_pixel - ((T)observed_x);
			residuals[1] = y_pixel - ((T)observed_y);

			//std::cout << "Observed point: [" << observed_x << ", " << observed_y << "] <-> reprojected point [" << ((ceres::Jet<double, 10>)x_pixel).a << ", " << ((ceres::Jet<double, 10>)y_pixel).a << "]" << std::endl;
			//std::cout << "Residual: [" << ((ceres::Jet<double, 10>)residuals[0]).a << ", " << ((ceres::Jet<double, 10>)residuals[1]).a <<"]" << std::endl;

			return true;
		}

		// Factory to hide the construction of the CostFunction object from the client code.
		static ceres::CostFunction*
		Create(const double observed_x, const double observed_y, const int& imgWidth, const int& imgHeight)
		{
			return (new ceres::AutoDiffCostFunction<OrthographicQuaternionReprojectionError,
													2, // 2 Residuals
													4, // 4 Quaternion components
													1, // offsetX
													1, // offsetY
													1, // scale
													4> // 4 Point components
				(new OrthographicQuaternionReprojectionError(observed_x, observed_y, imgWidth, imgHeight)));
		}

		double observed_x;
		double observed_y;
		const int imgWidth;
		const int imgHeight;
	};
}

#endif //ORTHO_SFM_ORTHOGRAPHICREPROJECTIONERROR_H
