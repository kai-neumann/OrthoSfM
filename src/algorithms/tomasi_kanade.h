/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#ifndef ORTHO_SFM_TOMASI_KANADE_H
#define ORTHO_SFM_TOMASI_KANADE_H

#include <data_structures/Camera.h>
#include <algorithms/orthographic/OrthographicCamera.h>
#include <utility>
#include <vector>
#include <data_structures/view.h>
#include <data_structures/group.h>
#include <data_structures/track.h>
#include "ReconstructionAlgorithm.h"
#include <Eigen/Dense>

namespace orthosfm
{

	std::vector<std::shared_ptr<Camera>> getOrderedCameraVector(const ViewGroup& group, const std::vector<View>& views);

	std::pair<std::vector<Eigen::Matrix3d>, std::vector<Eigen::Matrix3d>>
	tomasiKanadeFactorization(const ViewGroup& group, const std::vector<View>& views, const std::vector<Track>& tracks);

	std::vector<std::shared_ptr<Camera>>
	basisVectorToCameraVector(const ViewGroup& group, const std::vector<View>& views, std::vector<Eigen::Matrix3d> model);

	std::pair<std::vector<Eigen::Matrix3d>, std::vector<Eigen::Matrix3d>>
	robustlyEstimateTomasiKanadeFactorization(const ViewGroup& group, const std::vector<View>& views, const std::vector<
		Track>& tracks);

	std::vector<Eigen::Matrix3d> resolveAmbiguity(const std::pair<std::vector<Eigen::Matrix3d>,
																  std::vector<Eigen::Matrix3d>>& models, const ViewGroup& group, const std::vector<
		std::shared_ptr<Camera>>& globalCameras, const std::vector<View>& views, const ReconstructionAlgorithm* algorithm);

	bool isTomasiKanadeResultUsable(const std::vector<Eigen::Matrix3d>& cameras);

	struct TomasiKanadeCostFunctor
	{
		const Eigen::MatrixXd RStar;
		TomasiKanadeCostFunctor(Eigen::MatrixXd rstar)
			: RStar(rstar)
		{

		}

		template<typename T>
		bool operator()(const T* const x, T* residual) const
		{
			// Create a matrix from the given input
			Eigen::Matrix<T, 3, 3> QMatrix;
			QMatrix << (T)x[0], (T)x[1], (T)x[2], (T)x[3], (T)x[4], (T)x[5], (T)x[6], (T)x[7], (T)x[8];

			// Calcualte the number of cameras
			int cameraNumber = RStar.rows() / 2;

			Eigen::Matrix<T, 3, 3> QSquared = QMatrix * QMatrix.transpose();

			for (int frameID = 0; frameID < cameraNumber; frameID++)
			{
				// Build the camera coordinate frame vectors i and j
				Eigen::Matrix<T, 3, 1> i;
				i(0, 0) = (T)RStar(frameID, 0);
				i(1, 0) = (T)RStar(frameID, 1);
				i(2, 0) = (T)RStar(frameID, 2);

				Eigen::Matrix<T, 3, 1> j;
				j(0, 0) = (T)RStar(frameID + cameraNumber, 0);
				j(1, 0) = (T)RStar(frameID + cameraNumber, 1);
				j(2, 0) = (T)RStar(frameID + cameraNumber, 2);

				// iT*Q*Q*i = 1
				Eigen::Matrix<T, 1, 1> res1 = (i.transpose() * QSquared * i);
				residual[3 * frameID + 0] = (T)(res1(0, 0) - 1.0);
				// jT*Q*Q*j = 1
				Eigen::Matrix<T, 1, 1> res2 = (j.transpose() * QSquared * j);
				residual[3 * frameID + 1] = (T)(res2(0, 0) - 1.0);
				// iT*Q*Q*j = 0
				Eigen::Matrix<T, 1, 1> res3 = (i.transpose() * QSquared * j);
				residual[3 * frameID + 2] = (T)(res3(0, 0));
			}

			// Also add contraints to remove ambiguity->Places First camera on x, y plane looking in z - direction
			Eigen::Matrix<T, 3, 1> i0;
			i0(0, 0) = (T)RStar(0, 0);
			i0(1, 0) = (T)RStar(0, 1);
			i0(2, 0) = (T)RStar(0, 2);
			Eigen::Matrix<T, 3, 1> j0;
			j0(0, 0) = (T)RStar(cameraNumber, 0);
			j0(1, 0) = (T)RStar(cameraNumber, 1);
			j0(2, 0) = (T)RStar(cameraNumber, 2);

			Eigen::Matrix<T, 3, 1> constraint1 = QMatrix.transpose() * i0;
			Eigen::Matrix<T, 3, 1> constraint2 = QMatrix.transpose() * j0;

			residual[3 * cameraNumber + 0] = (T)(constraint1(0, 0) - 1.0);
			residual[3 * cameraNumber + 1] = (T)constraint1(1, 0);
			residual[3 * cameraNumber + 2] = (T)constraint1(2, 0);
			residual[3 * cameraNumber + 3] = (T)constraint2(0, 0);
			residual[3 * cameraNumber + 4] = (T)(constraint2(1, 0) - 1.0);
			residual[3 * cameraNumber + 5] = (T)constraint2(2, 0);

			// Add Norm of each column of Q -> Should be around 1
			// Stops optimizer from setting Q as zero matrix as local minimum and therefore increases convergence
			// This is NOT necessary if the solver is stable enough on its own.
			/*residual[3 * cameraNumber + 6] = (T)(1.0 - QMatrix.col(0).norm());
			residual[3 * cameraNumber + 7] = (T)(1.0 - QMatrix.col(1).norm());
			residual[3 * cameraNumber + 8] = (T)(1.0 - QMatrix.col(2).norm());*/

			return true;
		}
	};

}

#endif //ORTHO_SFM_TOMASI_KANADE_H
