/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#ifndef ORTHO_SFM_SYNTHETICTESTS_H
#define ORTHO_SFM_SYNTHETICTESTS_H

#include <iostream>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include "dataset_generation.h"
#include "random"

#include <sfm/reconstruct.h>
#include <algorithms/orthographic/OrthographicReconstructionAlgorithm.h>
#include <algorithms/orthographic_quaternion/OrthoQuaternionRecoAlgorithm.h>
#include <algorithms/ReconstructionAlgorithm.h>

namespace orthosfm
{
	struct TestBenchDataEntry
	{
		std::vector<double> angularErrors;
		int datasetID = 0;
		int algorithmID = 0;
		double noisePercentage = 0;

		// Store the calculated mean and std
		double meanError;
		double stdError;

		TestBenchDataEntry(int datasetID, int algorithmID, std::vector<double> angularErrors, double noiseLevel)
		{
			this->datasetID = datasetID;
			this->algorithmID = algorithmID;
			this->angularErrors = angularErrors;
			this->noisePercentage = noiseLevel;
		}

		void calculateMetrics()
		{
			// First calculate the mean
			double sum = 0;
			for (double angularError: angularErrors)
			{
				// Add to mean sum
				sum += angularError;
			}

			// Calculate the mean
			meanError = sum / ((double)angularErrors.size());

			// Then calculate the standard deviation
			double squaredSum = 0;
			for (double angularError: angularErrors)
			{
				// Add to squared sum
				squaredSum += pow(angularError - meanError, 2);
			}

			stdError = sqrt(squaredSum / ((double)angularErrors.size()));
		}
	};

	void runSyntheticRobustnessTests(int argc, char** argv);
}

#endif //ORTHO_SFM_SYNTHETICTESTS_H