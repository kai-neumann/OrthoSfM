/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#include "synthethic_tests.h"

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

void orthosfm::runSyntheticRobustnessTests(int argc, char** argv) {
    std::cout << "Starting testbench" << std::endl;

    // Create a new image
    cv::Mat progressBar = cv::Mat(50, 500, CV_8UC3);

    // Setup the algorithms that should be evaluated this time
    std::vector<std::shared_ptr<ReconstructionAlgorithm>> algorithms;
    algorithms.push_back(std::make_shared<OrthographicReconstructionAlgorithm>());
    algorithms.push_back(std::make_shared<OrthoQuaternionReconstructionAlgorithm>());

    // Load the objects that should be used for evaluation as point clouds
    std::vector<std::string> modelFiles;
    auto resource_path = boost::filesystem::absolute(argv[0]).parent_path() / "resources";
    for(auto & p : boost::filesystem::directory_iterator(resource_path)){
        modelFiles.push_back(p.path().string());
    }

    // Generate the datasets
    std::vector<TestDataset> datasets = generateDatasets(modelFiles);

    // Only use the first dataset
    /*std::vector<TestDataset> chosenDatasets;
    chosenDatasets.push_back(datasets[0]);
    datasets = chosenDatasets;*/

    // the number of samples to calculate
    double samples = 101;                        // How many different steps should be sampled between (and including) the minimum and maximum

    // Observation noise
    bool useObservationNoise = true;            // Randomly perturb the reprojected pixel positions
    double freqObservationNoise = 1.0;          // Relative frequency of how often this error occurs (between 0 and 1)
    double startObservationNoiseStrength = 0.0; // The maximum amount of pixels an observation is perturbed
    double endObservationNoiseStrength = 100.0;  // The maximum amount of pixels an observation is perturbed
    double obsNoiseStepSize = (endObservationNoiseStrength - startObservationNoiseStrength)/(samples-1);

    // Matching noise
    bool useMatchingNoise = false;              // Randomly jumble up some tracks
    double freqMatchingNoise = 0.5;             // Relative frequency of how often this error occurs (between 0 and 1)

    // Define the range of noise levels that should be tested over the runtime
    double startNoisePercentage = 100;    // The amount of noise (in percent) at the start of the experiments
    double endNoisePercentage = 100;     // The maximum amount of noise in percent (0-100)
    double stepSize = (endNoisePercentage - startNoisePercentage)/(samples-1);

    // Setup the random distribution
    std::uniform_real_distribution<double> noiseChance(0,100);
    std::default_random_engine re;

    // Store the results
    std::vector<TestBenchDataEntry> results;

    for(int sampleID=0; sampleID<samples; sampleID++) {
        // Calculate the current noise percentage
        double currentNoisePercentage = startNoisePercentage + sampleID * stepSize;
        double currentNoiseStrength = startObservationNoiseStrength + sampleID * obsNoiseStepSize;
        std::cout << "Sample point " << sampleID << " -> " << currentNoisePercentage << "% noise with strength " << currentNoiseStrength << std::endl;
    }

    // Keep a counter over all runs
    int recoRun = 0;
    int maxRecoCount = samples*datasets.size()*algorithms.size();

    for(int sampleID=0; sampleID<samples; sampleID++) {
        // Calculate the current noise percentage
        double currentNoisePercentage = startNoisePercentage + sampleID*stepSize;
        double currentNoiseStrength = startObservationNoiseStrength + sampleID * obsNoiseStepSize;
        std::cout << "Executing tests with noise percentage: " << currentNoisePercentage << " and noise strength " << currentNoiseStrength << std::endl;

        // Initialize the noise function with the correct strength
        std::normal_distribution<double> pixelPerturbation(0, currentNoiseStrength);

        // Loop over all datasets
        for(int datasetID=0; datasetID<datasets.size(); datasetID++) {
            auto& dataset = datasets[datasetID];

            // Apply perturbations to the dataset
            for(int trackID=0; trackID<dataset.tracks.size(); trackID++) {
                // Loop over all features
                for(int featureID=0; featureID<dataset.tracks[trackID].size(); featureID++) {
                    if(useObservationNoise) {
                        // Randomize between 0 and 100
                        double diceRoll = noiseChance(re);

                        // If the roll is smaller than the weighted noise percentage: Modify this feature
                        if(diceRoll < currentNoisePercentage*freqObservationNoise) {
                            dataset.tracks[trackID].get(featureID).x += pixelPerturbation(re);
                            dataset.tracks[trackID].get(featureID).y += pixelPerturbation(re);
                        }
                    }
                    if(useMatchingNoise) {
                        // TODO
                    }
                }
            }

            // Loop over all algorithms
            for(int algorithmID=0; algorithmID<algorithms.size(); algorithmID++) {
                recoRun++;
                std::cout << "Running reconstruction " << recoRun << " out of " << maxRecoCount << std::endl;

                // Create a string for displaying
                std::string progressString = std::to_string(recoRun) + "/" + std::to_string(maxRecoCount);

                int baseLine = 0;
                cv::Size textSize = cv::getTextSize(progressString, cv::FONT_HERSHEY_PLAIN, 2, 4, &baseLine);

                // Show progress bar
                cv::rectangle(progressBar, cv::Point(0, 0), cv::Point(500, 50), cv::Scalar(128, 128, 128), -1);
                cv::rectangle(progressBar, cv::Point(0, 0), cv::Point((int)(500*((double) recoRun)/((double) maxRecoCount)), 50), cv::Scalar(100, 255, 60), -1);
                cv::putText(progressBar, progressString, cv::Point(250 - (textSize.width/2), 35), cv::FONT_HERSHEY_PLAIN, 2, CV_RGB(0, 0, 0),4);
                cv::putText(progressBar, progressString, cv::Point(250 - (textSize.width/2), 35), cv::FONT_HERSHEY_PLAIN, 2, CV_RGB(255, 255, 255),2);

                cv::imshow("Testbench Progress", progressBar);
                cv::waitKey(1);

                // Create a new working copy of the perturbed dataset
                TestDataset workingCopy = dataset;

                // Execute the algorithm on the dataset
                reconstruction_config config;
                std::vector<std::shared_ptr<Camera>> estimatedPoses = runPoseEstimation(workingCopy.views, algorithms[algorithmID], workingCopy.tracks, config);

                // Get the basis of the second cameras
                Eigen::Matrix3d basisCamera2;
                basisCamera2.col(0) = estimatedPoses[1]->getXAxis();
                basisCamera2.col(1) = estimatedPoses[1]->getYAxis();
                basisCamera2.col(2) = estimatedPoses[1]->getZAxis();

                // Convert to angle representation
                Eigen::Vector3d cameraAngles = OrthographicCamera::basisToPhiThetaRho(basisCamera2, true);

                // Check if the phi angle is positive (should always be positive in our test cases)
                bool isCoordinateSystemInverted = cameraAngles[0] < 0;
                if(isCoordinateSystemInverted) {
                    std::cout << "The coordinate system is inverted!!!" << std::endl;
                }

                // Calculate the distance of the estimated cameras to the ground truth
                std::vector<double> angleError;
                for(int i=0; i<estimatedPoses.size(); i++) {
                    // Find the ground truth with the same id
                    for(int j=0; j<dataset.groundTruthCameras.size(); j++) {
                        if(estimatedPoses[i]->getView()->getID() == dataset.groundTruthCameras[j]->getView()->getID()) {
                            // Get the bases of both cameras
                            Eigen::Matrix3d basis1;
                            basis1.col(0) = estimatedPoses[i]->getXAxis();
                            basis1.col(1) = estimatedPoses[i]->getYAxis();
                            basis1.col(2) = estimatedPoses[i]->getZAxis();

                            Eigen::Matrix3d basis2;
                            basis2.col(0) = dataset.groundTruthCameras[j]->getXAxis();
                            basis2.col(1) = dataset.groundTruthCameras[j]->getYAxis();
                            basis2.col(2) = dataset.groundTruthCameras[j]->getZAxis();

                            // Flip the first basis if necessary
                            if(isCoordinateSystemInverted) {
                                // Create angle based representation of old pose
                                Eigen::Vector3d oldCamAngles = OrthographicCamera::basisToPhiThetaRho(basis1, true);

                                Eigen::Matrix3d transformToAlt;
                                transformToAlt << 1, 0, 0, 0, 1, 0, 0, 0, -1;
                                basis1 = transformToAlt * basis1 * transformToAlt;

                                // Create angle based representation of the new pose
                                Eigen::Vector3d newCamAngles = OrthographicCamera::basisToPhiThetaRho(basis1, true);

                                std::cout << "Flipped camera [phi: " << oldCamAngles.x() << "; theta: " << oldCamAngles.y() << "; roll: " << oldCamAngles.z() << "]";
                                std::cout << "to [phi: " << newCamAngles.x() << "; theta: " << newCamAngles.y() << "; roll: " << newCamAngles.z() << "]" << std::endl;
                            }

                            // Convert both bases to quaternions
                            Eigen::Quaterniond q1(basis1);
                            Eigen::Quaterniond q2(basis2);

                            // Measure the angular distance
                            double distance = q1.angularDistance(q2);
                            angleError.push_back(distance);

                            break;
                        }
                    }
                }

                // Store calcualted data
                //results.emplace_back(datasetID, algorithmID, angleError, currentNoisePercentage);
                results.emplace_back(datasetID, algorithmID, angleError, currentNoiseStrength);
            }

        }
    }

    // Merge the results with the same algorithmID and noisePercentage
    std::vector<TestBenchDataEntry> mergedResults;
    for(const auto& result : results) {
        // Search an entry with the same algorithm and noise percentage
        bool found = false;
        for(int i=0; i<mergedResults.size(); i++) {
            if(mergedResults[i].noisePercentage == result.noisePercentage && mergedResults[i].algorithmID == result.algorithmID) {
                // Add all entries of the result to the merged result
                mergedResults[i].angularErrors.insert(mergedResults[i].angularErrors.end(), result.angularErrors.begin(), result.angularErrors.end());

                found = true;
                break;
            }
        }

        // If there was no matching entry: Add this one to the list
        if(!found) {
            mergedResults.push_back(result);
        }
    }

    // Average the mean and std error per algorithm over all datasets -> Only one data point per sample per algorithm
    for(int i=0; i<mergedResults.size(); i++) {
        mergedResults[i].calculateMetrics();
    }

    // Convert to vectors for easier plotting
    std::vector<std::vector<std::pair<double, double>>> timeSeries;
    timeSeries.resize(algorithms.size());
    for(int i=0; i<mergedResults.size(); i++) {
        timeSeries[mergedResults[i].algorithmID].emplace_back(mergedResults[i].noisePercentage, mergedResults[i].meanError);
    }

    // Create a figure
    plt::figure_size(1200, 1000);
    plt::title("Angular error over noise strength");
    plt::xlabel("Noise strength");
    plt::ylabel("Mean angular error");

    // Plot all algorithms
    for(int i=0; i<timeSeries.size(); i++) {
        // Create an x and y array
        std::vector<double> x;
        std::vector<double> y;

        for(int j=0; j<timeSeries[i].size(); j++) {
            x.push_back(timeSeries[i][j].first);
            y.push_back(timeSeries[i][j].second);
        }

        // Plot
        plt::named_plot(algorithms[i]->getName(), x, y);
    }

    // Show the legend
    plt::legend();

    // Show the plot
    plt::show();
}
