/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#include "full_pipeline_tests.h"

#include <iostream>
#include <string>
#include <vector>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <util/common.h>
#include <data_structures/project_io.h>
#include <data_structures/camera_io.h>
#include <util/timing.h>

#include <algorithms/orthographic/OrthographicCamera.h>

void saveToResults(const std::string& projectFolderRoot, const std::vector<std::string>& imageDatasetNames, const std::vector<orthosfm::RunConfiguration>& configs,
                   const std::vector<std::pair<int, orthosfm::RunConfiguration>>& combinations, const std::vector<orthosfm::FullPipelineResult>& results) {
    // Build the results as a list of strings
    std::vector<std::string> resultStrings;

    // Create the header
    std::string header = "Algorithm Configuration;Metric";
    for(const auto& name : imageDatasetNames) {
        header += ";" + name;
    }
    resultStrings.push_back(header);

    // Build the lines per configuration
    for(int configID=0; configID<configs.size(); configID++) {
        // Initialize the lines
        std::string lineErr = configs[configID].name + ";Mean Angular Error [degrees]";
        std::string lineStd = configs[configID].name + ";Std Angular Error";
        std::string lineErrPos = configs[configID].name + ";Mean Position Error";
        std::string lineStdPos = configs[configID].name + ";Std Position Error";
        std::string lineMeanRuntime = configs[configID].name + ";Mean Runtime [s]";
        std::string lineMeanPoseTime = configs[configID].name + ";Mean Pose Runtime [s]";

        // Loop over all datasets
        for(int datasetID=0; datasetID<imageDatasetNames.size(); datasetID++) {
            // Get a combination with that dataset and config
            bool found = false;
            for(int i=0; i<combinations.size(); i++) {
                if(combinations[i].first == datasetID && combinations[i].second == configs[configID]) {
                    // Add to lines
                    lineErr += ";" + std::to_string(results[i].meanAngularError);
                    lineStd += ";" + std::to_string(results[i].stdAngularError);
                    lineErrPos += ";" + std::to_string(results[i].meanPositionError);
                    lineStdPos += ";" + std::to_string(results[i].stdPositionError);
                    lineMeanRuntime += ";" + std::to_string(results[i].runtime);
                    lineMeanPoseTime += ";" + std::to_string(results[i].poseEstimationTime);

                    // Set the flag
                    found = true;
                    break;
                }
            }

            if(!found) {
                lineErr +=";-";
                lineStd +=";-";
                lineErrPos +=";-";
                lineStdPos +=";-";
                lineMeanRuntime +=";-";
                lineMeanPoseTime +=";-";
            }
        }
        resultStrings.push_back(lineErr);
        resultStrings.push_back(lineStd);
        resultStrings.push_back(lineErrPos);
        resultStrings.push_back(lineStdPos);
        resultStrings.push_back(lineMeanRuntime);
        resultStrings.push_back(lineMeanPoseTime);
    }

    for(int i=0; i<resultStrings.size(); i++) {
        std::cout << resultStrings[i] << std::endl;
    }

    std::ofstream outfile;
    outfile.open(projectFolderRoot + "/results.csv");
    for(int i=0; i<resultStrings.size(); i++) {
        outfile << resultStrings[i] << std::endl;
    }
    outfile.close();
}

std::vector<orthosfm::FullPipelineResult> evaluateResultsFromFolders(const std::string& projectFolderRoot, const std::string& datasetFolder,
                                                        const std::vector<std::string>& imageDatasetNames,
                                                        const std::vector<orthosfm::RunConfiguration>& configs,
                                                        const std::vector<std::pair<int, orthosfm::RunConfiguration>>& combinations,
                                                        int repetitionsCount) {

    // Initialize result arrays for the mean / std accuracy as well as the runtimes
    std::vector<orthosfm::FullPipelineResult> results;

    std::string time_measurements_file_name = "/time_measurements.txt";
    std::string camera_file_name = "/cameras.txt";
    Eigen::Vector3d positionOffset = Eigen::Vector3d(0,0,0);

    //std::string time_measurements_file_name = "/time_matlab.txt";
    //std::string camera_file_name = "/cameras_matlab.txt";
    //Eigen::Vector3d positionOffset = Eigen::Vector3d(0,0,-1);

    // Load the references per dataset
    std::vector<std::vector<orthosfm::ReferenceCamera>> references;
    for(const auto& name : imageDatasetNames) {
        // Get the reference file
        std::string referenceFilePath = datasetFolder + "/" + name + "/references.txt";
        if(!boost::filesystem::exists(referenceFilePath) || !boost::filesystem::is_regular_file(referenceFilePath)) {
            std::cout << "Error: No reference file found for dataset '" << name << "'" << std::endl;
            return {};
        }

        std::vector<orthosfm::ReferenceCamera> cameras;

        // Read the file line by line
        std::ifstream file(referenceFilePath);
        std::string line;
        while (std::getline(file, line)) {
            // Tokenize the string
            std::vector<std::string> splitted;
            boost::split(splitted, line, boost::is_any_of(";"));

            // Create a reference struct
			orthosfm::ReferenceCamera cam;

            // Define coordinate transform
            Eigen::Matrix3d coordTransform;
            coordTransform << -1, 0, 0, 0, 0, -1, 0, 1, 0;

            // Spherical fix matrix (from OrthographicCamera)
            Eigen::Matrix3d R;
            R << 1, 0, 0, 0, 0, -1, 0, 1, 0;
            //coordTransform = R * coordTransform;

            // Extract the name
            cam.name = splitted[0];

            // Extract the position
            cam.position = -coordTransform * Eigen::Vector3d(std::stod(splitted[4]), std::stod(splitted[8]), std::stod(splitted[12]));

            // Extract the rotation matrix
            Eigen::Matrix3d mat;
            mat <<  std::stod(splitted[1]), std::stod(splitted[2]), std::stod(splitted[3]),
                    std::stod(splitted[5]), std::stod(splitted[6]), std::stod(splitted[7]),
                    std::stod(splitted[9]), std::stod(splitted[10]), std::stod(splitted[11]);
            //mat.transposeInPlace();

            // Apply transformation
            mat = coordTransform * mat;

            // Convert to phi theta rho and print (for now)
            /*Eigen::Vector3d angles = OrthographicCamera::basisToPhiThetaRho(mat, false);

            std::cout << "Reference Camera " << cam.name << " [phi: " << (angles[0]*180)/M_PI << "; theta: " << (angles[1] * 180) / M_PI <<
                      "; roll: " << (angles[2] * 180) / M_PI << "]"<< std::endl;*/

            // Create an orthographic camera
			orthosfm::View v(0, "");
            v.setDimensions(2048, 2048);
			orthosfm::OrthographicCamera orthoCam(v);
            orthoCam.convertFromAxis(mat.col(0), mat.col(1), mat.col(2));
            orthoCam.print();

            Eigen::Matrix3d extracted;
            extracted.col(0) = orthoCam.getXAxis();
            extracted.col(1) = orthoCam.getYAxis();
            extracted.col(2) = orthoCam.getZAxis();

            // Convert to quaternion
            Eigen::Quaternion<double> quaternion(extracted);
            cam.rotation = quaternion;

            /*std::cout << "Loaded origin [" << cam.position.x() << ", " << cam.position.y() << ", " << cam.position.z()
                << "] <-> Calculated origin: [" << orthoCam.getOrigin().x() << ", " << orthoCam.getOrigin().y() << ", " << orthoCam.getOrigin().z() << "]" << std::endl;*/

            //cam.position = orthoCam.getOrigin();

            // Add the reference to the list
            cameras.push_back(cam);
        }

        // Add all cameras to the list of references
        references.push_back(cameras);
    }

    for (int combiID = 0; combiID < combinations.size(); combiID++) {
        std::vector<double> angularErrors;
        std::vector<double> positionErrors;
        std::vector<double> runtimes;
        std::vector<double> poseEstimationTimes;

        for (int repetition = 0; repetition < repetitionsCount; repetition++) {
            // Create a unique project folder
            std::string iterProjectFolder =
                    projectFolderRoot + "/" + orthosfm::zfill(combiID + 1, 3) + "_" + orthosfm::zfill(repetition, 3) + "_" +
                    imageDatasetNames[combinations[combiID].first] + "_" + combinations[combiID].second.name;

            // Check if the folder exists
            if(!boost::filesystem::exists(iterProjectFolder)) {
                std::cout << "Failed to find result folder " << iterProjectFolder << std::endl;
                continue;
            }

            // Check if the time measurements and camera file exists
            if(!boost::filesystem::exists(iterProjectFolder + time_measurements_file_name) || !boost::filesystem::exists(iterProjectFolder + camera_file_name)) {
                std::cout << "Missing time measurements or camera files!" << std::endl;
                continue;
            }

            // Load the runtimes
            orthosfm::TimeMeasurements measurements = orthosfm::runtimesFromTxt(iterProjectFolder + time_measurements_file_name);

            runtimes.push_back(measurements.totalTime);
            poseEstimationTimes.push_back(measurements.poseEstimationTime);

            // Get the corresponding references
            std::vector<orthosfm::ReferenceCamera> correspondingReferences = references[combinations[combiID].first];

            // Load the cameras from a file
            std::vector<orthosfm::CameraTransform> calculatedCameras = orthosfm::importCameraFileAsMatrix(iterProjectFolder + camera_file_name);
            for(auto& cam : calculatedCameras) {
                cam.transform.block(0, 3, 3, 1) += positionOffset;
            }

            // Find out if the coordinate system is flipped
            bool isSolutionFlipped = false;
            {
                Eigen::Vector3d referenceDirection = (correspondingReferences[1].position.normalized() -
                                                      correspondingReferences[0].position.normalized()).normalized();

                // Extract the two origins
                Eigen::Vector3d ori1 = calculatedCameras[0].transform.block(0, 3, 3, 1);
                Eigen::Vector3d ori2 = calculatedCameras[1].transform.block(0, 3, 3, 1);

                Eigen::Vector3d calculatedDirection = (ori2.normalized() - ori1.normalized()).normalized();

                if (referenceDirection.dot(calculatedDirection) < 0) {
                    isSolutionFlipped = true;
                    std::cout << "The solution is flipped!" << std::endl;
                }
            }


            for (const auto &cam: calculatedCameras) {
                // Find the corresponding reference
                for (const auto &ref: correspondingReferences) {
                    if (cam.imageName == ref.name) {
                        // Convert the calculated camera rotation to a rotation matrix
                        Eigen::Matrix3d mat = cam.transform.block(0, 0, 3, 3);

                        // Also extract the position
                        Eigen::Vector3d pos = cam.transform.block(0, 3, 3, 1);

                        std::cout << orthosfm::vec2str(pos) << "<->" << orthosfm::vec2str(ref.position) << std::endl;

                        // Apply the flip matrix, if necessary

                        if (isSolutionFlipped) {
                            Eigen::Matrix3d transformToAlt;
                            transformToAlt << 1, 0, 0, 0, 1, 0, 0, 0, -1;
                            mat = transformToAlt * mat * transformToAlt;

                            Eigen::Matrix3d transformToAltPos;
                            transformToAltPos << -1, 0, 0, 0, -1, 0, 0, 0, 1;
                            pos = transformToAltPos * pos;
                        }

                        // Create a quaternion from the matrix
                        Eigen::Quaternion<double> quat(mat);

                        // Calculate the distance in radians
                        double distance = ref.rotation.angularDistance(quat);
                        //std::cout << "Error " << distance << std::endl;

                        double angleInDegrees = (distance*180)/M_PI;

                        // Put the result into the array
                        angularErrors.push_back(abs(angleInDegrees));

                        // Also evaluate the position error
                        double posErr = (pos.normalized() - ref.position.normalized()).norm();
                        positionErrors.push_back(abs(posErr));

                        // We can stop the search here
                        break;
                    }
                }
            }

        }

        // Calculate mean and std
        std::pair<double, double> errorMetrics = orthosfm::meanAndStd(angularErrors);
        std::pair<double, double> errorPosMetrics = orthosfm::meanAndStd(positionErrors);
        std::pair<double, double> runtimeMetrics = orthosfm::meanAndStd(runtimes);
        std::pair<double, double> poseTimeMetrics = orthosfm::meanAndStd(poseEstimationTimes);

        // Create a result
		orthosfm::FullPipelineResult result;
        result.meanAngularError = errorMetrics.first;
        result.stdAngularError = errorMetrics.second;
        result.meanPositionError = errorPosMetrics.first;
        result.stdPositionError = errorPosMetrics.second;
        result.runtime = runtimeMetrics.first;
        result.poseEstimationTime = poseTimeMetrics.first;
        result.configurationID = combiID;

        // Add to list
        results.push_back(result);
    }

    return results;
}

std::vector<orthosfm::FullPipelineResult> averagePerDataset(std::vector<orthosfm::FullPipelineResult> results,
                                                  std::vector<std::string> imageDatasetNames,
                                                  std::vector<std::pair<int, orthosfm::RunConfiguration>> combinations) {
    std::cout << "Averaging!" << std::endl;

    // Init the output
    std::vector<orthosfm::FullPipelineResult> filtered;

    // Loop over all datasets
    for(const auto& name : imageDatasetNames) {
        std::vector<double> angularMeanErrors;
        std::vector<double> angularStdErrors;
        std::vector<double> positionMeanErrors;
        std::vector<double> positionStdErrors;
        std::vector<double> runtimes;
        std::vector<double> poseEstimationTimes;
        std::vector<int> mergedIDs;

        // Loop over all results
        for(const auto& res : results) {
            // If this result corresponds to this dataset
            if(imageDatasetNames[combinations[res.configurationID].first] == name) {
                angularMeanErrors.push_back(res.meanAngularError);
                angularStdErrors.push_back(res.stdAngularError);
                positionMeanErrors.push_back(res.meanPositionError);
                positionStdErrors.push_back(res.stdPositionError);
                runtimes.push_back(res.runtime);
                poseEstimationTimes.push_back(res.poseEstimationTime);
                mergedIDs.push_back(res.configurationID);
            }
        }

        // Calculate mean and std
        std::pair<double, double> meanMetrics = orthosfm::meanAndStd(angularMeanErrors);
        std::pair<double, double> stdMetrics = orthosfm::meanAndStd(angularStdErrors);
        std::pair<double, double> meanPosMetrics = orthosfm::meanAndStd(positionMeanErrors);
        std::pair<double, double> stdPosMetrics = orthosfm::meanAndStd(positionStdErrors);
        std::pair<double, double> runtimeMetrics = orthosfm::meanAndStd(runtimes);
        std::pair<double, double> poseTimeMetrics = orthosfm::meanAndStd(poseEstimationTimes);

        // Create a result
        for(const auto& id : mergedIDs) {
			orthosfm::FullPipelineResult result;
            result.meanAngularError = meanMetrics.first;
            result.stdAngularError = stdMetrics.first;
            result.meanPositionError = meanPosMetrics.first;
            result.stdPositionError = stdPosMetrics.first;
            result.runtime = runtimeMetrics.first;
            result.poseEstimationTime = poseTimeMetrics.first;
            result.configurationID = id;
            filtered.push_back(result);
        }
    }

    return filtered;
}

void orthosfm::runFullPipelineTests(const std::string& datasetFolder, const std::string& projectFolderRoot, const std::string& executablePath) {
    std::cout << "Starting testbench for full pipeline tests!" << std::endl;
    bool SKIP_RECONSTRUCTIONS = false;
    bool AVERAGE_PER_DATASET = false;

    // First clean the old root folder
    if(boost::filesystem::is_directory(projectFolderRoot) && !SKIP_RECONSTRUCTIONS) {
        boost::filesystem::remove_all(projectFolderRoot);
        boost::filesystem::create_directory(projectFolderRoot);
    }

    // Get the working directory
    // FIXME: This only works for windows right now. Needs to be adapted for linux!
    std::string sfmExecutable = executablePath;
    if(!boost::filesystem::exists(sfmExecutable)) {
        std::cout << "ERROR: No sfm executable at the specified path!" << std::endl;
		return;
    }

    // Define all image datasets that should be evaluated
    std::vector<std::string> imageDatasetNames;
    //imageDatasetNames.emplace_back("Suzanne_GroupOfThree");

    /*imageDatasetNames.emplace_back("Suzanne_Circle");
    imageDatasetNames.emplace_back("Suzanne_3Lat");
    imageDatasetNames.emplace_back("Suzanne_3Lat_rotated");
    imageDatasetNames.emplace_back("Rings_Circle");
    imageDatasetNames.emplace_back("Rings_3Lat");
    imageDatasetNames.emplace_back("Rings_3Lat_rotated");
    imageDatasetNames.emplace_back("Dragon_Circle");
    imageDatasetNames.emplace_back("Dragon_3Lat");
    imageDatasetNames.emplace_back("Dragon_3Lat_rotated");*/

    /*imageDatasetNames.emplace_back("01_Dragon_horizontal");
    imageDatasetNames.emplace_back("02_Dragon_vertical");
    imageDatasetNames.emplace_back("03_Dragon_rotated");
    imageDatasetNames.emplace_back("04_Suzanne_horizontal");
    imageDatasetNames.emplace_back("05_Suzanne_vertical");
    imageDatasetNames.emplace_back("06_Suzanne_rotated");
    imageDatasetNames.emplace_back("07_Rings_horizontal");
    imageDatasetNames.emplace_back("08_Rings_vertical");
    imageDatasetNames.emplace_back("09_Rings_rotated");*/

    // Real datasets
    imageDatasetNames.emplace_back("Elephant");
    imageDatasetNames.emplace_back("Valdivia");

    std::vector<std::vector<SFM_SOLVER_TYPE>> supportedSolvers;
    //supportedSolvers.push_back(std::vector<SFM_SOLVER_TYPE>{SFM_SOLVER_TYPE::ORTHO_EULER_HORIZONTAL, SFM_SOLVER_TYPE::ORTHO_EULER_HORIZONTAL_VERTICAL, SFM_SOLVER_TYPE::ORTHO_EULER_ALL_DOF, SFM_SOLVER_TYPE::ORTHO_QUATERNION});

    /*supportedSolvers.push_back(std::vector<SFM_SOLVER_TYPE>{SFM_SOLVER_TYPE::ORTHO_EULER_HORIZONTAL, SFM_SOLVER_TYPE::ORTHO_EULER_HORIZONTAL_VERTICAL, SFM_SOLVER_TYPE::ORTHO_EULER_ALL_DOF, SFM_SOLVER_TYPE::ORTHO_QUATERNION});
    supportedSolvers.push_back(std::vector<SFM_SOLVER_TYPE>{SFM_SOLVER_TYPE::ORTHO_EULER_HORIZONTAL_VERTICAL, SFM_SOLVER_TYPE::ORTHO_EULER_ALL_DOF, SFM_SOLVER_TYPE::ORTHO_QUATERNION});
    supportedSolvers.push_back(std::vector<SFM_SOLVER_TYPE>{SFM_SOLVER_TYPE::ORTHO_EULER_ALL_DOF, SFM_SOLVER_TYPE::ORTHO_QUATERNION});
    supportedSolvers.push_back(std::vector<SFM_SOLVER_TYPE>{SFM_SOLVER_TYPE::ORTHO_EULER_HORIZONTAL, SFM_SOLVER_TYPE::ORTHO_EULER_HORIZONTAL_VERTICAL, SFM_SOLVER_TYPE::ORTHO_EULER_ALL_DOF, SFM_SOLVER_TYPE::ORTHO_QUATERNION});
    supportedSolvers.push_back(std::vector<SFM_SOLVER_TYPE>{SFM_SOLVER_TYPE::ORTHO_EULER_HORIZONTAL_VERTICAL, SFM_SOLVER_TYPE::ORTHO_EULER_ALL_DOF, SFM_SOLVER_TYPE::ORTHO_QUATERNION});
    supportedSolvers.push_back(std::vector<SFM_SOLVER_TYPE>{SFM_SOLVER_TYPE::ORTHO_EULER_ALL_DOF, SFM_SOLVER_TYPE::ORTHO_QUATERNION});
    supportedSolvers.push_back(std::vector<SFM_SOLVER_TYPE>{SFM_SOLVER_TYPE::ORTHO_EULER_HORIZONTAL, SFM_SOLVER_TYPE::ORTHO_EULER_HORIZONTAL_VERTICAL, SFM_SOLVER_TYPE::ORTHO_EULER_ALL_DOF, SFM_SOLVER_TYPE::ORTHO_QUATERNION});
    supportedSolvers.push_back(std::vector<SFM_SOLVER_TYPE>{SFM_SOLVER_TYPE::ORTHO_EULER_HORIZONTAL_VERTICAL, SFM_SOLVER_TYPE::ORTHO_EULER_ALL_DOF, SFM_SOLVER_TYPE::ORTHO_QUATERNION});
    supportedSolvers.push_back(std::vector<SFM_SOLVER_TYPE>{SFM_SOLVER_TYPE::ORTHO_EULER_ALL_DOF, SFM_SOLVER_TYPE::ORTHO_QUATERNION});*/

    // Real datasets
    supportedSolvers.push_back(std::vector<SFM_SOLVER_TYPE>{SFM_SOLVER_TYPE::ORTHO_EULER_HORIZONTAL, SFM_SOLVER_TYPE::ORTHO_EULER_HORIZONTAL_VERTICAL, SFM_SOLVER_TYPE::ORTHO_EULER_ALL_DOF, SFM_SOLVER_TYPE::ORTHO_QUATERNION});
    supportedSolvers.push_back(std::vector<SFM_SOLVER_TYPE>{SFM_SOLVER_TYPE::ORTHO_EULER_HORIZONTAL, SFM_SOLVER_TYPE::ORTHO_EULER_HORIZONTAL_VERTICAL, SFM_SOLVER_TYPE::ORTHO_EULER_ALL_DOF, SFM_SOLVER_TYPE::ORTHO_QUATERNION});


    // Check if the datasets exist
    if(boost::filesystem::exists(datasetFolder) && boost::filesystem::is_directory(datasetFolder)) {
        for(const auto& name : imageDatasetNames) {
            if(!boost::filesystem::exists(datasetFolder + "/" + name)) {
                std::cout << "Error: Failed to find dataset with name '" << name << "'" << std::endl;
                return;
            }
        }
    }
    else {
        std::cout << "Error: Invalid dataset folder!" << std::endl;
        return;
    }

    // Define the different run configurations
    std::vector<RunConfiguration> configs;
    /*configs.emplace_back(SFM_SOLVER_TYPE::ORTHO_EULER_HORIZONTAL, "Euler_Phi");
    configs.emplace_back(SFM_SOLVER_TYPE::ORTHO_EULER_HORIZONTAL_VERTICAL, "Euler_Phi_Theta");
    configs.emplace_back(SFM_SOLVER_TYPE::ORTHO_EULER_ALL_DOF, "Euler_All");*/
    configs.emplace_back(SFM_SOLVER_TYPE::ORTHO_QUATERNION, "Quaternion");

    // Create all combinations of datasets and run configurations
    std::vector<std::pair<int, RunConfiguration>> combinations;
    for(int i=0; i<imageDatasetNames.size(); i++) {
        for(const auto& config : configs) {
            // Check if the dataset supports the solver specified in the config
            if(std::find(supportedSolvers[i].begin(), supportedSolvers[i].end(), config.solverType) != supportedSolvers[i].end()) {
                combinations.emplace_back(i, config);
            }
        }
    }

    // Define how many repetitions to do
    int repetitionsCount = 5;

    if(SKIP_RECONSTRUCTIONS) {
        // Load results from file
        std::vector<FullPipelineResult> results = evaluateResultsFromFolders(projectFolderRoot, datasetFolder,
                                                                             imageDatasetNames, configs, combinations,
                                                                             repetitionsCount);

        // If wished: Average per dataset
        if(AVERAGE_PER_DATASET) {
            results = averagePerDataset(results, imageDatasetNames, combinations);
        }

        // Save to csv
        saveToResults(projectFolderRoot, imageDatasetNames, configs, combinations, results);
        return;
    }

    // For each combination
    for(int combiID=0; combiID<combinations.size(); combiID++) {
        for(int repetition=0; repetition<repetitionsCount; repetition++) {
            std::cout << "\nRunning combination " << combiID+1 << "/" << combinations.size() << " ["
            << imageDatasetNames[combinations[combiID].first] << "; " << combinations[combiID].second.name
            << "] with repetition " << repetition << std::endl;

            // Create a unique project folder
            std::string iterProjectFolder = projectFolderRoot + "/" + zfill(combiID+1, 3) + "_" + zfill(repetition, 3) + "_" + imageDatasetNames[combinations[combiID].first] + "_" + combinations[combiID].second.name;

            // Create a reconstruction config
            reconstruction_config reco_config;
            reco_config.solver = combinations[combiID].second.solverType;
            reco_config.projectFolder = iterProjectFolder;
            reco_config.imageFolder = datasetFolder + "/" + imageDatasetNames[combinations[combiID].first] + "/Images";
            reco_config.maskFolder = datasetFolder + "/" + imageDatasetNames[combinations[combiID].first] + "/Masks";

            // Clean the project folder before use
            createProject(reco_config.projectFolder, true);

            // Do a full reconstruction
            try {
                // Directly call the reconstruction code
                //cameras = reconstruct(reco_config);

                // Parse the call to sfm pga
                char command[1024];
               // std::sprintf(command, "%s %s %s --mask-folder %s --downscale-factor=1 --overwrite --solver %i --export-pairwise-tracks", sfmExecutable.c_str(),
               //          reco_config.projectFolder.c_str(), reco_config.imageFolder.c_str(), reco_config.maskFolder.c_str(), solverTypeToIndex(reco_config.solver));

                std::sprintf(command, "%s %s %s --downscale-factor=1 --overwrite --solver %i --export-pairwise-tracks", sfmExecutable.c_str(),
                             reco_config.projectFolder.c_str(), reco_config.imageFolder.c_str(), solverTypeToIndex(reco_config.solver));

                std::cout << "Calling command: '" << command << "'" << std::endl;

                // Call the command
                system(command);
            }
            catch(const std::exception& e){
                continue;
            }


        }

        // Load results from file
        std::vector<FullPipelineResult> results = evaluateResultsFromFolders(projectFolderRoot, datasetFolder,
                                                                             imageDatasetNames, configs, combinations,
                                                                             repetitionsCount);
        // Save to csv
        saveToResults(projectFolderRoot, imageDatasetNames, configs, combinations, results);
    }



}
