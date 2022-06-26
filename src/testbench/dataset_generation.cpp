/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#include "dataset_generation.h"
#include <random>
#include <fstream>
#include <boost/algorithm/string.hpp>

std::vector<std::shared_ptr<orthosfm::OrthographicCamera>> orthosfm::generateGroundTruthCameras(const std::vector<View>& views) {
    std::vector<std::shared_ptr<OrthographicCamera>> cameras;

    std::uniform_real_distribution<double> thetaRnd(-30,30);
    std::uniform_real_distribution<double> rollRnd(-30,30);
    std::default_random_engine re;

    // Place 16 Cameras in 22.5 degree distances with random theta and roll
    for(int i=0; i<16;i++) {
        // Make the first camera identity
        if(i == 0) {
            std::shared_ptr<OrthographicCamera> cam = std::make_shared<OrthographicCamera>(views[i], 0, 0, 0);
            cameras.push_back(cam);
            cam->print();
        }
        else {
            std::shared_ptr<OrthographicCamera> cam = std::make_shared<OrthographicCamera>(views[i],  22.5*i, thetaRnd(re), rollRnd(re));
            cameras.push_back(cam);
            cam->print();
        }


    }

    return cameras;
}

std::vector<orthosfm::TestDataset> orthosfm::generateDatasets(const std::vector<std::string>& modelPaths) {
    // Create the views
    std::vector<View> views;
    views.reserve(16);
    for(int i=0; i<16; i++) {
        views.emplace_back(i, "Synthetic view " + std::to_string(i));
        views[i].setDimensions(2048, 2048);
    }

    // Create the ground truth cameras
    std::vector<std::shared_ptr<OrthographicCamera>> cameras = generateGroundTruthCameras(views);

    // Load the models as point clouds
    std::vector<PointCloud> pointClouds = loadPointClouds(modelPaths);

    // Create datasets based on the point clouds
    std::vector<TestDataset> datasets;
    for(const auto& pointCloud : pointClouds) {
        // Create a new dataset
        TestDataset dataset;
        dataset.views = views;
        dataset.groundTruthCameras = cameras;

        // Keep a global feature count
        int counter = 0;

        // Create a track for each point
        for(int pointID=0; pointID<pointCloud.size(); pointID++) {
            // Create a track
            Track track;

            // Project the point onto all cameras
            for(const auto& cam : cameras) {
                // Convert the 3d point to a homogenous 4d point
                Eigen::Vector4d homogenousPoint = Eigen::Vector4d(pointCloud[pointID].x(), pointCloud[pointID].y(), pointCloud[pointID].z(), 1);

                Eigen::Vector2d pixels = cam->projectPointOntoImagePlane(homogenousPoint);
                track.add(Feature(cam->getView()->getID(), pointID, counter, pixels.x(), pixels.y()));
                counter++;
            }

            // Add track to dataset
            dataset.tracks.push_back(track);
        }

        datasets.push_back(dataset);
    }

    // FixME: Include normals in point clouds to filter out which camera can actually see which point

    // Return the datasets
    return datasets;
}

std::vector<PointCloud> orthosfm::loadPointClouds(const std::vector<std::string> &modelPaths) {
    // Init the output vector
    std::vector<PointCloud> clouds;

    // Loop over the models
    for(int i=0; i<modelPaths.size(); i++) {
        // Create a point cloud
        PointCloud cloud;

        // Store if we are currenly reading the header
        bool readingHeader = true;

        // Read file line by line
        std::ifstream file(modelPaths[i]);
        std::string line;
        while (std::getline(file, line))
        {
            if(readingHeader) {
                if(boost::starts_with(line, "end_header")) {
                    readingHeader = false;
                }
                continue;
            }


            // Tokenize the string
            std::vector<std::string> splitted;
            boost::split(splitted,line,boost::is_any_of(" "));

            // Create a Point
            Eigen::Vector3d vec(std::stod(splitted[0]), std::stod(splitted[1]), std::stod(splitted[2]));
            cloud.push_back(vec);
        }

        // Add to output
        clouds.push_back(cloud);

        // Print
        std::cout << "Read point cloud with " << cloud.size() << " points" << std::endl;
    }

    return clouds;
}
