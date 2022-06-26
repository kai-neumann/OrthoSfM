/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#include "camera_io.h"

#include "fstream"
#include "iostream"
#include <boost/algorithm/string.hpp>

void orthosfm::exportCamerasToFile(const std::vector<std::shared_ptr<Camera>> &cameras, const std::string &filePath) {
    std::cout << "Exporting alignment to " << filePath << std::endl;

    std::ofstream outfile;
    outfile.open(filePath);

    // Loop over all cameras
    for(const auto& cam : cameras) {
        // Create a 4x4 matrix from the camera
        Eigen::Matrix4d matrix;
        matrix <<   cam->getXAxis().x(), cam->getYAxis().x(), cam->getZAxis().x(), cam->getOrigin().x(),
                    cam->getXAxis().y(), cam->getYAxis().y(), cam->getZAxis().y(), cam->getOrigin().y(),
                    cam->getXAxis().z(), cam->getYAxis().z(), cam->getZAxis().z(), cam->getOrigin().z(),
                    0, 0, 0, 1;

        // Serialize to string
        outfile << cam->getView()->getImageName() + ";";
        outfile << std::to_string(matrix(0, 0)) + "," + std::to_string(matrix(0, 1)) + "," + std::to_string(matrix(0, 2)) + "," + std::to_string(matrix(0, 3)) + ",";
        outfile << std::to_string(matrix(1, 0)) + "," + std::to_string(matrix(1, 1)) + "," + std::to_string(matrix(1, 2)) + "," + std::to_string(matrix(1, 3)) + ",";
        outfile << std::to_string(matrix(2, 0)) + "," + std::to_string(matrix(2, 1)) + "," + std::to_string(matrix(2, 2)) + "," + std::to_string(matrix(2, 3)) + ",";
        outfile << std::to_string(matrix(3, 0)) + "," + std::to_string(matrix(3, 1)) + "," + std::to_string(matrix(3, 2)) + "," + std::to_string(matrix(3, 3));
        outfile << std::endl;
    }

    outfile.close();
}

std::vector<orthosfm::CameraTransform> orthosfm::importCameraFileAsMatrix(const std::string &filePath) {
    // Init the output vector
    std::vector<CameraTransform> outMatrices;

    // Read file line by line
    std::ifstream file(filePath);
    std::string line;
    while (std::getline(file, line)) {
        // Init the new output
        CameraTransform transform;

        // Tokenize the string
        std::vector<std::string> splitted;
        boost::split(splitted,line,boost::is_any_of(";"));
        transform.imageName = splitted[0];

        std::vector<std::string> s;
        boost::split(s,splitted[1],boost::is_any_of(","));
        Eigen::Matrix4d mat;
        mat <<  std::stod(s[0]), std::stod(s[1]), std::stod(s[2]), std::stod(s[3]),
                std::stod(s[4]), std::stod(s[5]), std::stod(s[6]), std::stod(s[7]),
                std::stod(s[8]), std::stod(s[9]), std::stod(s[10]), std::stod(s[11]),
                std::stod(s[12]), std::stod(s[13]), std::stod(s[14]), std::stod(s[15]);

        transform.transform = mat;
        outMatrices.push_back(transform);
    }

    return outMatrices;
}
