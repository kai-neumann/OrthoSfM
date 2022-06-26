/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#include <fstream>
#include "timing.h"
#include <vector>
#include <boost/algorithm/string.hpp>

double orthosfm::elapsedSeconds(const std::chrono::time_point<std::chrono::steady_clock> &start, const std::chrono::time_point<std::chrono::steady_clock> &end) {
    return ((double)std::chrono::duration_cast<std::chrono::nanoseconds> (end - start).count())/1000000000.;
}

void orthosfm::saveRuntimesToTxt(const std::string& path, double init, double track, double pose, double total) {
    std::ofstream outfile;
    outfile.open(path);

    outfile << "Initialization Time [s] = " << init << std::endl;
    outfile << "Track Building Time [s] = " << track << std::endl;
    outfile << "Pose Estimation Time [s] = " << pose << std::endl;
    outfile << "Total Time [s] = " << total << std::endl;

    outfile.close();
}

orthosfm::TimeMeasurements orthosfm::runtimesFromTxt(const std::string &path) {
    TimeMeasurements measurements;

    // Read file line by line
    std::ifstream file(path);
    std::string line;
    int currentLine = 0;
    while (std::getline(file, line))
    {
        // Tokenize the string
        std::vector<std::string> splitted;
        boost::split(splitted,line,boost::is_any_of("="));

        if(currentLine == 0) measurements.initTime = std::stod(splitted[1]);
        if(currentLine == 1) measurements.trackBuildingTime = std::stod(splitted[1]);
        if(currentLine == 2) measurements.poseEstimationTime = std::stod(splitted[1]);
        if(currentLine == 3) measurements.totalTime = std::stod(splitted[1]);

        currentLine++;
    }

    return measurements;
}
