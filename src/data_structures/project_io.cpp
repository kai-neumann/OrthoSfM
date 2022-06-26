/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#include "project_io.h"

#include <boost/filesystem.hpp>
#include <boost/exception/exception.hpp>
#include "iostream"

bool orthosfm::createProject(const std::string& projectPath, bool overwrite) {
    // Check if there already is a folder at the given project path
    if(boost::filesystem::exists(projectPath) && boost::filesystem::is_directory(projectPath) && !boost::filesystem::is_empty(projectPath)) {
        // If overwrite is set
        if(overwrite) {
            // Check if the existing directory is a project
            if(isProject(projectPath)) {
                // Clean the old project
                cleanExistingProject(projectPath);
            }
            else {
                std::cerr << "ERROR: The specified folder can not be overwritten as it does not contain a project" << std::endl;
                return false;
            }
        }
        // Else print an error
        else {
            std::cerr << "ERROR: The specified project folder is not empty. To overwrite it use the \"--overwrite\" argument" << std::endl;
            return false;
        }
    }

    // Create a new directory in the specified location
    try {
        boost::filesystem::create_directory(projectPath);
    }
    catch(boost::exception& exception){
        std::cerr << "ERROR: Failed to create the specified project folder. Exiting." << std::endl;
        return false;
    }

    // Create a small text file to indicate that this is a project
    std::ofstream outfile;
    outfile.open(projectPath + "/project.txt");
    outfile << "This is a reconstruction project created by sfm-pga." << std::endl;
    outfile.close();

    return true;
}

bool orthosfm::isProject(const std::string& dir) {
    return boost::filesystem::exists(dir) && boost::filesystem::is_directory(dir) && boost::filesystem::exists(dir + "/project.txt") && boost::filesystem::is_regular_file(dir + "/project.txt");
}

void orthosfm::cleanExistingProject(const std::string& dir) {
    std::cout << "Deleting contents of previous project in same project folder.." << std::endl;
    boost::filesystem::remove_all(dir);
}
