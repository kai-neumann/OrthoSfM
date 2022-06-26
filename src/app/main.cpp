/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
 */

#include <iostream>
#include <string>
#include <exception>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <sfm/reconstruct.h>
#include <data_structures/view.h>
#include <data_structures/project_io.h>


int main(int argc, char** argv) {
    // Declare additional supported options.
    std::string projectFolder, imageFolder, maskFolder, trackFile;
    int downscaleFactor = 1;
    int solverType = 0;
    boost::program_options::options_description desc("OrthoSfM: A simple Structure from Motion (SfM) implementation primarily for orthographic images \n\nUsage:\tOrthoSfM.exe [project folder] [image folder] --option1 value1 --option2 value2 "
                                                     "\nor\tOrthoSfM.exe --project-folder [project folder] --image-folder [image folder] --option1 value1 --option2 value2\n\nAllowed Additional options");
    desc.add_options()
            ("help", "produce help message")
            ("project-folder", boost::program_options::value<std::string>(&projectFolder), "The folder where all temporary files and results should be saved. This folder is required.")
            ("image-folder", boost::program_options::value<std::string>(&imageFolder), "A folder containing the images that should be used as input. This folder is required.")
            ("calculated-tracks", boost::program_options::value<std::string>(&trackFile), "A track file containing previously calculated tracks can be specified to avoid calculating them on the fly")
            ("export-pairwise-tracks", "Exports the tracks in a pairwise fashion usefull as input to other sfm implementations")
            ("mask-folder", boost::program_options::value<std::string>(&maskFolder), "A folder containing a mask per image in the format {imageName}_mask.png")
            ("downscale-factor", boost::program_options::value<int>(&downscaleFactor), "The number of times the input images are downscaled before using them for calculation")
            ("overwrite", "Resets any old project in the same project folder and overwrites it with the new project.")
            ("solver", boost::program_options::value<int>(&solverType), "The solver used on the dataset [0 = Quaternion, 1 = EulerHorizontal, 2 = EulerHorizontalVertical, 3 = EulerAll]")
            ;

    boost::program_options::positional_options_description p;
    p.add("project-folder", 1);
    p.add("image-folder", 1);

    boost::program_options::variables_map vm;
    try {
        boost::program_options::store(boost::program_options::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
        boost::program_options::notify(vm);
    }
    catch(std::exception& e){
        std::cerr << "Error: Invalid program arguments!" << std::endl << std::endl;
        std::cout << desc << "\n";
        return EXIT_FAILURE;
    }

    if (vm.count("help")) {
        std::cout << desc << "\n";
        return 1;
    }

    // Check if a project folder was specified
    if(!vm.count("project-folder")) {
        std::cerr << "Error: Please specify a project folder!\n" << std::endl;
        std::cerr << desc << "\n";
        return EXIT_FAILURE;
    }

    // Check if an image folder was specified
    if(!vm.count("image-folder")) {
        std::cerr << "Error: Please specify an image folder!\n" << std::endl;
        std::cerr << desc << "\n";
        return EXIT_FAILURE;
    }

    // Check if the image folder is valid
    if(!boost::filesystem::exists(imageFolder)) {
        std::cerr << "Error: The specifed image folder does not exist!\n" << std::endl;
        return EXIT_FAILURE;
    }

    // If a calculated tracks file was specified: Check if it exists
    if(vm.count("calculated-tracks") && (!boost::filesystem::exists(trackFile) || !boost::filesystem::is_regular_file(trackFile))) {
        std::cerr << "Error: The specifed track file is invalid or does not exist!\n" << std::endl;
        return EXIT_FAILURE;
    }

    // Read in if the tracks should be exported
    bool exportPairwiseTracks = vm.count("export-pairwise-tracks");

    // If a mask folder was specified: Check if it exists
    if(vm.count("mask-folder") && (!boost::filesystem::exists(maskFolder) || !boost::filesystem::is_directory(maskFolder))) {
        std::cerr << "Error: The specifed mask folder is invalid or does not exist!\n" << std::endl;
        return EXIT_FAILURE;
    }

    // Get the solver type
	orthosfm::SFM_SOLVER_TYPE solverTypeStruct = orthosfm::SFM_SOLVER_TYPE::ORTHO_QUATERNION;
    if(vm.count("solver")) {
        solverTypeStruct = orthosfm::solverTypeFromIndex(solverType);
        if(solverType == orthosfm::SFM_SOLVER_TYPE::NONE) {
            return EXIT_FAILURE;
        }
    }


    // Create the project
    if(!orthosfm::createProject(projectFolder, vm.count("overwrite"))) {
        return EXIT_FAILURE;
    }

    // If everything is in order: Start the reconstruction!
    std::cout << "========== Starting reconstruction ==========" << std::endl;
    std::cout << "Project Folder = " << projectFolder << std::endl;
    std::cout << "Image Folder = " << imageFolder << std::endl;
    std::cout << "Solver Type = " << solverTypeToString(solverTypeStruct) << std::endl;
    std::cout << "=============================================" << std::endl << std::endl;

    // Fill the config struct with settings
	orthosfm::reconstruction_config config;
    config.projectFolder = projectFolder;
    config.imageFolder = imageFolder;
    config.maskFolder = maskFolder;
    config.trackFile = trackFile;
    config.downscaleFactor = downscaleFactor;
    config.solver = solverTypeStruct;
    config.exportPairwiseTracks = exportPairwiseTracks;

    // Run the reconstruction
    reconstruct(config);

    return EXIT_SUCCESS;
}

