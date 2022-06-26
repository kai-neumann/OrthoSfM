/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#include "synthethic_tests.h"
#include "full_pipeline_tests.h"

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

int main(int argc, char** argv) {
    // Declare additional supported options.
    std::string projectFolder, datasetFolder, executable;
    boost::program_options::options_description desc("OrthoSfM testbench. A simple application to systematically test and evaluate the sfm-pga software \n\nUsage:\torthosfm-testbench.exe [project folder] [dataset folder] [executable path] --option1 value1 --option2 value2 "
                                                     "\nor\torthosfm-testbench.exe --project-folder [project folder] --dataset-folder [image folder] --executable [executable path] --option1 value1 --option2 value2\n\nAllowed Additional options");
    desc.add_options()
            ("help", "produce help message")
            ("project-folder", boost::program_options::value<std::string>(&projectFolder), "The folder where all temporary files and results should be saved. This folder is required.")
            ("dataset-folder", boost::program_options::value<std::string>(&datasetFolder), "A folder containing the images that should be used as input. Only necessary for non-synthetic tests.")
            ("executable", boost::program_options::value<std::string>(&executable), "A path to the OrthoSfM-app executable that should be called during testing")
            ("synthetic", "Specify if synthetic data should be used for evaluating the robustness of the software.")
            ;

    boost::program_options::positional_options_description p;
    p.add("project-folder", 1);
    p.add("dataset-folder", 1);
    p.add("executable", 1);

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

	// Check if an executable was specified
	if(!vm.count("executable")) {
		std::cerr << "Error: Please specify the path to the executable!\n" << std::endl;
		std::cerr << desc << "\n";
		return EXIT_FAILURE;
	}

    if(!vm.count("synthetic") && !vm.count("dataset-folder")) {
        std::cerr << "Error: Please specify a dataset folder to run dataset based tests! Alternatively switch to synthetic tests using the '--synthetic' flag.\n" << std::endl;
        std::cerr << desc << "\n";
        return EXIT_FAILURE;
    }

    if(vm.count("synthetic")) {
		orthosfm::runSyntheticRobustnessTests(argc, argv);
    }
    else {
		orthosfm::runFullPipelineTests(datasetFolder, projectFolder, executable);
    }

}

