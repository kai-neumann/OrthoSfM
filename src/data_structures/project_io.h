/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#ifndef ORTHO_SFM_PROJECTIO_H
#define ORTHO_SFM_PROJECTIO_H

#include <string>

namespace orthosfm
{

	// Create a project, and overwrite the old one if necessary
	bool createProject(const std::string& projectPath, bool overwrite);

	// Check if a directory is a reconstruction project
	bool isProject(const std::string& dir);

	// Clean a given project directory
	void cleanExistingProject(const std::string& dir);
}

#endif //ORTHO_SFM_PROJECTIO_H
