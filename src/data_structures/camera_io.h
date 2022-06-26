/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#ifndef ORTHO_SFM_CAMERAIO_H
#define ORTHO_SFM_CAMERAIO_H

#include <vector>
#include <Eigen/Dense>

#include <data_structures/Camera.h>

namespace orthosfm
{
	struct CameraTransform
	{
		std::string imageName;
		Eigen::Matrix4d transform;
	};

	// Export cameras to a file
	void exportCamerasToFile(const std::vector<std::shared_ptr<Camera>>& cameras, const std::string& filePath);

	// Import cameras as 4x4 matrices
	std::vector<CameraTransform> importCameraFileAsMatrix(const std::string& filePath);
}

#endif //ORTHO_SFM_CAMERAIO_H