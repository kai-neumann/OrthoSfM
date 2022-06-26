/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#include "ReconstructionAlgorithm.h"

void orthosfm::ReconstructionAlgorithm::mergeIntoGlobal(const std::vector<std::shared_ptr<Camera>> &localCameras, std::vector<std::shared_ptr<Camera>> &globalCameras) {
    // For each local camera
    for(const auto& cam : localCameras) {
        // Check if there is an existing global camera with the same id
        bool found = false;
        for(const auto& globalCam : globalCameras) {
            if(cam->getView()->getID() == globalCam->getView()->getID()) {
                found = true;
                break;
            }
        }

        // If it was not found: add it to the list of global cameras
        if(!found) {
            globalCameras.push_back(cam);
        }
    }
}
