/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#include "common.h"
#include <opencv2/opencv.hpp>
#include <data_structures/colored_point_cloud.h>
#include <sstream>


std::vector<std::string> orthosfm::getImagesInFolder(const std::string& folder) {
    std::vector<std::string> images;

    // Check if the given folder exists and is actually a folder
    if(!boost::filesystem::exists(folder) || !boost::filesystem::is_directory(folder)) {
        std::cout << "Error: The specified image folder does not exit or is invalid." << std::endl;
    }

    // Iterate over all contents of the folder
    for(auto& entry : boost::make_iterator_range(boost::filesystem::directory_iterator(boost::filesystem::path(folder)), {})) {
        if(is_regular_file(entry)) {
            // Get the file extension
            std::string ext = boost::filesystem::extension(entry);

            // Check if this is a known image extension
            if(ext == ".tiff" || ext == ".tif" || ext == ".png" || ext == ".jpeg" || ext == ".jpg") {
                // Add to output list
                images.push_back(absolute(entry.path()).string());
            }
        }
    }

    return images;
}

std::string orthosfm::zfill(const int &value, const int &zeros) {
    // Convert the value to a string
    std::string str = std::to_string(value);

    if(zeros > str.size())
        str.insert(0, zeros - str.size(), '0');

    return str;
}

std::string orthosfm::type2str(int type) {
    std::string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch ( depth ) {
        case CV_8U:  r = "8U"; break;
        case CV_8S:  r = "8S"; break;
        case CV_16U: r = "16U"; break;
        case CV_16S: r = "16S"; break;
        case CV_32S: r = "32S"; break;
        case CV_32F: r = "32F"; break;
        case CV_64F: r = "64F"; break;
        default:     r = "User"; break;
    }

    r += "C";
    r += (chans+'0');

    return r;
}

std::vector<orthosfm::Track> orthosfm::filterTracksToAvailableCameras(const std::vector<std::shared_ptr<Camera>> &cameras, const std::vector<Track> &tracks,
                                                  bool onlyFullSizeTracks, bool keepAdditionalCamera) {
    // Create a vector of ids
    std::vector<unsigned int> ids;
    for(const auto& cam : cameras) {
        ids.push_back(cam->getView()->getID());
    }

    // call overloaded method
    return filterTracksToAvailableCameras(ids, tracks, onlyFullSizeTracks, keepAdditionalCamera);
}

std::vector<orthosfm::Track> orthosfm::filterTracksToAvailableCameras(const std::vector<unsigned int> &ids, const std::vector<Track> &tracks,
                                                  bool onlyFullSizeTracks, bool keepAdditionalCamera) {
    // Initialize the output list
    std::vector<Track> filtered;

    // Create a set of available camera ids
    std::set<unsigned int> availableIDs;
    for(const auto& id : ids) {
        availableIDs.insert(id);
    }

    // Loop over all tracks
    for(const auto& track : tracks) {
        // Create a new track
        Track cur;

        // Loop over all features of the track
        for(int featureID=0; featureID<track.size(); featureID++) {
            // If the features viewID is available: Add it to the new track
            if(availableIDs.find(track.get(featureID).viewID) != availableIDs.end()) {
                cur.add(track.get(featureID));
            }
        }


        // If only full tracks should be kept: Check if the new track contains exactly all camera
        if(onlyFullSizeTracks) {
            if(cur.size() == ids.size()) {
                if(keepAdditionalCamera) {
                    filtered.push_back(track);
                }
                else {
                    filtered.push_back(cur);
                }

            }
        }
        else {
            // If the track contains at least two features: Add it to the output
            if(cur.size() > 1) {
                if(keepAdditionalCamera) {
                    filtered.push_back(track);
                }
                else {
                    filtered.push_back(cur);
                }

            }
        }


    }

    return filtered;
}

void orthosfm::savePointsToPLY(const std::string &outPath, std::vector<std::shared_ptr<Camera>>& cameras, const std::vector<Track> &tracks, const std::shared_ptr<ReconstructionAlgorithm>& algorithm) {
    // Count the number of points inside the cameras
    int pointsCount = 0;
    for(int trackID=0; trackID<tracks.size(); trackID++) {
        if(tracks[trackID].hasPoint()) {
            pointsCount++;
        }
    }

    if (boost::filesystem::exists(outPath)) {
        boost::filesystem::remove(outPath);
    }

    std::ofstream filestream;
    filestream.open(outPath);

    filestream << "ply" << std::endl;
    filestream << "format ascii 1.0" << std::endl;
    filestream << "element vertex " << pointsCount << std::endl;
    filestream << "property float x" << std::endl;
    filestream << "property float y" << std::endl;
    filestream << "property float z" << std::endl;
    filestream << "property uchar red" << std::endl;
    filestream << "property uchar green" << std::endl;
    filestream << "property uchar blue" << std::endl;
    filestream << "end_header" << std::endl;

    for(int trackID=0; trackID<tracks.size(); trackID++) {
        // Skip if no point available
        if(!tracks[trackID].hasPoint()) {
            continue;
        }

        // Encode the point
        filestream << tracks[trackID].getPoint().x() << " ";
        filestream << tracks[trackID].getPoint().y() << " ";
        filestream << tracks[trackID].getPoint().z() << " ";

        // For now always use the color of the first feature
        filestream << (int)tracks[trackID].get(0).r << " ";
        filestream << (int)tracks[trackID].get(0).g << " ";
        filestream << (int)tracks[trackID].get(0).b << std::endl;
    }

    filestream.close();

    std::cout << "Saved sparse cloud to ply." << std::endl;
}

void orthosfm::printCameras(const std::vector<std::shared_ptr<Camera>> &cameras, bool ordered) {
    if(ordered) {
        // Get the highest camera id
        unsigned int maxID = 0;
        for (const auto &cam: cameras) {
            if (cam->getView()->getID() > maxID) {
                maxID = cam->getView()->getID();
            }
        }

        for(unsigned int i=0; i<maxID+1; i++) {
            for (const auto &cam: cameras) {
                if (cam->getView()->getID() == i) {
                    cam->print();
                }
            }
        }
    }
    else {
        for(const auto& cam : cameras) {
            cam->print();
        }
    }


    std::cout << std::endl;
}

std::pair<double, double> orthosfm::meanAndStd(const std::vector<double>& values) {
    // First calculate the mean
    double sum = 0;
    for(double val : values) {
        // Add to mean sum
        sum += val;
    }

    // Calculate the mean
    double meanError = sum / ((double) values.size());

    // Then calculate the standard deviation
    double squaredSum = 0;
    for(double val : values) {
        // Add to squared sum
        squaredSum += pow(val - meanError, 2);
    }

    double stdError = sqrt(squaredSum / ((double) values.size()));

    return std::make_pair(meanError, stdError);
}

unsigned int orthosfm::solverTypeToIndex(const SFM_SOLVER_TYPE &solver) {
    switch (solver) {
        case ORTHO_QUATERNION:
            return 0;
        case ORTHO_EULER_HORIZONTAL:
            return 1;
        case ORTHO_EULER_HORIZONTAL_VERTICAL:
            return 2;
        case ORTHO_EULER_ALL_DOF:
            return 3;
        default:
            return 99;
    }
}

orthosfm::SFM_SOLVER_TYPE orthosfm::solverTypeFromIndex(const int &index) {
    switch (index) {
        case 0:
            return SFM_SOLVER_TYPE::ORTHO_QUATERNION;
        case 1:
            return SFM_SOLVER_TYPE::ORTHO_EULER_HORIZONTAL;
            break;
        case 2:
            return SFM_SOLVER_TYPE::ORTHO_EULER_HORIZONTAL_VERTICAL;
            break;
        case 3:
            return SFM_SOLVER_TYPE::ORTHO_EULER_ALL_DOF;
        default:
            std::cerr << "Error: Invalid solver type!\n" << std::endl;
            return SFM_SOLVER_TYPE::NONE;
    }
}

std::string orthosfm::solverTypeToString(const SFM_SOLVER_TYPE& solver) {
    switch (solver) {
        case ORTHO_QUATERNION:
            return "Quaternion based orthographic sfm solver";
        case ORTHO_EULER_HORIZONTAL:
            return "Euler angle based orthographic sfm solver restricted to horizontal rotation";
        case ORTHO_EULER_HORIZONTAL_VERTICAL:
            return "Euler angle based orthographic sfm solver restricted to horizontal and vertical rotation";
        case ORTHO_EULER_ALL_DOF:
            return "Euler angle based orthographic sfm solver";
        default:
            return "INVALID SOLVER";
    }
}

void orthosfm::propagateColorsToTracks(std::vector<Track> &tracks, const std::vector<View> &views) {
    // Create a map from view ID to the index in the list
    std::unordered_map<unsigned int, int> viewIDtoListID;
    for(int i=0; i<views.size(); i++) {
        viewIDtoListID.insert(std::make_pair(views[i].getID(), i));
    }

    // Loop over all tracks
    #pragma omp parallel for
    for(int i=0; i<tracks.size(); i++) {
        for(int featureID=0; featureID<tracks[i].size(); featureID++) {
            // Create point for lookup
            cv::Point point(
                    (int)fmax(fmin(tracks[i].get(featureID).x, views[viewIDtoListID.at(tracks[i].get(featureID).viewID)].getPixels().cols-1), 0),
                    (int)fmax(fmin(tracks[i].get(featureID).y, views[viewIDtoListID.at(tracks[i].get(featureID).viewID)].getPixels().rows-1), 0)
            );

            // Get the color at the pixel position
            cv::Vec3b color = views[viewIDtoListID.at(tracks[i].get(featureID).viewID)].getPixels().at<cv::Vec3b>(point);

            // BGR
            tracks[i].get(featureID).b = color[0];
            tracks[i].get(featureID).g = color[1];
            tracks[i].get(featureID).r = color[2];
        }
    }
}

std::string orthosfm::vec2str(const Eigen::Vector3d &vec) {
    std::stringstream str;
    str << "[" << vec.x() << ", " << vec.y() << ", " << vec.z() << "]";
    return str.str();
}

