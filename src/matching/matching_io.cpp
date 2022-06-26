/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#include "matching_io.h"

#include <fstream>
#include <boost/algorithm/string.hpp>

#include <util/common.h>

void orthosfm::saveTracksToFile(const std::vector<Track>& tracks, const std::string& outPath) {
    std::cout << "Saving " << tracks.size() << " tracks to " << outPath << std::endl;

    std::ofstream outfile;

    outfile.open(outPath);
    // Loop over all tracks
    for(int trackID=0; trackID<tracks.size(); trackID++) {
        // First write the number of features this track contains
        outfile << tracks[trackID].size() << ";";

        // Then loop over all features
        for(int featureID=0; featureID<tracks[trackID].size(); featureID++) {
            // Append all information of the feature
            outfile << tracks[trackID].get(featureID).viewID << ";";
            outfile << tracks[trackID].get(featureID).localFeatureID << ";";
            outfile << tracks[trackID].get(featureID).globalFeatureID << ";";
            outfile << tracks[trackID].get(featureID).x << ";";
            outfile << tracks[trackID].get(featureID).y << ";";
            outfile << tracks[trackID].get(featureID).r << ";";
            outfile << tracks[trackID].get(featureID).g << ";";
            outfile << tracks[trackID].get(featureID).b;

            // Only insert a ";" if this is not the last feature
            if(featureID < tracks[trackID].size() - 1) {
                outfile << ";";
            }
        }

        // Finally insert a line break
        outfile << std::endl;
    }

    outfile.close();
}

void orthosfm::loadTracksFromFile(std::vector<Track> &tracks, const std::string &inPath) {
    // Clear any tracks that might be currently contained inside the vector
    tracks.clear();

    // Read file line by line
    std::ifstream file(inPath);
    std::string line;
    while (std::getline(file, line))
    {
        // Tokenize the string
        std::vector<std::string> splitted;
        boost::split(splitted,line,boost::is_any_of(";"));

        // Create a track
        Track track;

        // Read in the number of features of this track
        int featuresCount = std::stoi(splitted[0]);

        // Read the features after each other
        int rollingIndex = 1;
        for(int featureID=0; featureID<featuresCount; featureID++) {
            // Create a new Feature
            Feature f(std::stoi(splitted[rollingIndex]), /*viewID*/
                      std::stoi(splitted[rollingIndex+1]), /*featureID*/
                      std::stoi(splitted[rollingIndex+2]), /*global eatureID*/
                      std::stof(splitted[rollingIndex+3]), /*x pos*/
                      std::stof(splitted[rollingIndex+4]) /*y pos*/);

            f.r = std::stoi(splitted[rollingIndex+5]);
            f.g = std::stoi(splitted[rollingIndex+6]);
            f.b = std::stoi(splitted[rollingIndex+7]);

            // Add the feature to the track
            track.add(f);

            // Increase the rolling index by the number of fields
            rollingIndex += 8;
        }

        // Add the track to the output
        tracks.push_back(track);
    }
}

void orthosfm::saveTracksToPairwiseFiles(const std::vector<Track> &tracks, const std::vector<View>& views, const std::string &folder) {
    // Loop over all (unique) combinations of two views
    for(int i=0; i<views.size(); i++) {
        for(int j=i+1; j<views.size(); j++) {
            // Put view ids into a vector
            std::vector<unsigned int> ids;
            ids.push_back(views[i].getID());
            ids.push_back(views[j].getID());

            // Filter the tracks to those two views
            std::vector<Track> filtered = filterTracksToAvailableCameras(ids, tracks, true, false);

            // If there are not tracks for the two cameras: We can stop here
            if(filtered.empty()) continue;

            // Create a file name from the IDs
            std::string filePath = folder + "/" + zfill(views[i].getID(), 3) + "_" + zfill(views[j].getID(), 3) + ".txt";

            // Open an output file
            std::ofstream outfile;
            outfile.open(filePath);

            // Loop over all tracks
            for(int trackID=0; trackID<filtered.size(); trackID++) {
                // Loop over all current ids
                for(int k=0; k<ids.size(); k++) {
                    // Loop over all features of the track
                    for(int featureID=0; featureID<filtered[trackID].size(); featureID++) {
                        // If the view id of the feature matches the current ID we can append it to the output
                        if(filtered[trackID].get(featureID).viewID == ids[k]) {
                            outfile << filtered[trackID].get(featureID).x << " " << filtered[trackID].get(featureID).y;

                            if(k == 0) {
                                outfile << " ";
                            }
                            else {
                                outfile << std::endl;
                            }
                        }
                    }
                }
            }
        }
    }
}