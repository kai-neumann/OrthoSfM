/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#include "group.h"

#include <util/common.h>

void orthosfm::buildGroups(std::deque<ViewGroup> &groups, const std::vector<View> &views, const std::vector<Track>& tracks, int groupSize) {
    std::cout << "Building groups of size " << groupSize << " ";

    // Clear current queue
    groups.clear();

    // Create a list of all ids that still need to be assigned (excluding the first group)
    std::set<unsigned int> toAssign;
    for(int i=2; i<views.size(); i++) {
        toAssign.insert(views[i].getID());
    }

    // Also keep track which cameras are used already
    std::set<unsigned int> used;

    // Build the first group (containing view 0 and 1)
    ViewGroup initialGroup;
    initialGroup.ids.push_back(views[0].getID());
    initialGroup.ids.push_back(views[1].getID());
    std::pair<ViewGroup, int> suggestion = completeGroup(initialGroup, tracks, toAssign, groupSize);
    groups.push_back(suggestion.first);

    // Remove the indices of the first group from the list of remaining ids
    for(auto id : suggestion.first.ids) {
        toAssign.erase(id);
        used.insert(id);
    }

    // While there are still views that need to be added
    while (!toAssign.empty()) {
        std::cout << toAssign.size() << " remaining cameras to assign to groups" << std::endl;
        // Build all possible combinations of groupSize-1 cameras from the list of already added cameras
        std::vector<ViewGroup> seedGroups = getAllPossibleCombinations(used, groupSize - 1);

        // Loop over all groups and try to complete them
        std::vector<std::pair<ViewGroup, int>> completed;
        completed.reserve(seedGroups.size());
        for(const auto& group : seedGroups) {
            completed.push_back(completeGroup(group, tracks, toAssign, groupSize));
        }

        // Loop over all completed groups to find the best
        int mostSharedTracks = -1;
        ViewGroup* bestGroup = nullptr;
        for(int i=0; i<completed.size(); i++) {
            if(completed[i].second > mostSharedTracks) {
                bestGroup = &completed[i].first;
                mostSharedTracks = completed[i].second;
            }
        }

        //std::cout << "The best group has " << mostSharedTracks << " shared tracks!" << std::endl;

        if(mostSharedTracks == 0) {
            std::cerr << std::endl << "Error while building groups. A view did not contain any matches to any other views. The reconstruction may not succeed." << std::endl;
        }

        // Add the group with the most shared tracks to the list
        groups.push_back(*bestGroup);

        // Add the newly assigned id to the list
        for(auto id : bestGroup->ids) {
            // Check if it has already been added to the list
            if(used.find(id) != used.end()) {
                continue;
            }

            toAssign.erase(id);
            used.insert(id);
        }
    }

    printGroups(groups);
    std::cout << "--> built " << groups.size() << " groups." << std::endl;

}

std::pair<orthosfm::ViewGroup, int> orthosfm::completeGroup(const ViewGroup &seedGroup, const std::vector<Track>& tracks,
                                        const std::set<unsigned int> &remainingIds, int groupSize) {
    // Create a working copy of the group
    ViewGroup group = seedGroup;

    // Store how many shared tracks are added
    int addedTracks = 0;

    // Convert the remaining id list to a vector
    std::vector<unsigned int> remainingIDVec;
    for(const auto& id : remainingIds) {
        remainingIDVec.push_back(id);
    }

    // Pre-filter the tracks based on the seed group
    std::vector<Track> seedPreFilteredTracks = filterTracksToAvailableCameras(group.ids, tracks, false, true);

    // As long as the group is not full
    while (group.ids.size() < groupSize) {
        std::vector<unsigned int> groupIDs = group.ids; // For debugging

        // Pre filter the tracks on the current group
        std::vector<Track> preFilteredTracks = filterTracksToAvailableCameras(group.ids, seedPreFilteredTracks, false, true);

        // Get the view that has the most shared tracks with all previously added views
        unsigned int mostSharedTrack = 0;
        unsigned int bestViewID = 0;

        #pragma omp parallel for
        for(int i=0; i<remainingIDVec.size(); i++) {
            unsigned int id = remainingIDVec[i];

            // Skip, if the view is already part of the group
            if(std::find(group.ids.begin(), group.ids.end(), id) != group.ids.end()) {
                continue;
            }

            // Create a list of ids for filtering
            std::vector<unsigned int> completedIds = group.ids;
            completedIds.push_back(id);

            // Filter the tracks to the number of ids
            std::vector<Track> filteredTracks = filterTracksToAvailableCameras(completedIds, preFilteredTracks, true, false);

            // The score of this combination is defined as the number of shared (full size) tracks
            unsigned int score = filteredTracks.size();

            // If the sum is higher than the currently highest element
            #pragma omp critical
            {
                if(score > mostSharedTrack) {
                    bestViewID = id;
                    mostSharedTrack = score;
                }
            };

        }

        // Add the best view to the group
        group.ids.push_back(bestViewID);
        addedTracks = mostSharedTrack;
    }

    group.tracks = addedTracks;
    return std::make_pair(group, addedTracks);
}

std::vector<orthosfm::ViewGroup> orthosfm::getAllPossibleCombinations(const std::set<unsigned int> &ids, int combinationSize) {
    // Convert into a vector for sorting
    std::vector<unsigned int> sortedIDs;
    sortedIDs.reserve(ids.size());
    for(const unsigned int& id : ids) {
        sortedIDs.push_back(id);
    }

    // Sort
    std::sort(sortedIDs.begin(), sortedIDs.end());

    // Initialize the groups with one element each
    std::vector<ViewGroup> currentGroups;
    for(int i=0; i<sortedIDs.size(); i++) {
        ViewGroup g;
        g.ids.push_back(sortedIDs[i]);
        currentGroups.push_back(g);
    }

    // If the combinations should only contain one element, we can stop here
    if(combinationSize <= 1) {
        return currentGroups;
    }

    // Else: Add elements for n iterations
    for(int i=0; i<combinationSize-1; i++) {
        // Build a new list with longer combinations
        std::vector<ViewGroup> extendedGroups;

        // Loop over all current groups
        for(const auto& g : currentGroups) {
            // Loop over all possible ids
            for(int i=0; i<sortedIDs.size(); i++) {
                // Check if the last element of the group is bigger than (or equal to) the current id
                if(g.ids.back() >= sortedIDs[i]) {
                    continue;
                }

                // Create a copy of the current group
                ViewGroup copy = g;

                // Add the id to the group
                copy.ids.push_back(sortedIDs[i]);

                // Add to the extended groups
                extendedGroups.push_back(copy);
            }
        }

        // Apply the extended groups to the current groups for the next iteration
        currentGroups = extendedGroups;
    }


    return currentGroups;
}

void orthosfm::printGroups(const std::deque<ViewGroup> &groups) {
    // Print the groups
    for(int i=0; i<groups.size(); i++) {
        std::cout << "Group " << i << ": [";
        for(int j=0; j<groups[i].ids.size(); j++) {
            std::cout << groups[i].ids[j];

            if(j < groups[i].ids.size() - 1) {
                std::cout << ", ";
            }
        }
        std::cout << "] --> " << groups[i].tracks << " tracks" << std::endl;
    }
}

void orthosfm::printGroups(const std::vector<ViewGroup> &groups) {
    // Print the groups
    for(int i=0; i<groups.size(); i++) {
        std::cout << "Group " << i << ": [";
        for(int j=0; j<groups[i].ids.size(); j++) {
            std::cout << groups[i].ids[j];

            if(j < groups[i].ids.size() - 1) {
                std::cout << ", ";
            }
        }
        std::cout << "] --> " << groups[i].tracks << " tracks" << std::endl;
    }
}

std::string orthosfm::ViewGroup::toString() {
    std::string str = "[";
    for(int i=0; i<ids.size(); i++) {
        str += std::to_string(ids[i]);
        if(i < ids.size()-1) {
            str += ", ";
        }
    }
    str += "]";
    return str;
}
