/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#ifndef ORTHO_SFM_GROUP_H
#define ORTHO_SFM_GROUP_H

#include <vector>
#include <deque>
#include <set>

#include <data_structures/view.h>
#include <data_structures/track.h>

namespace orthosfm
{
// Define the ViewGroup data structure
	struct ViewGroup
	{
		std::vector<unsigned int> ids;
		double tracks;
		std::string toString();
	};

	void
	buildGroups(std::deque<ViewGroup>& groups, const std::vector<View>& views, const std::vector<Track>& tracks, int groupSize);

// Completes a group as best as it can and returns the score
	std::pair<ViewGroup, int>
	completeGroup(const ViewGroup& seedGroup, const std::vector<Track>& tracks, const std::set<unsigned int>& remainingIds, int groupSize);

	std::vector<ViewGroup> getAllPossibleCombinations(const std::set<unsigned int>& ids, int combinationSize);

	void printGroups(const std::deque<ViewGroup>& groups);
	void printGroups(const std::vector<ViewGroup>& groups);
}


#endif //ORTHO_SFM_GROUP_H
