/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#ifndef ORTHO_SFM_MATCHING_H
#define ORTHO_SFM_MATCHING_H

#include <vector>
#include "iostream"

#include <data_structures/track.h>
#include <data_structures/view.h>

//#ifdef ENABLE_SIFT

// Define the maximum allowed number of features
namespace orthosfm
{

	const int MAX_FEATURES = 32768;

	void extractFeaturesForAllImages(std::vector<View>& views);

	std::vector<Track> runExhaustivePairwiseMatching(std::vector<View>& views, bool refineWithHomography);

	std::vector<Track>
	calculatePairwiseMatchesThreadSafe(const View& view1, const View& view2, bool refineWithHomography);

	std::vector<Track> filterTracksWithMasks(const std::vector<Track>& pairwiseTracks, const std::vector<View>& views);

	std::vector<Track> filterDuplicateTracks(const std::vector<Track>& pairwiseTracks);

	std::vector<Track> mergePairwiseTracks(const std::vector<Track>& pairwiseTracks);

	void visualizeMatches(const std::vector<Track>& tracks, const View& view1, const View& view2);

}

#endif //ORTHO_SFM_MATCHING_H