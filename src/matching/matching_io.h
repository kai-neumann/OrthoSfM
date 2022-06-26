/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#ifndef ORTHO_SFM_MATCHINGIO_H
#define ORTHO_SFM_MATCHINGIO_H

#include <vector>
#include "iostream"

#include <data_structures/view.h>
#include <data_structures/track.h>

namespace orthosfm
{
	void saveTracksToFile(const std::vector<Track>& tracks, const std::string& outPath);
	void loadTracksFromFile(std::vector<Track>& tracks, const std::string& inPath);

	void
	saveTracksToPairwiseFiles(const std::vector<Track>& tracks, const std::vector<View>& views, const std::string& folder);
}

#endif // ORTHO_SFM_MATCHINGIO_H