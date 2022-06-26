/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#ifndef ORTHO_SFM_MATCHINGMVE_H
#define ORTHO_SFM_MATCHINGMVE_H

#include <data_structures/view.h>
#include <data_structures/track.h>

namespace orthosfm
{
	std::vector<Track> calculateTracksUsingMVE(const std::vector<View>& views, const std::string& projectFolder, int downscaleFactor);
}

#endif // ORTHO_SFM_MATCHINGMVE_H
