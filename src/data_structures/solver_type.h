/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#ifndef ORTHO_SFM_SOLVERTYPE_H
#define ORTHO_SFM_SOLVERTYPE_H

namespace orthosfm
{
	enum SFM_SOLVER_TYPE
	{
		ORTHO_QUATERNION,
		ORTHO_EULER_HORIZONTAL,
		ORTHO_EULER_HORIZONTAL_VERTICAL,
		ORTHO_EULER_ALL_DOF,
		NONE
	};
}

#endif //ORTHO_SFM_SOLVERTYPE_H
