///////////////////////////////////
// Arif Pramudwiatmoko           //
// Konagaya Laboratory           //
// Tokyo Institute fo Technology //
///////////////////////////////////

/*This file is for providing atomic radii data.
Atomic Radii reference (Calculated atomic radii):

Clementi, E.; Raimond, D. L.; Reinhardt, W. P. (1967). "Atomic Screening Constants from SCF Functions. II. Atoms with 37 to 86 Electrons".
		Journal of Chemical Physics. 47 (4): 1300–1307. Bibcode:1967JChPh..47.1300C. doi:10.1063/1.1712084.
*/

#pragma once

#include <string>

namespace AtmRad
{
	class AtomRadii {
	private:
		int N = 15;
		std::string element[15] = {" H", "HE", "LI",  "BE",  " B",  " C",  " N",  " O",  " F",  "NE",  "NA",  "MG",  "AL",  "SI",  " P"};
		float radius[15] = {0.053f, 0.031f, 0.167f, 0.112f, 0.087f, 0.067f, 0.056f, 0.048f, 0.042f, 0.038f, 0.190f, 0.145f, 0.118f, 0.111f, 0.098f};

	public:
		float getRadius(std::string atmName);
		float getRadiusDNA(std::string atmName);
	};
}