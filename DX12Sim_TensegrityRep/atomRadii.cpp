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

#include "atomRadii.h"

float AtmRad::AtomRadii::getRadius(std::string atmName) {
	for (int i = 0; i < N; i++) {
		if (!atmName.compare(element[i]))
			return radius[i];
	}
	return 0.0f;
}

float AtmRad::AtomRadii::getRadiusDNA(std::string atmName) {
	if (!atmName.compare(" H"))
		return radius[0];
	else if (!atmName.compare(" C"))
		return radius[5];
	else if (!atmName.compare(" N"))
		return radius[6];
	else if (!atmName.compare(" O"))
		return radius[7];
	else if (!atmName.compare(" P"))
		return radius[14];
	return 0.0f;
}

/*if (!atmName.compare(element[i])) {
	return radius[i];
}*/