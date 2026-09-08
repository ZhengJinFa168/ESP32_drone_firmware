#pragma once

// Include the available filters
#include "modules/logic/attitude_estimator/ComplementaryFilter.h"
// #include "modules/logic/attitude_estimator/MadgwickFilter.h" // Future use

// Select the active filter type
using ActiveAttitudeFilter = ComplementaryFilter;

