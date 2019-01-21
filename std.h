#include <math.h>
#include <stdint.h>



// STUFF TO FIX COMPILING RANSAC IN WINDOWS: ALL ARRAYS ARE NOW FIXED SIZE:
#define MAX_ITERATIONS	500
#define MAX_SAMPLES		30
#define VECTOR_DIMENSION	1

// Missing defines windows does not know
#define true 1
#define false 0
#define bool int
#define inline

// Paparazzi stuff
#define RadOfDeg(X)  ((X)*3.1415926535897f/180.0f)
#define Bound(_x, _min, _max) { if (_x > _max) _x = _max; else if (_x < _min) _x = _min; }
