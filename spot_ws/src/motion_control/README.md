# Motion Control

Currently we only support a triangular 8-phase gait. In this gait, we move a single leg at a time. In between moving legs, we shift the robot's center of gravity to prepare for the next leg to move. There are four leg motions and four shifts before you get back to the beginning of the cycle (hence 8-phase).

## TODO

Other gait options
* stance based instead of swing-based
* ML model