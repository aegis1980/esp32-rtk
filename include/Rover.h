#ifndef ROVER_H
#define ROVER_H

#include <L298N.h>

class Rover
{
private:
    L298N _tracks[2];
    void _updateTrackSpeed(int track_i, signed short speed);

public:
    Rover(int in1L, int in2L, int enL,int in1R, int in2R, int enR);


    void updateSpeed(signed short speedL,signed short speedR);
};

#endif