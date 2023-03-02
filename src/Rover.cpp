
#include <L298N.h>

#include "Rover.h"

    const int TRACK_L = 0; 
    const char TRACK_R = 1; 


    Rover::Rover(int in1L, int in2L, int enL,int in1R, int in2R, int enR)
{     
    _tracks[0]= L298N(in1L, in2L, enL);
    _tracks[1]= L298N(in1R, in2R, enR);
}

void Rover::updateSpeed(signed short speedL, signed short speedR){
    _updateTrackSpeed(TRACK_L, speedL);
    _updateTrackSpeed(TRACK_R, speedR);
}

void Rover::_updateTrackSpeed(int track_i, signed short speed){

    if (speed == 0)
    {
        _tracks[track_i].stop();  
    } 
    else
    {
        _tracks[track_i].setSpeed(speed);
        if (signal > 0){
            _tracks[track_i].forward();
        } else {
            _tracks[track_i].backward();
        }
    }
}
