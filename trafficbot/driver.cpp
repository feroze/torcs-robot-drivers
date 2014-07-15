#include "driver.h"
#include <stdio.h>

Driver::Driver(int index)
{
    INDEX = index;
}

/* Called for every track change or new race. */
void Driver::initTrack(tTrack* t, void *carHandle, void **carParmHandle, tSituation *s)
{
    track = t;
    *carParmHandle = NULL;
}

/* Update my private data every timestep */
void Driver::update(tCarElt* car, tSituation *s)
{
    trackangle = RtTrackSideTgAngleL(&(car->_trkPos));
    angle = trackangle - car->_yaw;
    NORM_PI_PI(angle);
}


/* Drive during race. */
void Driver::drive(tCarElt* car, tSituation *s)
{

    update(car, s);
    memset(&car->ctrl, 0, sizeof(tCarCtrl));

    float angle;
    const float SC = 1.0;

    angle = RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
    NORM_PI_PI(angle); // put the angle back in the range from -PI to PI
    angle -= SC*car->_trkPos.toMiddle/car->_trkPos.seg->width;

    // set up the values to return
    if (INDEX == 0) {
        car->ctrl.steer = angle / car->_steerLock +0.3 ;
        car->ctrl.gear = 1; // first gear
        car->ctrl.accelCmd = 0.5; // 30% accelerator pedal
        car->ctrl.brakeCmd = 0.0; // no brakes
    }

    if (INDEX == 1) {
        car->ctrl.steer = angle / car->_steerLock - 0.2;
        car->ctrl.gear = 1; // first gear
        car->ctrl.accelCmd = 0.3; // 30% accelerator pedal
        car->ctrl.brakeCmd = 0.0; // no brakes
    }
}
