#ifndef _DRIVER_H_
#define _DRIVER_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <tgf.h>
#include <track.h>
#include <car.h>
#include <raceman.h>
#include <robottools.h>
#include <robot.h>

class Driver {
    public:
        Driver(int index);
        void initTrack(tTrack* t, void *carHandle, void **carParmHandle, tSituation *s);
        void drive(tCarElt* car, tSituation *s);
        //~Driver();

    private:
        void update(tCarElt* car, tSituation *s);
        int INDEX;

        float trackangle;
        float angle;


        /* class variables */
        tTrack* track;
};

#endif // _DRIVER_H_
