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
        //~Driver();

        /* callback functions called from TORCS */
        void initTrack(tTrack* t, void *carHandle, void **carParmHandle, tSituation *s);
        void newRace(tCarElt* car, tSituation *s);
        void drive(tCarElt* car, tSituation *s);
        void endRace(tCarElt *car, tSituation *s);

    private:

        bool isStuck(tCarElt* car);
        void update(tCarElt* car, tSituation *s);

        /* per robot global data */
        int stuck;
        float trackangle;
        float angle;

        /* data that should stay constant after first initialization */
        int MAX_UNSTUCK_COUNT;
        int INDEX;

        /* class constants */
        static const float MAX_UNSTUCK_ANGLE;
        static const float UNSTUCK_TIME_LIMIT;



        /* class variables */
        tTrack* track;
};

#endif // _DRIVER_H_
