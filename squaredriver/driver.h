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

#include "linalg.h"
#include "opponent.h"

class Opponents;
class Opponent;

class Driver {
    public:
        Driver(int index);
        //~Driver();

        /* callback functions called from TORCS */
        void initTrack(tTrack* t, void *carHandle, void **carParmHandle, tSituation *s);
        void newRace(tCarElt* car, tSituation *s);
        void drive(tSituation *s);
        int pitCommand(tSituation *s);
        void endRace(tSituation *s);
        tCarElt *getCarPtr() { return car; }
        tTrack *getTrackPtr() { return track; }
        float getSpeed() { return speed; }

    private:

        bool isStuck();
        void update(tSituation *s);
        float getAllowedSpeed(tTrackSeg *segment);
        float getAccel();
        float getDistToSegEnd();
        float getBrake();
        int getGear();
        float getSteer();
        v2d getTargetPoint();
        float getOvertakeOffset();

        float filterABS(float brake);
        float filterTCL(float accel);
        float filterTCL_RWD();
        float filterTCL_FWD();
        float filterTCL_4WD();
        void initTCLfilter();
        float filterTrk(float accel);
        float filterBColl(float brake);
        float filterSColl(float steer);

        void initCa();
        void initCw();

        /* per robot global data */
        int stuck;
        float trackangle;
        float angle;
        float mass;         /* mass of car + fuel */
        float speed;        /* speed in track direction */
        tCarElt *car;       /* pointer to tCarElt struct */
        float myoffset;     /* overtake offset sideways */
        Opponents *opponents;
        Opponent *opponent;

        /* data that should stay constant after first initialization */
        int MAX_UNSTUCK_COUNT;
        int INDEX;
        float CARMASS;      /* mass of the car only */
        float CA;           /* aerodynamic downforce coefficient */
        float CW;           /* aerodynamic drag coefficient */
        float (Driver::*GET_DRIVEN_WHEEL_SPEED)();


        /* class constants */
        static const float MAX_UNSTUCK_ANGLE;
        static const float UNSTUCK_TIME_LIMIT;
        static const float MAX_UNSTUCK_SPEED;
        static const float MIN_UNSTUCK_DIST;
        static const float G;
        static const float FULL_ACCEL_MARGIN;
        static const float SHIFT;
        static const float SHIFT_MARGIN;
        static const float ABS_SLIP;
        static const float ABS_MINSPEED;
        static const float TCL_SLIP;
        static const float TCL_MINSPEED;
        static const float LOOKAHEAD_CONST;
        static const float LOOKAHEAD_FACTOR;
        static const float WIDTHDIV;
        static const float SIDECOLL_MARGIN;
        static const float BORDER_OVERTAKE_MARGIN;
        static const float OVERTAKE_OFFSET_INC;

        /* class variables */
        tTrack* track;
};

#endif // _DRIVER_H_
