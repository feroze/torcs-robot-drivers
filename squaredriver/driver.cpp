#include "driver.h"

const float Driver::MAX_UNSTUCK_ANGLE = 15.0/180.0*PI;      /* [radians] */
const float Driver::UNSTUCK_TIME_LIMIT = 2.0;               /* [s] */
const float Driver::MAX_UNSTUCK_SPEED = 5.0;                /* [m/s] */
const float Driver::MIN_UNSTUCK_DIST = 3.0;
const float Driver::G = 9.81;                               /* [m/(s*s)] */
const float Driver::FULL_ACCEL_MARGIN = 1.0;
const float Driver::SHIFT = 0.9;                            /* [-] (% of rpmredline) */
const float Driver::SHIFT_MARGIN = 4.0;     

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

/* Start a new race. */
void Driver::newRace(tCarElt* car, tSituation *s)
{
    MAX_UNSTUCK_COUNT = int(UNSTUCK_TIME_LIMIT/RCM_MAX_DT_ROBOTS);
    stuck = 0;
    this->car = car;
    CARMASS = GfParmGetNum(car->_carHandle, SECT_CAR, PRM_MASS, NULL, 1000.0);
    initCa();
    initCw();
}

/* Drive during race. */
void Driver::drive(tSituation *s)
{
    update(s);

    memset(&car->ctrl, 0, sizeof(tCarCtrl));

    if (isStuck()) {
        car->ctrl.steer = -angle / car->_steerLock;
        car->ctrl.gear = -1; // reverse gear
        car->ctrl.accelCmd = 0.5; // 30% accelerator pedal
        car->ctrl.brakeCmd = 0.0; // no brakes
    } else {
        float steerangle = angle - car->_trkPos.toMiddle/car->_trkPos.seg->width;
        car->ctrl.steer = steerangle / car->_steerLock;
        car->ctrl.gear = 1; // first gear
        car->ctrl.accelCmd = 0.3; // 30% accelerator pedal
        if (car->ctrl.brakeCmd == 0.0) {
            car->ctrl.accelCmd = getAccel();
        } else {
            car->ctrl.accelCmd = 0.0;
        }
    }
}

/* Set pitstop commands. */
int Driver::pitCommand(tSituation *s)
{
    return ROB_PIT_IM; /* return immediately */
}


/* End of the current race */
void Driver::endRace(tSituation *s)
{
}


/* Compute the allowed speed on a segment */
float Driver::getAllowedSpeed(tTrackSeg *segment)
{
    if (segment->type == TR_STR) {
        return FLT_MAX;
    } else {
        float mu = segment->surface->kFriction;
        return sqrt(mu*G*segment->radius);
    }
}


/* Compute the length to the end of the segment */
float Driver::getDistToSegEnd()
{
    if (car->_trkPos.seg->type == TR_STR) {
        return car->_trkPos.seg->length - car->_trkPos.toStart;
    } else {
        return (car->_trkPos.seg->arc - car->_trkPos.toStart)*car->_trkPos.seg->radius;
    }
}


/* Compute fitting acceleration */
float Driver::getAccel()
{
    float allowedspeed = getAllowedSpeed(car->_trkPos.seg);
    float gr = car->_gearRatio[car->_gear + car->_gearOffset];
    float rm = car->_enginerpmRedLine;
    if (allowedspeed > car->_speed_x + FULL_ACCEL_MARGIN) {
        return 1.0;
    } else {
        return allowedspeed/car->_wheelRadius(REAR_RGT)*gr /rm;
    }
}

/***************************************************************************
 *
 * utility functions
 *
***************************************************************************/


float Driver::getBrake()
{
    tTrackSeg *segptr = car->_trkPos.seg;
    float currentspeedsqr = car->_speed_x*car->_speed_x;
    float mu = segptr->surface->kFriction;
    float maxlookaheaddist = currentspeedsqr/(2.0*mu*G);
    float lookaheaddist = getDistToSegEnd();

    float allowedspeed = getAllowedSpeed(segptr);
    if (allowedspeed < car->_speed_x) return 1.0;

    segptr = segptr->next;
    while (lookaheaddist < maxlookaheaddist) {
        allowedspeed = getAllowedSpeed(segptr);
        if (allowedspeed < car->_speed_x) {
            float allowedspeedsqr = allowedspeed*allowedspeed;
            float brakedist = (currentspeedsqr - allowedspeedsqr) / (2.0*mu*G);
            if (brakedist > lookaheaddist) {
                return 1.0;
            }
        }
        lookaheaddist += segptr->length;
        segptr = segptr->next;
    }
    return 0.0;
}

/* Compute gear */
int Driver::getGear()
{
    if (car->_gear <= 0) return 1;
    float gr_up = car->_gearRatio[car->_gear + car->_gearOffset];
    float omega = car->_enginerpmRedLine/gr_up;
    float wr = car->_wheelRadius(2);

    if (omega*wr*SHIFT < car->_speed_x) {
        return car->_gear + 1;
    } else {
        float gr_down = car->_gearRatio[car->_gear + car->_gearOffset - 1];
        omega = car->_enginerpmRedLine/gr_down;
        if (car->_gear > 1 && omega*wr*SHIFT > car->_speed_x + SHIFT_MARGIN) {
            return car->_gear - 1;
        }
    }
    return car->_gear;
}

/* Update my private data every timestep */
void Driver::update(tSituation *s)
{
    trackangle = RtTrackSideTgAngleL(&(car->_trkPos));
    angle = trackangle - car->_yaw;
    NORM_PI_PI(angle);
    mass = CARMASS + car->_fuel;
}


/* Check if I'm stuck */
bool Driver::isStuck()
{
    if (fabs(angle) > MAX_UNSTUCK_ANGLE &&
        car->_speed_x < MAX_UNSTUCK_SPEED &&
        fabs(car->_trkPos.toMiddle) > MIN_UNSTUCK_DIST) {
        if (stuck > MAX_UNSTUCK_COUNT && car->_trkPos.toMiddle*angle < 0.0) {
            return true;
        } else {
            stuck++;
            return false;
        }
    } else {
        stuck = 0;
        return false;
    }
}


/* Compute aerodynamic downforce coefficient CA */
void Driver::initCa()
{
    char *WheelSect[4] = {SECT_FRNTRGTWHEEL, SECT_FRNTLFTWHEEL, SECT_REARRGTWHEEL, SECT_REARLFTWHEEL};
    float rearwingarea = GfParmGetNum(car->_carHandle, SECT_REARWING, PRM_WINGAREA, (char*) NULL, 0.0);
    float rearwingangle = GfParmGetNum(car->_carHandle, SECT_REARWING, PRM_WINGANGLE, (char*) NULL, 0.0);
    float wingca = 1.23*rearwingarea*sin(rearwingangle);

    float cl = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_FCL, (char*) NULL, 0.0) +
               GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_RCL, (char*) NULL, 0.0);
    float h = 0.0;
    int i;
    for (i = 0; i < 4; i++)
        h += GfParmGetNum(car->_carHandle, WheelSect[i], PRM_RIDEHEIGHT, (char*) NULL, 0.20);
    h*= 1.5; h = h*h; h = h*h; h = 2.0 * exp(-3.0*h);
    CA = h*cl + 4.0*wingca;
}


/* Compute aerodynamic drag coefficient CW */
void Driver::initCw()
{
    float cx = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_CX, (char*) NULL, 0.0);
    float frontarea = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_FRNTAREA, (char*) NULL, 0.0);
    CW = 0.645*cx*frontarea;
}
