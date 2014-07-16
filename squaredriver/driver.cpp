#include "driver.h"

#define BT_SECT_PRIV "bt private"
#define BT_ATT_FUELPERLAP "fuelperlap"


const float Driver::MAX_UNSTUCK_ANGLE = 15.0/180.0*PI;      /* [radians] */
const float Driver::UNSTUCK_TIME_LIMIT = 2.0;               /* [s] */
const float Driver::MAX_UNSTUCK_SPEED = 5.0;                /* [m/s] */
const float Driver::MIN_UNSTUCK_DIST = 3.0;
const float Driver::G = 9.81;                               /* [m/(s*s)] */
const float Driver::FULL_ACCEL_MARGIN = 1.0;
const float Driver::SHIFT = 0.9;                            /* [-] (% of rpmredline) */
const float Driver::SHIFT_MARGIN = 4.0;     
const float Driver::ABS_SLIP = 0.9;                         /* [-] range [0.95..0.3] */
const float Driver::ABS_MINSPEED = 3.0;                     /* [m/s] */
const float Driver::TCL_SLIP = 0.9;                         /* [-] range [0.95..0.3] */
const float Driver::TCL_MINSPEED = 3.0;                     /* [m/s] */
const float Driver::LOOKAHEAD_CONST = 17.0;                 /* [m] */
const float Driver::LOOKAHEAD_FACTOR = 0.33;                /* [-] */
const float Driver::WIDTHDIV = 4.0; 
const float Driver::SIDECOLL_MARGIN = 2.0;                  /* [m] */
const float Driver::BORDER_OVERTAKE_MARGIN = 0.5;           /* [m] */
const float Driver::OVERTAKE_OFFSET_INC = 0.1;  

Driver::Driver(int index)
{
    INDEX = index;
}


/* Called for every track change or new race. */
void Driver::initTrack(tTrack* t, void *carHandle, void **carParmHandle, tSituation *s)
{
    track = t;

    char buffer[256];
    /* get a pointer to the first char of the track filename */
    char* trackname = strrchr(track->filename, '/') + 1;

    switch (s->_raceType) {
        case RM_TYPE_PRACTICE:
            sprintf(buffer, "drivers/bt/%d/practice/%s", INDEX, trackname);
            break;
        case RM_TYPE_QUALIF:
            sprintf(buffer, "drivers/bt/%d/qualifying/%s", INDEX, trackname);
            break;
        case RM_TYPE_RACE:
            sprintf(buffer, "drivers/bt/%d/race/%s", INDEX, trackname);
            break;
        default:
            break;
    }

    *carParmHandle = GfParmReadFile(buffer, GFPARM_RMODE_STD);
    if (*carParmHandle == NULL) {
        sprintf(buffer, "drivers/bt/%d/default.xml", INDEX);
        *carParmHandle = GfParmReadFile(buffer, GFPARM_RMODE_STD);
    }

    float fuel = GfParmGetNum(*carParmHandle, BT_SECT_PRIV, BT_ATT_FUELPERLAP, (char*)NULL, 5.0);
    fuel *= (s->_totLaps + 1.0);
    GfParmSetNum(*carParmHandle, SECT_CAR, PRM_FUEL, (char*)NULL, MIN(fuel, 100.0));

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
    initTCLfilter();

    /* initialize the list of opponents */
    opponents = new Opponents(s, this);
    opponent = opponents->getOpponentPtr();
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
        car->ctrl.steer = getSteer();
        car->ctrl.gear = getGear();
        car->ctrl.brakeCmd = filterABS(getBrake());
        if (car->ctrl.brakeCmd == 0.0) {
            car->ctrl.accelCmd = filterTCL(getAccel());
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


/***************************************************************************
 *
 * utility functions
 *
***************************************************************************/


/* Compute the allowed speed on a segment */
float Driver::getAllowedSpeed(tTrackSeg *segment)
{
    if (segment->type == TR_STR) {
        return FLT_MAX;
    } else {
        float arc = 0.0;
        tTrackSeg *s = segment;
        
        while (s->type == segment->type && arc < PI/2.0) {
            arc += s->arc;
            s = s->next;
        }

        arc /= PI/2.0;
        float mu = segment->surface->kFriction;
        float r = (segment->radius + segment->width/2.0)/sqrt(arc);
        return sqrt((mu*G*r)/(1.0 - MIN(1.0, r*CA*mu/mass)));
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


/* compute steer value */
float Driver::getSteer()
{
    float targetAngle;
    v2d target = getTargetPoint();

    targetAngle = atan2(target.y - car->_pos_Y, target.x - car->_pos_X);
    targetAngle -= car->_yaw;
    NORM_PI_PI(targetAngle);
    return targetAngle / car->_steerLock;
}


/* compute target point for steering */
/* compute target point for steering */
v2d Driver::getTargetPoint()
{
    tTrackSeg *seg = car->_trkPos.seg;
    float lookahead = LOOKAHEAD_CONST + car->_speed_x*LOOKAHEAD_FACTOR;
    float length = getDistToSegEnd();
    float offset = getOvertakeOffset();

    while (length < lookahead) {
        seg = seg->next;
        length += seg->length;
    }

    length = lookahead - length + seg->length;
    v2d s;
    s.x = (seg->vertex[TR_SL].x + seg->vertex[TR_SR].x)/2.0;
    s.y = (seg->vertex[TR_SL].y + seg->vertex[TR_SR].y)/2.0;

    if ( seg->type == TR_STR) {
        v2d d, n;
        n.x = (seg->vertex[TR_EL].x - seg->vertex[TR_ER].x)/seg->length;
        n.y = (seg->vertex[TR_EL].y - seg->vertex[TR_ER].y)/seg->length;
        n.normalize();
        d.x = (seg->vertex[TR_EL].x - seg->vertex[TR_SL].x)/seg->length;
        d.y = (seg->vertex[TR_EL].y - seg->vertex[TR_SL].y)/seg->length;
        return s + d*length + offset*n;
    } else {
        v2d c, n;
        c.x = seg->center.x;
        c.y = seg->center.y;
        float arc = length/seg->radius;
        float arcsign = (seg->type == TR_RGT) ? -1.0 : 1.0;
        arc = arc*arcsign;
        s = s.rotate(c, arc);
        n = c - s;
        n.normalize();
        return s + arcsign*offset*n;
    }
}

/* Compute an offset to the target point */
float Driver::getOvertakeOffset()
{
    int i;
    float catchdist, mincatchdist = FLT_MAX;
    Opponent *o = NULL;

    for (i = 0; i < opponents->getNOpponents(); i++) {
        if (opponent[i].getState() & OPP_FRONT) {
            catchdist = opponent[i].getCatchDist();
            if (catchdist < mincatchdist) {
                mincatchdist = catchdist;
                o = &opponent[i];
            }
        }
    }

    if (o != NULL) {
        float w = o->getCarPtr()->_trkPos.seg->width/WIDTHDIV-BORDER_OVERTAKE_MARGIN;
        float otm = o->getCarPtr()->_trkPos.toMiddle;
        if (otm > 0.0 && myoffset > -w) myoffset -= OVERTAKE_OFFSET_INC;
        else if (otm < 0.0 && myoffset < w) myoffset += OVERTAKE_OFFSET_INC;
    } else {
        if (myoffset > OVERTAKE_OFFSET_INC) myoffset -= OVERTAKE_OFFSET_INC;
        else if (myoffset < -OVERTAKE_OFFSET_INC) myoffset += OVERTAKE_OFFSET_INC;
        else myoffset = 0.0;
    }
    return myoffset;
}


/* Update my private data every timestep */
void Driver::update(tSituation *s)
{
    trackangle = RtTrackSideTgAngleL(&(car->_trkPos));
    angle = trackangle - car->_yaw;
    NORM_PI_PI(angle);
    mass = CARMASS + car->_fuel;
    speed = Opponent::getSpeed(car);
    opponents->update(s, this);
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

/* Brake filter for collision avoidance */
float Driver::filterBColl(float brake)
{
    float currentspeedsqr = car->_speed_x*car->_speed_x;
    float mu = car->_trkPos.seg->surface->kFriction;
    float cm = mu*G*mass;
    float ca = CA*mu + CW;
    int i;

    for (i = 0; i < opponents->getNOpponents(); i++) {
        if (opponent[i].getState() & OPP_COLL) {
            float allowedspeedsqr = opponent[i].getSpeed();
            allowedspeedsqr *= allowedspeedsqr;
            float brakedist = mass*(currentspeedsqr - allowedspeedsqr) / (2.0*(cm + allowedspeedsqr*ca));
            if (brakedist > opponent[i].getDistance()) {
                return 1.0;
            }
        }
    }
    return brake;
}

/* Steer filter for collision avoidance */
float Driver::filterSColl(float steer)
{
    int i;
    float sidedist = 0.0, fsidedist = 0.0, minsidedist = FLT_MAX;
    Opponent *o = NULL;

    /* get the index of the nearest car (o) */
    for (i = 0; i < opponents->getNOpponents(); i++) {
        if (opponent[i].getState() & OPP_SIDE) {
            sidedist = opponent[i].getSideDist();
            fsidedist = fabs(sidedist);
            if (fsidedist < minsidedist) {
                minsidedist = fsidedist;
                o = &opponent[i];
            }
        }
    }

    /* if there is another car handle the situation */
    if (o != NULL) {
        float d = fsidedist - o->getWidth();
        /* near enough */
        if (d < SIDECOLL_MARGIN) {
            /* compute angle between cars */
            tCarElt *ocar = o->getCarPtr();
            float diffangle = ocar->_yaw - car->_yaw;
            NORM_PI_PI(diffangle);
            const float c = SIDECOLL_MARGIN/2.0;
            d = d - c;
            if (d < 0.0) d = 0.0;
            float psteer = diffangle/car->_steerLock;
            return steer*(d/c) + 2.0*psteer*(1.0-d/c);
        }
    }
    return steer;
}

/* Antilocking filter for brakes */
float Driver::filterABS(float brake)
{
    if (car->_speed_x < ABS_MINSPEED) return brake;
    int i;
    float slip = 0.0;
    for (i = 0; i < 4; i++) {
        slip += car->_wheelSpinVel(i) * car->_wheelRadius(i) / car->_speed_x;
    }
    slip = slip/4.0;
    if (slip < ABS_SLIP) brake = brake*slip;
    return brake;
}


/* TCL filter for accelerator pedal */
float Driver::filterTCL(float accel)
{
    if (car->_speed_x < TCL_MINSPEED) return accel;
    float slip = car->_speed_x/(this->*GET_DRIVEN_WHEEL_SPEED)();
    if (slip < TCL_SLIP) {
        accel = 0.0;
    }
    return accel;
}


/* Traction Control (TCL) setup */
void Driver::initTCLfilter()
{
    const char *traintype = GfParmGetStr(car->_carHandle, SECT_DRIVETRAIN, PRM_TYPE, VAL_TRANS_RWD);
    if (strcmp(traintype, VAL_TRANS_RWD) == 0) {
        GET_DRIVEN_WHEEL_SPEED = &Driver::filterTCL_RWD;
    } else if (strcmp(traintype, VAL_TRANS_FWD) == 0) {
        GET_DRIVEN_WHEEL_SPEED = &Driver::filterTCL_FWD;
    } else if (strcmp(traintype, VAL_TRANS_4WD) == 0) {
        GET_DRIVEN_WHEEL_SPEED = &Driver::filterTCL_4WD;
    }
}


/* TCL filter plugin for rear wheel driven cars */
float Driver::filterTCL_RWD()
{
    return (car->_wheelSpinVel(REAR_RGT) + car->_wheelSpinVel(REAR_LFT)) *
            car->_wheelRadius(REAR_LFT) / 2.0;
}


/* TCL filter plugin for front wheel driven cars */
float Driver::filterTCL_FWD()
{
    return (car->_wheelSpinVel(FRNT_RGT) + car->_wheelSpinVel(FRNT_LFT)) *
            car->_wheelRadius(FRNT_LFT) / 2.0;
}


/* TCL filter plugin for all wheel driven cars */
float Driver::filterTCL_4WD()
{
    return (car->_wheelSpinVel(FRNT_RGT) + car->_wheelSpinVel(FRNT_LFT)) *
            car->_wheelRadius(FRNT_LFT) / 4.0 +
           (car->_wheelSpinVel(REAR_RGT) + car->_wheelSpinVel(REAR_LFT)) *
            car->_wheelRadius(REAR_LFT) / 4.0;
}

/* Hold car on the track */
float Driver::filterTrk(float accel)
{
    tTrackSeg* seg = car->_trkPos.seg;

    if (car->_speed_x < MAX_UNSTUCK_SPEED) return accel;
    
    if (seg->type == TR_STR) {
        float tm = fabs(car->_trkPos.toMiddle);
        float w = seg->width/WIDTHDIV;
        if (tm > w) return 0.0; else return accel;
    } else {
        float sign = (seg->type == TR_RGT) ? -1 : 1;
        if (car->_trkPos.toMiddle*sign > 0.0) {
            return accel;
        } else {
            float tm = fabs(car->_trkPos.toMiddle);
            float w = seg->width/WIDTHDIV;
            if (tm > w) return 0.0; else return accel;
        }
    }
}

