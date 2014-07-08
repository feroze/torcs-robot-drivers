/***************************************************************************

    file                 : multibot.cpp
    created              : Tue Jul 8 11:04:52 IST 2014
    copyright            : (C) 2002 Feroze Naina

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifdef _WIN32
#include <windows.h>
#endif

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

static tTrack	*curTrack;

static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s); 
static void newrace(int index, tCarElt* car, tSituation *s); 
static void drive(int index, tCarElt* car, tSituation *s); 
static void endrace(int index, tCarElt *car, tSituation *s);
static void shutdown(int index);
static int  InitFuncPt(int index, void *pt); 


#define BUFSIZE 20
#define NBBOTS 2

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
    car->ctrl.steer = angle / car->_steerLock;
    car->ctrl.gear = 1; // first gear
    car->ctrl.accelCmd = 0.3; // 30% accelerator pedal
    car->ctrl.brakeCmd = 0.0; // no brakes
}

static Driver *driver[NBBOTS];
static const char* botname[NBBOTS] = {
    "multibot 1", "multibot 2"
};
static const char* botdesc[NBBOTS] = {
     "multibot 1", "multibot 2"
 };


/* 
 * Module entry point  
 */ 
extern "C" int 
multibot(tModInfo *modInfo) 
{
    int i;
    memset(modInfo, 0, 10*sizeof(tModInfo));

    for (i = 0; i < NBBOTS; i++) {
        modInfo[i].name    = strdup(botname[i]);    // name of the module (short).
        modInfo[i].desc    = strdup(botdesc[i]);    // Description of the module (can be long).
        modInfo[i].fctInit = InitFuncPt;            // Init function.
        modInfo[i].gfId    = ROB_IDENT;             // Supported framework version.
        modInfo[i].index   = i;                     // Indices from 0 to 9.
    }
    return 0;
} 

/* Module interface initialization. */
static int 
InitFuncPt(int index, void *pt) 
{ 
    tRobotItf *itf  = (tRobotItf *)pt; 

    // Create robot instance for index.
    driver[index] = new Driver(index);

    itf->rbNewTrack = initTrack; /* Give the robot the track view called */ 
				 /* for every track change or new race */ 
    itf->rbNewRace  = newrace; 	 /* Start a new race */
    itf->rbDrive    = drive;	 /* Drive during race */
    itf->rbPitCmd   = NULL;
    itf->rbEndRace  = endrace;	 /* End of the current race */
    itf->rbShutdown = shutdown;	 /* Called before the module is unloaded */
    itf->index      = index; 	 /* Index used if multiple interfaces */
    return 0; 
} 

/* Called for every track change or new race. */
static void  
initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s) 
{ 
driver[index]->initTrack(track, carHandle, carParmHandle, s);
} 

/* Start a new race. */
static void  
newrace(int index, tCarElt* car, tSituation *s) 
{ 
} 

/* Drive during race. */
static void  
drive(int index, tCarElt* car, tSituation *s) 
{ 

    driver[index]->drive(car, s);

}

/* End of the current race */
static void
endrace(int index, tCarElt *car, tSituation *s)
{
}

/* Called before the module is unloaded */
static void
shutdown(int index)
{
}

