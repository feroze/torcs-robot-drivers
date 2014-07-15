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

static tTrack   *curTrack;

static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s); 
static void newrace(int index, tCarElt* car, tSituation *s); 
static void drive(int index, tCarElt* car, tSituation *s); 
static void endrace(int index, tCarElt *car, tSituation *s);
static void shutdown(int index);
static int  InitFuncPt(int index, void *pt); 


#define BUFSIZE 20
#define NBBOTS 2

#include "driver.h"


static Driver *driver[NBBOTS];
static const char* botname[NBBOTS] = {
    "trafficbot 1", "trafficbot 2"
};
static const char* botdesc[NBBOTS] = {
     "trafficbot 1", "trafficbot 2"
 };


/* 
 * Module entry point  
 */ 
extern "C" int 
trafficbot(tModInfo *modInfo) 
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
    itf->rbNewRace  = newrace;   /* Start a new race */
    itf->rbDrive    = drive;     /* Drive during race */
    itf->rbPitCmd   = NULL;
    itf->rbEndRace  = endrace;   /* End of the current race */
    itf->rbShutdown = shutdown;  /* Called before the module is unloaded */
    itf->index      = index;     /* Index used if multiple interfaces */
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
    //free(botname[index]);
    delete driver[index];
}

