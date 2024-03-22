
#include <string>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include "mpose.h"
#include "steensy.h"
#include "uservice.h"
#include "sencoder.h"
#include "utime.h"
#include "cmotor.h"
#include "cservo.h"
#include "medge.h"
#include "cedge.h"
#include "cmixer.h"
#include "sdist.h"

#include "keepRight.h"

// create class object
KeepRight keepRight;


void KeepRight::setup()
{ // ensure there is default values in ini-file
  if (not ini["KeepRight"].has("log"))
  { // no data yet, so generate some default values
    ini["KeepRight"]["log"] = "true";
    ini["KeepRight"]["run"] = "false";
    ini["KeepRight"]["print"] = "true";
  }
  // get values from ini-file
  toConsole = ini["KeepRight"]["print"] == "true";
  //
  if (ini["KeepRight"]["log"] == "true")
  { // open logfile
    std::string fn = service.logPath + "log_KeepRight.txt";
    logfile = fopen(fn.c_str(), "w");
    fprintf(logfile, "%% Mission KeepRight logfile\n");
    fprintf(logfile, "%% 1 \tTime (sec)\n");
    fprintf(logfile, "%% 2 \tMission state\n");
    fprintf(logfile, "%% 3 \t%% Mission status (mostly for debug)\n");
  }
  setupDone = true;
}

KeepRight::~KeepRight()
{
  terminate();
}

void KeepRight::run(int startFromState)
{
  if (not setupDone)
    setup();
  // if (ini["KeepRight"]["run"] == "false")
  //   return;
  UTime t("now");
  bool finished = false;
  bool lost = false;
  state = 0;
  if (startFromState > 0 ) {
    state = startFromState;
  }
  oldstate = state;
  const int MSL = 100;
  char s[MSL];
  bool gateFound = false;

  float gateDistThreshold = 0.30;  // In meters
  
  toLog("KeepRight started");
  
  while (not finished and not lost and not service.stop)
  {
    switch (state)
    {
      case 0:
        // Startup sequence
        pose.resetPose();
        state = 1;
        break;
      case 1:
        toLog("going forward!");
        mixer.setVelocity(0.30); // Slow start
        mixer.setTurnrate(0); // No turn
        state = 2;
        break;
      case 2: // forward until distance, then look for edge
        if (pose.dist > 0.1)
        {
          //toLog("Continue until edge is found");
          toLog("starting edge mode");
          mixer.setEdgeMode(defaultLeftEdge, 0);
          mixer.setVelocity(0.25);
          mixer.setInModeTurnrate(1);
          state = 3;
          pose.dist = 0;
        }
        else if (t.getTimePassed() > 10)
        { // line should be found within 10 seconds, else lost
          toLog("never got going ._.");
          lost = true;
        }
        break;
      case 3: 
        // Go through the first gate
        // When the gate has been passed, go right or left depending on the defaultLeftEdge bool. #TODO
        if (gateFound)
        {
          gateFound = false;
          finished = true;
          break;
        }
        else if (!gateFound) {
          // probe for gate using left distance sensor (dist.dist[1])
          if (dist.dist[1] < gateDistThreshold) {
            toLog("found gate");
            gateFound = true;
          }

        }
        else if (t.getTimePassed() > 100)
        { // line should be found within 10 seconds, else lost
          toLog("failed to find line after 10 sec / 30cm");
          lost = true;
        }
        break;
      case 4: // Continue turn until right edge is almost reached, then follow right edge
        if (medge.edgeValid and medge.rightEdge > -0.04 and pose.turned > 0.3)
        {
          toLog("Line detected, that is OK to follow");
          mixer.setEdgeMode(false /* right */, -0.03 /* offset */);
          mixer.setVelocity(0.3);
          state = 5;
          pose.dist = 0;
        }
        else if (t.getTimePassed() > 10)
        {
          toLog("Time passed, no crossing line");
          lost = true;
        }
        else if (pose.dist > 1.0)
        {
          toLog("Driven too long");
          lost = true;
        }
        break;
      case 5: // follow edge until crossing line, the go straight
        if (medge.width > 0.075 and pose.dist > 0.2)
        { // go straight
          mixer.setTurnrate(0);
          pose.dist = 0;
          state = 6;
        }
        else if (t.getTimePassed() > 10)
        {
          toLog("too long time");
          finished = true;
        }
        else if (not medge.edgeValid)
        {
          toLog("Lost line");
          lost = true;
        }
        break;
      case 6: // continue straight for 1 meter
        if (pose.dist > 1)
        { // done
          toLog("done");
          mixer.setVelocity(0);
          finished = true;
        }
        else if (t.getTimePassed() > 10)
        {
          toLog("too long time");
          lost = true;
        }
        break;
      default:
        lost = true;
        break;
    }
    if (state != oldstate)
    { // C-type string print
      snprintf(s, MSL, "State change from %d to %d", oldstate, state);
      toLog(s);
      oldstate = state;
      t.now();
    }
    // wait a bit to offload CPU (4000 = 4ms)
    usleep(4000);
  }
  if (lost)
  { // there may be better options, but for now - stop
    toLog("KeepRight got lost - stopping");
    mixer.setVelocity(0);
    mixer.setTurnrate(0);
  }
  else
    toLog("KeepRight finished successfully");
}


void KeepRight::terminate()
{ //
  if (logfile != nullptr)
    fclose(logfile);
  logfile = nullptr;
}

void KeepRight::toLog(const char* message)
{
  UTime t("now");
  if (logfile != nullptr)
  {
    fprintf(logfile, "%lu.%04ld %d %% %s\n", t.getSec(), t.getMicrosec()/100,
            oldstate,
            message);
  }
  if (toConsole)
  {
    printf("%lu.%04ld %d %% %s\n", t.getSec(), t.getMicrosec()/100,
           oldstate,
           message);
  }
}
