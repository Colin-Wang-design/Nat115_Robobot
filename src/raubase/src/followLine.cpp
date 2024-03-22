
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

#include "followLine.h"

// create class object
FollowLine followLine;


void FollowLine::setup()
{ // ensure there is default values in ini-file
  if (not ini["FollowLine"].has("log"))
  { // no data yet, so generate some default values
    ini["FollowLine"]["log"] = "true";
    ini["FollowLine"]["run"] = "false";
    ini["FollowLine"]["print"] = "true";
  }
  // get values from ini-file
  toConsole = ini["FollowLine"]["print"] == "true";
  //
  if (ini["FollowLine"]["log"] == "true")
  { // open logfile
    std::string fn = service.logPath + "log_FollowLine.txt";
    logfile = fopen(fn.c_str(), "w");
    fprintf(logfile, "%% Mission FollowLine logfile\n");
    fprintf(logfile, "%% 1 \tTime (sec)\n");
    fprintf(logfile, "%% 2 \tMission state\n");
    fprintf(logfile, "%% 3 \t%% Mission status (mostly for debug)\n");
  }
  setupDone = true;
}

FollowLine::~FollowLine()
{
  terminate();
}

void FollowLine::run()
{
  if (not setupDone)
    setup();
  // if (ini["FollowLine"]["run"] == "false")
  //   return;
  UTime t("now");
  bool finished = false;
  bool lost = false;
  state = 1;
  oldstate = state;
  const int MSL = 100;
  char s[MSL];
  //
  toLog("FollowLine started");
  //
  while (not finished and not lost and not service.stop)
  {
    switch (state)
    {
      case 1:
        // start slow driving
        pose.resetPose();
        toLog("forward 0.25 m/sec");
        mixer.setVelocity(0.25);
        mixer.setTurnrate(0);
        state = 2;
        break;
      case 2: // forward until distance, then look for edge
        if (pose.dist > 0.25)
        {
          //toLog("Continue until edge is found");
          toLog("settings edge mode");
          mixer.setEdgeMode(true/*follow left edge*/, 0);
          mixer.setVelocity(0.2); // slow
          state = 3;
          t.now();
          pose.dist = 0;
        }
        else if (t.getTimePassed() > 10)
        { // line should be found within 10 seconds, else lost
          toLog("failed to find line after 10 sec");
          lost = true;
        }
        break;
      case 3: // forward 20cm looking for line, then turn left
        if (pose.dist > 0.2)
        {
          //toLog("found line, turn left");
          // set to edge control, left side and 0 offset
          /* mixer.setVelocity(0.15); // slow
          mixer.setTurnrate(-0.8); // rad/s */
          pose.resetPose();
          mixer.setVelocity(0.06);
          mixer.setDesiredHeading(M_PI/2);
          state = 4;
          pose.dist = 0;
          pose.turned = 0;
          t.now();
          
        }
        else if (t.getTimePassed() > 100)
        { // line should be found within 10 seconds, else lost
          toLog("failed to find line after 10 sec / 30cm");
          lost = true;
        }
        break;
      case 4: // Continue turn until right edge is almost reached, then follow right edge
        if (medge.edgeValid /*and medge.leftEdge > 0.04*/ and pose.turned >= M_PI/2)
        {
          toLog("Line detected, that is OK to follow");
          pose.resetPose();
          mixer.setEdgeMode(true /* left */, -0.03 /* offset */);
          mixer.setVelocity(0.3);
          mixer.setTurnrate(-0.5);
          state = 5;
          t.now();
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
    toLog("FollowLine got lost - stopping");
    mixer.setVelocity(0);
    mixer.setTurnrate(0);
  }
  else
    toLog("FollowLine finished successfully");
}


void FollowLine::terminate()
{ //
  if (logfile != nullptr)
    fclose(logfile);
  logfile = nullptr;
}

void FollowLine::toLog(const char* message)
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
