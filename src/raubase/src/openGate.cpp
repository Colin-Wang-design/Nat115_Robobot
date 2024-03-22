
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

#include "openGate.h"

// create class object
OpenGate openGate;


void OpenGate::setup()
{ // ensure there is default values in ini-file
  if (not ini["openGate"].has("log"))
  { // no data yet, so generate some default values
    ini["openGate"]["log"] = "true";
    ini["openGate"]["run"] = "true";
    ini["openGate"]["print"] = "true";
  }
  // get values from ini-file
  toConsole = ini["openGate"]["print"] == "true";
  //
  if (ini["openGate"]["log"] == "true")
  { // open logfile
    std::string fn = service.logPath + "log_openGate.txt";
    logfile = fopen(fn.c_str(), "w");
    fprintf(logfile, "%% Mission openGate logfile\n");
    fprintf(logfile, "%% 1 \tTime (sec)\n");
    fprintf(logfile, "%% 2 \tMission state\n");
    fprintf(logfile, "%% 3 \t%% Mission status (mostly for debug)\n");
  }
  setupDone = true;
}

OpenGate::~OpenGate()
{
  terminate();
}

void OpenGate::run(int startFromState)
{
  if (not setupDone)
    setup();
  // if (ini["openGate"]["run"] == "false")
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

  float gateDistThreshold = 0.4;  // In meters
  
  toLog("openGate started");
  
  while (not finished and not lost and not service.stop)
  {
    switch (state)
    {
      case 0:
        // Startup sequence
        pose.resetPose();
        t.now();
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
        // Go found the tunnel gate first #TODO
        if (gateFound)
        {
          gateFound = false;
          finished = true;
          break;
        }
        else if (!gateFound) {
          // probe for gate using left distance sensor (dist.dist[1])
          if (dist.dist[1] < gateDistThreshold) {
            toLog("Tunnel found gate!");
            gateFound = true;
            mixer.setVelocity(-0.15); // reverse the Robobot 10cm
            mixer.setTurnrate(0); 
            pose.dist = 0;
            state = 4;
          }

        }
        else if (t.getTimePassed() > 100)
        { // line should be found within 10 seconds, else lost
          toLog("failed to find line after 10 sec / 30cm");
          lost = true;
        }
        break;
      case 4: //Do a First left pi/2 turn
        if (pose.dist < -0.13)
        {
          toLog("Start Turn");
          mixer.setVelocity(0.06);
          //mixer.setTurnrate(1);
          mixer.setDesiredHeading(M_PI/2);
          pose.turned = 0;
          t.now();
          state = 5;
        }
        
        //pose.dist = 0;
        
        break;
      case 5: // Second 20cm forward
        if (t.getTimePassed() > 0.8)
        { 
          /* pose.resetPose();
          //mixer.setVelocity(0.06);
          mixer.setDesiredHeading(-M_PI/2); */
          //pose.dist = 0;
          //finished = true;
          //pose.resetPose();
          mixer.setVelocity(0.30); // Slow forward
          mixer.setTurnrate(0); // No turn
          //t.now();
          pose.dist = 0;
          state = 6;
        }
        else if (t.getTimePassed() > 10)
        {
          toLog("too long time");
          finished = true;
        }
        /* else if (not medge.edgeValid)
        {
          toLog("Lost line");
          lost = true;
        } */
        break;
      case 6: // Then do a pi/2 right turn
        if (pose.dist >= 0.19)
        { // done
          pose.resetPose();
          //mixer.setVelocity(0.06);
          mixer.setDesiredHeading(-M_PI/2);
          //finished = true;
          pose.turned = 0;
          t.now();
          state = 7;
        }
        else if (t.getTimePassed() > 10)
        {
          toLog("too long time");
          lost = true;
        }
        break;
      case 7: // move forward 10cm
        if (t.getTimePassed() > 0.8)
        { // done
          mixer.setVelocity(0.30); // Slow forward
          mixer.setTurnrate(0); // No turn
          pose.dist = 0;
          //t.now();
          state = 8;
        }
        else if (t.getTimePassed() > 10)
        {
          toLog("too long time");
          lost = true;
        }
        break;
        case 8: // Continue until edge is found
          if (pose.dist > 0.2)
          { // done
            pose.resetPose();
            //mixer.setEdgeMode(true /* left */, -0.01 /* offset */);
            toLog("Continue until edge is found");
            mixer.setVelocity(0.06);
            //mixer.setTurnrate(1);
            mixer.setDesiredHeading(M_PI/1.5);
            pose.turned = 0;
            t.now();
            state = 9;
            //finished = true;
            
          }
          break;
        case 81:

        break;
        case 9: // finish Last pi/2 turn
          if (true || medge.width > 0.05)
          { // done  
            toLog("found line, turn left");
            // set to edge control, left side and 0 offset
            mixer.setVelocity(0.2); // slow
            mixer.setTurnrate(0.45); // rad/s
            pose.dist = 0;
            pose.turned = 0;
            state = 10;
            //finished = true;
          }
          else if (t.getTimePassed() > 10 or pose.dist > 0.6)
        { // line should be found within 10 seconds, else lost
          toLog("failed to find line after 10 sec / 30cm");
          lost = true;
        }
          break;
        case 10: // Continue turn until right edge is almost reached, then follow right edge
          if (medge.edgeValid and medge.rightEdge > -0.04 and pose.turned > 0.45)
          {
            toLog("Line detected, that is OK to follow");
            mixer.setEdgeMode(false /* right */, -0.03 /* offset */);
            mixer.setVelocity(0.2);
            pose.dist = 0;
            state = 11;
            
          }
          else if (t.getTimePassed() > 20)
          {
            toLog("too long time");
            lost = true;
          }
          else if (pose.dist > 1.0)
          {
            toLog("Driven too long");
            state = 90;
          }
        break;
          break;
        case 11: // follow edge until crossing line, the go straight
          if (pose.dist > 1.25)
          { // go straight
            /* mixer.setVelocity(0);
            mixer.setTurnrate(0);
            finished = true; */
            mixer.setVelocity(0.55);
            toLog("Go find the start gate");
            pose.dist = 0;
            state = 12;
          }
          else if (t.getTimePassed() > 10)
          {
            toLog("too long time");
            lost = true;
          }
          break;
        case 12:// 
          if (pose.dist > 1 and dist.dist[1] <= 0.3)
          { // wall found
            toLog("Start gate found");
            mixer.setVelocity(0.3);
            pose.dist = 0;
            state = 13;
            //finished = true;
          }
          else if (t.getTimePassed() > 100)
          {
            toLog("too long time");
            lost = true;
          }
          else if (pose.dist > 3)
          {
            toLog("too far");
            lost = true;
          }
          break;
        case 13:// Turn around
          if (pose.dist >= 0.5)
          { 
            pose.resetPose();
            mixer.setVelocity(0.06);
            mixer.setDesiredHeading(M_PI);//Turn 180 degree
            //mixer.setTurnrate(0.9);
            t.now();
            pose.turned = 0;
            state = 131;
            //finished = true;
          }
        break;
        case 131:
          if(t.getTimePassed() > 0.8 and pose.turned >0)
          {
            mixer.setVelocity(0.1); // slow
            mixer.setTurnrate(0.9); // rad/s
            pose.dist = 0;
            pose.turned = 0;
            state = 14;
          }
          else if(t.getTimePassed() > 0.8)
          {
            mixer.setVelocity(0.1);
            mixer.setTurnrate(-0.9);
            pose.turned = 0;
            state = 14;
          }
        break;
        case 14:
          if(/*pose.turned >= M_PI || t.getTimePassed() > 1 and */medge.edgeValid and medge.rightEdge > -0.04 and pose.turned > 0)
          {
            //pose.resetPose();
            toLog("Start race!");
            //mixer.setTurnrate(0.3);
            mixer.setEdgeMode(false /* right */, -0.03 /* offset */);
            mixer.setVelocity(0.25);//Slow start
            pose.dist = 0;
            state = 15;
          }
          else if(medge.edgeValid and medge.leftEdge > -0.04 and pose.turned < 0)
          {
            toLog("Start race!");
            mixer.setEdgeMode(true /* left */, -0.03 /* offset */);
            mixer.setVelocity(0.25);//Slow start
            pose.dist = 0;
            state = 15;
          }
          else if (t.getTimePassed() > 10)
          {
            toLog("too long time");
            lost = true;
          }
          else if (pose.dist > 1.0)
          {
            toLog("Driven too long");
            state = 90;
          }
        break;
        case 15:
          if(dist.dist[1] <= 0.3)
          {
            mixer.setEdgeMode(false /* right */, -0.03 /* offset */);//Along the right edge
            mixer.setVelocity(0.45);//speed up
            toLog("Speed up to 0.5");
            pose.dist = 0;
            state = 16;
          }
          else if (pose.dist > 1.5)
          {
            lost = true;
          }
        break;
        case 16:
          if(pose.dist >= 0.15)
          {
            mixer.setEdgeMode(false /* right */, -0.03 /* offset */);//Along the right edge
            mixer.setVelocity(0.5);//speed up
            toLog("Speed up to 0.8");
            pose.dist = 0;
            state = 17;
          }
        break;
        case 17:
          if(pose.dist >= 4.5)
          {
            mixer.setEdgeMode(false /* right */, -0.03 /* offset */);//Along the right edge
            mixer.setVelocity(0.5);//speed up
            toLog("Almost reach the turn, speed down");
            pose.dist = 0;
            state = 18;
          }
        break;
        case 18:
          if(pose.dist >= 0.5)
          {
            mixer.setVelocity(0.4);//speed down
            toLog("Speed dowm");
            pose.dist = 0;
            state = 19;
          }
        break;
        case 19:
          if(pose.dist >= 0.3)
          {
            mixer.setVelocity(0.55);//speed up
            toLog("Speed up");
            state = 20;
          }
        break;
        case 20:
          if(pose.dist > 1.2)
          {
            toLog("Alomost reach the end speed down!");
            mixer.setVelocity(0.45); //speed down
            state = 21;
          }
        break;
        case 21:
          if(dist.dist[1] <= 0.3)
          {
            mixer.setVelocity(0.0);//stop reach end
            //pose.dist = 0;
            //state = 16;
            toLog("Reach the end");
            finished = true;
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
    toLog("openGate got lost - stopping");
    mixer.setVelocity(0);
    mixer.setTurnrate(0);
  }
  else
    toLog("openGate finished successfully");
    /* mixer.setVelocity(0);
    mixer.setTurnrate(0); */
}


void OpenGate::terminate()
{ //
  if (logfile != nullptr)
    fclose(logfile);
  logfile = nullptr;
}

void OpenGate::toLog(const char* message)
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
