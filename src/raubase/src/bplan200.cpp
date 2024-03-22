/*  
 * 
 * Copyright © 2023 DTU,
 * Author:
 * Christian Andersen jcan@dtu.dk
 * 
 * The MIT License (MIT)  https://mit-license.org/
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the “Software”), to deal in the Software without restriction, 
 * including without limitation the rights to use, copy, modify, merge, publish, distribute, 
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software 
 * is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies 
 * or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE 
 * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
 * THE SOFTWARE. */

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
#include <stdio.h>



#include "bplan200.h"

// create class object
BPlan200 plan200;


void BPlan200::setup()
{ // ensure there is default values in ini-file
  if (not ini["plan200"].has("log"))
  { // no data yet, so generate some default values
    ini["plan200"]["log"] = "true";
    ini["plan200"]["run"] = "false";
    ini["plan200"]["print"] = "true";
  }
  // get values from ini-file
  toConsole = ini["plan200"]["print"] == "true";
  //
  if (ini["plan200"]["log"] == "true")
  { // open logfile
    std::string fn = service.logPath + "log_plan200.txt";
    logfile = fopen(fn.c_str(), "w");
    fprintf(logfile, "%% Mission plan200 logfile\n");
    fprintf(logfile, "%% 1 \tTime (sec)\n");
    fprintf(logfile, "%% 2 \tMission state\n");
    fprintf(logfile, "%% 3 \t%% Mission status (mostly for debug)\n");
  }
  setupDone = true;
}

BPlan200::~BPlan200()
{
  terminate();
}



void BPlan200::run()
{
  if (not setupDone){
    setup();
    }
  if (ini["plan40"]["run"] == "false"){
    return;
    }
  UTime t("now");
  bool finished = false;
  bool lost = false;

  state = 1;
  oldstate = state;
  const int MSL = 100;
  char s[MSL];
  double start_turn_time = 0.0;

  toLog("Plan200 started");
  
  while (not finished and not lost and not service.stop)
  {
    switch (state){
      case 1:
        pose.resetPose();
        mixer.setVelocity(0.25);
        mixer.setTurnrate(0);
        state = 2;
        break;
      case 2: 
        if (pose.dist >= 0.3){
          mixer.setTurnrate(0.2);
          state = 3;
          break;
        }
      case 3:
        if(t.getTimePassed() > 4){
          mixer.setTurnrate(0);
          state = 4;
          break;
        }
      case 4:
        if(t.getTimePassed() > 4){
            mixer.setTurnrate(0.3);
            state = 4;
            break;
          }

    }

    if (state != oldstate)
    {
      oldstate = state;
      toLog("state start");
      // reset time in new state
      t.now();
    }
    // wait a bit to offload CPU
    usleep(2000);

  }


}


void BPlan200::terminate()
{ //
  if (logfile != nullptr)
    fclose(logfile);
  logfile = nullptr;
}

void BPlan200::toLog(const char* message)
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

