/* 

CUSTOM PLAN TO TEST FOLLOWING THE WHITE LINE

*/
#pragma once


using namespace std;

/**
 * Class intended to accomplish a short mission,
 * e.g. one challenge or part of a challenge
 * */
class KeepRight
{
public:
  /**
   * destructor */
    ~KeepRight();
  /** setup and request data */
  void setup();
  /**
   * run this mission */
  void run(int startFromState);
  /**
   * terminate */
  void terminate();

private:
  /**
   * Write a timestamped message to log */
  void toLog(const char * message);
  /// added to log
  int state, oldstate;
  /// private stuff
  // debug print to console
  bool toConsole = true;
  // logfile
  FILE * logfile = nullptr;
  bool setupDone = false;
  // default params that can be loaded using robot.ini
  float defaultVelocity = 0.1; // 0.1m/s slow!
  float defaultTurnrate = 0.0;
  bool defaultLeftEdge = false; // If true, will follow the left
};

/**
 * Make this visible to the rest of the software */
extern KeepRight keepRight;

