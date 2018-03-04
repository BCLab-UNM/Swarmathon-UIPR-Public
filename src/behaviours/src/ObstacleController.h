#ifndef OBSTACLECONTOLLER_H
#define OBSTACLECONTOLLER_H

#include "Controller.h"
#include "Tag.h"
#include "PID.h"
#include "DriveController.h"
#include <angles/angles.h> 

class ObstacleController : virtual Controller
{
public:
  ObstacleController();

  Result result;

  void Reset() override;
  Result DoWork() override;
  void setSonarData(float left, float center, float right);
  void setCurrentLocation(Point currentLocation);
  void setTagData(vector<Tag> tags);
  bool ShouldInterrupt() override;
  bool HasWork() override;
  void setIgnoreCenterSonar();
  void setCurrentTimeInMilliSecs( long int time );
  void setTargetHeld ();

  //EDITED
  bool getObstacleInfo();

  // Checks if a target is held and if so resets the state of the obestacle controller otherwise does nothing
  void setTargetHeldClear();
  //Asked by logiccontroller to determine if drive controller should have its waypoints cleared
  bool getShouldClearWaypoints() {bool tmp = clearWaypoints; clearWaypoints = false; return tmp;}

  PID pid; 
  PIDConfig pidC, pidC2;
  void setPIDController(PIDConfig pidC, PIDConfig pidC2);

protected:

  void ProcessData();

private:

  // Try not to run over the collection zone
  void avoidCollectionZone();

  // Try not to run into a physical object
  void avoidObstacle();
  void follow_Wall();
  vector<double> distRead; //distances read from sonars
  int getDirection(); //returns direction (-1 or 1)
  int direction; //-1 for following right wall. 1 for folloeing left wall
  int size; //reenges vector size
  int minIndex; //min and max indexes for angleMin calc
  int maxIndex; 
  double angleMin, distMin; //minimun angle and distance between object and rover vars
  double e, diffE;  //error vars

  // Are there AprilTags in the camera view that mark the collection zone
  // and are those AprilTags oriented towards or away from the camera.
  bool checkForCollectionZoneTags( vector<Tag> );

  const float K_angular = 1.0; //radians a second turn rate to avoid obstacles
  const float reactivate_center_sonar_threshold = 0.8; //reactive center sonar if it goes back above this distance, assuming it is deactivated
  const int targetCountPivot = 6; ///unused variable
  const float obstacleDistancePivot = 0.2526; //used in angleMin calc
  const float triggerDistance = 0.8;

  /*
     * Member variables
     */


  bool obstacleInterrupt; //records if obstacle has interupted
  bool obstacleDetected;  //records if an obstacle has been detected
  bool obstacleAvoided; //record if an obstacke has been avoided
  bool clearWaypoints = false;  //record if drivecontrollers waypoints should be cleared

  float left = 0; //distance on left ultrasound
  float center = 0; //distance on center ultrasound
  float right = 0; //distance on right ultrasound

  unsigned int count_left_collection_zone_tags;
  unsigned int count_right_collection_zone_tags;

  // Ignore the center sonar because we are carrying a target
  bool ignore_center_sonar = false;

  Point currentLocation;

  long int current_time;
  long int timeSinceTags;
  long int delay;

  bool targetHeld = false;
  bool previousTargetState = false;

  bool phys = false; // Physical obstacle
  bool collection_zone_seen = false; // The obstacle is the collection zone

  bool set_waypoint = false;
  bool can_set_waypoint = false;

  float camera_offset_correction = 0.020; //meters;
};

#endif // OBSTACLECONTOLLER_H
