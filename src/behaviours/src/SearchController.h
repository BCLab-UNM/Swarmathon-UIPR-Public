#ifndef SEARCH_CONTROLLER
#define SEARCH_CONTROLLER

#include <random_numbers/random_numbers.h>
#include "Controller.h"

/**
 * This class implements the search control algorithm for the rovers. The code
 * here should be modified and enhanced to improve search performance.
 */
class SearchController : virtual Controller {

public:

  SearchController();

  void Reset() override;

  // performs search pattern
  Result DoWork() override;
  bool ShouldInterrupt() override;
  bool HasWork() override;

  // sets the value of the current location
  //void UpdateData(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D centerLocation);
  void SetCurrentLocation(Point currentLocation);
  void SetCenterLocation(Point centerLocation);
  void SetSuccesfullPickup();


  // EDIT
  //** Variables

  //**

  //** Functions
  void setObstacleDetected(bool var);
  void getID(int myId){this->myId = myId;}
  void getTotalIds(int totalIds){this->totalIds = totalIds;}

  //Receives point lists. This is only used by Logic Controller.
  void setVisitedVector(std::vector<Point> visitedVector){this->visitedPoints = visitedVector;}

  //Prepare a Point struct to be shared with Logic Controller. Use it in Search Controller.
  void setVisitedPoint(Point searchedPoint){this->latestVisitedPoint = searchedPoint;}

  //Flag to control when to publish a Point to Master. This is verified in Logic Controller and ROSadapter.
  void setVisitedPointFlag (bool visitedPointFlag){this->publishVisitedPointFlag = visitedPointFlag;}

  //This is the method that will be used in Logic Controller to share a Point struct that was set using the setVisitedPoint.
  const Point getVisitedPoint(){return this->latestVisitedPoint;}

  //This is to verify if we Publish the point in Master.
  const bool getVisitedFlag(){return this->publishVisitedPointFlag;}
  //**
  //

protected:

  void ProcessData();
  void lawnMowerSearch(int wallLocation, int myId);
  void smartRandomSearch(int wallLocation, int myId);
  void triangleSearch(int myId);
  Point SearchController::setSearchLocation(float x, float y)

private:

  Result result;

  random_numbers::RandomNumberGenerator* rng;
  Point currentLocation;
  Point centerLocation;
  Point searchLocation;
  bool outTravel;
  bool searchObstacle;
  int attemptCount = 0;
  bool leftAdjust;
  bool rightAdjust;
  int mapSize = 15;
  int triangularSection = 1;
  float magnitude;
  float angle;

  // Search state
  // Flag to allow special behaviour for the first waypoint
  bool first_waypoint = true;
  bool succesfullPickup = false;

  // EDIT
  //** Variables
  int myId;
  int totalIds;
  bool publishVisitedPointFlag = true;

  //This Struct is used to share a Visited point with Logic Controller.
  Point latestVisitedPoint;
  //This Vector is used to know all the points that were published into Master.
  std::vector<Point> visitedPoints;
  //**

  //** Functions

  //**
  //

};

#endif /* SEARCH_CONTROLLER */
