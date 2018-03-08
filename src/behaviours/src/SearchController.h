#ifndef SEARCH_CONTROLLER
#define SEARCH_CONTROLLER

#include <random_numbers/random_numbers.h>
#include "Controller.h"
#include "Triangle.h"
#include <thread> 

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
  void CreateThread();
  void newPointManager();

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

  Point setSearchLocation(float x, float y);
  
  float degToRad(float deg);
  float radToDeg(float rad);

  void ProcessData();
  
private:

  Point currentLocation, centerLocation, searchLocation;
  Result result;
  random_numbers::RandomNumberGenerator* rng;

  int pointCounter, triangleSel,sideSel,pointLimit;
  int attemptCount = 0;
  int mapSize;
  int triangularSection = 1;

  float ghostWall;
  float magnitude;
  float angle, unknownAngle;
  float triangleSquare;
  float sideBoundary;

  bool searchObstacle;
  bool movingLeft, movingRight = false;

  // Search state
  // Flag to allow special behaviour for the first waypoint
  bool first_waypoint = true;
  bool first_side_waypoint = true;
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
  vector<Point> visitedLoc;
  //**

  //** Functions
  int getMapSize();
  void setTriangleSquareArea(float area);
  void setGhostWall(float ghostWall);
  void setSideBoundary();
  void giveTask2Robot();
  void triangleSearch(int myId, int triangularSection,float triangleSquare);
  void sideSearch(int myId, int sideSection,float triangleSquare);
  
  //**
  //

};

#endif /* SEARCH_CONTROLLER */
