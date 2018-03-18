#ifndef SEARCH_CONTROLLER
#define SEARCH_CONTROLLER

#include <random_numbers/random_numbers.h>
#include "Controller.h"
#include "Triangle.h"

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
  // --------------------------------------------------------------- // Jomar --------------------------------------------------------------------------


  void aTagDetected();
  void droppedOFF();
  void cantSeeTarget();
  void FidelityImprovement();

  // Getters
	bool getSdropped() { return Sdropped; }
  bool getDeleteVector() { return deleteVector; }
  bool getCantSeeTargetDontRepeat() { return cantSeeTargetDontRepeat; }
  bool getTagDetectedCatched(){ return TagDetectedCatched; }
  Point getSearchLocation() {return searchLocation; }

  void setClusterLocation (std::vector<Point> clusterLocation){this->clusterLocation = clusterLocation;}
  bool getClusterAssigned(){return this->clusterAssigned;}
  
  // Setters
  void setSdropped(bool Sdropped) { this->Sdropped = Sdropped; }
  void setDeleteVector(bool deleteVector) { this->deleteVector = deleteVector; }
  void setCantSeeTargetDontRepeat(bool cantSeeTargetDontRepeat) { this->cantSeeTargetDontRepeat = cantSeeTargetDontRepeat; }
  void setTagDetectedCatched(bool TagDetectedCatched){ this->TagDetectedCatched = TagDetectedCatched; }
  void setClusterAssigned(bool newValue){this->clusterAssigned = newValue;}
  void setNeedNewPoint (bool pointInsideObstacle){this->pointInsideObstacle = pointInsideObstacle; }
  // --------------------------------------------------------------- // Jomar --------------------------------------------------------------------------

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
  //void setClusterLocation (std::vector<Point> clusterLoc){this->clusterLocation = clusterLoc;}
  //**
  //
  // --------------------------------------------------------------- // Jomar --------------------------------------------------------------------------
protected:

  Point setSearchLocation(float x, float y);
  
  float degToRad(float deg);
  float radToDeg(float rad);

  void ProcessData();
  
private:

  Point currentLocation, centerLocation, searchLocation, previousLoc;
  Result result;
  random_numbers::RandomNumberGenerator* rng;

  int pointCounter, triangleSel,sideSel,trianglePointLimit,triangleSel2;
  int attemptCount = 0;
  int mapSize;
  int triangularSection = 1;
  
  bool Sdropped = false;
  bool cantSeeTargetDontRepeat = false;
  bool arrives = false;
  bool deleteVector = false;
  bool fidImprovement = false;
  bool TagDetectedCatched = true;
  bool pointInsideObstacle  = false;
  bool firstClusterCommand = true;
  bool clustred = false;
  
  float ghostWall;
  float magnitude;
  float angle, unknownAngle;
  float triangleSquare;
  float sideBoundary;
  float sideOffset;

  bool searchObstacle;
  bool goRight,goLeft;
  bool sideInit;
  bool finals;


  // Search state
  // Flag to allow special behaviour for the first waypoint
  bool first_waypoint;
  bool first_side_waypoint;
  bool succesfullPickup = false;

  // EDIT
  //** Variables
  bool clusterAssigned;
  int myId;
  int totalIds;
  bool publishVisitedPointFlag = true;
  vector<Point> clusterLocation;
  //This Struct is used to share a Visited point with Logic Controller.
  Point latestVisitedPoint;
  //This Vector is used to know all the points that were published into Master.
  std::vector<Point> visitedPoints;
  vector<Point> visitedLoc;
  vector<Point> SavedPointsVector;
  //**

  //** Functions
  int getMapSize();
  void setTriangleSquareArea(float area);
  void setGhostWall(float ghostWall);
  void setSideBoundary();
  void giveTask2Robot();
  void triangleSearch(int myId, int triangularSection,float triangleSquare);
  void sideSearch(int myId, int sideSection,float triangleSquare,float offset);
  void thenGoRight();
  void thenGoLeft();
  bool checkAvailableDistance(int sideSel);
  float getSideOffset();
  void randomWalk();
  void displayVector(float magnitude, float angle);
  Point generateRandomTriangleLoc(float firstBound, float secondBound, float triangleSquare);
  float angleTraslation(float newAngle);
  //**
  //
  bool x_first_waypoint = true;
  bool x_front_pared = false;
  bool x_front_giro_pared = false;
  bool x_front_pared_finish = false;


// --------------------------------------------------------------- // Jomar --------------------------------------------------------------------------


};

#endif /* SEARCH_CONTROLLER */
