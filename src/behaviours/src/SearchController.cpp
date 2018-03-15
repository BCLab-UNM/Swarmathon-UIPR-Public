#include "SearchController.h"
#include <angles/angles.h>

SearchController::SearchController()
{
  rng = new random_numbers::RandomNumberGenerator();
  currentLocation.x = 0;
  currentLocation.y = 0;
  currentLocation.theta = 0;

  centerLocation.x = 0;
  centerLocation.y = 0;
  centerLocation.theta = 0;
  result.PIDMode = FAST_PID;
  result.fingerAngle = M_PI / 2;
  result.wristAngle = M_PI / 4;
  outTravel = true;
  searchObstacle = false;
  leftAdjust = false;
  rightAdjust = false;

  reached = true;
  previous = currentLocation;
}

void SearchController::Reset()
{
  result.reset = false;
}

void SearchController::setNeedNewPoint(bool pointInsideObstacle)
{
  this->pointInsideObstacle = pointInsideObstacle;
}

Point SearchController::getSearchLocation(){
  return searchLocation;
}

Result SearchController::DoWork()
{

  cout << "Total IDs " << totalIds << endl;

  if (hypot(previous.x - currentLocation.x, previous.y - currentLocation.y) < .3)
  {
    reached = true;
  }
  if (pointInsideObstacle)
  {
    cout << "Point is inside obstacle. Creating new Point!" << endl; //Debug

    if (myId == 1)
    {
      lawnMowerSearch(-4, myId);
    }

    else if (myId == 2)
    {
      lawnMowerSearch(4, myId);
    }

    else if (myId == 3)
    {
      smartRandomSearch(-4, myId);
    }
    reached = false;
    pointInsideObstacle = false;
  }
  else if (reached)
  {
    if (myId == 1)
    {
      lawnMowerSearch(-4, myId);
    }

    else if (myId == 2)
    {
      lawnMowerSearch(4, myId);
    }

    else if (myId == 3)
    {
      smartRandomSearch(-4, myId);
    }
    previous = searchLocation;
    reached = false;
  }
  else
  {
    searchLocation = previous;
  }

  //cout << "SL: " <<
  result.wpts.waypoints.clear();
  result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);

  return result;
}

void SearchController::setObstacleDetected(bool var)
{
  this->searchObstacle = var;
}

void SearchController::SetCenterLocation(Point centerLocation)
{

  float diffX = this->centerLocation.x - centerLocation.x;
  float diffY = this->centerLocation.y - centerLocation.y;
  this->centerLocation = centerLocation;

  if (!result.wpts.waypoints.empty())
  {
    result.wpts.waypoints.back().x -= diffX;
    result.wpts.waypoints.back().y -= diffY;
  }
}

void SearchController::smartRandomSearch(int wallLocation, int myId)
{
  double number;
  bool xAccepted = false;
  bool yAccepted = false;
  bool shield = false;
  result.type = waypoint;

  cout << "My ID: " << myId << endl;
  cout << "Publishing visited point: (" << currentLocation.x << "," << currentLocation.y << ")" << endl;
  setVisitedPoint(currentLocation);
  setVisitedPointFlag(true);

  if (first_waypoint)
  {
    first_waypoint = false;
    cout << "Searching for first given location!!" << endl;
    searchLocation.x = -.1;
    searchLocation.y = -.1;
  }
  else
  {
    number = rng->uniformReal(0, -4);
    searchLocation.y = number;
    cout << "Now looking for: (" << searchLocation.x << "," << searchLocation.y << ")" << endl;
  }
}

void SearchController::lawnMowerSearch(int wallLocation, int myId)
{
  //Right Side Robot will search cuadrant I and IV.
  //Left Side Robot will search cuadrant II and III.

  float miniWall;   // imaginary wall to avoid colliding with Physical wall.
  float centerLine; //imaginary line to understand that we reached the Y axis.

  result.type = waypoint;
  cout << "My ID " << myId << endl;

  if (wallLocation < 0) //This if is to know the location of the wall. And set some variables for the behavior of the each robot.
  {
    cout << "Wall is Negative" << endl;
    miniWall = -.5;
    centerLine = 0.01;
  }
  else
  {
    cout << "Wall is Positive" << endl;
    miniWall = .5;
    centerLine = -0.01;
  }

  cout << "Initial Location: (" << currentLocation.x << "," << currentLocation.y << ")" << endl;

  // first location of each robot
  if (first_waypoint)
  {
    first_waypoint = false;
    cout << "Searching for first given location!!" << endl;
    //searchLocation.theta = 0;

    switch (myId)
    {
    case 1:
    {
      searchLocation.x = -.5;
      searchLocation.y = 5;
      break;
    }
    case 2:
    {
      searchLocation.x = .5;
      searchLocation.y = -5; //(wallLocation / -1 - .5); //Modify when whole map configuration is done.
    }
    case 3:
    {
      //Robot 3
    }
    case 4:
    {
      //Robot 4
    }
    }
  }
  else //If the robot reached the first point to visit.
  {
    //this->visitedPoints.push_back(currentLocation); //saving visted location for future usage.(Random Searh)
    //cout << "Point visited last was: (" << this->visitedPoints.at(visitedPoints.size() - 1).x << "," << this->visitedPoints.at(visitedPoints.size() - 1).y << ")" << endl;
    cout << "-Calculating new position-" << endl;

    //if we reach the wall boundary and we havent turned left, adjust left. Left side Robot
    if ((currentLocation.x <= (wallLocation - miniWall)) == true && leftAdjust == false && myId == 1)
    {
      //cout << "-Reached mini wall-" << endl;
      searchLocation.x = currentLocation.x;
      searchLocation.y = currentLocation.y - 1;
      cout << "Now looking for: (" << searchLocation.x << "," << searchLocation.y << ")" << endl;
      leftAdjust = true;
    }
    //if we reached the wall boundary and we haven't turned left, adjust left. Right side Robot
    else if ((currentLocation.x >= (wallLocation - miniWall)) == true && leftAdjust == false && myId == 2)
    {
      //cout << "-Reached mini wall-" << endl;
      searchLocation.x = currentLocation.x;
      searchLocation.y = currentLocation.y + 1;
      cout << "Now looking for: (" << searchLocation.x << "," << searchLocation.y << ")" << endl;
      leftAdjust = true;
    }
    //if we reached the center axis and we havent turned right, adjust right. Left side Robot
    else if ((currentLocation.x > centerLine == true) && rightAdjust == false && myId == 1)
    {
      //cout << "-Reached center line-" << endl;
      searchLocation.x = currentLocation.x;
      searchLocation.y = currentLocation.y - 1;
      cout << "Now looking for: (" << searchLocation.x << "," << searchLocation.y << ")" << endl;
      rightAdjust = true;
      leftAdjust = false;
    }
    //if we reached the center axis and we havent turned right, adjust right. Right side Robot
    else if ((currentLocation.x < centerLine == true) && rightAdjust == false && myId == 2)
    {
      //cout << "-Reached center line-" << endl;
      searchLocation.x = currentLocation.x;
      searchLocation.y = currentLocation.y + 1;
      cout << "Now looking for: (" << searchLocation.x << "," << searchLocation.y << ")" << endl;
      rightAdjust = true;
      leftAdjust = false;
    }
    //if we turned left and no other condition have been met, keep moving toward center axis.
    else if (leftAdjust == true)
    {
      //cout << "-Walking towards center-" << endl;
      rightAdjust = false;
      searchLocation.y = currentLocation.y;

      if (miniWall < 0) //if Left Side Robot
      {
        searchLocation.x = currentLocation.x + 1;
      }
      else
      { //if Right Side Robot
        searchLocation.x = currentLocation.x - 1;
      }
    }
    //if we are going outside the base searching.
    else
    {
      //cout << "-Walking towards wall-" << endl;
      if (miniWall < 0) //if Left Side Robot
      {
        searchLocation.x = currentLocation.x - 1;
      }
      else
      { //if Right Side Robot
        searchLocation.x = currentLocation.x + 1;
      }
      searchLocation.y = currentLocation.y;
      cout << "Now looking for: (" << searchLocation.x << "," << searchLocation.y << ")" << endl;
    }
  }
}

void SearchController::SetCurrentLocation(Point currentLocation)
{
  this->currentLocation = currentLocation;
}

void SearchController::ProcessData()
{
}

bool SearchController::ShouldInterrupt()
{
  ProcessData();

  return false;
}

bool SearchController::HasWork()
{
  return true;
}

void SearchController::SetSuccesfullPickup()
{
  succesfullPickup = true;
}
