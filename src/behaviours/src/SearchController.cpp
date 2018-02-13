#include "SearchController.h"
#include <angles/angles.h>

SearchController::SearchController() {
  rng = new random_numbers::RandomNumberGenerator();
  currentLocation.x = 0;
  currentLocation.y = 0;
  currentLocation.theta = 0;

  centerLocation.x = 0;
  centerLocation.y = 0;
  centerLocation.theta = 0;
  result.PIDMode = FAST_PID;
  result.fingerAngle = M_PI/2;
  result.wristAngle = M_PI/4;
  outTravel = true;
  searchObstacle = false;
  leftAdjust = false;
  rightAdjust = false;
  triangularSection = 1;
  magnitude = 0;
  angle = 0;

}

void SearchController::Reset() {
  result.reset = false;
}

Result SearchController::DoWork() {


    cout << "Total IDs " << totalIds << endl;

    if(myId == 1)
    {
        triangularSearch(myId,triangularSection);
    }

    else if(myId == 2)
    {
    }

    else if(myId == 3)
    {

    }
    result.wpts.waypoints.clear();
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);

    return result;
  }

void SearchController::triangleSearch(int myId,int triangularSection)
{
    //Developing code for first traingular section
    switch(triangularSection)
    {
      case 1: //This section is from degrees 0->45
        {
           magnitude = rng->uniformReal(0,9.1);
           angle = rng->uniformReal(0,45);

           searchLocation.x = magnitude * cos(angle);
           searchLocation.y = magnitude * sin(angle);

           cout << "Looking for location: (" << searchLocation.x << "," << searchLocation.y << ")" << endl;
        }
      default:
        {
          cout << "Error in Triangular Section!!" << endl;
        }
    }
}
void SearchController::setObstacleDetected(bool var)
{
  this->searchObstacle = var;
}

void SearchController::SetCenterLocation(Point centerLocation) {

  float diffX = this->centerLocation.x - centerLocation.x;
  float diffY = this->centerLocation.y - centerLocation.y;
  this->centerLocation = centerLocation;

  if (!result.wpts.waypoints.empty())
  {
  result.wpts.waypoints.back().x -= diffX;
  result.wpts.waypoints.back().y -= diffY;
  }

}

void SearchController::smartRandomSearch(int wallLocation, int myId){
}

void SearchController::lawnMowerSearch(int wallLocation,int myId){
}

void SearchController::SetCurrentLocation(Point currentLocation) {
  this->currentLocation = currentLocation;
}

void SearchController::ProcessData() {
}

bool SearchController::ShouldInterrupt(){
  ProcessData();

  return false;
}

bool SearchController::HasWork() {
  return true;
}

void SearchController::SetSuccesfullPickup() {
  succesfullPickup = true;
}
