#include "SearchController.h"
#include <angles/angles.h>

SearchController::SearchController() {

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

  first_waypoint = true;

}

void SearchController::Reset() {
  result.reset = false;
}

Result SearchController::DoWork() {
    result.type = waypoint;    

  if (fidImprovement) {
    cout << "Using Fidelity" << endl;
    FidelityImprovement();
  }

  else{
    //cout << "Total IDs " << totalIds << endl;
    
    if(myId == 1)
    {
      cout << "Lawnmower running" << endl;
      //smartRandomSearch(6.8,myId);
      lawnMowerSearch(-6.8,myId);
    }

    else if(myId == 2)
    {
      lawnMowerSearch(6.8,myId);
    }

    else if(myId == 3)
    {
      smartRandomSearch(-6.8,myId);
    }

  
    result.wpts.waypoints.clear();
    cout << "FW: ("<< searchLocation.x << "," << searchLocation.y << ")" << endl;
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);
  }
    return result;
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
  double number;
  bool xAccepted = false;
  bool yAccepted = false;
  bool shield = false;
  result.type = waypoint;

  cout << "My ID: " << myId << endl;
  cout << "Publishing visited point: (" << currentLocation.x << "," << currentLocation.y << ")" << endl;
  setVisitedPoint(currentLocation);
  setVisitedPointFlag(true);

/*
  if(visitedPoints.size() == 0)
  {
    cout << "-There are no Published Visited Locations-" << endl;
    //cout << "-Going to first X value-" << endl;
    //number = .1;
  }
  else{
    cout << "------------------------------------------" << endl;
     for(int i = 0; i <= visitedPoints.size() - 1; i++)
     {
       cout << "Location #" << i + 1 << "(" << visitedPoints.at(i).x << "," << visitedPoints.at(0).y << ")" << endl;
     }
    cout << "------------------------------------------" << endl;
  }
*/

if(first_waypoint)
{
  first_waypoint = false;
  cout << "Searching for first given location!!" << endl;
  searchLocation.theta = 0;
  searchLocation.x = -.1;
  searchLocation.y = -.1;
}
else{
  number = rng->uniformReal(0,-6.5);
  /*
  while(xAccepted == false)
  {


    for(int i = 0; i <= (this->visitedPoints.size() - 1); i++)
    {
      cout <<"Verifying numbers" << endl;
      if(abs(number - visitedPoints.at(i).x) <= .1)
      {
        shield = true;
        break;
      }
    }
    if(shield)
    {
      cout << "Generated X value rejected!!" << endl;
      shield = false;
      xAccepted = false;
    }
    else{
      cout << "Generated X value accepted" << endl;
      xAccepted = true;
      }
    }
    */
    searchLocation.x = number;
    /*
    shield = false;
    while(yAccepted == false)
    {
      number = rng->uniformReal(0,-6.5);
      for(int i = 0; i <= (this->visitedPoints.size() - 1); i++)
      {
        cout <<"Verifying numbers" << endl;
        if(abs(number - visitedPoints.at(i).y) <= .1)
        {
          shield = true;
          break;
        }
      }
      if(shield)
      {
        cout << "Generated Y value rejected!!" << endl;
        shield = false;
        yAccepted = false;
      }
      else{
        cout << "Generated Y value accepted" << endl;
        yAccepted = true;
      }
    }
    */
    number = rng->uniformReal(0,-6.5);
    searchLocation.y = number;
    cout << "Now looking for: (" << searchLocation.x << "," << searchLocation.y <<")" << endl;
  }
  //this->visitedPoints.push_back(searchLocation);
}

void SearchController::lawnMowerSearch(int wallLocation,int myId){
  //Right Side Robot will search cuadrant I and IV.
  //Left Side Robot will search cuadrant II and III.

  float miniWall; // imaginary wall to avoid colliding with Physical wall.
  float centerLine; //imaginary line to understand that we reached the Y axis.

  result.type = waypoint;
  cout << "My ID " << myId << endl;


  if(wallLocation < 0)//This if is to know the location of the wall. And set some variables for the behavior of the each robot.
  {
    cout << "Wall is Negative" << endl;
    miniWall = -.5;
    centerLine = 0.01;
  }
  else{
  cout << "Wall is Positive" << endl;
   miniWall = .5;
   centerLine = -0.01;
  }

  cout << "Publishing visited point: (" << currentLocation.x << "," << currentLocation.y << ")" << endl;
  setVisitedPoint(currentLocation);
  setVisitedPointFlag(true);

  cout << "-Looking for Published visited locations-" << endl;

/*
  if(visitedPoints.size() == 0)
  {
    cout << "-There are no Published Visited Locations-" << endl;
  }
  else{
    cout << "------------------------------------------" << endl;
     for(int i = 0; i <= visitedPoints.size() - 1; i++)
     {
       cout << "Location #" << i + 1 << "(" << visitedPoints.at(i).x << "," << visitedPoints.at(0).y << ")" << endl;
     }
    cout << "------------------------------------------" << endl;
   }
   */
// first location of each robot

  cout << "First WP: "<< first_waypoint << endl;

  if (first_waypoint)
  {
    first_waypoint = false;
    cout << "Searching for first given location!!" << endl;
    searchLocation.theta = 0;

    switch (myId) {
      case 1:
      {
        searchLocation.x = -1;
        searchLocation.y = (wallLocation/-1 - .5) ;
        break;
      }
      case 2:
      {
        searchLocation.x = 1;
        searchLocation.y = (wallLocation/-1 - .5); //Modify when whole map configuration is done.
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
      if((currentLocation.x <= (wallLocation - miniWall)) == true && leftAdjust == false && myId == 1)
      {
        //cout << "-Reached mini wall-" << endl;
        searchLocation.x = currentLocation.x;
        searchLocation.y = currentLocation.y - 1;
        cout << "Now looking for: (" << searchLocation.x << "," << searchLocation.y <<")" << endl;
        leftAdjust = true;
      }
      //if we reached the wall boundary and we haven't turned left, adjust left. Right side Robot
      else if((currentLocation.x >= (wallLocation - miniWall)) == true && leftAdjust == false && myId == 2)
      {
        //cout << "-Reached mini wall-" << endl;
        searchLocation.x = currentLocation.x;
        searchLocation.y = currentLocation.y + 1;
        cout << "Now looking for: (" << searchLocation.x << "," << searchLocation.y <<")" << endl;
        leftAdjust = true;
      }
      //if we reached the center axis and we havent turned right, adjust right. Left side Robot
      else if((currentLocation.x > centerLine == true) && rightAdjust == false && myId == 1)
      {
        //cout << "-Reached center line-" << endl;
        searchLocation.x = currentLocation.x;
        searchLocation.y = currentLocation.y - 1;
        cout << "Now looking for: (" << searchLocation.x << "," << searchLocation.y <<")" << endl;
        rightAdjust = true;
        leftAdjust = false;
      }
      //if we reached the center axis and we havent turned right, adjust right. Right side Robot
      else if((currentLocation.x < centerLine == true) && rightAdjust == false && myId == 2)
      {
        //cout << "-Reached center line-" << endl;
        searchLocation.x = currentLocation.x;
        searchLocation.y = currentLocation.y + 1;
        cout << "Now looking for: (" << searchLocation.x << "," << searchLocation.y <<")" << endl;
        rightAdjust = true;
        leftAdjust = false;
      }
      //if we turned left and no other condition have been met, keep moving toward center axis.
      else if(leftAdjust == true)
      {
        //cout << "-Walking towards center-" << endl;
        rightAdjust = false;
        searchLocation.y = currentLocation.y;

        if(miniWall < 0)//if Left Side Robot
        {
          searchLocation.x = currentLocation.x +.3;
        }
        else{ //if Right Side Robot
          searchLocation.x = currentLocation.x -.3;
        }
      }
      //if we are going outside the base searching.
      else{
        //cout << "-Walking towards wall-" << endl;
        if(miniWall < 0)//if Left Side Robot
        {
          searchLocation.x = currentLocation.x -.3;
        }
        else{ //if Right Side Robot
          searchLocation.x = currentLocation.x +.3;
        }
        searchLocation.y = currentLocation.y;
        cout << "Now looking for: (" << searchLocation.x << "," << searchLocation.y <<")" << endl;
      }
  }
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

//----------------------------------------------------------------- Fidelity Improvement (Jomar) -------------------------------------------------------------------------

void SearchController::FidelityImprovement(){ //Enters when detects a Tag

  if (getSdropped()){ // if drops the tag, go to the saved location

          cout << "S-1" << '\n';

          x_first_waypoint = false;
          x_front_pared = false;
          x_front_giro_pared = false;

          if (getDeleteVector()) { // enter if there is more than one element in the vector
            SavedPointsVector.pop_back(); // delete the last saved point visited
          }


          if(SavedPointsVector.size() == 1) { // if only one element, get out of the 'if' statement
            setDeleteVector(false);
            arrives = true;
            Sdropped = false;
          }

          else{
            setDeleteVector(true); // if not, enter again to the statement.
          }
          result.wpts.waypoints.clear();
          result.wpts.waypoints.insert(result.wpts.waypoints.begin(), SavedPointsVector[SavedPointsVector.size() - 1]);
          std::cout << "Dropped: Getting Back to: " << SavedPointsVector[SavedPointsVector.size() - 1].x << ", " << SavedPointsVector[SavedPointsVector.size() - 1].y << "\n\n";

  }


  else if (arrives){ // else if arrives to the saved location

          cout << "S-2" << '\n';
          SavedPointsVector.pop_back(); //delete the last element
          arrives = false;
          fidImprovement = false; // continue with the search algorithm

          //in wich part it had stayed when it detected the tag
          if (searchProcess == 0 || searchProcess == 2) {
              x_front_pared = true;
              x_front_giro_pared = false;
          }

          else if (searchProcess == 1) {
              x_front_giro_pared = true;
              x_front_pared = false;
          }

          result.wpts.waypoints.clear();
          result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocSaved);
          std::cout << "fromSavedtoWall: " << searchLocSaved.x << ", " << searchLocSaved.y << ", " << searchLocSaved.theta << "\n\n";

  }

}

void SearchController::aTagDetected(){
  // If detects do it only one time
  if (getTagDetectedCatched()){
      setDeleteVector(false);
      arrives = false;
      SavedPointsVector.push_back(currentLocation);


      std::cout << "Tag Detected!!!" << '\n';

      for (size_t j = 0; j != SavedPointsVector.size(); j++) {
        if (j == 0){
          std::cout << "\nPoints Saved Array: [\n(" << SavedPointsVector[j].x << ", " << SavedPointsVector[j].y << ", " << SavedPointsVector[j].theta << ")" << '\n';
        }
        else if (j == arrayCounter && j != 0) {
            std::cout << "(" << SavedPointsVector[j].x << ", " << SavedPointsVector[j].y << ", " << SavedPointsVector[j].theta << ")]" << "\n\n";
        }
        else{
          std::cout << "(" << SavedPointsVector[j].x << ", " << SavedPointsVector[j].y << ", " << SavedPointsVector[j].theta << ")" << '\n';
        }
      }

      arrayCounter = arrayCounter + 1;
      setTagDetectedCatched(false);
      }
}




void SearchController::droppedOFF(){
  // If dropped it, do it only one time
  if (!getTagDetectedCatched()){
    std::cout << "SearchController::droppedOFF()" << '\n';
    fidImprovement = true;

    Sdropped = true;
    setTagDetectedCatched(true);
  }
}


//----------------------------------------------------------------- Fidelity Improvement --------------------------------------------------------------------------------
