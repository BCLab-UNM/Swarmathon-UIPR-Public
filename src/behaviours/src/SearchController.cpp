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
}

void SearchController::Reset() {
  result.reset = false;
}

/**
 * This code implements a basic random walk search.
 */
Result SearchController::DoWork() {
    result.type = waypoint;
    Point searchLocation;

  // if (x_first_waypoint) {
  //   cout << "SearchController::DoWork()" << "\n\n";
  //   result.type = waypoint;
  //   Point searchLocation;
  //   //cout << "Sdropped = " << Sdropped << "\n\n";
  //
  //
  //   searchLocation.x = -2;
  //   searchLocation.y = 0;
  //   result.wpts.waypoints.clear();
  //   result.wpts.waypoints.push_back(searchLocation);
  //   x_first_waypoint = true;
  // }
  //
  // else{
  //   result.type = behavior;
  //   result.b = wait;
  // }




  if (fidImprovement) {
    FidelityImprovement();
  }

  else{

  if (x_first_waypoint) {
        cout << "S-4" << '\n';
        searchProcess = 0; //stast
        x_first_waypoint = false;
        searchLocation.x = 0;
        searchLocation.y = 5;
        searchLocation.theta = hypot(searchLocation.x - currentLocation.x, searchLocation.y - currentLocation.y);
        x_front_pared = true;
        std::cout << "Starting Point: " << searchLocation.x << ", " << searchLocation.y << ", " << searchLocation.theta << "\n\n";
        result.wpts.waypoints.clear();
        result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);
        searchLocSaved = searchLocation;
  }


  else {
        cout << "Entro al else" << '\n';

        if (x_front_pared){
          cout << "S-5" << '\n';
          searchProcess = 1;
          x_front_pared = false;
          searchLocation.x = currentLocation.x;
          searchLocation.y = -currentLocation.y;
          searchLocation.theta = hypot(searchLocation.x - currentLocation.x, searchLocation.y - currentLocation.y);
          x_front_giro_pared = true;
          std::cout << "Hacia la pared: " << searchLocation.x << ", " << searchLocation.y << ", " << searchLocation.theta << "\n\n";
          searchLocSaved = searchLocation;
          result.wpts.waypoints.clear();
          result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);
        }

        else if (x_front_giro_pared){
          cout << "S-6" << '\n';
          searchProcess = 2;
          x_front_giro_pared = false;
          searchLocation.x = currentLocation.x - 0.5;
          searchLocation.y = currentLocation.y;
          searchLocation.theta = hypot(searchLocation.x - currentLocation.x, searchLocation.y - currentLocation.y);
          x_front_pared = true;
          std::cout << "Girando: " << searchLocation.x << ", " << searchLocation.y << ", " << searchLocation.theta << "\n\n";
          searchLocSaved = searchLocation;
          result.wpts.waypoints.clear();
          result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);
        }

  }

  }

    cout << "TagDetectedCatched = " << getTagDetectedCatched() << "\n\n";
    return result;

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
