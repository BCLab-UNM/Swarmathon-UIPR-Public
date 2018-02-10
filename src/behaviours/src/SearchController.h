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

  // Setters
  void setSdropped(bool Sdropped) { this->Sdropped = Sdropped; }
  void setDeleteVector(bool deleteVector) { this->deleteVector = deleteVector; }
  void setCantSeeTargetDontRepeat(bool cantSeeTargetDontRepeat) { this->cantSeeTargetDontRepeat = cantSeeTargetDontRepeat; }
  void setTagDetectedCatched(bool TagDetectedCatched){ this->TagDetectedCatched = TagDetectedCatched; }


  // --------------------------------------------------------------- // Jomar --------------------------------------------------------------------------


protected:

  void ProcessData();

private:

  Point currentLocation;
  Point centerLocation;
  Point searchLocation;


  //struct for returning data to ROS adapter
  Result result;


  // Search state
  // Flag to allow special behaviour for the first waypoint

  // --------------------------------------------------------------- // Jomar --------------------------------------------------------------------------

  Point searchLocSaved;
  vector<Point> SavedPointsVector;

  int arrayCounter = 0;
  int searchProcess = 0;

  bool succesfullPickup = false;
  bool x_first_waypoint = true;
  bool x_front_pared = false;
  bool x_front_giro_pared = false;
  bool x_front_pared_finish = false;


  bool Sdropped = false;
  bool cantSeeTargetDontRepeat = false;
  bool arrives = false;
  bool deleteVector = false;
  bool fidImprovement = false;
  bool TagDetectedCatched = true;

// --------------------------------------------------------------- // Jomar --------------------------------------------------------------------------


};

#endif /* SEARCH_CONTROLLER */
