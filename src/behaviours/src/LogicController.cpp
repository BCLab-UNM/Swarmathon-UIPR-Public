#include "LogicController.h"

LogicController::LogicController()
{

  logicState = LOGIC_STATE_INTERRUPT;
  processState = PROCCESS_STATE_SEARCHING;

  ProcessData();

  control_queue = priority_queue<PrioritizedController>();
}

LogicController::~LogicController() {}

void LogicController::Reset()
{
  logicState = LOGIC_STATE_INTERRUPT;
  processState = PROCCESS_STATE_SEARCHING;

  ProcessData();

  control_queue = priority_queue<PrioritizedController>();
}

//******************************************************************************
// This function is called every 1/10th of a second by the ROSAdapter
// The logical flow if the behaviours is controlled here by using an interrupt,
// haswork, and priority queue system.
Result LogicController::DoWork()
{
  Result result;

  //first a loop runs through all the controllers who have a priority of 0 or above witht he largest number being
  //most important. A priority of less than 0 is an ignored controller use -1 for standards sake.
  //if any controller needs and interrupt the logic state is changed to interrupt
  for (PrioritizedController cntrlr : prioritizedControllers)
  {
    if (cntrlr.controller->ShouldInterrupt() && cntrlr.priority >= 0)
    {
      logicState = LOGIC_STATE_INTERRUPT;
      //do not break all shouldInterupts may need calling in order to properly pre-proccess data.
    }
  }

  //logic state switch
  switch (logicState)
  {

  //when an interrupt has been thorwn or there are no pending control_queue.top().actions logic controller is in this state.
  case LOGIC_STATE_INTERRUPT:
  {
    //Reset the control queue
    control_queue = priority_queue<PrioritizedController>();

    //check what controllers have work to do all that say yes will be added to the priority queue.
    for (PrioritizedController cntrlr : prioritizedControllers)
    {
      if (cntrlr.controller->HasWork())
      {
        if (cntrlr.priority < 0)
        {
          continue;
        }
        else
        {
          control_queue.push(cntrlr);
        }
      }
    }

    //if no controlers have work report this to ROS Adapter and do nothing.
    if (control_queue.empty())
    {
      result.type = behavior;
      result.b = wait;
      break;
    }
    else
    {
      //default result state if someone has work this safe gaurds against faulty result types
      result.b = noChange;
    }

    //take the top member of the priority queue and run their do work function.

    result = control_queue.top().controller->DoWork();

    //anaylyze the result that was returned and do state changes accordingly
    //behavior types are used to indicate behavior changes of some form
    if (result.type == behavior)
    {

      //ask for an external reset so the state of the controller is preserved untill after it has returned a result and
      //gotten a chance to communicate with other controllers
      if (result.reset)
      {
        controllerInterconnect(); //allow controller to communicate state data before it is reset
        control_queue.top().controller->Reset();
      }

      //ask for the procces state to change to the next state or loop around to the begining
      if (result.b == nextProcess)
      {
        if (processState == _LAST - 1)
        {
          processState = _FIRST;
          searchController.setTagDetectedCatched(true);  // Jomar
          pickUpController.setDontRepeatSeeTarget(true); // Jomar
        }
        else
        {
          processState = (ProcessState)((int)processState + 1);
        }
      }

      //ask for the procces state to change to the previouse state or loop around to the end
      else if (result.b == prevProcess)
      {
        if (processState == _FIRST)
        {
          processState = (ProcessState)((int)_LAST - 1);
        }
        else
        {
          processState = (ProcessState)((int)processState - 1);
        }
      }

      //update the priorites of the controllers based upon the new process state.
      if (result.b == nextProcess || result.b == prevProcess)
      {
        ProcessData();
        result.b = wait;
        driveController.Reset(); // It is assumed that the drive controller may
                                 // be in a bad state if interrupted, so reset it.
      }
      break;
    }

    //precision driving result types are when a controller wants direct command of the robots actuators
    //logic controller facilitates the command pass through in the LOGIC_STATE_PRECISION_COMMAND switch case
    else if (result.type == precisionDriving)
    {

      logicState = LOGIC_STATE_PRECISION_COMMAND;
      break;
    }

    //waypoints are also a pass through facilitated command but with a slightly diffrent overhead
    //they are handled in the LOGIC_STATE_WAITING switch case
    else if (result.type == waypoint)
    {

      logicState = LOGIC_STATE_WAITING;
      driveController.SetResultData(result);
      // Fall through on purpose to "case LOGIC_STATE_WAITING:"
    }

  }
  // ***************************************************************************
  // END LOGIC_STATE_INTERUPT
  // ***************************************************************************

  // ***************************************************************************
  // BEGIN LOGIC_STATE_WAITING
  // ***************************************************************************

    //this case is primarly when logic controller is waiting for drive controller to reach its last waypoint
  case LOGIC_STATE_WAITING:
  {
    //ask drive controller how to drive
    //commands to be passed the ROS Adapter as left and right wheel PWM values in the result struct are returned
    result = driveController.DoWork();

    //when out of waypoints drive controller will through an interrupt however unlike other controllers
    //drive controller is not on the priority queue so it must be checked here
    if (result.type == behavior)
    {
      if (driveController.ShouldInterrupt())
      {
        logicState = LOGIC_STATE_INTERRUPT;
      }
    }
    break;
  } //end of waiting case*****************************************************************************************

    //used for precision driving pass through
  case LOGIC_STATE_PRECISION_COMMAND:
  {

    // Unlike waypoints, precision commands change every update tick, so we ask
    // the controller for new commands on every update tick.
    result = control_queue.top().controller->DoWork();

    // Pass the driving commands to the drive controller so it can interpret them.
    driveController.SetResultData(result);

    // The interpreted commands are turned into proper initial_spiral_offset
    // motor commands to be passed the ROS Adapter such as left and right wheel
    // PWM values in the result struct.
    result = driveController.DoWork();
    break;

  } //end of precision case****************************************************************************************
  } //end switch statment******************************************************************************************

  // bad! causes node to crash

  // Allow the controllers to communicate data between each other,
  // depending on the processState.
  controllerInterconnect();

  // Give the ROSAdapter the final decision on how it should drive.
  return result;
}

void LogicController::UpdateData()
{
  // EDIT
  if (init)
  {
    cout << "My Id Loc:" << myIdLoc << endl;
    searchController.getID(myIdLoc);
    init = false;
  }
  searchController.setClusterLocation(clusterPoints);
  searchController.getTotalIds(totalIds);
  searchController.setVisitedVector(visitedPoints);
  searchController.setVisitedPointFlag(publishVisitedPointFlag);

  //
}

void LogicController::ProcessData()
{

  //this controller priority is used when searching
  if (processState == PROCCESS_STATE_SEARCHING)
  {
    prioritizedControllers = {
        PrioritizedController{0, (Controller *)(&searchController)},
        PrioritizedController{10, (Controller *)(&obstacleController)},
        PrioritizedController{15, (Controller *)(&pickUpController)},
        PrioritizedController{5, (Controller *)(&range_controller)},
        PrioritizedController{-1, (Controller *)(&dropOffController)},
        PrioritizedController{-1, (Controller *)(&manualWaypointController)}};
  }

  //this priority is used when returning a target to the center collection zone
  else if (processState == PROCCESS_STATE_TARGET_PICKEDUP)
  {
    prioritizedControllers = {
        PrioritizedController{-1, (Controller *)(&searchController)},
        PrioritizedController{15, (Controller *)(&obstacleController)},
        PrioritizedController{-1, (Controller *)(&pickUpController)},
        PrioritizedController{10, (Controller *)(&range_controller)},
        PrioritizedController{1, (Controller *)(&dropOffController)},
        PrioritizedController{-1, (Controller *)(&manualWaypointController)}};
  }
  //this priority is used when returning a target to the center collection zone
  else if (processState == PROCCESS_STATE_DROP_OFF)
  {
    prioritizedControllers = {
        PrioritizedController{-1, (Controller *)(&searchController)},
        PrioritizedController{-1, (Controller *)(&obstacleController)},
        PrioritizedController{-1, (Controller *)(&pickUpController)},
        PrioritizedController{10, (Controller *)(&range_controller)},
        PrioritizedController{1, (Controller *)(&dropOffController)},
        PrioritizedController{-1, (Controller *)(&manualWaypointController)}};
  }
  else if (processState == PROCESS_STATE_MANUAL)
  {
    // under manual control only the manual waypoint controller is active
    prioritizedControllers = {
        PrioritizedController{-1, (Controller *)(&searchController)},
        PrioritizedController{-1, (Controller *)(&obstacleController)},
        PrioritizedController{-1, (Controller *)(&pickUpController)},
        PrioritizedController{-1, (Controller *)(&range_controller)},
        PrioritizedController{-1, (Controller *)(&dropOffController)},
        PrioritizedController{5, (Controller *)(&manualWaypointController)}};
  }
}

bool LogicController::ShouldInterrupt()
{
  ProcessData();

  // The logic controller is the top level controller and will never have to
  // interrupt. It is only the lower level controllers that may need to interupt.
  return false;
}

bool LogicController::HasWork()
{
  // The LogicController class is a special case. It will never have work to
  // do because it is always handling the work of the other controllers.
  return false;
}


// This function will deal with inter-controller communication. Communication
// that needs to occur between specific low level controllers is done here.
//
// The type of communication may or may not depend on the processState.
//
//                       /<----> ControllerA
// LogicController <---->|                  \__ inter-controller communication
//                       |                  /
//                       \<----> ControllerB
void LogicController::controllerInterconnect()
{
  searchController.setNeedNewPoint(obstacleController.needNewPoint()); //Hector added
  obstacleController.setDroppedOff(dropOffController.getDroppedOff()); //Hector added
  obstacleController.setTagDetected(pickUpController.TagDetected());   //Hector added
  // EDIT
  if (searchController.getVisitedFlag())
  {
    publishVisitedPointFlag = searchController.getVisitedFlag();
    latestVisitedPoint = searchController.getVisitedPoint();
  }

  if (searchController.getClusterAssigned())
  {
    // Erase first cluster location and allow Fieldity to take care of it
    clusterPoints.erase(clusterPoints.begin());
    // Reset flag so if robot detects new cluster it can later erase the point
    searchController.setClusterAssigned(false);
  }
  //

  if (processState == PROCCESS_STATE_SEARCHING)
  {
    //obstacle needs to know if the center ultrasound should be ignored
    if (pickUpController.GetIgnoreCenter())
    {
      obstacleController.setIgnoreCenterSonar();
    }

    //pickup controller annouces it has pickedup a target
    if (dropOffController.getDroppedOff()){ //Hector Changed

    if(!pickUpController.GetTargetHeld())
    {
      obstacleController.setTargetHeldClear();
    }
    else{
      dropOffController.SetTargetPickedUp();
      obstacleController.setTargetHeld();
    }
      
    }

    if (pickUpController.TagDetected())
    {
      searchController.aTagDetected();
    }

    if (pickUpController.getCantSeeTargetDontRepeat())
    {
      searchController.setCantSeeTargetDontRepeat(true);
    }

    else
    {
      searchController.setCantSeeTargetDontRepeat(false);
    }

    if (searchController.getCantSeeTargetDontRepeat())
    {
      pickUpController.setCantSeeTargetDontRepeat(true);
    }

    else
    {
      pickUpController.setCantSeeTargetDontRepeat(false);
    }

  }

  if (processState == PROCCESS_STATE_DROP_OFF)
  {
    if (dropOffController.NotHasTag())
    {
      searchController.droppedOFF();
    }
  }

  //ask if drop off has released the target from the claws yet
  if (!dropOffController.HasTarget())
  {

    obstacleController.setTargetHeldClear();
  }

  //obstacle controller is running driveController needs to clear its waypoints
  if (obstacleController.getShouldClearWaypoints())
  {
    driveController.Reset();
  }
}

// Recieves position in the world inertial frame (should rename to SetOdomPositionData)
void LogicController::SetPositionData(Point currentLocation)
{
  searchController.SetCurrentLocation(currentLocation);
  dropOffController.SetCurrentLocation(currentLocation);
  obstacleController.setCurrentLocation(currentLocation);
  driveController.SetCurrentLocation(currentLocation);
  manualWaypointController.SetCurrentLocation(currentLocation);
}

// Recieves position in the world frame with global data (GPS)
void LogicController::SetMapPositionData(Point currentLocation)
{
  range_controller.setCurrentLocation(currentLocation);
}

void LogicController::SetVelocityData(float linearVelocity, float angularVelocity)
{
  driveController.SetVelocityData(linearVelocity, angularVelocity);
}

void LogicController::SetMapVelocityData(float linearVelocity, float angularVelocity)
{
}

void LogicController::SetAprilTags(vector<Tag> tags)
{
  pickUpController.SetTagData(tags);
  obstacleController.setTagData(tags);
  dropOffController.SetTargetData(tags);
}

void LogicController::SetSonarData(float left, float center, float right)
{

  pickUpController.SetSonarData(center);

  obstacleController.setSonarData(left, center, right); //change from (lefts,center,right) to (lefta,centera,righta)
}

// Called once by RosAdapter in guarded init
void LogicController::SetCenterLocationOdom(Point centerLocationOdom)
{

  centerAvg.x += centerLocationOdom.x;
  centerAvg.y += centerLocationOdom.y;

  centerCounter++;
  if (centerCounter == 30)
  {
    centerAvg.x = centerAvg.x / 30;
    centerAvg.y = centerAvg.y / 30;

    searchController.SetCenterLocation(centerAvg);
    dropOffController.SetCenterLocation(centerAvg);

    centerAvg.x = 0;
    centerAvg.y = 0;
    centerCounter = 0;
  }
}

void LogicController::AddManualWaypoint(Point manualWaypoint, int waypoint_id)
{
  manualWaypointController.AddManualWaypoint(manualWaypoint, waypoint_id);
}

void LogicController::RemoveManualWaypoint(int waypoint_id)
{
  manualWaypointController.RemoveManualWaypoint(waypoint_id);
}

std::vector<int> LogicController::GetClearedWaypoints()
{
  return manualWaypointController.ReachedWaypoints();
}

void LogicController::setVirtualFenceOn(RangeShape *range)
{
  range_controller.setRangeShape(range);
  range_controller.setEnabled(true);
}

void LogicController::setVirtualFenceOff()
{
  range_controller.setEnabled(false);
}

void LogicController::SetCenterLocationMap(Point centerLocationMap)
{
}

void LogicController::SetCurrentTimeInMilliSecs(long int time)
{
  current_time = time;
  dropOffController.SetCurrentTimeInMilliSecs(time);
  pickUpController.SetCurrentTimeInMilliSecs(time);
  obstacleController.setCurrentTimeInMilliSecs(time);
}

void LogicController::SetModeAuto()
{
  if (processState == PROCESS_STATE_MANUAL)
  {
    // only do something if we are in manual mode
    this->Reset();
    manualWaypointController.Reset();
  }
}
void LogicController::SetModeManual()
{
  if (processState != PROCESS_STATE_MANUAL)
  {
    logicState = LOGIC_STATE_INTERRUPT;
    processState = PROCESS_STATE_MANUAL;
    ProcessData();
    control_queue = priority_queue<PrioritizedController>();
    driveController.Reset();
  }
}
