#include "ObstacleController.h"
#include <cmath>
#include <iostream>

ObstacleController::ObstacleController()
{
  obstacleAvoided = true;
  obstacleDetected = false;
  obstacleInterrupt = false;
  result.PIDMode = CONST_PID; //use the const PID to turn at a constant speed
  direction = 0;
}

//note, not a full reset as this could cause a bad state
//resets the interupt and knowledge of an obstacle or obstacle avoidance only.
void ObstacleController::Reset() {
  obstacleAvoided = true;
  obstacleDetected = false;
  obstacleInterrupt = false;
  delay = current_time;
  direction = 0;
}

bool ObstacleController::getObstacleInfo()
{
  return obstacleDetected;
}

int ObstacleController::getDirection(){
  if (left < right){
    direction = -1;
  }
   else {
     direction = 1;
   } 
  return direction;
}

// Avoid crashing into objects detected by the ultraound
// void ObstacleController::avoidObstacle() {

//     //always turn right to avoid obstacles
//     if (right < 0.8 || center < 0.8 || left < 0.8) {
//       cout << "Pared encontrada en (" << currentLocation.x << "," << currentLocation.y << ")";
//       obstacleDetected = true;
//       result.type = precisionDriving;

//       result.pd.cmdAngular = -K_angular;

//       result.pd.setPointVel = 0.0;
//       result.pd.cmdVel = 0.0;
//       result.pd.setPointYaw = 0;
//     }
// }

// void ObstacleController::follow_Wall() {
//     //cout << "Current - " << "x: " <<currentLocation.x << " y: " << currentLocation.y << " theta: " << currentLocation.theta << endl;
    
//     //Add distances read to vector so min and max index can be calculated.
//     distRead.push_back(left);
//     distRead.push_back(center);
//     distRead.push_back(right);
//     //cout << "Added to list: " << distRead[0] << " " << distRead[1] << " " << distRead[2] << endl;
  
//     // Calculate min index
//     int size = distRead.size();
//     int direction = getDirection();
  
//     minIndex = size*(direction+1)/4;
//     maxIndex = size*(direction+3)/4;

//     for (int i = minIndex; i < maxIndex; i++){
//       if (distRead[i] < distRead[i+1] && distRead[i] > 0.1){
//         minIndex = i + 1;
//       }
//     }

//     cout <<"minIndex is = " << minIndex << endl;

//     angleMin = (minIndex - size/2)*M_PI/8;
//     distMin = distRead[minIndex];
//     diffE = (distMin - triggerDistance) - e;
//     e = distMin - triggerDistance;

//     cout <<"angleMin is = " << angleMin << endl;

//     result.type = precisionDriving;
//     result.pd.cmdAngular = direction*(10*e + 5*diffE) + K_angular * (angleMin - M_PI * direction/2); //PD controller
//     //cout <<"direction is = " << direction << endl;
//     //cout << "My Angular Vel is: " << result.pd.cmdAngular << endl;

//     if (right < triggerDistance || center < triggerDistance || left < triggerDistance){
//       result.pd.cmdVel = 0.0;
//       //result.pd.setPointYaw = currentLocation.theta;
//       //result.pd.cmdAngular = ;
//       //distRead.clear();
//     }
//     else if (right < triggerDistance * 2 || center < triggerDistance * 2 || left < triggerDistance * 2){
//       result.pd.cmdVel = 0.5 * 255;
//       result.pd.cmdAngular = direction*(10*e + 5*diffE) + K_angular * (angleMin - M_PI * direction/2);;
//       //result.pd.setPointYaw = currentLocation.theta;
//       cout << "Found Obstacle!, my Vel is: " << result.pd.cmdVel <<  " my anglar vel is: " << result.pd.cmdAngular << " my current heading is: " << 
//            currentLocation.theta << endl;
//       //distRead.clear();
//     }
//     else if (fabs(angleMin) > 1.57){
//        result.pd.cmdVel = 0.4 * 255;
//          result.pd.cmdAngular = direction*(10*e + 5*diffE) + K_angular * (angleMin - M_PI * direction/2);;
//        //result.pd.setPointYaw = currentLocation.theta;
//        cout << "Angle min case!" << endl;
//        //distRead.clear();
//      }
//     else if (right > triggerDistance && center > triggerDistance && left > triggerDistance){
//       result.pd.cmdAngular = 0.0;
//       result.pd.setPointVel = 0.0;
//       result.pd.cmdVel = 0.0;
//       result.pd.setPointYaw = 0;
//       distRead.clear();
//       cout << "No obstacle detected!" << endl;
//      }
      
// }
// void ObstacleController::follow_Wall() {
//     //cout << "Current - " << "x: " <<currentLocation.x << " y: " << currentLocation.y << " theta: " << currentLocation.theta << endl;
    
//     //Add distances read to vector so min and max index can be calculated.
//     distRead.push_back(left);
//     distRead.push_back(center);
//     distRead.push_back(right);
//     //cout << "Added to list: " << distRead[0] << " " << distRead[1] << " " << distRead[2] << endl;
  
//     // Calculate min index
//     int size = distRead.size();
//     int direction = getDirection();
  
//     minIndex = size*(direction+1)/4;
//     maxIndex = size*(direction+3)/4;

//     for (int i = minIndex; i < maxIndex; i++){
//       if (distRead[i] < distRead[i+1] && distRead[i] > 0.1){
//         minIndex = i + 1;
//       }
//     }

//     cout <<"minIndex is = " << minIndex << endl;

//     angleMin = (minIndex - size/2)*M_PI/8;
//     distMin = distRead[minIndex];
//     diffE = (distMin - triggerDistance) - e;
//     e = distMin - triggerDistance;
    
//     distRead.clear();
//     cout <<"angleMin is = " << angleMin << endl;

//     result.type = precisionDriving;
//     result.pd.cmdVel = 0.0;
//     result.pd.cmdAngular = direction*(10*e + 5*diffE) + K_angular * (angleMin - M_PI * direction/2); //PD controller

//     if (right < triggerDistance || center < triggerDistance || left < triggerDistance){
//       result.pd.cmdVel = 0.0;
//     }
//     else if (right < triggerDistance * 2 || center < triggerDistance * 2 || left < triggerDistance * 2){
//       result.pd.cmdVel = 0.5 * 255;
//       //result.pd.cmdAngular = 0.0;
//       cout << "Found Obstacle!, my Vel is: " << result.pd.cmdVel  << " my current heading is: " << currentLocation.theta << endl;
//     }
//     else if (fabs(angleMin) > 1.57){
//        result.pd.cmdVel = 0.4 * 255;
//        //result.pd.cmdAngular = 0.0;
//        cout << "Angle min case!" << endl;
//      }
//     else if (right > triggerDistance && center > triggerDistance && left > triggerDistance){
//       result.pd.cmdAngular = 0.0;
//       result.pd.setPointVel = 0.0;
//       result.pd.cmdVel = 255;
//       result.pd.setPointYaw = 0;
//       cout << "No obstacle detected!" << endl;
//      }
// }

void ObstacleController::follow_Wall() {
    //cout << "Current - " << "x: " <<currentLocation.x << " y: " << currentLocation.y << " theta: " << currentLocation.theta << endl;
    
    //Add distances read to vector so min and max index can be calculated.
    distRead.push_back(left);
    distRead.push_back(center);
    distRead.push_back(right);
    //cout << "Added to list: " << distRead[0] << " " << distRead[1] << " " << distRead[2] << endl;
  
    // Calculate min index
    int size = distRead.size();
    int direction = getDirection();
  
    minIndex = size*(direction+1)/4;
    maxIndex = size*(direction+3)/4;

    for (int i = minIndex; i < maxIndex; i++){
      if (distRead[i] < distRead[i+1] && distRead[i] > 0.1){
        minIndex = i + 1;
      }
    }

    cout <<"minIndex is = " << minIndex << endl;

    angleMin = (minIndex - size/2)*M_PI/8;
    distMin = distRead[minIndex];
    diffE = (distMin - triggerDistance) - e;
    e = distMin - triggerDistance;
    
    distRead.clear();
    cout <<"angleMin is = " << angleMin << endl;

    result.type = precisionDriving;
    result.pd.cmdVel = 0.0;
    result.pd.cmdAngular = direction*(10*e + 5*diffE) + K_angular * (angleMin - M_PI * direction/2); //PD controller

    if (right < triggerDistance || center < triggerDistance || left < triggerDistance){
      result.pd.cmdVel = 0.0;
    }
    else if (right < triggerDistance * 2 || center < triggerDistance * 2 || left < triggerDistance * 2){
      result.pd.cmdVel = 0.5 * 255;
      result.pd.cmdAngular = 0.0;
      cout << "Found Obstacle!, my Vel is: " << result.pd.cmdVel  << " my current heading is: " << currentLocation.theta << endl;
    }
    else if (fabs(angleMin) > 1.57){
       result.pd.cmdVel = 0.4 * 255;
       result.pd.cmdAngular = 0.0;
       cout << "Angle min case!" << endl;
     }
    else if (right > triggerDistance && center > triggerDistance && left > triggerDistance){
      result.pd.cmdAngular = 0.0;
      result.pd.setPointVel = 0.0;
      result.pd.cmdVel = 0.0;
      result.pd.setPointYaw = 0;
      cout << "No obstacle detected!" << endl;
     }
}
// A collection zone was seen in front of the rover and we are not carrying a target
// so avoid running over the collection zone and possibly pushing cubes out.
void ObstacleController::avoidCollectionZone() {

    cout << "Base detected" << endl;
    result.type = precisionDriving;

    result.pd.cmdVel = 0.0;

    // Decide which side of the rover sees the most april tags and turn away
    // from that side
    if(count_left_collection_zone_tags < count_right_collection_zone_tags) {
      result.pd.cmdAngular = K_angular;
    } else {
      result.pd.cmdAngular = -K_angular;
    }

    result.pd.setPointVel = 0.0;
    result.pd.cmdVel = 0.0;
    result.pd.setPointYaw = 0;
}


Result ObstacleController::DoWork() {

  clearWaypoints = true;
  set_waypoint = true;
  result.PIDMode = CONST_PID;
  // The obstacle is an april tag marking the collection zone
  if(collection_zone_seen){
    avoidCollectionZone();
  }
  else {
    follow_Wall();
  }

  //if an obstacle has been avoided
  if (can_set_waypoint) {
    cout << "Im inside new waypoint if!"  << endl;
    can_set_waypoint = false; //only one waypoint is set
    set_waypoint = false;
    clearWaypoints = false;
    // result.type = waypoint;

    // result.PIDMode = FAST_PID; //use fast pid for waypoints
    // Point forward; 
    // //waypoint is directly ahead of current heading
    // forward.x = currentLocation.x + (0.5 * cos(currentLocation.theta));
    // forward.y = currentLocation.y + (0.5 * sin(currentLocation.theta));  
    // result.wpts.waypoints.clear();
    // result.wpts.waypoints.push_back(forward);
  }
  return result;
}


void ObstacleController::setSonarData(float sonarleft, float sonarcenter, float sonarright) {
  left = sonarleft;
  right = sonarright;
  center = sonarcenter;

  ProcessData();
}

void ObstacleController::setCurrentLocation(Point currentLocation) {
  this->currentLocation = currentLocation;
}

void ObstacleController::ProcessData() {

  //timeout timer for no tag messages
  //this is used to set collection zone seen to false beacuse
  //there is no report of 0 tags seen
  long int Tdifference = current_time - timeSinceTags;
  float Td = Tdifference/1e3;
  if (Td >= 0.5) {
    collection_zone_seen = false;
    phys= false;
    if (!obstacleAvoided)
    {
      can_set_waypoint = true;
    }
  }

  //If we are ignoring the center sonar
  if(ignore_center_sonar){
    //If the center distance is longer than the reactivation threshold
    if(center > reactivate_center_sonar_threshold){
      //currently do not re-enable the center sonar instead ignore it till the block is dropped off
      //ignore_center_sonar = false; //look at sonar again beacuse center ultrasound has gone long
    }
    else{
      //set the center distance to "max" to simulated no obstacle
      center = 3;
    }
  }
  else {
    //this code is to protect against a held block causing a false short distance
    //currently pointless due to above code
    if (center < 3.0) {
      result.wristAngle = 0.7;
    }
    else {
      result.wristAngle = -1;
    }
  }

  //if any sonar is below the trigger distance set physical obstacle true
  if (left < triggerDistance || right < triggerDistance || center < triggerDistance)
  {
    phys = true;
    timeSinceTags = current_time;
  }

  //if physical obstacle or collection zone visible
  if (collection_zone_seen || phys)
  {
    obstacleDetected = true;
    obstacleAvoided = false;
    can_set_waypoint = false;
  }
  else
  {
    obstacleAvoided = true;
  }
}

// Report April tags seen by the rovers camera so it can avoid
// the collection zone
// Added relative pose information so we know whether the
// top of the AprilTag is pointing towards the rover or away.
// If the top of the tags are away from the rover then treat them as obstacles.
void ObstacleController::setTagData(vector<Tag> tags){
  collection_zone_seen = false;
  count_left_collection_zone_tags = 0;
  count_right_collection_zone_tags = 0;

  // this loop is to get the number of center tags
  if (!targetHeld) {
    for (int i = 0; i < tags.size(); i++) { //redundant for loop
      if (tags[i].getID() == 256) {

	collection_zone_seen = checkForCollectionZoneTags( tags );
        timeSinceTags = current_time;
      }
    }
  }
}

bool ObstacleController::checkForCollectionZoneTags( vector<Tag> tags ) {

  for ( auto & tag : tags ) {

    // Check the orientation of the tag. If we are outside the collection zone the yaw will be positive so treat the collection zone as an obstacle.
    //If the yaw is negative the robot is inside the collection zone and the boundary should not be treated as an obstacle.
    //This allows the robot to leave the collection zone after dropping off a target.
    if ( tag.calcYaw() > 0 )
      {
	// checks if tag is on the right or left side of the image
	if (tag.getPositionX() + camera_offset_correction > 0) {
	  count_right_collection_zone_tags++;

	} else {
	  count_left_collection_zone_tags++;
	}
      }

  }


  // Did any tags indicate that the robot is inside the collection zone?
  return count_left_collection_zone_tags + count_right_collection_zone_tags > 0;

}

//obstacle controller should inrerupt is based upon the transition from not seeing and obstacle to seeing an obstacle
bool ObstacleController::ShouldInterrupt() {

  //if we see and obstacle and havent thrown an interrupt yet
  if(obstacleDetected && !obstacleInterrupt)
  {
    obstacleInterrupt = true;
    return true;
  }
  else
  {
    //if the obstacle has been avoided and we had previously detected one interrupt to change to waypoints
    if(obstacleAvoided && obstacleDetected)
    {
      Reset();
      return true;
    } else {
      return false;
    }
  }
}

bool ObstacleController::HasWork() {
  //there is work if a waypoint needs to be set or the obstacle hasnt been avoided
  if (can_set_waypoint && set_waypoint)
  {
    return true;
  }

  return !obstacleAvoided;
}

//ignore center ultrasound
void ObstacleController::setIgnoreCenterSonar(){
  ignore_center_sonar = true;
}

void ObstacleController::setCurrentTimeInMilliSecs( long int time )
{
  current_time = time;
}

void ObstacleController::setTargetHeld() {
  targetHeld = true;

  //adjust current state on transition from no cube held to cube held
  if (previousTargetState == false) {
    obstacleAvoided = true;
    obstacleInterrupt = false;
    obstacleDetected = false;
    previousTargetState = true;
  }
}

void ObstacleController::setTargetHeldClear()
{
  //adjust current state on transition from cube held to cube not held
  if (targetHeld)
  {
    Reset();
    targetHeld = false;
    previousTargetState = false;
    ignore_center_sonar = false;
  }
}
