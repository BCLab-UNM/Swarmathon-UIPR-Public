

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

  searchObstacle = false;
  leftAdjust = false;
  rightAdjust = false;
  sideSel = 1;
  pointCounter =0;
  magnitude = 0;
  angle = 0;
  unknownAngle = 0;
  triangleSel = 1;
  first_side_waypoint = true;
}

void SearchController::Reset() {
  result.reset = false;
}

Result SearchController::DoWork() {


    cout << "Total IDs " << totalIds << endl;
    if(totalIds <=3)
    {
      triangleSquare = mapSize/2 - 2.5; // Triangle square in semi should be 5mts. 
    }

    else
    {
      triangleSquare = mapSize/2 - 2.5;
      //Put here the size of the triangle square boundarie. 
    }

    if(myId == 1)
    {
      /*
      if(pointCounter == 9)
      {
        pointCounter = 0;
        sideSel++;
      }
      else{
        pointCounter++;
      }
      sideSearch(myId,sideSel,triangleSquare);
      */
      //Before of deciding where to search check the public list of triangles to know their state. 
        if(pointCounter == 8)
        {
          pointCounter = 0;
          //Look for triangle availability. 
          cout << "Limited Point search reached!!" << endl;
          triangleSel++;
        }
        else{
          pointCounter++;
        }
        triangleSearch(myId,triangleSel,triangleSquare);
    }

    else if(myId == 2)
    {
      if(pointCounter == 8)
        {
          pointCounter = 0;
          //Look for triangle availability. 
          cout << "Limited Point search reached!!" << endl;
          triangleSel++;
        }
        else{
            pointCounter++;
        }
        triangleSearch(myId,triangleSel,triangleSquare);
    }

    else if(myId == 3)
    
    {
      sideSearch(myId,3,triangleSquare);
    }
    else if(myId == 4)
    {
      sideSearch(myId,1,triangleSquare);
    }
    result.wpts.waypoints.clear();
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);

    return result;
  }

void SearchController::sideSearch(int myId,int sideSection,float triangleSquare)
{
  result.type = waypoint;
  float sideBoundary = mapSize/2 - triangleSquare; //Variable to set maximum size of zig zag search. 
  switch(sideSection)
  {
    case 1:
    {
      cout << "Looking for side section # 1" << endl;

      if(first_side_waypoint)
      {
        cout << "Looking for first location" << endl;
        first_side_waypoint = false; 
        searchLocation = setSearchLocation(mapSize/2 - .8, mapSize/2 - .8);
        movingLeft = true;
        break;
      }

      else
      {
        cout << "Calculating new position." << endl;
        if(movingLeft)
        {
          cout << "Moving Left" << endl;

          /*angle = 5*M_PI/4;
          angle = M_PI/4;
          magnitude = sqrt(pow((mapSize/2 - .5) - triangleSquare,2) + pow((mapSize/2 - .5)- triangleSquare,2));
          cout << "Magnitude: " << magnitude << endl;
          //this->searchLocation = setSearchLocation(magnitude * cos(angle),magnitude * sin(angle));
          this->searchLocation.theta = 5*M_PI/4;
          */

          this->searchLocation = setSearchLocation(currentLocation.x - 1.5,currentLocation.y - sideBoundary);
          movingLeft = false;
          movingRight = true;
          break;
        }

        else if(movingRight)
        {
          cout << "Moving Right" << endl;
         // this->searchLocation.theta = 3*M_PI/4;
         this->searchLocation = setSearchLocation(currentLocation.x - 1.5,currentLocation.y + sideBoundary);
          movingLeft =true;
          movingRight = false;
          break;
        }
        

      }
    }

  case 2:
    {
      cout << "Looking for side section # 2" << endl;

      if(first_side_waypoint)
      {
        cout << "Looking for first location" << endl;
        first_side_waypoint = false; 
        searchLocation = setSearchLocation(mapSize/2 - .8, triangleSquare);
        movingRight = true;
        break;
      }

      else
      {
        cout << "Calculating new position." << endl;
        if(movingLeft)
        {
          cout << "Moving Left" << endl;

          /*angle = 5*M_PI/4;
          angle = M_PI/4;
          magnitude = sqrt(pow((mapSize/2 - .5) - triangleSquare,2) + pow((mapSize/2 - .5)- triangleSquare,2));
          cout << "Magnitude: " << magnitude << endl;
          //this->searchLocation = setSearchLocation(magnitude * cos(angle),magnitude * sin(angle));
          this->searchLocation.theta = 5*M_PI/4;
          */

          this->searchLocation = setSearchLocation(currentLocation.x + sideBoundary,currentLocation.y - 1.5);
          movingLeft = false;
          movingRight = true;
          break;
        }

        else if(movingRight)
        {
          cout << "Moving Right" << endl;
         // this->searchLocation.theta = 3*M_PI/4;
         this->searchLocation = setSearchLocation(currentLocation.x - sideBoundary,currentLocation.y - 1.5);
          movingLeft =true;
          movingRight = false;
          break;
        }
        

      }
    }

  case 3:
    {
      cout << "Looking for side section # 3" << endl;
    
      if(first_side_waypoint)
      {
        cout << "Looking for first location" << endl;
        first_side_waypoint = false; 
        searchLocation = setSearchLocation(mapSize/2 - .5, (mapSize/2 - .5) * -1);
        movingRight = true;
        break;
      }

      else
      {
        cout << "Calculating new position." << endl;
        if(movingLeft)
        {
          cout << "Moving Left" << endl;

          /*angle = 5*M_PI/4;
          angle = M_PI/4;
          magnitude = sqrt(pow((mapSize/2 - .5) - triangleSquare,2) + pow((mapSize/2 - .5)- triangleSquare,2));
          cout << "Magnitude: " << magnitude << endl;
          //this->searchLocation = setSearchLocation(magnitude * cos(angle),magnitude * sin(angle));
          this->searchLocation.theta = 5*M_PI/4;
          */

          this->searchLocation = setSearchLocation(currentLocation.x - 1.5,currentLocation.y - sideBoundary);
          movingLeft = false;
          movingRight = true;
          break;
        }

        else if(movingRight)
        {
          cout << "Moving Right" << endl;
         // this->searchLocation.theta = 3*M_PI/4;
         this->searchLocation = setSearchLocation(currentLocation.x - 1.5,currentLocation.y + sideBoundary);
          movingLeft =true;
          movingRight = false;
          break;
        }
        

      }
    }

    

    case 4:
    {
      cout << "Looking for side section # 4" << endl;

      if(first_side_waypoint)
      {
        cout << "Looking for first location" << endl;
        first_side_waypoint = false; 
        searchLocation = setSearchLocation((mapSize/2 - .8) * -1, triangleSquare);
        movingLeft = true;
        break;
      }

      else
      {
        cout << "Calculating new position." << endl;
        if(movingLeft)
        {
          cout << "Moving Left" << endl;

          /*angle = 5*M_PI/4;
          angle = M_PI/4;
          magnitude = sqrt(pow((mapSize/2 - .5) - triangleSquare,2) + pow((mapSize/2 - .5)- triangleSquare,2));
          cout << "Magnitude: " << magnitude << endl;
          //this->searchLocation = setSearchLocation(magnitude * cos(angle),magnitude * sin(angle));
          this->searchLocation.theta = 5*M_PI/4;
          */

          this->searchLocation = setSearchLocation(currentLocation.x + sideBoundary,currentLocation.y - 1.5);
          movingLeft = false;
          movingRight = true;
          break;
        }

        else if(movingRight)
        {
          cout << "Moving Right" << endl;
         // this->searchLocation.theta = 3*M_PI/4;
         this->searchLocation = setSearchLocation(currentLocation.x - sideBoundary,currentLocation.y - 1.5);
          movingLeft =true;
          movingRight = false;
          break;
        }
        

      }
    }
    default:
    {

    }
  }
}
void SearchController::triangleSearch(int myId,int triangularSection, float triangleSquare)
{
  float xLoc = 0;
  float yLoc = 0;
  bool pointAccepted = false;
  result.type = waypoint;
    //Developing code for first traingular section
    switch(triangularSection)
    {
      case 1: //This section is from degrees 0->45
        {
          if(first_waypoint) 
          {
            cout << "--Looking for first given location.--" << endl;
            this->searchLocation = setSearchLocation(1,.5);
            first_waypoint = false;
            break;
          }

          else{
            this->visitedPoints.push_back(currentLocation);
            while(pointAccepted == false)
            {
              angle = rng->uniformReal(0,M_PI/4);
              angle = radToDeg(angle);

              unknownAngle = 180 - (angle + 90);
              unknownAngle = degToRad(unknownAngle);
              magnitude = rng->uniformReal(1,(sin(M_PI/2) * triangleSquare)/sin(unknownAngle));
              cout << "Vector: (" << magnitude << "," << angle <<")" << endl;
              angle = degToRad(angle); 
              xLoc = magnitude * cos(angle);
              yLoc = magnitude * sin(angle);

              cout <<"Current Location: (" << currentLocation.x <<"," << currentLocation.y << ")" << endl;

              for(int i = 0; i <= this->visitedPoints.size()-1; i++)
              {
                cout << "Validating new location" << endl;
                if(((xLoc >= this->visitedPoints.at(i).x - .5) && (xLoc <= this->visitedPoints.at(i).x + .5)) && ((yLoc >= this->visitedPoints.at(i).y - .5)  && (yLoc <= this->visitedPoints.at(i).y + .5)))
                {
                  pointAccepted = false;
                  break;
                }
                else{
                  pointAccepted = true;
                }
              }
              if(pointAccepted)
              {
                cout << "Point accepted!" << endl;
                this->searchLocation = setSearchLocation(xLoc,yLoc);
              }
              else{
                cout << "Point not accepted. Generating other location!!" << endl;
                cout << "Rejected location: (" << xLoc << "," << yLoc << ")" << endl;
              }
              
            }
            
            break;
          } 
        } 

        case 2: //This section is from degrees 45->90
        {
          if(first_waypoint) 
          {
            cout << "--Looking for first given location.--" << endl;
            this->searchLocation = setSearchLocation(.5,1);
            first_waypoint = false;
            break;
          }

          else{
            angle = rng->uniformReal(M_PI/4,M_PI/2);
            angle = radToDeg(angle);

            unknownAngle = 180 - (angle -45 + 90);
            unknownAngle = degToRad(unknownAngle);

            magnitude = rng->uniformReal(1,(sin(M_PI/2) * triangleSquare)/sin(unknownAngle));

            
            //magnitude = rng->uniformReal(0,9.1);
            cout << "Vector: (" << magnitude << "," << angle <<")" << endl;
            angle = degToRad(angle);
            this->searchLocation = setSearchLocation(magnitude * cos(angle),magnitude * sin(angle));
            cout << "Looking for location: (" << searchLocation.x << "," << searchLocation.y << ")" << endl;
            break;
          } 
        }

        case 3: //This section is from degrees 90->135
        {
          if(first_waypoint) 
          {
            cout << "--Looking for first given location.--" << endl;
            this->searchLocation = setSearchLocation(-.5,1);
            first_waypoint = false;
            break;
          }

          else{
            angle = rng->uniformReal(M_PI/2,3*M_PI/4);
            angle = radToDeg(angle);

            unknownAngle = 180 - (angle - 90 + 90);
            unknownAngle = degToRad(unknownAngle);

            magnitude = rng->uniformReal(1,(sin(M_PI/2) * triangleSquare)/sin(unknownAngle));

            
            //magnitude = rng->uniformReal(0,9.1);
            cout << "Vector: (" << magnitude << "," << angle <<")" << endl;
            angle = degToRad(angle);
            this->searchLocation = setSearchLocation(magnitude * cos(angle),magnitude * sin(angle));
            cout << "Looking for location: (" << searchLocation.x << "," << searchLocation.y << ")" << endl;
            break;
          } 
        } 
         case 4: //This section is from degrees 135->180
        {
          if(first_waypoint) 
          {
            cout << "--Looking for first given location.--" << endl;
            this->searchLocation = setSearchLocation(-1,.5);
            first_waypoint = false;
            break;
          }

          else{
            angle = rng->uniformReal(3*M_PI/4,M_PI);
            angle = radToDeg(angle);

            unknownAngle = 180 - (angle - 135 + 90);
            unknownAngle = degToRad(unknownAngle);

            magnitude = rng->uniformReal(1,(sin(M_PI/2) * triangleSquare)/sin(unknownAngle));

            
            //magnitude = rng->uniformReal(0,9.1);
            cout << "Vector: (" << magnitude << "," << angle <<")" << endl;
            angle = degToRad(angle);
            this->searchLocation = setSearchLocation(magnitude * cos(angle),magnitude * sin(angle));
            cout << "Looking for location: (" << searchLocation.x << "," << searchLocation.y << ")" << endl;
            break;
          } 
        }
        case 5: //This section is from degrees 180->225
        {
          if(first_waypoint) 
          {
            cout << "--Looking for first given location.--" << endl;
            this->searchLocation = setSearchLocation(-1,-.5);
            first_waypoint = false;
            break;
          }

          else{
            angle = rng->uniformReal(M_PI,5*M_PI/4);
            angle = radToDeg(angle);

            unknownAngle = 180 - (angle - 180 + 90);
            unknownAngle = degToRad(unknownAngle);

            magnitude = rng->uniformReal(1,(sin(M_PI/2) * triangleSquare)/sin(unknownAngle));

            
            //magnitude = rng->uniformReal(0,9.1);
            cout << "Vector: (" << magnitude << "," << angle <<")" << endl;
            angle = degToRad(angle);
            this->searchLocation = setSearchLocation(magnitude * cos(angle),magnitude * sin(angle));
            cout << "Looking for location: (" << searchLocation.x << "," << searchLocation.y << ")" << endl;
            break;
          } 
        }
        case 6: //This section is from degrees 225->270
        {
          if(first_waypoint) 
          {
            cout << "--Looking for first given location.--" << endl;
            this->searchLocation = setSearchLocation(-.5,-1);
            first_waypoint = false;
            break;
          }

          else{
            angle = rng->uniformReal(5*M_PI/4,3*M_PI/2);
            angle = radToDeg(angle);

            unknownAngle = 180 - (angle - 225 + 90);
            unknownAngle = degToRad(unknownAngle);

            magnitude = rng->uniformReal(1,(sin(M_PI/2) * triangleSquare)/sin(unknownAngle));

            
            //magnitude = rng->uniformReal(0,9.1);
            cout << "Vector: (" << magnitude << "," << angle <<")" << endl;
            angle = degToRad(angle);
            this->searchLocation = setSearchLocation(magnitude * cos(angle),magnitude * sin(angle));
            cout << "Looking for location: (" << searchLocation.x << "," << searchLocation.y << ")" << endl;
            break;
          } 
        }
        case 7: //This section is from degrees 270->315
        {
          if(first_waypoint) 
          {
            cout << "--Looking for first given location.--" << endl;
            this->searchLocation = setSearchLocation(.5,-1);
            first_waypoint = false;
            break;
          }

          else{
            angle = rng->uniformReal(3*M_PI/2,7*M_PI/4);
            angle = radToDeg(angle);

            unknownAngle = 180 - (angle - 270 + 90);
            unknownAngle = degToRad(unknownAngle);

            magnitude = rng->uniformReal(1,(sin(M_PI/2) * triangleSquare)/sin(unknownAngle));

            
            //magnitude = rng->uniformReal(0,9.1);
            cout << "Vector: (" << magnitude << "," << angle <<")" << endl;
            angle = degToRad(angle);
            this->searchLocation = setSearchLocation(magnitude * cos(angle),magnitude * sin(angle));
            cout << "Looking for location: (" << searchLocation.x << "," << searchLocation.y << ")" << endl;
            break;
          } 
        }
        case 8: //This section is from degrees 315->360
        {
          if(first_waypoint) 
          {
            cout << "--Looking for first given location.--" << endl;
            this->searchLocation = setSearchLocation(1,-.5);
            first_waypoint = false;
            break;
          }

          else{
            angle = rng->uniformReal(7*M_PI/4,2*M_PI);
            angle = radToDeg(angle);

            unknownAngle = 180 - (angle - 315 + 90);
            unknownAngle = degToRad(unknownAngle);

            magnitude = rng->uniformReal(1,(sin(M_PI/2) * triangleSquare)/sin(unknownAngle));

            
            //magnitude = rng->uniformReal(0,9.1);
            cout << "Vector: (" << magnitude << "," << angle <<")" << endl;
            angle = degToRad(angle);
            this->searchLocation = setSearchLocation(magnitude * cos(angle),magnitude * sin(angle));
            cout << "Looking for location: (" << searchLocation.x << "," << searchLocation.y << ")" << endl;
            break;
          } 
        }
      default:
        {
          cout << "Error in Triangular Section!!" << endl;
          break;
        }
    }
}
Point SearchController::setSearchLocation(float x, float y)
{
  Point searchPoint;
  searchPoint.x = x;
  searchPoint.y = y;
  return searchPoint;
}

float SearchController::radToDeg(float rad)
{
  return (rad * 180/M_PI);
}
float SearchController::degToRad(float deg)
{
  return (deg * M_PI/180);
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
