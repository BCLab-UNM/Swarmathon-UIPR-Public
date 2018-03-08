

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

  searchObstacle = false;
 
  pointCounter = 0; //Counter used to set a limit of visited points per area.

  //Variables used for triangle search.
  rng = new random_numbers::RandomNumberGenerator();
  magnitude = 0;
  angle = 0;
  unknownAngle = 0; //Used Sine law to calculate maximum Hypo depending on generated Angle
  triangleSel = 1; //Variable used to iterate through triangle areas.
  triangleSquare = 0; // Maximum sqaure area for triangle search.


  //Variables used for ZIG ZAG search.
  first_side_waypoint = true; // First waypoint of a side area.
  pointLimit = 14; //Variable to limit visited points in any area. Every side gives a particular value.
  sideSel = 1; //Variable used to iterate through side areas.
  ghostWall = .8; //Variable to evade walls. 
}

void SearchController::Reset() {
  result.reset = false;
}
Result SearchController::DoWork() {

    

    result.wpts.waypoints.clear();
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);

    return result;
  }

void SearchController::newPointManager(){
  cout << "New Thread running!" << endl;
  while(true){

    // Verifiy if points in vector are the maximum size
    // if it is NOT the max size then generate a new point
    // else do nothing

    if(getMapSize() == 15) //Preliminars?
    {
      setTriangleSquareArea(10); // 10x10mts area for triangle square.(5mts each side)
    }
    else{ // Semi/Finals?

    }

    giveTask2Robot(); //Verify ID and give the robot a task. 

  }
}

void SearchController::giveTask2Robot()
  {
    switch(this->myId)
    {
      case 1:
      {
         if(pointCounter == pointLimit)
      {
        pointCounter = 0;
        sideSel++;
        first_side_waypoint = true;
      }
      else{
        pointCounter++;
      }
      sideSearch(myId,sideSel,triangleSquare);
  
      //Before of deciding where to search check the public list of triangles to know their state.

      /* 
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
        */

        break;
    }

      case 2:
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
        break;
      }
    
      case 3:
      {
        sideSearch(myId,3,triangleSquare);
        break;
      }

      case 4:
      {
        sideSearch(myId,1,triangleSquare);
        break;
      }

      case 5:
      {
        //5th robot
        break;
      }

      case 6:
      {
        //6th robot
        break;
      }

      default:
      {
        //Randomizer
        break;
      }

    }//End of switch
  }
void SearchController::setTriangleSquareArea(float area)
  {
    this->triangleSquare = area;
  }

int SearchController::getMapSize()
  {
  //Method to identify maximum foraging area. 
    if(totalIds <=3)
    {
      cout << "Map Size: 15x15mts" << endl;
      this->mapSize = 15; //15mts by 15mts map size.
    }
    else{
      cout << "Map Size: 22x22mts" << endl;
      this->mapSize = 22; //22mts by 22mts map size. 
    }

    return this->mapSize;
  }


void SearchController::setSideBoundary()
  {
    this->sideBoundary = (this->mapSize/2 - this->triangleSquare/2) - ghostWall;
  }

void SearchController::setGhostWall(float ghostWall)
  {
    this->ghostWall = ghostWall;
  }

void SearchController::sideSearch(int myId,int sideSection,float triangleSquare)
  {
    result.type = waypoint;
    setSideBoundary();
    switch(sideSection)
    {
      cout << "Looking for side section #" << sideSection << endl;

      case 1://Top side
      {
        if(first_side_waypoint)
        {
          cout << "Looking for first location" << endl;
          first_side_waypoint = false; 
          searchLocation = setSearchLocation(mapSize/2 - this->ghostWall, mapSize/2 - this->ghostWall);
          movingLeft = true;
          break;
        }

        else
        {
          cout << "Calculating new position." << endl;
          if(movingLeft)
          {
            cout << "Moving Left" << endl;
            this->searchLocation = setSearchLocation(currentLocation.x - 1 ,currentLocation.y - sideBoundary);
            movingLeft = false;
            movingRight = true;
            break;
          }

          else if(movingRight)
          {
            cout << "Moving Right" << endl;
            this->searchLocation = setSearchLocation(currentLocation.x - 1 ,currentLocation.y + sideBoundary);
            movingLeft =true;
            movingRight = false;
            break;
          }
        }
      }
    case 2://Left Side
      {
        pointLimit = 11;
        if(first_side_waypoint)
        {
          first_side_waypoint = false;
          movingLeft = true; 
          cout << "Looking for first location" << endl;
          searchLocation = setSearchLocation((mapSize/2 - this->ghostWall) * -1, triangleSquare);
          break;
        }
        else
        {
          cout << "Calculating new position." << endl;
          if(movingLeft)
          {
            movingLeft = false;
            movingRight = true;
            cout << "Moving Left" << endl;
            this->searchLocation = setSearchLocation(currentLocation.x + sideBoundary,currentLocation.y - 1);
            break;
          }
          else if(movingRight)
          {
            movingLeft =true;
            movingRight = false;
            cout << "Moving Right" << endl;
            this->searchLocation = setSearchLocation(currentLocation.x - sideBoundary,currentLocation.y - 1);
            break;
          }
        }
      }

    case 3://Bottom Side
      {
        pointLimit = 12;    
        if(first_side_waypoint)
        {
          first_side_waypoint = false;
          movingRight = true;
          cout << "Looking for first location" << endl; 
          searchLocation = setSearchLocation((mapSize/2 - this->ghostWall) * -1, (mapSize/2 - this->ghostWall) * -1);
          break;
        }
        else
        {
          cout << "Calculating new position." << endl;
          if(movingLeft)
          {
            movingLeft = false;
            movingRight = true;
            cout << "Moving Left" << endl;
            this->searchLocation = setSearchLocation(currentLocation.x + 1,currentLocation.y - sideBoundary);
            break;
          }

          else if(movingRight)
          {
            movingLeft =true;
            movingRight = false;
            cout << "Moving Right" << endl;
            this->searchLocation = setSearchLocation(currentLocation.x + 1,currentLocation.y + sideBoundary);
            break;
          }
          

        }
      }

      case 4://Right Side
      {
        pointLimit = 11;
        if(first_side_waypoint)
        {
          movingRight = true;
          first_side_waypoint = false; 
          cout << "Looking for first location" << endl;
          searchLocation = setSearchLocation(mapSize/2 - this->ghostWall, triangleSquare * -1);
          break;
        }
        else
        {
          cout << "Calculating new position." << endl;
          if(movingLeft)
          {
            movingLeft = false;
            movingRight = true;
            cout << "Moving Left" << endl;
            this->searchLocation = setSearchLocation(currentLocation.x + sideBoundary,currentLocation.y + 1);
            break;
          }

          else if(movingRight)
          {
            movingLeft = true;
            movingRight = false;
            cout << "Moving Right" << endl;
            this->searchLocation = setSearchLocation(currentLocation.x - sideBoundary,currentLocation.y + 1);
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
            /*
            if(visitedLoc.size() == 0)
              {
                cout << "-There are no Published Visited Locations-" << endl;
              }
            else{
                cout << "------------------------------------------" << endl;
                for(int i = 0; i <= visitedLoc.size() - 1; i++)
                {
                  cout << "Location #" << i + 1 << "(" << visitedLoc.at(i).x << "," << visitedLoc.at(i).y << ")" << endl;
                }
                cout << "------------------------------------------" << endl;
              }
              */

            if(first_waypoint) 
            {
              cout << "--Looking for first given location.--" << endl;
              this->searchLocation = setSearchLocation(1,.5);
              first_waypoint = false;
              break;
            }

            else{
              this->visitedLoc.push_back(currentLocation);
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

                for(int i = 0; i <= this->visitedLoc.size()-1; i++)
                {
                  cout << "Validating new location" << endl;
                  if(((xLoc >= this->visitedLoc.at(i).x - .5) && (xLoc <= this->visitedLoc.at(i).x + .5)) && ((yLoc >= this->visitedPoints.at(i).y - .5)  && (yLoc <= this->visitedPoints.at(i).y + .5)))
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


void SearchController::CreateThread(){
    thread pointCreatorThread (&SearchController::newPointManager, this);
    pointCreatorThread.detach();
}