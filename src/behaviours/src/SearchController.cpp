#include "SearchController.h"
#include <angles/angles.h>

SearchController::SearchController()
{
  currentLocation.x = 0;
  currentLocation.y = 0;
  currentLocation.theta = 0;
  centerLocation.x = 0;
  centerLocation.y = 0;
  centerLocation.theta = 0;
  result.PIDMode = FAST_PID;
  result.fingerAngle = M_PI / 2;
  result.wristAngle = M_PI / 4;

  searchObstacle = false;

  pointCounter = 0; //Counter used to set a limit of visited points per area.

  //Variables used for triangle search.
  rng = new random_numbers::RandomNumberGenerator();
  magnitude = 0;
  angle = 0;
  unknownAngle = 0; //Used Sine law to calculate maximum Hypo depending on generated Angle
  triangleSel = 1;  //Variable used to iterate through triangle areas.
  triangleSel2 = 5;
  triangleSquare = 0; // Maximum sqaure area for triangle search.
  first_waypoint = true;
  trianglePointLimit = 9;

  //Variables used for ZIG ZAG search.
  first_side_waypoint = true; // First waypoint of a side area.
  sideSel = 1;                //Variable used to iterate through side areas.
  ghostWall = .8;             //Variable to evade walls.
  goRight = false;
  goLeft = false;
  first_side_waypoint = true;
  sideInit = true;
  sideOffset = 0;
  finals = false;

  reachedFidelity = false;
  clusterAssigned = false;
}

void SearchController::Reset()
{
  result.reset = false;
}

void SearchController::displayVector(float magnitude, float angle)
{
  cout << "Vector: (" << magnitude << "mts," << angle << "°)" << endl;
}

bool SearchController::checkAvailableDistance(int sideSel)
{
  cout << "Checking for distance availabilty!" << endl;

  switch (sideSel)
  {
  case 1:
  {
    if (abs(currentLocation.x - (mapSize / 2 * -1)) >= 3 + this->sideOffset)
    {
      cout << "Distance left is > 1" << endl;
      return true;
      break;
    }
    else
    {
      cout << "Distance left is < 1" << endl;
      return false;
      break;
    }
  }

  case 2:
  {
    if (abs(currentLocation.y - (mapSize / 2 * -1)) >= 3 + this->sideOffset)
    {
      cout << "Distance left is > 1" << endl;
      return true;
      break;
    }
    else
    {
      cout << "Distance left is < 1" << endl;
      return false;
      break;
    }
  }

  case 3:
  {
    if (abs(currentLocation.x - (mapSize / 2)) >= 3 + this->sideOffset)
    {
      cout << "Distance left is > 1" << endl;
      return true;
      break;
    }
    else
    {
      cout << "Distance left is < 1" << endl;
      return false;
      break;
    }
  }

  case 4:
  {
    if (abs(currentLocation.y - (mapSize / 2)) >= 3 + this->sideOffset)
    {
      cout << "Distance left is > 1" << endl;
      return true;
      break;
    }
    else
    {
      cout << "Distance left is < 1" << endl;
      return false;
      break;
    }
  }

  default:
  {
    cout << "Trying to verify available distance. Emergency side change!" << endl;
    return false;
    break;
  }
  }
}
Result SearchController::DoWork()
{
  cout << "Searching for new Location!!" << endl;

  if (getMapSize() == 15) //Preliminars?
  {
    setTriangleSquareArea(10); // 10x10mts area for triangle square.(5mts each side) -- Hector Changed
    this->finals = false;
  }
  else
  {                            // Semi/Finals?
    setTriangleSquareArea(16); // 10x10mts area for triangle square.(5mts each side)
    this->finals = true;
  }

  if (fidImprovement)
  {
    cout << "Using Fidelity" << endl;

    if (hypot(fidelitySavedPoint.x - currentLocation.x, fidelitySavedPoint.y - currentLocation.y) > 0.2 && reachedFidelity)
    {
      this->searchLocation = this->fidelitySavedPoint;
    }
    else{
      reachedFidelity = false;
      FidelityImprovement();
      this->fidelitySavedPoint = this->searchLocation;
    }
  }
  else
  {
    giveTask2Robot(); //Verify ID and give the robot a task.
  }

  result.wpts.waypoints.clear();
  result.wpts.waypoints.insert(result.wpts.waypoints.begin(), this->searchLocation);
  return result;
}

void SearchController::giveTask2Robot()
{
  switch (this->myId)
  {
  case 1:
  {
    if (pointCounter == trianglePointLimit)
    {
      pointCounter = 0;
      //Look for triangle availability.
      cout << "Limited Point search reached!!" << endl;
      triangleSel++;
    }
    else
    {
      pointCounter++;
    }

    if (triangleSel > 4)
    {
      randomWalk();
    }
    else
    {
      triangleSearch(myId, triangleSel, triangleSquare);
    }

    //this->searchLocation = setSearchLocation(-5,1);
    break;
  }

  case 2:
  {

    if (pointCounter == trianglePointLimit)
    {
      pointCounter = 0;
      //Look for triangle availability.
      cout << "Limited Point search reached!!" << endl;
      triangleSel2++;
    }
    else
    {
      pointCounter++;
    }

    if (triangleSel2 > 8)
    {
      randomWalk();
    }
    else
    {

      triangleSearch(myId, triangleSel2, triangleSquare);
    }

    break;
  }

  case 3:
  {
    if (sideInit)
    {
      sideSel = 1;
      this->sideInit = false;
    }
    if (clusterLocation.empty() == false)
    {

      if (firstClusterCommand)
      {
        previousLoc = setSearchLocation(currentLocation.x, currentLocation.y);
        this->firstClusterCommand = false;
      }
      this->searchLocation.x = clusterLocation.at(0).x;
      this->searchLocation.y = clusterLocation.at(0).y;
      clusterAssigned = true;
      clustered = true;
      std::cout << "Location Cluster = " << searchLocation.x << ", " << searchLocation.y << '\n';
      break;
    }
    if (clustered)
    {
      this->searchLocation = previousLoc;
      clustered = !clustered;
      break;
    }
    else
    {
      if (checkAvailableDistance(sideSel) == false)
      {
        first_side_waypoint = true;
        if(this->finals)
        {
          this->sideOffset = getSideOffset();
        }
        if (sideSel == 4)
        {
          sideSel = 1;
          if(this->finals == false)
          {
            this->sideOffset = getSideOffset();
          }
        }
        else
        {
          sideSel++;
        }
      }
    }
    sideSearch(myId, sideSel, triangleSquare, sideOffset);
    break;
  }

  case 4:
  {
    if (sideInit)
    {
      sideSel = 2;
      this->sideInit = false;
    }
    if (checkAvailableDistance(sideSel) == false)
    {
      first_side_waypoint = true;
      if (sideSel == 4)
      {
        this->sideOffset = getSideOffset();
        sideSel = 1;
      }
      else
      {
        sideSel++;
      }
    }
    sideSearch(myId, sideSel, triangleSquare, sideOffset);
    break;
  }

  case 5:
  {
    //5th robot
    if (sideInit)
    {
      sideSel = 3;
      this->sideInit = false;
    }
    if (checkAvailableDistance(sideSel) == false)
    {
      first_side_waypoint = true;
      this->sideOffset = getSideOffset();
      if (sideSel == 4)
      {
        sideSel = 1;
      }
      else
      {
        sideSel++;
      }
    }
    sideSearch(myId, sideSel, triangleSquare, sideOffset);
    break;
  }

  case 6:
  {
    //6th robot
    if (sideInit)
    {
      sideSel = 4;
      this->sideInit = false;
    }
    if (checkAvailableDistance(sideSel) == false)
    {
      first_side_waypoint = true;
      this->sideOffset = getSideOffset();
      if (sideSel == 4)
      {
        sideSel = 1;
      }
      else
      {
        sideSel++;
      }
    }
    sideSearch(myId, sideSel, triangleSquare, sideOffset);
    break;
  }

  default:
  {
    cout << "Did not received an ID. Random walking" << endl;
    randomWalk();
    break;
  }

  } //End of switch
}
void SearchController::randomWalk()
{
  result.type = waypoint;
  switch (this->myId)
  {
    case 1:
    {
      this->searchLocation = generateRandomTriangleLoc(0.2, 3, mapSize / 2 - 2.5);
      break;
    }

    case 2:
    {
      this->searchLocation = generateRandomTriangleLoc(3.3, 6.1, mapSize / 2 - 2.5);
      break;
    }

    default:
    {
      this->searchLocation = generateRandomTriangleLoc(.2,6.1, mapSize/2 - 2.5);
      break;
    }
  }
}
float SearchController::getSideOffset()
{
  if (this->finals)
  {
    return .1;
  }
  else
  {
    return .4;
  }
}

float SearchController::angleTraslation(float newAngle)
{
  float translationAngle;

  if (newAngle >= 0 && newAngle <= 45)
  {
    translationAngle = 0;
  }
  else if (newAngle >= 45 && newAngle <= 90)
  {
    translationAngle = 45;
  }
  else if (newAngle >= 90 && newAngle <= 135)
  {
    translationAngle = 90;
  }
  else if (newAngle >= 135 && newAngle <= 180)
  {
    translationAngle = 135;
  }
  else if (newAngle >= 180 && newAngle <= 225)
  {
    translationAngle = 180;
  }
  else if (newAngle >= 225 && newAngle <= 270)
  {
    translationAngle = 225;
  }
  else if (newAngle >= 270 && newAngle <= 315)
  {
    translationAngle = 270;
  }
  else if (newAngle >= 315 && newAngle <= 360)
  {
    translationAngle = 315;
  }
  return translationAngle;
}
Point SearchController::generateRandomTriangleLoc(float firstBound, float secondBound, float maxBoundary)
{
  Point newLoc;
  float newAngle;
  float unknownedAngle;
  float magnituded;
  newAngle = rng->uniformReal(firstBound, secondBound);
  newAngle = radToDeg(newAngle);
  unknownedAngle = 180 - (newAngle - angleTraslation(newAngle) + 90);

  unknownedAngle = degToRad(unknownedAngle);
  magnituded = rng->uniformReal(1.5, (sin(M_PI / 2) * maxBoundary) / sin(unknownedAngle));

  displayVector(magnituded, newAngle);
  newAngle = degToRad(newAngle);
  newLoc.x = magnituded * cos(newAngle);
  newLoc.y = magnituded * sin(newAngle);

  return newLoc;
}
void SearchController::triangleSearch(int myId, int triangularSection, float triangleSquare)
{
  result.type = waypoint;
  //Developing code for first traingular section
  switch (triangularSection)
  {
  case 1: //This section is from degrees 0->45
  {
    if (first_waypoint)
    {
      cout << "--Looking for first given location.--" << endl;
      this->searchLocation = setSearchLocation(1, 0.5); //Hector changed
      first_waypoint = false;
      break;
    }

    else
    {

      if (hypot(searchLocation.y - currentLocation.y, searchLocation.x - currentLocation.x) >= .15 && pointInsideObstacle == false)
      {
        //first_side_waypoint = true;
        this->searchLocation = searchLocation;
        break;
      }
      else
      {
        this->searchLocation = generateRandomTriangleLoc(0, M_PI / 4, triangleSquare);
        break;
      }
    }
  }

  case 2: //This section is from degrees 45->90
  {
    if (first_waypoint)
    {
      cout << "--Looking for first given location.--" << endl;
      this->searchLocation = setSearchLocation(.5, 1);
      first_waypoint = false;
      break;
    }

    else
    {
      if (hypot(searchLocation.y - currentLocation.y, searchLocation.x - currentLocation.x) >= .15 && pointInsideObstacle == false)
      {
        //first_side_waypoint = true;
        this->searchLocation = searchLocation;
        break;
      }
      else
      {
        this->searchLocation = generateRandomTriangleLoc(M_PI / 4, M_PI / 2, triangleSquare);
        cout << "Looking for location: (" << searchLocation.x << "," << searchLocation.y << ")" << endl;
        break;
      }
    }
  }

  case 3: //This section is from degrees 90->135
  {
    if (first_waypoint)
    {
      cout << "--Looking for first given location.--" << endl;
      this->searchLocation = setSearchLocation(-.5, 1);
      first_waypoint = false;
      break;
    }

    else
    {

      if (hypot(searchLocation.y - currentLocation.y, searchLocation.x - currentLocation.x) >= .15 && pointInsideObstacle == false)
      {
        //first_side_waypoint = true;
        this->searchLocation = searchLocation;
        break;
      }
      else
      {
        this->searchLocation = generateRandomTriangleLoc(M_PI / 2, 3 * M_PI / 4, triangleSquare);
        cout << "Looking for location: (" << searchLocation.x << "," << searchLocation.y << ")" << endl;
        break;
      }
    }
  }
  case 4: //This section is from degrees 135->180
  {
    if (first_waypoint)
    {
      cout << "--Looking for first given location.--" << endl;
      this->searchLocation = setSearchLocation(-1, .5);
      first_waypoint = false;
      break;
    }

    else
    {
      if (hypot(searchLocation.y - currentLocation.y, searchLocation.x - currentLocation.x) >= .15 && pointInsideObstacle == false)
      {
        //first_side_waypoint = true;
        this->searchLocation = searchLocation;
        break;
      }
      else
      {
        this->searchLocation = generateRandomTriangleLoc(3 * M_PI / 4, M_PI, triangleSquare);
        cout << "Looking for location: (" << searchLocation.x << "," << searchLocation.y << ")" << endl;
        break;
      }
    }
  }
  case 5: //This section is from degrees 180->225
  {
    if (first_waypoint)
    {
      cout << "--Looking for first given location.--" << endl;
      this->searchLocation = setSearchLocation(-1, -.5);
      first_waypoint = false;
      break;
    }

    else
    {
      if (hypot(searchLocation.y - currentLocation.y, searchLocation.x - currentLocation.x) >= .15 && pointInsideObstacle == false)
      {
        //first_side_waypoint = true;
        this->searchLocation = searchLocation;
        break;
      }
      else
      {
        this->searchLocation = generateRandomTriangleLoc(M_PI, 5 * M_PI / 4, triangleSquare);
        cout << "Looking for location: (" << searchLocation.x << "," << searchLocation.y << ")" << endl;
        break;
      }
    }
  }
  case 6: //This section is from degrees 225->270
  {
    if (first_waypoint)
    {
      cout << "--Looking for first given location.--" << endl;
      this->searchLocation = setSearchLocation(-.5, -1);
      first_waypoint = false;
      break;
    }

    else
    {
      if (hypot(searchLocation.y - currentLocation.y, searchLocation.x - currentLocation.x) >= .15 && pointInsideObstacle == false)
      {
        //first_side_waypoint = true;
        this->searchLocation = searchLocation;
        break;
      }
      else
      {
        this->searchLocation = generateRandomTriangleLoc(5 * M_PI / 4, 3 * M_PI / 2, triangleSquare);
        cout << "Looking for location: (" << searchLocation.x << "," << searchLocation.y << ")" << endl;
        break;
      }
    }
  }
  case 7: //This section is from degrees 270->315
  {
    if (first_waypoint)
    {
      cout << "--Looking for first given location.--" << endl;
      this->searchLocation = setSearchLocation(.5, -1);
      first_waypoint = false;
      break;
    }

    else
    {
      if (hypot(searchLocation.y - currentLocation.y, searchLocation.x - currentLocation.x) >= .15 && pointInsideObstacle == false)
      {
        //first_side_waypoint = true;
        this->searchLocation = searchLocation;
        break;
      }

      else
      {
        this->searchLocation = generateRandomTriangleLoc(3 * M_PI / 2, 7 * M_PI / 4, triangleSquare);
        cout << "Looking for location: (" << searchLocation.x << "," << searchLocation.y << ")" << endl;
        break;
      }
    }
  }
  case 8: //This section is from degrees 315->360
  {
    if (first_waypoint)
    {
      cout << "--Looking for first given location.--" << endl;
      this->searchLocation = setSearchLocation(1, -.5);
      first_waypoint = false;
      break;
    }

    else
    {
      if (hypot(searchLocation.y - currentLocation.y, searchLocation.x - currentLocation.x) >= .15 && pointInsideObstacle == false)
      {
        //first_side_waypoint = true;
        this->searchLocation = searchLocation;
        break;
      }
      else
      {
        angle = rng->uniformReal(7 * M_PI / 4, 2 * M_PI);
        this->searchLocation = generateRandomTriangleLoc(7 * M_PI / 4, 2 * M_PI, triangleSquare);
        cout << "Looking for location: (" << searchLocation.x << "," << searchLocation.y << ")" << endl;
        break;
      }
    }
  }
  default:
  {
    cout << "Did not received triangulsr section. Random Walking" << endl;
    randomWalk();
    break;
  }
  }
}
void SearchController::setTriangleSquareArea(float area)
{
  this->triangleSquare = area / 2;
}

int SearchController::getMapSize()
{
  //Method to identify maximum foraging area.
  if (totalIds <= 3)
  {
    cout << "Map Size: 15x15mts" << endl;
    this->mapSize = 15; //15mts by 15mts map size. -- Hector changed for physical test!!!
  }
  else
  {
    cout << "Map Size: 22x22mts" << endl;
    this->mapSize = 22; //22mts by 22mts map size.
  }

  return this->mapSize;
}

void SearchController::setSideBoundary()
{
  this->sideBoundary = (this->mapSize / 2 - this->triangleSquare) - this->ghostWall;
}

void SearchController::setGhostWall(float ghostWall)
{
  this->ghostWall = ghostWall;
}

void SearchController::thenGoRight()
{
  cout << "Moving Left" << endl;
  this->goLeft = false;
  this->goRight = true;
}

void SearchController::thenGoLeft()
{
  cout << "Moving Right" << endl;
  this->goLeft = true;
  this->goRight = false;
}
void SearchController::sideSearch(int myId, int sideSection, float triangleSquare, float offset)
{
  result.type = waypoint;
  setSideBoundary();
  cout << "Looking for side section #" << sideSection << endl;
  switch (sideSection)
  {
  case 1: //Top side
  {

    if (first_side_waypoint)
    {
      cout << "Looking for first location" << endl;
    this->sideOffset = getSideOffset();  
    first_side_waypoint = false;
      this->searchLocation = setSearchLocation(mapSize / 2 - this->ghostWall - offset, mapSize / 2 - this->ghostWall);
      goLeft = true;
      break;
    }

    else
    {
      if (hypot(searchLocation.y - currentLocation.y, searchLocation.x - currentLocation.x) >= .15 && pointInsideObstacle == false)
      {
        //first_side_waypoint = true;
        this->searchLocation = searchLocation;
        break;
      }
      else
      {
        cout << "Calculating new position." << endl;
        if (goLeft)
        {
          this->searchLocation = setSearchLocation(currentLocation.x - 1, currentLocation.y - sideBoundary);
          thenGoRight();
          break;
        }

        else if (goRight)
        {
          this->searchLocation = setSearchLocation(currentLocation.x - 1, currentLocation.y + sideBoundary);
          thenGoLeft();
          break;
        }
      }
    }
  }
  case 2: //Left Side
  {
    if (first_side_waypoint)
    {
      first_side_waypoint = false;
      goLeft = true;
      cout << "Looking for first location" << endl;
      this->searchLocation = setSearchLocation((mapSize / 2 - this->ghostWall) * -1, mapSize / 2 - this->ghostWall - offset);
      break;
    }
    else
    {
      if (hypot(searchLocation.y - currentLocation.y, searchLocation.x - currentLocation.x) >= .15 && pointInsideObstacle == false)
      {
        //first_side_waypoint = true;
        this->searchLocation = searchLocation;
        break;
      }
      else
      {
        cout << "Calculating new position." << endl;
        if (goLeft)
        {
          thenGoRight();
          this->searchLocation = setSearchLocation(currentLocation.x + sideBoundary, currentLocation.y - 1);
          break;
        }
        else if (goRight)
        {
          thenGoLeft();
          this->searchLocation = setSearchLocation(currentLocation.x - sideBoundary, currentLocation.y - 1);
          break;
        }
      }
    }
  }

  case 3: //Bottom Side
  {
    if (first_side_waypoint)
    {
      first_side_waypoint = false;
      goLeft = true;
      cout << "Looking for first location" << endl;
     this->searchLocation = setSearchLocation((mapSize / 2 - this->ghostWall + offset) * -1, (mapSize / 2 - this->ghostWall) * -1);
      break;
    }
    else
    {
      if (hypot(searchLocation.y - currentLocation.y, searchLocation.x - currentLocation.x) >= .15 && pointInsideObstacle == false)
      {
        //first_side_waypoint = true;
        this->searchLocation = searchLocation;
        break;
      }
      else
      {
        cout << "Calculating new position." << endl;
        if (goLeft)
        {
          thenGoRight();
          this->searchLocation = setSearchLocation(currentLocation.x + 1, currentLocation.y + sideBoundary);
          break;
        }

        else if (goRight)
        {
          thenGoLeft();
          this->searchLocation = setSearchLocation(currentLocation.x + 1, currentLocation.y - sideBoundary);
          break;
        }
      }
    }
  }

  case 4: //Right Side
  {
    if (first_side_waypoint)
    {
      goLeft = true;
      first_side_waypoint = false;
      cout << "Looking for first location" << endl;
      this->searchLocation = setSearchLocation(mapSize / 2 - this->ghostWall, (mapSize / 2 - this->ghostWall + offset) * -1);
      break;
    }
    else
    {
      if (hypot(searchLocation.y - currentLocation.y, searchLocation.x - currentLocation.x) >= .15 && pointInsideObstacle == false)
      {
        //first_side_waypoint = true;
        this->searchLocation = searchLocation;
        break;
      }

      else
      {
        cout << "Calculating new position." << endl;
        if (goLeft)
        {
          thenGoRight();
          this->searchLocation = setSearchLocation(currentLocation.x - sideBoundary, currentLocation.y + 1);
          break;
        }

        else if (goRight)
        {
          thenGoLeft();
          this->searchLocation = setSearchLocation(currentLocation.x + sideBoundary, currentLocation.y + 1);
          break;
        }
      }
    }
  }
  default:
  {

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
  return (rad * 180 / M_PI);
}
float SearchController::degToRad(float deg)
{
  return (deg * M_PI / 180);
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

//----------------------------------------------------------------- Fidelity Improvement (Jomar) -------------------------------------------------------------------------

void SearchController::FidelityImprovement()
{ //Enters when detects a Tag


  if (getSdropped())
  { // if drops the tag, go to the saved location
    reachedFidelity = true;
    cout << "S-1" << '\n';

    if (getDeleteVector())
    {                               // enter if there is more than one element in the vector
      SavedPointsVector.pop_back(); // delete the last saved point visited
    }

    if (SavedPointsVector.size() == 1)
    { // if only one element, get out of the 'if' statement
      setDeleteVector(false);
      arrives = true;
      Sdropped = false;
      cout << "FINISHED FIDELITY" << endl;
    }

    else
    {
      reachedFidelity = false;
      setDeleteVector(true); // if not, enter again to the statement.
    }

    //result.wpts.waypoints.clear();
    //result.wpts.waypoints.insert(result.wpts.waypoints.begin(), SavedPointsVector[SavedPointsVector.size() - 1]);
    this->searchLocation = SavedPointsVector[SavedPointsVector.size() -1];
    std::cout << "Dropped: Getting Back to: " << SavedPointsVector[SavedPointsVector.size() - 1].x << ", " << SavedPointsVector[SavedPointsVector.size() - 1].y << "\n\n";
  }

  else if (arrives)
  { // else if arrives to the saved location

    cout << "S-2" << '\n';
    SavedPointsVector.pop_back(); //delete the last element
    arrives = false;
    fidImprovement = false; // continue with the search algorithm
  }
}

void SearchController::aTagDetected()
{
  // If detects do it only one time
  if (getTagDetectedCatched())
  {
    setDeleteVector(false);
    arrives = false;
    SavedPointsVector.push_back(currentLocation);

    std::cout << "Tag Detected!!!" << '\n';
    int arrayCounter, searchProcess = 0;
    for (size_t j = 0; j != SavedPointsVector.size(); j++)
    {
      if (j == 0)
      {
        std::cout << "\nPoints Saved Array: [\n(" << SavedPointsVector[j].x << ", " << SavedPointsVector[j].y << ", " << SavedPointsVector[j].theta << ")" << '\n';
      }
      else if (j == arrayCounter && j != 0)
      {
        std::cout << "(" << SavedPointsVector[j].x << ", " << SavedPointsVector[j].y << ", " << SavedPointsVector[j].theta << ")]"
                  << "\n\n";
      }
      else
      {
        std::cout << "(" << SavedPointsVector[j].x << ", " << SavedPointsVector[j].y << ", " << SavedPointsVector[j].theta << ")" << '\n';
      }
    }

    arrayCounter = arrayCounter + 1;
    setTagDetectedCatched(false);
  }
}

void SearchController::droppedOFF()
{
  // If dropped it, do it only one time
  if (!getTagDetectedCatched())
  {
    std::cout << "SearchController::droppedOFF()" << '\n';
    fidImprovement = true;
    
    Sdropped = true;
    setTagDetectedCatched(true);
  }
}

//----------------------------------------------------------------- Fidelity Improvement --------------------------------------------------------------------------------
