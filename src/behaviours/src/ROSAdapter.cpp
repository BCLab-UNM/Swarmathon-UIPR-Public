#include <ros/ros.h>

// ROS libraries
#include <angles/angles.h>
#include <random_numbers/random_numbers.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// ROS messages
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <std_msgs/Float32MultiArray.h>
#include "swarmie_msgs/Waypoint.h"

// Include Controllers
#include "LogicController.h"
#include <vector>
#include "Point.h"
#include "Tag.h"

// To handle shutdown signals so the node quits
// properly in response to "rosnode kill"
#include <ros/ros.h>
#include <signal.h>

#include <exception> // For exception handling

// Edit
#include <stdlib.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int64MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include <std_msgs/Bool.h>
#include <chrono> // Time measurments
//

using namespace std;

// Define Exceptions
// Define an exception to be thrown if the user tries to create
// a RangeShape using invalid dimensions
class ROSAdapterRangeShapeInvalidTypeException : public std::exception
{
public:
  ROSAdapterRangeShapeInvalidTypeException(std::string msg)
  {
    this->msg = msg;
  }

  virtual const char *what() const throw()
  {
    std::string message = "Invalid RangeShape type provided: " + msg;
    return message.c_str();
  }

private:
  std::string msg;
};

//Cluster Points Subscriber
ros::Subscriber clusterPointSubs;
//Cluster Points Publisher
ros::Publisher clusterPointPub;

std_msgs::Float64MultiArray clusterCoordArray;
bool clusterPointSaved = false;
void clusterHandler(const std_msgs::Float64MultiArray &message);

// Random number generator
random_numbers::RandomNumberGenerator *rng;

// Create logic controller

LogicController logicController;

void humanTime();

// Behaviours Logic Functions
void sendDriveCommand(double linearVel, double angularVel);
void openFingers();   // Open fingers to 90 degrees
void closeFingers();  // Close fingers to 0 degrees
void raiseWrist();    // Return wrist back to 0 degrees
void lowerWrist();    // Lower wrist to 50 degrees
void resultHandler(); // Not Used/Dead Code, prototype has no definition

Point updateCenterLocation();    //calls transformMapCenterToOdom, returns a center location in ODOM frame
void transformMapCentertoOdom(); //checks ODOMs perceived idea of where the center is with a stored GPS center coordinate and adjusts ODOM center value to account for drift

// Numeric Variables for rover positioning
geometry_msgs::Pose2D currentLocation;        //current location using ODOM
geometry_msgs::Pose2D currentLocationMap;     //current location using GPS
geometry_msgs::Pose2D currentLocationAverage; //an average of the robots current location

geometry_msgs::Pose2D centerLocation;       //Not used, dead code
geometry_msgs::Pose2D centerLocationMap;    //A GPS point of the center location, used to help reduce drift from ODOM
geometry_msgs::Pose2D centerLocationOdom;   //The centers location based on ODOM
geometry_msgs::Pose2D centerLocationMapRef; //Variable used in TransformMapCenterToOdom, can be moved to make it local instead of global

int currentMode = 0;
const float behaviourLoopTimeStep = 0.1;    //time between the behaviour loop calls
const float status_publish_interval = 1;    //time between publishes
const float heartbeat_publish_interval = 2; //time between heartbeat publishes
const float waypointTolerance = 0.1;        //10 cm tolerance.

// used for calling code once but not in main
bool initilized = false; //switched to true after running through state machine the first time, initializes base values

float linearVelocity = 0;  //forward speed, POSITIVE = forward, NEGATIVE = backward
float angularVelocity = 0; //turning speed, POSITIVE = left, NEGATIVE = right

float prevWrist = 0;    //last wrist angle
float prevFinger = 0;   //last finger angle
long int startTime = 0; //stores time when robot is swtiched on
float minutesTime = 0;  //time in minutes
float hoursTime = 0;    //time in hours

float drift_tolerance = 0.5; // the perceived difference between ODOM and GPS values before shifting the values up or down, in meters

Result result; //result struct for passing and storing values to drive robot

std_msgs::String msg; //used for passing messages to the GUI

geometry_msgs::Twist velocity;
char host[128];       //rovers hostname
string publishedName; //published hostname
char prev_state_machine[128];

// Publishers
ros::Publisher stateMachinePublish; //publishes state machine status
ros::Publisher status_publisher;    //publishes rover status
ros::Publisher fingerAnglePublish;  //publishes gripper angle to move gripper fingers
ros::Publisher wristAnglePublish;   //publishes wrist angle to move wrist
ros::Publisher infoLogPublisher;    //publishes a message to the infolog box on GUI
ros::Publisher driveControlPublish; //publishes motor commands to the motors
ros::Publisher heartbeatPublisher;  //publishes ROSAdapters status via its "heartbeat"
// Publishes swarmie_msgs::Waypoint messages on "/<robot>/waypooints"
// to indicate when waypoints have been reached.
ros::Publisher waypointFeedbackPublisher; //publishes a waypoint to travel to if the rover is given a waypoint in manual mode

// Subscribers
ros::Subscriber joySubscriber;
ros::Subscriber modeSubscriber;
ros::Subscriber targetSubscriber;
ros::Subscriber odometrySubscriber;
ros::Subscriber mapSubscriber;
ros::Subscriber virtualFenceSubscriber;
// manualWaypointSubscriber listens on "/<robot>/waypoints/cmd" for
// swarmie_msgs::Waypoint messages.
ros::Subscriber manualWaypointSubscriber; //receives manual waypoints given from GUI

// Timers
ros::Timer stateMachineTimer;
ros::Timer publish_status_timer;
ros::Timer publish_heartbeat_timer;

// records time for delays in sequenced actions, 1 second resolution.
time_t timerStartTime;

// An initial delay to allow the rover to gather enough position data to
// average its location.
unsigned int startDelayInSeconds = 30;
float timerTimeElapsed = 0;

//Transforms
tf::TransformListener *tfListener;

// OS Signal Handler
void sigintEventHandler(int signal);

//Callback handlers
void joyCmdHandler(const sensor_msgs::Joy::ConstPtr &message);                      //for joystick control
void modeHandler(const std_msgs::UInt8::ConstPtr &message);                         //for detecting which mode the robot needs to be in
void targetHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr &tagInfo); //receives and stores April Tag Data using the TAG class
void odometryHandler(const nav_msgs::Odometry::ConstPtr &message);                  //receives and stores ODOM information
void mapHandler(const nav_msgs::Odometry::ConstPtr &message);                       //receives and stores GPS information
void virtualFenceHandler(const std_msgs::Float32MultiArray &message);               //Used to set an invisible boundary for robots to keep them from traveling outside specific bounds
void manualWaypointHandler(const swarmie_msgs::Waypoint &message);                  //Receives a waypoint (from GUI) and sets the coordinates
void behaviourStateMachine(const ros::TimerEvent &);                                //Upper most state machine, calls logic controller to perform all actions
void publishStatusTimerEventHandler(const ros::TimerEvent &event);                  //Publishes "ONLINE" when rover is successfully connected
void publishHeartBeatTimerEventHandler(const ros::TimerEvent &event);
void sonarHandler(const sensor_msgs::Range::ConstPtr &sonarLeft, const sensor_msgs::Range::ConstPtr &sonarCenter, const sensor_msgs::Range::ConstPtr &sonarRight); //handles ultrasound data and stores data

//EDIT

//** Communication
ros::Publisher idPublish;
ros::Subscriber idSubscriber;
ros::Publisher idFlagPublisher;
ros::Subscriber idFlagSubscriber;
ros::Publisher coordPub;
ros::Subscriber coordSub;
ros::Publisher initLocPub;
ros::Subscriber initLocSub;
ros::Publisher localIDPublisher;
ros::Subscriber localIDSubscriber;
ros::Publisher leadRobotPublisher;
ros::Subscriber leadRobotSubscriber;

std_msgs::UInt8 idcount;
std_msgs::Bool ifUseFlag;
std_msgs::Bool leadRobotFlagPub;
std_msgs::Float64MultiArray coordArray;
std_msgs::Float64MultiArray initLoc;
std_msgs::Int64MultiArray newIds;

void idHandler(const std_msgs::UInt8::ConstPtr &message);
void idFlagHandler(const std_msgs::Bool::ConstPtr &message);
void leadFlagHandler(const std_msgs::Bool::ConstPtr &message);
void coordHandler(const std_msgs::Float64MultiArray &message);
void initialCoord(const std_msgs::Float64MultiArray &message);
void localIDHandler(const std_msgs::Int64MultiArray &message);

bool idGlobalFlag;
bool leadRobotFlag;
vector<Point> initialLocations;
//**

//** Other
int waitTime;
bool doIHaveID = false;
auto start_time = chrono::high_resolution_clock::now();
auto end_time = chrono::high_resolution_clock::now();
auto timeWaited = chrono::duration_cast<chrono::microseconds>(end_time - start_time).count();
Point baseLocation;
bool clusterClose = false;
int idCounter = 0;
vector<Point> startPoints;
vector<float> startPointsDistance;
vector<int> startPointRearranged;

vector<vector<float>> distancesFromPoints;
vector<bool> idTaken;

int mapSize;
float ghostWall;
float triangleSquare;

void setStartPoints();
bool totalIDsChanged(int);
Point pointBuilder(float, float);

int tempTotal = 0;
int counterForRobots;
bool localIDFlag = true;
int recenterCounter = 0;
bool firstOffSet = true;
bool enoughIDs = false;
//**

//

// Converts the time passed as reported by ROS (which takes Gazebo simulation rate into account) into milliseconds as an integer.
long int getROSTimeInMilliSecs();

int main(int argc, char **argv)
{

  gethostname(host, sizeof(host));
  string hostname(host);

  if (argc >= 2)
  {
    publishedName = argv[1];
    cout << "Welcome to the world of tomorrow " << publishedName
         << "!  Behaviour turnDirectionule started." << endl;
  }
  else
  {
    publishedName = hostname;
    cout << "No Name Selected. Default is: " << publishedName << endl;
  }

  // NoSignalHandler so we can catch SIGINT ourselves and shutdown the node
  ros::init(argc, argv, (publishedName + "_BEHAVIOUR"), ros::init_options::NoSigintHandler);
  ros::NodeHandle mNH;

  // Register the SIGINT event handler so the node can shutdown properly
  signal(SIGINT, sigintEventHandler);

  joySubscriber = mNH.subscribe((publishedName + "/joystick"), 10, joyCmdHandler);
  modeSubscriber = mNH.subscribe((publishedName + "/mode"), 1, modeHandler);
  targetSubscriber = mNH.subscribe((publishedName + "/targets"), 10, targetHandler);
  odometrySubscriber = mNH.subscribe((publishedName + "/odom/filtered"), 10, odometryHandler);
  mapSubscriber = mNH.subscribe((publishedName + "/odom/ekf"), 10, mapHandler);
  virtualFenceSubscriber = mNH.subscribe(("/virtualFence"), 10, virtualFenceHandler);
  manualWaypointSubscriber = mNH.subscribe((publishedName + "/waypoints/cmd"), 10, manualWaypointHandler);
  message_filters::Subscriber<sensor_msgs::Range> sonarLeftSubscriber(mNH, (publishedName + "/sonarLeft"), 10);
  message_filters::Subscriber<sensor_msgs::Range> sonarCenterSubscriber(mNH, (publishedName + "/sonarCenter"), 10);
  message_filters::Subscriber<sensor_msgs::Range> sonarRightSubscriber(mNH, (publishedName + "/sonarRight"), 10);

  // Edit
  idSubscriber = mNH.subscribe(("global/id"), 1, idHandler);
  idPublish = mNH.advertise<std_msgs::UInt8>(("global/id"), 1);

  idFlagSubscriber = mNH.subscribe(("global/idFlag"), 1, idFlagHandler);
  idFlagPublisher = mNH.advertise<std_msgs::Bool>(("global/idFlag"), 1);

  coordSub = mNH.subscribe(("/coordenates"), 10, coordHandler);
  coordPub = mNH.advertise<std_msgs::Float64MultiArray>("/coordenates", 50);

  initLocSub = mNH.subscribe(("/initialLocations"), 10, initialCoord);
  initLocPub = mNH.advertise<std_msgs::Float64MultiArray>("/initialLocations", 50);

  localIDSubscriber = mNH.subscribe(("/localIDs"), 10, localIDHandler);
  localIDPublisher = mNH.advertise<std_msgs::Int64MultiArray>("/localIDs", 50);

  leadRobotSubscriber = mNH.subscribe(("/leadFlag"), 1, leadFlagHandler);
  leadRobotPublisher = mNH.advertise<std_msgs::Bool>(("/leadFlag"), 1);

  //

  //=========Cluster Public and subscribe initialization=========||

  clusterPointSubs = mNH.subscribe(("/clusterCoord"), 50, clusterHandler);
  clusterPointPub = mNH.advertise<std_msgs::Float64MultiArray>("/clusterCoord", 50);

  //=============================================================||
  //

  status_publisher = mNH.advertise<std_msgs::String>((publishedName + "/status"), 1, true);
  stateMachinePublish = mNH.advertise<std_msgs::String>((publishedName + "/state_machine"), 1, true);
  fingerAnglePublish = mNH.advertise<std_msgs::Float32>((publishedName + "/fingerAngle/cmd"), 1, true);
  wristAnglePublish = mNH.advertise<std_msgs::Float32>((publishedName + "/wristAngle/cmd"), 1, true);
  infoLogPublisher = mNH.advertise<std_msgs::String>("/infoLog", 1, true);
  driveControlPublish = mNH.advertise<geometry_msgs::Twist>((publishedName + "/driveControl"), 10);
  heartbeatPublisher = mNH.advertise<std_msgs::String>((publishedName + "/behaviour/heartbeat"), 1, true);
  waypointFeedbackPublisher = mNH.advertise<swarmie_msgs::Waypoint>((publishedName + "/waypoints"), 1, true);

  //publishers
  status_publisher = mNH.advertise<std_msgs::String>((publishedName + "/status"), 1, true);                   //publishes rover status
  stateMachinePublish = mNH.advertise<std_msgs::String>((publishedName + "/state_machine"), 1, true);         //publishes state machine status
  fingerAnglePublish = mNH.advertise<std_msgs::Float32>((publishedName + "/fingerAngle/cmd"), 1, true);       //publishes gripper angle to move gripper finger
  wristAnglePublish = mNH.advertise<std_msgs::Float32>((publishedName + "/wristAngle/cmd"), 1, true);         //publishes wrist angle to move wrist
  infoLogPublisher = mNH.advertise<std_msgs::String>("/infoLog", 1, true);                                    //publishes a message to the infolog box on GUI
  driveControlPublish = mNH.advertise<geometry_msgs::Twist>((publishedName + "/driveControl"), 10);           //publishes motor commands to the motors
  heartbeatPublisher = mNH.advertise<std_msgs::String>((publishedName + "/behaviour/heartbeat"), 1, true);    //publishes ROSAdapters status via its "heartbeat"
  waypointFeedbackPublisher = mNH.advertise<swarmie_msgs::Waypoint>((publishedName + "/waypoints"), 1, true); //publishes a waypoint to travel to if the rover is given a waypoint in manual mode

  //timers
  publish_status_timer = mNH.createTimer(ros::Duration(status_publish_interval), publishStatusTimerEventHandler);
  stateMachineTimer = mNH.createTimer(ros::Duration(behaviourLoopTimeStep), behaviourStateMachine);

  publish_heartbeat_timer = mNH.createTimer(ros::Duration(heartbeat_publish_interval), publishHeartBeatTimerEventHandler);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Range, sensor_msgs::Range, sensor_msgs::Range> sonarSyncPolicy;

  message_filters::Synchronizer<sonarSyncPolicy> sonarSync(sonarSyncPolicy(10), sonarLeftSubscriber, sonarCenterSubscriber, sonarRightSubscriber);
  sonarSync.registerCallback(boost::bind(&sonarHandler, _1, _2, _3));

  tfListener = new tf::TransformListener();
  std_msgs::String msg;
  msg.data = "Log Started";
  infoLogPublisher.publish(msg);

  stringstream ss;
  ss << "Rover start delay set to " << startDelayInSeconds << " seconds";
  msg.data = ss.str();
  infoLogPublisher.publish(msg);

  if (currentMode != 2 && currentMode != 3)
  {
    // ensure the logic controller starts in the correct mode.
    logicController.SetModeManual();
  }

  timerStartTime = time(0);

  ros::spin();

  return EXIT_SUCCESS;
}

// This is the top-most logic control block organised as a state machine.
// This function calls the dropOff, pickUp, and search controllers.
// This block passes the goal location to the proportional-integral-derivative
// controllers in the abridge package.
void behaviourStateMachine(const ros::TimerEvent &)
{

  std_msgs::String stateMachineMsg;

  // time since timerStartTime was set to current time
  timerTimeElapsed = time(0) - timerStartTime;

  // init code goes here. (code that runs only once at start of
  // auto mode but wont work in main goes here)

  if (!initilized)
  {

    if (timerTimeElapsed > startDelayInSeconds)
    {
      cout << "Initializing Robots....." << endl;
      cout << "Please ensure that all rovers are deployed at the same time." << endl;
      cout << "This will ensure a sucessful communication and rover ID distribution." << endl;
      // initialization has run
      initilized = true;
      //TODO: this just sets center to 0 over and over and needs to change
      Point centerOdom;
      centerOdom.x = 1.3 * cos(currentLocation.theta);
      centerOdom.y = 1.3 * sin(currentLocation.theta);
      centerOdom.theta = centerLocation.theta;
      // EDIT
      baseLocation = centerOdom;
      //
      logicController.SetCenterLocationOdom(centerOdom);

      Point centerMap;
      centerMap.x = currentLocationMap.x + (1.3 * cos(currentLocationMap.theta));
      centerMap.y = currentLocationMap.y + (1.3 * sin(currentLocationMap.theta));
      centerMap.theta = centerLocationMap.theta;
      logicController.SetCenterLocationMap(centerMap);

      centerLocationMap.x = centerMap.x;
      centerLocationMap.y = centerMap.y;

      centerLocationOdom.x = centerOdom.x;
      centerLocationOdom.y = centerOdom.y;

      startTime = getROSTimeInMilliSecs();

      idcount.data = 0;
      ifUseFlag.data = false;
      waitTime = 5;   //rng->uniformReal(1, 100); // Save random number from 1 to 100
      waitTime *= 10; // multiply random number by 10 to get how much time in milliseconds will be spent wating for turn
      start_time = chrono::high_resolution_clock::now();
      while (!doIHaveID)
      {
        end_time = chrono::high_resolution_clock::now();
        timeWaited = chrono::duration_cast<chrono::microseconds>(end_time - start_time).count();

        if (timeWaited >= waitTime)
        {
          // try taking ID from master

          if (!idGlobalFlag)
          {
            // Turn semaphore ON
            ifUseFlag.data = true;
            idFlagPublisher.publish(ifUseFlag);

            // Get and Increment ID
            idcount.data = ++logicController.totalIds;
            idPublish.publish(idcount);

            // Save ID
            logicController.myIdPub = logicController.totalIds;

            // Publish Initial location
            initLoc.data.push_back(currentLocation.x - baseLocation.x);
            initLoc.data.push_back(currentLocation.y - baseLocation.y);

            Point temp;
            temp.x = initLoc.data.at(0);
            temp.y = initLoc.data.at(1);

            initialLocations.push_back(temp);

            // Turn semaphore OFF
            ifUseFlag.data = false;
            idFlagPublisher.publish(ifUseFlag);

            if (logicController.myIdPub == 1)
            {
              leadRobotFlagPub.data = true;
              leadRobotPublisher.publish(leadRobotFlagPub);

              counterForRobots = 0;
            }
            else
            {
              initLocPub.publish(initLoc);
            }

            // Rover has ID
            doIHaveID = true;
          }
        }

        if (timeWaited == 1000)
        {
          //reset time counter
          start_time = chrono::high_resolution_clock::now();
        }
      }
      tempTotal = logicController.totalIds;
    }

    else
    {
      return;
    }
  }


  if (localIDFlag)
  {

    //Set starting points on vector
    setStartPoints();

    // Set robot task by where their staring location
    if (logicController.myIdPub == 1 && (logicController.totalIds == 3 || logicController.totalIds == 6))
    {
      cout << "ROS Verify" << endl;
      counterForRobots++;
      float distance;
      // Just in case a new robot connects too late
      // Reason: If a new rover connects then finished vectors
      // will be missing one or more distances
      if (counterForRobots >= 1) // Init
      {
        tempTotal = logicController.totalIds;
        newIds.data.resize(tempTotal, 0);

        //localIDPublisher.publish(newIds);

        vector<float> tempVector;
        distancesFromPoints.resize(tempTotal, tempVector);

        idTaken.resize(tempTotal, false);
      }

      // Reset Counter if a new robot connects to ROS
      if (totalIDsChanged(tempTotal))
      {
        counterForRobots = 0;
      }

      if (counterForRobots >= 100 || (logicController.totalIds == 6 && counterForRobots != 0)) // After 1 second
      {
        // Create 2D vector for each ID with all distances to all points
        // Iterate through IDs

        vector<float> pointDistances;
        //vector<int> idByDistances;
        //Iterate through start Points with amount of IDs available
        for (int i = 0; i < tempTotal; i++)
        {
          // Iterate through ID init Locations
          for (int j = 0; j < tempTotal; j++)
          {
            distance = hypot(startPoints.at(i).x - initialLocations.at(j).x, startPoints.at(i).y - initialLocations.at(j).y);
            pointDistances.push_back(distance);
          }

          float closestDistance = 1000000;
          int closestID = -1; // -1 means no ID has taken point

          //Assign ID to closest Point
          for (int j = 0; j < tempTotal; j++)
          {
            if (pointDistances.at(j) < closestDistance)
            {
              if (!idTaken.at(j))
              {
                // If point is taken
                if (closestID != -1)
                {
                  // Reset previous ID to not taken
                  idTaken.at(closestID) = false; // This is old ID
                }

                // Save all values to new ID and set it as taken
                closestDistance = pointDistances.at(j);
                closestID = j;
                idTaken.at(closestID) = true; // This is new ID
                //}
              }
            }
          }
          newIds.data.at(i) = closestID;

          pointDistances.clear();
        }
        reverse(newIds.data.begin(), newIds.data.end());

        // Add 1 to match public IDs
        for (int i = 0; i < newIds.data.size(); i++)
        {
          newIds.data.at(i) = newIds.data.at(i) + 1;
        }

        localIDPublisher.publish(newIds);

        leadRobotFlagPub.data = false;
        leadRobotPublisher.publish(leadRobotFlagPub);
        localIDFlag = false;
      }
    
    }
      
      else if (logicController.myIdPub == 1)
      {
        // Wait for more robots
        cout << "Waiting for more robots" << endl;
        //counterForRobots++;
        cout << "Counter: " << counterForRobots <<endl;
      }
      else if(leadRobotFlag) {
         // Wait until lead robot assigns ID
         cout << "Waiting for lead to finish" << endl;
      }
      else if(logicController.myIdLoc == 0){
        // Logic so robot takes point closest available point
        cout << "Have no ID" << endl;
        
      }
      else {
        cout << "Default" << endl;
        localIDFlag = false;
      }

      // Clear vector for next iteration
      startPoints.clear();
      cout << "Rover Local ID = " << logicController.myIdLoc << endl;

  }

  // Robot is in automode
  else if (currentMode == 2 || currentMode == 3)
  {
    //cout << "Started Auto Mode" << endl;
  cout << "Total Ids" << logicController.totalIds << endl;
    humanTime();

    // EDIT
    // Update communication data on Controllers
    logicController.UpdateData();

    //

    //update the time used by all the controllers
    logicController.SetCurrentTimeInMilliSecs(getROSTimeInMilliSecs());

    //update center location
    logicController.SetCenterLocationOdom(updateCenterLocation());

    //ask logic controller for the next set of actuator commands
    result = logicController.DoWork();

    // EDIT
    if (logicController.getVisitedFlag())
    {
      // Do not publish visited point until search controller sets a new point
      logicController.setVisitedFlag(false);

      coordArray.data.push_back(logicController.getVisitedPoint().x);
      coordArray.data.push_back(logicController.getVisitedPoint().y);

      coordPub.publish(coordArray);

      coordArray.data.clear();
    }
    //

    bool wait = false;

    //if a wait behaviour is thrown sit and do nothing untill logicController is ready
    if (result.type == behavior)
    {
      if (result.b == wait)
      {
        wait = true;
      }
    }

    //do this when wait behaviour happens
    if (wait)
    {
      sendDriveCommand(0.0, 0.0);
      std_msgs::Float32 angle;

      angle.data = prevFinger;
      fingerAnglePublish.publish(angle);
      angle.data = prevWrist;
      wristAnglePublish.publish(angle);
    }

    //normally interpret logic controllers actuator commands and deceminate them over the appropriate ROS topics
    else
    {

      sendDriveCommand(result.pd.left, result.pd.right);

      //Alter finger and wrist angle is told to reset with last stored value if currently has -1 value
      std_msgs::Float32 angle;
      if (result.fingerAngle != -1)
      {
        angle.data = result.fingerAngle;   //uses results struct with data sent back from logic controller to get angle data
        fingerAnglePublish.publish(angle); //publish angle data to the gripper fingers
        prevFinger = result.fingerAngle;   //store the last known gripper finger angle
      }

      if (result.wristAngle != -1)
      {
        angle.data = result.wristAngle;   //uses results struct with data sent back from logic controller to get angle data
        wristAnglePublish.publish(angle); //publish angle data to the gripper wrist
        prevWrist = result.wristAngle;    //store the last known gripper wrist angle
      }
    }

    //publishHandeling here
    //logicController.getPublishData(); suggested

    //adds a blank space between sets of debugging data to easily tell one tick from the next
    cout << endl;
  }

  // mode is NOT auto
  else //manual mode
  {
    humanTime();

    //cout << "centerOdom (" << centerOdom.x << ", " << centerOdom.y << ")" << endl;
    //cout << "centerMap (" << centerMap.x << ", " << centerMap.y << ")" << endl;

    //cout << "Current Location Odom (" << currentLocation.x - baseLocation.x << ", " << currentLocation.y - baseLocation.y << ")" << endl;
    //cout << "Current Location Map (" << currentLocation.x - 1.51278 << ", " << currentLocation.y - -0.362635 << ")" << endl;

    logicController.SetCurrentTimeInMilliSecs(getROSTimeInMilliSecs());

    // publish current state for the operator to see
    stateMachineMsg.data = "WAITING";

    // ask the logicController to get the waypoints that have been
    // reached.
    std::vector<int> cleared_waypoints = logicController.GetClearedWaypoints();

    for (std::vector<int>::iterator it = cleared_waypoints.begin();
         it != cleared_waypoints.end(); it++)
    {
      swarmie_msgs::Waypoint wpt;
      wpt.action = swarmie_msgs::Waypoint::ACTION_REACHED;
      wpt.id = *it;
      waypointFeedbackPublisher.publish(wpt);
    }
    result = logicController.DoWork(); //ask logic controller to run
    if (result.type != behavior || result.b != wait)
    {
      // if the logic controller requested that the robot drive, then
      // drive. Otherwise there are no manual waypoints and the robot
      // should sit idle. (ie. only drive according to joystick
      // input).
      sendDriveCommand(result.pd.left, result.pd.right);
    }
  }

  // publish state machine string for user, only if it has changed, though
  if (strcmp(stateMachineMsg.data.c_str(), prev_state_machine) != 0)
  {
    stateMachinePublish.publish(stateMachineMsg);
    sprintf(prev_state_machine, "%s", stateMachineMsg.data.c_str());
  }
}

void sendDriveCommand(double left, double right)
{
  velocity.linear.x = left,
  velocity.angular.z = right;

  // publish the drive commands
  driveControlPublish.publish(velocity);
}

bool totalIDsChanged(int previousTotal)
{
  bool diffAmount;

  if (previousTotal != logicController.totalIds)
  {
    diffAmount = true;
  }
  else
  {
    diffAmount = false;
  }
  return diffAmount;
}

void setStartPoints()
{
  if (logicController.totalIds <= 3)
  {
    mapSize = 14; //15mts by 15mts map size.
  }
  else
  {
    mapSize = 21; //22mts by 22mts map size.
  }

  ghostWall = .8; //Variable to evade walls.

  if (mapSize == 14)
  {
    triangleSquare = 8.5; // 10x10mts area for triangle square.(5mts each side)
  }
  else // Semi/Finals
  {
    // Set area
    triangleSquare = 16;
  }

  if (logicController.totalIds <= 3)
  {
    startPoints.push_back(pointBuilder(1.0, 0.5));
    startPoints.push_back(pointBuilder(-1.0, -0.5));
    startPoints.push_back(pointBuilder(mapSize / 2 - ghostWall, mapSize / 2 - ghostWall));
  }
  if (logicController.totalIds >= 4)
  {
    startPoints.push_back(pointBuilder(1.0, 0.5));
    startPoints.push_back(pointBuilder(-1.0, -0.5));
    startPoints.push_back(pointBuilder(mapSize / 2 - ghostWall, mapSize / 2 - ghostWall));
    startPoints.push_back(pointBuilder((mapSize / 2 - ghostWall) * -1, triangleSquare / 2));
    startPoints.push_back(pointBuilder((mapSize / 2 - ghostWall) * -1, (mapSize / 2 - ghostWall) * -1));
    startPoints.push_back(pointBuilder(mapSize / 2 - ghostWall, (mapSize / 2 - ghostWall) * -1));
  }
  reverse(startPoints.begin(), startPoints.end());
}

Point pointBuilder(float x, float y)
{
  Point newPoint;

  newPoint.x = x;
  newPoint.y = y;

  return newPoint;
}

/*************************
 * ROS CALLBACK HANDLERS *
 *************************/

void targetHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr &message)
{

  // Don't pass April tag data to the logic controller if the robot is not in autonomous mode.
  // This is to make sure autonomous behaviours are not triggered while the rover is in manual mode.
  if (currentMode == 0 || currentMode == 1)
  {
    return;
  }

  if (message->detections.size() > 0)
  {
    vector<Tag> tags;

    if (message->detections.size() > 2)
    {
      //std::cout << "There is a cluster. tag.size: " << message->detections.size() << '\n';
      //realCluster == true;
    }
    else
    {
      //std::cout << "Simple Tag" << '\n';
    }

    for (int i = 0; i < message->detections.size(); i++)
    {

      // Package up the ROS AprilTag data into our own type that does not rely on ROS.
      Tag loc;
      loc.setID(message->detections[i].id);

      // Pass the position of the AprilTag
      geometry_msgs::PoseStamped tagPose = message->detections[i].pose;

      loc.setPosition(make_tuple(tagPose.pose.position.x,
                                 tagPose.pose.position.y,
                                 tagPose.pose.position.z));

      // Pass the orientation of the AprilTag
      loc.setOrientation(::boost::math::quaternion<float>(tagPose.pose.orientation.x,
                                                          tagPose.pose.orientation.y,
                                                          tagPose.pose.orientation.z,
                                                          tagPose.pose.orientation.w));
      tags.push_back(loc);
    }

    if (message->detections.size() > 5)
    {
      bool anyBaseTags = false;

      // Verify if any tag detected is a Base tag, if so don't publish point
      for (int i = 0; i < tags.size(); i++)
      {
        if (tags.at(i).getID() == 256)
        {
          anyBaseTags = true;
          break;
        }
        else
        {
          anyBaseTags = false;
        }
      }

      if (!anyBaseTags)
      { // If no base tags then publish location

        clusterCoordArray.data.push_back((0.5 * cos(currentLocation.theta) + currentLocation.x - baseLocation.x));
        clusterCoordArray.data.push_back((0.5 * sin(currentLocation.theta) + currentLocation.y - baseLocation.y));
        clusterPointPub.publish(clusterCoordArray);
      }
    }

    logicController.SetAprilTags(tags);
  }
}

void modeHandler(const std_msgs::UInt8::ConstPtr &message)
{
  currentMode = message->data;
  if (currentMode == 2 || currentMode == 3)
  {
    logicController.SetModeAuto();
  }
  else
  {
    logicController.SetModeManual();
  }
  sendDriveCommand(0.0, 0.0);
}

void sonarHandler(const sensor_msgs::Range::ConstPtr &sonarLeft, const sensor_msgs::Range::ConstPtr &sonarCenter, const sensor_msgs::Range::ConstPtr &sonarRight)
{

  logicController.SetSonarData(sonarLeft->range, sonarCenter->range, sonarRight->range);
}

void odometryHandler(const nav_msgs::Odometry::ConstPtr &message)
{
  //Get (x,y) location directly from pose
  currentLocation.x = message->pose.pose.position.x;
  currentLocation.y = message->pose.pose.position.y;

  //Get theta rotation by converting quaternion orientation to pitch/roll/yaw
  tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z, message->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  currentLocation.theta = yaw;

  linearVelocity = message->twist.twist.linear.x;
  angularVelocity = message->twist.twist.angular.z;

  // EDIT
  // included - baselocation to both x and y
  Point currentLoc;
  currentLoc.x = currentLocation.x - baseLocation.x;
  currentLoc.y = currentLocation.y - baseLocation.y;
  currentLoc.theta = currentLocation.theta;
  logicController.SetPositionData(currentLoc);
  logicController.SetVelocityData(linearVelocity, angularVelocity);
}

// Allows a virtual fence to be defined and enabled or disabled through ROS
void virtualFenceHandler(const std_msgs::Float32MultiArray &message)
{
  // Read data from the message array
  // The first element is an integer indicating the shape type
  // 0 = Disable the virtual fence
  // 1 = circle
  // 2 = rectangle
  int shape_type = static_cast<int>(message.data[0]); // Shape type

  if (shape_type == 0)
  {
    logicController.setVirtualFenceOff();
  }
  else
  {
    // Elements 2 and 3 are the x and y coordinates of the range center
    Point center;
    center.x = message.data[1]; // Range center x
    center.y = message.data[2]; // Range center y

    // If the shape type is "circle" then element 4 is the radius, if rectangle then width
    switch (shape_type)
    {
    case 1: // Circle
    {
      if (message.data.size() != 4)
        throw ROSAdapterRangeShapeInvalidTypeException("Wrong number of parameters for circle shape type in ROSAdapter.cpp:virtualFenceHandler()");
      float radius = message.data[3];
      logicController.setVirtualFenceOn(new RangeCircle(center, radius));
      break;
    }
    case 2: // Rectangle
    {
      if (message.data.size() != 5)
        throw ROSAdapterRangeShapeInvalidTypeException("Wrong number of parameters for rectangle shape type in ROSAdapter.cpp:virtualFenceHandler()");
      float width = message.data[3];
      float height = message.data[4];
      logicController.setVirtualFenceOn(new RangeRectangle(center, width, height));
      break;
    }
    default:
    { // Unknown shape type specified
      throw ROSAdapterRangeShapeInvalidTypeException("Unknown Shape type in ROSAdapter.cpp:virtualFenceHandler()");
    }
    }
  }
}

void mapHandler(const nav_msgs::Odometry::ConstPtr &message)
{
  //Get (x,y) location directly from pose
  currentLocationMap.x = message->pose.pose.position.x;
  currentLocationMap.y = message->pose.pose.position.y;

  //Get theta rotation by converting quaternion orientation to pitch/roll/yaw
  tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z, message->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  currentLocationMap.theta = yaw;

  linearVelocity = message->twist.twist.linear.x;
  angularVelocity = message->twist.twist.angular.z;

  Point curr_loc;
  curr_loc.x = currentLocationMap.x - centerLocationOdom.x; //baseLocation.x;
  curr_loc.y = currentLocationMap.y - centerLocationOdom.y; //baseLocation.y;
  curr_loc.theta = currentLocationMap.theta;
  logicController.SetMapPositionData(curr_loc);
  logicController.SetMapVelocityData(linearVelocity, angularVelocity);
}

void joyCmdHandler(const sensor_msgs::Joy::ConstPtr &message)
{
  const int max_motor_cmd = 255;
  if (currentMode == 0 || currentMode == 1)
  { //takes data coming from joystick and stores into linear and angular variables
    float linear = abs(message->axes[4]) >= 0.1 ? message->axes[4] * max_motor_cmd : 0.0;
    float angular = abs(message->axes[3]) >= 0.1 ? message->axes[3] * max_motor_cmd : 0.0;

    float left = linear - angular;
    float right = linear + angular;
    //check to see if commands exceed MAX values, and if so set them to hard coded MAX value
    if (left > max_motor_cmd)
    {
      left = max_motor_cmd;
    }
    else if (left < -max_motor_cmd)
    {
      left = -max_motor_cmd;
    }

    if (right > max_motor_cmd)
    {
      right = max_motor_cmd;
    }
    else if (right < -max_motor_cmd)
    {
      right = -max_motor_cmd;
    }

    sendDriveCommand(left, right);
  }
}

void publishStatusTimerEventHandler(const ros::TimerEvent &)
{
  std_msgs::String msg;
  msg.data = "UIPR-Bayamón";
  status_publisher.publish(msg);
}

void manualWaypointHandler(const swarmie_msgs::Waypoint &message)
{
  Point wp;
  wp.x = message.x;
  wp.y = message.y;
  wp.theta = 0.0;
  switch (message.action)
  {
  case swarmie_msgs::Waypoint::ACTION_ADD:
    logicController.AddManualWaypoint(wp, message.id);
    break;
  case swarmie_msgs::Waypoint::ACTION_REMOVE:
    logicController.RemoveManualWaypoint(message.id);
    break;
  }
}

void sigintEventHandler(int sig)
{
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

void publishHeartBeatTimerEventHandler(const ros::TimerEvent &)
{
  std_msgs::String msg;
  msg.data = "";
  heartbeatPublisher.publish(msg);
}

long int getROSTimeInMilliSecs()
{
  // Get the current time according to ROS (will be zero for simulated clock until the first time message is recieved).
  ros::Time t = ros::Time::now();

  // Convert from seconds and nanoseconds to milliseconds.
  return t.sec * 1e3 + t.nsec / 1e6;
}

Point updateCenterLocation()
{
  transformMapCentertoOdom();

  Point tmp;

  tmp.x = centerLocationOdom.x - baseLocation.x; // - baseLocation.x;
  tmp.y = centerLocationOdom.y - baseLocation.y; //- baseLocation.y;

  recenterCounter++;

  if (firstOffSet || recenterCounter == 600)
  {
    firstOffSet = false;
    recenterCounter = 0;
    baseLocation.x = centerLocationOdom.x;
    baseLocation.y = centerLocationOdom.y;
  }
  return tmp;
}

void transformMapCentertoOdom()
{

  // map frame
  geometry_msgs::PoseStamped mapPose;

  // setup msg to represent the center location in map frame
  mapPose.header.stamp = ros::Time::now();

  mapPose.header.frame_id = publishedName + "/map";
  mapPose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, centerLocationMap.theta);
  mapPose.pose.position.x = centerLocationMap.x;
  mapPose.pose.position.y = centerLocationMap.y;
  geometry_msgs::PoseStamped odomPose;
  string x = "";

  try
  { //attempt to get the transform of the center point in map frame to odom frame.
    tfListener->waitForTransform(publishedName + "/map", publishedName + "/odom", ros::Time::now(), ros::Duration(1.0));
    tfListener->transformPose(publishedName + "/odom", mapPose, odomPose);
  }

  catch (tf::TransformException &ex)
  {
    ROS_INFO("Received an exception trying to transform a point from \"map\" to \"odom\": %s", ex.what());
    x = "Exception thrown " + (string)ex.what();
    std_msgs::String msg;
    stringstream ss;
    ss << "Exception in mapAverage() " + (string)ex.what();
    msg.data = ss.str();
    infoLogPublisher.publish(msg);
    cout << msg.data << endl;
  }

  // Use the position and orientation provided by the ros transform.
  centerLocationMapRef.x = odomPose.pose.position.x; //set centerLocation in odom frame
  centerLocationMapRef.y = odomPose.pose.position.y;

  // cout << "x ref : "<< centerLocationMapRef.x << " y ref : " << centerLocationMapRef.y << endl;

  float xdiff = centerLocationMapRef.x - centerLocationOdom.x;
  float ydiff = centerLocationMapRef.y - centerLocationOdom.y;

  float diff = hypot(xdiff, ydiff);

  if (diff > drift_tolerance)
  {
    centerLocationOdom.x += xdiff / diff; //adjust X
    centerLocationOdom.y += ydiff / diff; //adjust Y
  }

  //cout << "center x diff : " << centerLocationMapRef.x - centerLocationOdom.x << " center y diff : " << centerLocationMapRef.y - centerLocationOdom.y << endl;
  //cout << hypot(centerLocationMapRef.x - centerLocationOdom.x, centerLocationMapRef.y - centerLocationOdom.y) << endl;
}

void humanTime()
{

  float timeDiff = (getROSTimeInMilliSecs() - startTime) / 1e3;
  if (timeDiff >= 60)
  {
    minutesTime++;
    startTime += 60 * 1e3;
    if (minutesTime >= 60)
    {
      hoursTime++;
      minutesTime -= 60;
    }
  }
  timeDiff = floor(timeDiff * 10) / 10;

  double intP, frac;
  frac = modf(timeDiff, &intP);
  timeDiff -= frac;
  frac = round(frac * 10);
  if (frac > 9)
  {
    frac = 0;
  }

  //cout << "System has been Running for :: " << hoursTime << " : hours " << minutesTime << " : minutes " << timeDiff << "." << frac << " : seconds" << endl; //you can remove or comment this out it just gives indication something is happening to the log file
}

// EDIT
void idHandler(const std_msgs::UInt8::ConstPtr &message)
{
  logicController.totalIds = message->data;
}

void idFlagHandler(const std_msgs::Bool::ConstPtr &message)
{
  idGlobalFlag = message->data;
}

void coordHandler(const std_msgs::Float64MultiArray &message)
{
  Point tempPoint;
  tempPoint.x = message.data[0];
  tempPoint.y = message.data[1];

  logicController.visitedPoints.push_back(tempPoint);

  //printf("%d\n",element1 );
}

void clusterHandler(const std_msgs::Float64MultiArray &message)
{

  bool samePoint;
  float distanceToCluster = 0;
  Point clusterPoint;
  clusterPoint.x = message.data[0];
  clusterPoint.y = message.data[1];

  for (int i = 0; i < logicController.clusterPoints.size(); i++)
  {

    if (!logicController.clusterPoints.empty())
    {
      //cout << "Cluster Point: (" << logicController.clusterPoints.at(i).x << "," << logicController.clusterPoints.at(i).y << ")" << endl;
      distanceToCluster = hypot(logicController.clusterPoints.at(i).x - currentLocation.x, logicController.clusterPoints.at(i).y - currentLocation.y);
      //cout << "Distance to Cluster is: " << distanceToCluster << endl;
    }
    if (distanceToCluster < 3)
    {
      //std::cout << "It's close to the same cluster." << '\n';
      clusterClose = true;
      break;
    }

    else
    {
      //std::cout << "Not close" << '\n';
      clusterClose = false;
    }
  }

  if (clusterClose)
  {
    //Only add if vector is empty
    if (logicController.clusterPoints.empty())
    {
      logicController.clusterPoints.push_back(clusterPoint);
    }
  }
  else
  {
    logicController.clusterPoints.push_back(clusterPoint);
  }
}

//logicController.itsACluster = true;
void initialCoord(const std_msgs::Float64MultiArray &message)
{
  Point tempPoint;

  tempPoint.x = message.data[0];
  tempPoint.y = message.data[1];
  initialLocations.push_back(tempPoint);
}

void localIDHandler(const std_msgs::Int64MultiArray &message)
{

  if (message.data.at(0) != 0)
  {
    for (int i = 0; i < message.data.size(); i++)
    {

      if ((message.data.at(i)) == logicController.myIdPub)
      {
        logicController.myIdLoc = i + 1;
        break;
      }
    }
  }
}

void leadFlagHandler(const std_msgs::Bool::ConstPtr &message)
{
  leadRobotFlag = message->data;
}
//
