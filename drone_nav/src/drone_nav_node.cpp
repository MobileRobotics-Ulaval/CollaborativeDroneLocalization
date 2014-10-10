#include "drone_nav/drone_nav_node.h"

using namespace std;

DroneNav::DroneNav(ros::NodeHandle nodeHandle) : nodeHandle(nodeHandle)
{
    this->joyInitiated = false;
    this->navInitiated = false;
    this->poseInitiated = false;
    string topic;
    ros::param::get("~topic", topic); // _topic:="/cobra" when rosrun node
    ros::param::get("~ctrl", this->deadManSwitch); // _ctrl:=4 or 6

    int goal_radius;
    ros::param::get("~goal_radius", goal_radius); // _ctrl:=4 or 6
    this->autoCtrl.setGoalRadius(goal_radius);


    if(this->deadManSwitch > 11 || this->deadManSwitch < 0)
    {
        ROS_INFO("=== Default Parameter ===");

        this->deadManSwitch = 4; // L1
    }

    ROS_INFO("Dead man's switch button: %i", this->deadManSwitch);

    ROS_INFO("Namespace leader: %s", topic.c_str());

    this->createPublishers(topic);
    this->createSubscribers(topic);
    this->createServices(topic);
    this->setGoal();

    this->loop();
}

void DroneNav::setGoal(){
    geometry_msgs::Pose goal;
    goal.position.x = 2.625;
    this->autoCtrl.setGoal(goal);
}

void DroneNav::loop()
{
    ros::Rate loopRate(50);

    this->waitingInitiation(loopRate);

    int tickCount = 0;
    int maxCount = 20000;

    lastPoseReceived = ros::Time::now();
    while(ros::ok())
    {
       if(this->button["Y"] && (ros::Time::now() - lastPoseReceived > ros::Duration(10)))
       {
           this->pubLand.publish(std_msgs::Empty());
       }
       if(this->button["Y"] && (ros::Time::now() - lastPoseReceived > ros::Duration(1)))
       {
           this->poseInitiated = false;
           geometry_msgs::Twist level; //level our velocity
           this->pubTwist.publish(level);
       }

       this->getDroneState(tickCount, maxCount);

       this->takeoffObserver(loopRate);
       this->landingObserver(loopRate);
       this->resetObserver(loopRate);
       this->flatTrimObserver();
       this->controlObserver();

       ros::spinOnce();
       loopRate.sleep();
    }
}

void DroneNav::joyCallBack(const sensor_msgs::Joy& joyMsg)
{
    if(joyMsg.buttons[this->deadManSwitch])
    {
        this->enableJoystick(joyMsg);
    }
    else
    {
        this->disableJoystick();
    }
}

void DroneNav::navCallback(const ardrone_autonomy::Navdata& navMsg)
{
    this->droneData = navMsg;
    if(!this->navInitiated)
    {
        ROS_INFO("Navigation message initiated");

        this->navInitiated = true;
    }
}

void DroneNav::poseCallback(const geometry_msgs::PoseStamped& poseMsg)
{
    lastPoseReceived = ros::Time::now();
    this->poseData = poseMsg;
    if(!this->poseInitiated)
    {
        ROS_INFO("Pose message initiated");

        this->poseInitiated = true;
    }
}

void DroneNav::createPublishers(const string& topic)
{
    this->pubTwist = this->nodeHandle.advertise<geometry_msgs::Twist>(topic + "/cmd_vel", 1);
    this->pubReset = this->nodeHandle.advertise<std_msgs::Empty>(topic + "/ardrone/reset", 1);
    this->pubTakeoff = this->nodeHandle.advertise<std_msgs::Empty>(topic + "/ardrone/takeoff", 1);
    this->pubLand = this->nodeHandle.advertise<std_msgs::Empty>(topic + "/ardrone/land", 1);

    this->pubPath = this->nodeHandle.advertise<nav_msgs::Path>(topic + "/automous_command", 1);
    this->pubGoalMarker = this->nodeHandle.advertise<visualization_msgs::Marker>(topic + "/goal_marker", 1);
}

void DroneNav::createSubscribers(const string& topic)
{
    this->subJoy = this->nodeHandle.subscribe("/joy", 1, &DroneNav::joyCallBack, this);
    this->subNav = this->nodeHandle.subscribe(topic + "/ardrone/navdata", 1, &DroneNav::navCallback, this);
    this->subPose = this->nodeHandle.subscribe(topic + "/pose", 1, &DroneNav::poseCallback, this);
}

void DroneNav::createServices(const string& topic)
{
    this->clientFlatTrim = this->nodeHandle.serviceClient<std_srvs::Empty>(topic + "/ardrone/flattrim");
}

void DroneNav::waitingInitiation(ros::Rate loopRate)
{
    ROS_INFO("Waiting for initiation...");

    while((!this->joyInitiated || !this->navInitiated) && ros::ok())
    {
        ros::spinOnce();
        loopRate.sleep();
    }

    ROS_INFO("==== Finish initiation ====");
}

void DroneNav::getDroneState(int &tickCount,  int maxCount)
{
    tickCount++;

    if(tickCount == maxCount)
    {
        const char *states[] = {"Unknown", "Initiated", "Landed", "Flying", "Hovering", "Test?", "Take Off", "Flying", "Landing", "Looping?"};

        ROS_INFO("Battery = %g/100 state = %s", this->droneData.batteryPercent, states[this->droneData.state]);
        tickCount = 0;
    }

    if(ros::Time::now().toSec() > this->droneData.header.stamp.toSec() + 3.0)
    {
        ROS_ERROR("=== Too much delay on navigation message ===");
    }
}

void DroneNav::takeoffObserver(ros::Rate loopRate)
{
    if(this->button["A"])
    {
        while(this->droneData.state == 2 && !this->button["X"])
        {
            this->pubTakeoff.publish(std_msgs::Empty());
            ROS_INFO("Battery = %g/100 : Launching drone", this->droneData.batteryPercent);
            ros::spinOnce();
            loopRate.sleep();
        }

    }
}

void DroneNav::landingObserver(ros::Rate loopRate)
{
    if(this->button["B"])
    {
        while((this->droneData.state == 3 || this->droneData.state == 4) && !this->button["X"])
        {
            this->pubLand.publish(std_msgs::Empty());
            ROS_INFO("Battery = %g/100 : Landing drone", this->droneData.batteryPercent);
            ros::spinOnce();
            loopRate.sleep();
        }
    }
}

void DroneNav::resetObserver(ros::Rate loopRate)
{
    if(this->button["X"])
    {
        while(this->droneData.state == 0)
        {
            this->pubReset.publish(std_msgs::Empty());
            ROS_INFO("Battery = %g/100 : Reseting drone", this->droneData.batteryPercent);
            ros::spinOnce();
            loopRate.sleep();
        }
    }
}

void DroneNav::flatTrimObserver()
{
    if(this->button["Start"] && this->droneData.state == 2)
    {
        std_srvs::Empty emptySrv;
        this->clientFlatTrim.call(emptySrv);
        ROS_INFO("Battery = %g/100 : Flat trim calibration", this->droneData.batteryPercent);
    }
}

void DroneNav::controlObserver()
{
    if(this->button["Y"] && poseInitiated)
    {
        this->autonomousControlObserver();
    }
    else
    {
       this->movementObserver();
    }
}
void DroneNav::autonomousControlObserver()
{
    geometry_msgs::Twist twistMsg;
    nav_msgs::Path pathMsg;

    this->autoCtrl.generateCommand(twistMsg, pathMsg, this->poseData);
    this->pubTwist.publish(twistMsg);
    this->pubPath.publish(pathMsg);
    this->pubGoalMarker.publish(this->autoCtrl.generateGoalMarkerMessage());
}
void DroneNav::movementObserver()
{
    this->calibrateJoyAxis();
    geometry_msgs::Twist twistMsg;

    twistMsg.linear.x = this->axis["LX"] * MAX_SPEED;
    twistMsg.linear.y = this->axis["LY"] * MAX_SPEED;
    twistMsg.linear.z = this->altitudeObserver() * MAX_SPEED;
    twistMsg.angular.z = this->axis["RX"] * MAX_SPEED;
    this->pubTwist.publish(twistMsg);
}

void DroneNav::calibrateJoyAxis()
{
    if (fabs(this->axis["LX"]) < 0.01) {this->axis["LX"] = 0;}
    if (fabs(this->axis["LY"]) < 0.01) {this->axis["LY"] = 0;}
    if (fabs(this->axis["RX"]) < 0.01) {this->axis["RX"] = 0;}
    if (fabs(this->axis["RY"]) < 0.01) {this->axis["RY"] = 0;}
}

double DroneNav::altitudeObserver()
{
    if(this->button["R1"])
    {
        return 1;
    }
    else if(this->button["R2"])
    {
        return -1;
    }
    else
    {
        return 0.0;
    }
}

void DroneNav::enableJoystick(const sensor_msgs::Joy& joyMsg)
{
    if(!this->joyInitiated)
    {
        this->joyInitiated = true;
        ROS_INFO("Joystick initiated");
    }

    this->button["X"] = joyMsg.buttons[0];
    this->button["A"] = joyMsg.buttons[1];
    this->button["B"] = joyMsg.buttons[2];
    this->button["Y"] = joyMsg.buttons[3];
    this->button["R1"] = joyMsg.buttons[5];
    this->button["R2"] = joyMsg.buttons[7];
    this->button["Start"] = joyMsg.buttons[9];

    this->axis["LY"] = joyMsg.axes[0];
    this->axis["LX"] = joyMsg.axes[1];
    this->axis["RX"] = joyMsg.axes[2];
    this->axis["RY"] = joyMsg.axes[3];
}

void DroneNav::disableJoystick()
{
    for(map<string,bool>::iterator it = this->button.begin(); it != this->button.end(); ++it)
    {
        it->second = false;
    }

    for(map<string,double>::iterator it = this->axis.begin(); it != this->axis.end(); ++it)
    {
        it->second = 0.0;
    }
}
