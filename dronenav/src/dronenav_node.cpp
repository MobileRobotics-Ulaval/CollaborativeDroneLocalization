#include "dronenav/dronenav_node.h"

using namespace std;

DroneNav::DroneNav(ros::NodeHandle nodeHandle) : nodeHandle(nodeHandle)
{
    this->joyInitiated = false;
    this->navInitiated = false;
    string topic;
    ros::param::get("~topic", topic); // _topic:="/cobra" when rosrun node
    ros::param::get("~ctrl", this->deadManSwitch); // _ctrl:=4 or 6

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

    this->loop();
}

void DroneNav::loop()
{
    ros::Rate loopRate(50);

    this->waitingInitiation(loopRate);

    int tickCount = 0;
    int maxCount = 20000;

    while(ros::ok())
    {
        this->getDroneState(tickCount, maxCount);

        this->takeoffObserver(loopRate);
        this->landingObserver(loopRate);
        this->resetObserver(loopRate);
        this->movementObserver(loopRate);
        this->flatTrimObserver();
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

void DroneNav::createPublishers(const string& topic)
{
    this->pubTwist = this->nodeHandle.advertise<geometry_msgs::Twist>(topic + "/cmd_vel", 1);
    this->pubReset = this->nodeHandle.advertise<std_msgs::Empty>(topic + "/ardrone/reset", 1);
    this->pubTakeoff = this->nodeHandle.advertise<std_msgs::Empty>(topic + "/ardrone/takeoff", 1);
    this->pubLand = this->nodeHandle.advertise<std_msgs::Empty>(topic + "/ardrone/land", 1);
}

void DroneNav::createSubscribers(const string& topic)
{
    this->subJoy = this->nodeHandle.subscribe("/joy", 1, &DroneNav::joyCallBack, this);
    this->subNav = this->nodeHandle.subscribe(topic + "/ardrone/navdata", 1, &DroneNav::navCallback, this);
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
        const char *states[] = {"Unknown", "Initiated", "Landed", "Flying", "Hovering", "test", "6=impossible", "Flying"};

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
    if(this->button["Y"] && this->droneData.state == 2)
    {
        std_srvs::Empty emptySrv;
        this->clientFlatTrim.call(emptySrv);
        ROS_INFO("Battery = %g/100 : Flat trim calibration", this->droneData.batteryPercent);
    }
}

void DroneNav::movementObserver(ros::Rate loopRate)
{
    this->calibrateJoyAxis();
    geometry_msgs::Twist twistMsg;

    twistMsg.linear.x = this->axis["LX"] * MAX_SPEED;
    twistMsg.linear.y = this->axis["LY"] * MAX_SPEED;
    twistMsg.linear.z = this->altitudeObserver() * MAX_SPEED;
    twistMsg.angular.z = this->axis["RX"] * MAX_SPEED;
    this->pubTwist.publish(twistMsg);
    ros::spinOnce();
    loopRate.sleep();
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
