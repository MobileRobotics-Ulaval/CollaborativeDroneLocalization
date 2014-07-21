#include "ardrone_pose_estimator/ardrone_pose_estimator_node.h"

using namespace std;

namespace ardrone_pose_estimator
{

ARPE::ARPE(ros::NodeHandle node): m_nodehandle(node), m_joyInitiated(false), m_navInitiated(false){
    string topic;
    ros::param::get("~topic", topic);
    ros::param::get("~ctrl", m_button_deadman_switch);
    ROS_INFO("Namespace leader: %s", topic.c_str());
    ROS_INFO("Dead man's switch button: %i", m_button_deadman_switch);
    if(m_button_deadman_switch == 0 || topic == ""){
        ROS_ERROR("Invalid parameter");
        exit(0);
    }

    // Dynamic reconfiguration
    dynamic_reconfigure::Server<ardrone_pose_estimator::ardrone_pose_estimatorConfig>::CallbackType m_cb;
    m_cb = boost::bind(&ARPE::dynamicParametersCallback, this, _1, _2);
    m_dr_server.setCallback(m_cb);

    // Init input and output with the ARdrone driver
    m_pub_twist = node.advertise<geometry_msgs::Twist>(topic + "/cmd_vel", 1);

    m_joy_sub = node.subscribe("/joy", 1, &ARPE::joyCallback, this);
    m_nav_sub = node.subscribe(topic + "/ardrone/navdata", 1, &ARPE::navCallback, this);

    m_pub_empty_reset   = node.advertise<std_msgs::Empty>(topic + "/ardrone/reset", 1);
    m_pub_empty_takeoff = node.advertise<std_msgs::Empty>(topic + "/ardrone/takeoff", 1);
    m_pub_empty_land    = node.advertise<std_msgs::Empty>(topic + "/ardrone/land", 1);
}

void ARPE::joyCallback(const sensor_msgs::Joy& joy_msg_in){
    // 4 = l1; 6 =l2
    if(joy_msg_in.buttons[m_button_deadman_switch]){
        m_axeLX = joy_msg_in.axes[1];
        m_axeLY = joy_msg_in.axes[0];
        m_axeRX = joy_msg_in.axes[2];
        m_axeRY = joy_msg_in.axes[3];

        m_buttonX = joy_msg_in.buttons[0];
        m_buttonA = joy_msg_in.buttons[1];
        m_buttonB = joy_msg_in.buttons[2];
        m_buttonL1 = joy_msg_in.buttons[4];
        m_buttonR1 = joy_msg_in.buttons[5];
        m_buttonR2 = joy_msg_in.buttons[7];
        //ROS_INFO("Joystick updated");
        //m_current_joy = joy_msg_in

        if(!m_joyInitiated){
            ROS_INFO("Joystick initiated");
            m_joyInitiated = true;
        }
    }
    else{
        m_axeLX = m_axeLY = m_axeRX = m_axeRY = 0.0;
        m_buttonA = false;
        m_buttonB = false;
        m_buttonX = false;
        m_buttonL1 = false;
        m_buttonR1 = false;
        m_buttonR2 = false;
    }
}

void ARPE::navCallback(const ardrone_autonomy::Navdata& msg_in)
{
    //Take in state of ardrone
    m_drone = msg_in;
    if(!m_navInitiated){
        ROS_INFO("Navigation message initiated");
        m_navInitiated = true;
    }
}



void ARPE::loop(){

    ros::Rate loop_rate(50);


    ROS_INFO("Waiting for initiation");
    //ros::spin();
    while((!m_joyInitiated || !m_navInitiated) && ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
        continue;
    }

    ROS_INFO("===== Finish initiation =====");

    const char *states[] = {"Unknown", "Inited", "Landed", "Flying", "Hovering", "test", "6=impossible", "Flying"};


    geometry_msgs::Vector3 v3_msg;
    geometry_msgs::Twist twist_msg;
    std_msgs::Empty emp_msg;
    double cmd_x, cmd_y, cmd_z, cmd_teta;
    int tick_count = 0;
    while(ros::ok()){
        tick_count++;

       if(tick_count == 9){
          ROS_INFO("Battery = %f p state = %s", m_drone.batteryPercent, states[m_drone.state]);
          tick_count = 0;
       }

       if(ros::Time::now().toSec() > m_drone.header.stamp.toSec() + 3.0){
           ROS_ERROR("Too much delay on navigation Message!!!");
       }

        if (m_buttonA){
            //  State => 0: Unknown * 1: Inited * 2: Landed * 3,7: Flying * 4: Hovering * 5: Test
            while (m_drone.state == 2 && !m_buttonX){
                ROS_INFO("%f:Launching drone", m_drone.batteryPercent);

                m_pub_empty_takeoff.publish(emp_msg); //launches the drone
                ros::spinOnce();
                loop_rate.sleep();
            }//drone take off
        }
        if (m_buttonB){
            while (m_drone.state == 3 || m_drone.state == 4 && !m_buttonX){
                ROS_INFO("%f:landing drone", m_drone.batteryPercent);

                m_pub_empty_land.publish(emp_msg); //landing the drone
                ros::spinOnce();
                loop_rate.sleep();
            }//drone land
        }
        if (m_buttonX){
            double time_start=(double) ros::Time::now().toSec();
            while (m_drone.state == 0){
                ROS_INFO("%f:Reset drone", m_drone.batteryPercent);

                m_pub_empty_reset.publish(emp_msg); //reset the drone
                ros::spinOnce();
                loop_rate.sleep();
                if((double)ros::Time::now().toSec() > time_start+3.0){
                    ROS_ERROR("Time limit reached, unable reset ardrone");
                    break; //exit loop
                }
            }//drone reset
        }
        if (fabs(m_axeLX) < 0.01) {m_axeLX = 0;}

        if (fabs(m_axeLY) < 0.01) {m_axeLY = 0;}

        if (fabs(m_axeRX) < 0.01) {m_axeRX = 0;}
        if (fabs(m_axeRY) < 0.01) {m_axeRY = 0;}

        cmd_x = m_axeLX * m_max_speed;
        cmd_y = m_axeLY * m_max_speed;
        if(m_buttonR1)
             cmd_z = m_max_speed;
        else if(m_buttonR2)
             cmd_z = -1 * m_max_speed;
        else
            cmd_z = 0;
       //cmd_z = m_axeRY * m_max_speed;
        cmd_teta = m_axeRX * m_max_speed;

        twist_msg.linear.x  = cmd_x;
        twist_msg.linear.y  = cmd_y;
        twist_msg.linear.z  = cmd_z;
        twist_msg.angular.z = cmd_teta;

       // v3_msg.x = cmd_x;
        //v3_msg.y = cmd_y;
       // v3_msg.z = cmd_z;

       // m_pub_v3.publish(v3_msg);
        m_pub_twist.publish(twist_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void ARPE::dynamicParametersCallback(ardrone_pose_estimator::ardrone_pose_estimatorConfig &config, uint32_t level){

}

} //ardrone_pose_estimator

int main(int argc, char* argv[]){

    ros::init(argc, argv,  "ardrone_pose_estimator_node", ros::init_options::AnonymousName);

    ros::NodeHandle n;
    ardrone_pose_estimator::ARPE ardrone_pose_estimator(n);
    ardrone_pose_estimator.loop();
    ros::spin();
}
