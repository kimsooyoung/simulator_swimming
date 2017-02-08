#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h> //set position 용
#include <mavros_msgs/CommandBool.h> //arm용
#include <mavros_msgs/SetMode.h> //offboard 모드 설정용
#include <mavros_msgs/State.h> //mavros 메세지 활용용

float earth_x = 0.0;
float earth_y = 0.0;
float earth_z = 5.0;
float moon_x;
float moon_y;
float moon_z = 7.0;
float offset;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "start_moon_node");

    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State> ("mavros_moon/state", 10, state_cb);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped> ("mavros_moon/setpoint_position/local", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool> ("mavros_moon/cmd/arming");

    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode> ("mavros_moon/set_mode");


    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0); //period 0.05 s

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    //switch mode to OFFBOARD
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    //arming
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    
    //initial position
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = moon_x;
    pose.pose.position.y = moon_y;
    pose.pose.position.z = moon_z;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    ros::Time last_request = ros::Time::now();

    //round round
    while(ros::ok()){
		
	if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
                ROS_INFO("moon_Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("moon armed");
                }
                last_request = ros::Time::now();
            }
        }

    
	//get own position

	moon_x = earth_x;
	moon_y = earth_y + offset;
	moon_z = earth_z + 2;
	//goto own position
    	pose.pose.position.x = moon_x;
    	pose.pose.position.y = moon_y;
    	pose.pose.position.z = moon_z;
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

