#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h> //set position 용
#include <mavros_msgs/CommandBool.h> //arm용
#include <mavros_msgs/SetMode.h> //offboard 모드 설정용
#include <mavros_msgs/State.h> //mavros 메세지 활용용
#include "math.h" //수식 입력용
#include "solar_sys_formation/msgCoordinate.h"//메세지 정의용

double r = 3;
double theta;
double count=0.0;
double wn = 1.2;
float moon_x;
float moon_y;
float moon_z = 7;
float earth_x;
float earth_y;
float earth_z;
float btw_offset = 2;
int mode = 0;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}


void msgCallback(const solar_sys_formation::msgCoordinate::ConstPtr& msg) {
	//ROS_INFO("recieve msg : %f, %f, %f", msg->earth_x, msg->earth_y, msg->earth_z);
	earth_x = msg->earth_x;
	earth_y = msg->earth_y;
	earth_z = msg->earth_z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moon_node");

    ros::NodeHandle nh;

    ros::Subscriber ros_coordinate_sub = nh.subscribe<solar_sys_formation::msgCoordinate> ("ros_coordinate_msg", 100, msgCallback);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State> ("mavros_moon/state", 10, state_cb);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped> ("mavros_moon/setpoint_position/local", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool> ("mavros_moon/cmd/arming");

    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode> ("mavros_moon/set_mode");


    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(40.0); //period 0.025 s
    

	nh.param("moon_node/mode", mode, 0);
//	nh.param("wn", wn, 1.0);
//	nh.param("r", r, 2.0);
//	nh.param("center_x", center_x, 0.0);
//	nh.param("center_y", center_y, 0.0);
//	nh.param("center_z", center_z, 0.0);


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
    
    //initial posistion
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = moon_x;
    pose.pose.position.y = moon_y;
    pose.pose.position.z = moon_z;

    //send a few setpoints before starting
    for(int i = 40; ros::ok() && i > 0; --i){
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
                ROS_INFO("moon Offboard enabled");
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

	nh.param("moon_node/mode", mode,0);
//	ROS_INFO("moon mode = %d", mode);

	
	if(mode==0){
	pose.pose.position.x = 0;
    	pose.pose.position.y = 0;
    	pose.pose.position.z = 7;
        local_pos_pub.publish(pose);
	}
	else if(mode==1){
		//geting own position
		theta = wn*count*0.025;  //0.4rad/s if wn=0.4
		moon_x = earth_x + r*sin(theta);
		moon_y = earth_y + r*cos(theta);
		moon_z = earth_z + 2;

		//goto own position
	    	pose.pose.position.x = moon_x;
	    	pose.pose.position.y = moon_y;
	    	pose.pose.position.z = moon_z;
		local_pos_pub.publish(pose);
		count++;
	}
	else if(mode == 2){
		pose.pose.position.x = 0;
    		pose.pose.position.y = 0;
    		pose.pose.position.z = 0;
        	local_pos_pub.publish(pose);
	}

        ros::spinOnce();
        rate.sleep();
    }
	
	

    return 0;
}

