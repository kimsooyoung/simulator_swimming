#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h> //set position 용
#include <mavros_msgs/CommandBool.h> //arm용
#include <mavros_msgs/SetMode.h> //offboard 모드 설정용
#include <mavros_msgs/State.h> //mavros 메세지 활용용
#include "math.h" //수식 입력용
#include "solar_sys_formation/msgCoordinate.h"//메세지 정의용

double r = 5;
double theta;
double count=0.0;
double wn = 0.09;
float sun_x;
float sun_y;
float sun_z = 3.0;
float earth_x = 0.0;
float earth_y = 0.0;
float earth_z = 5.0;
int mode;

solar_sys_formation::msgCoordinate msg;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "earth_node");

    ros::NodeHandle nh;

    ros::Publisher ros_coordinate_pub = nh.advertise<solar_sys_formation::msgCoordinate> ("ros_coordinate_msg", 100);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State> ("mavros_earth/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped> ("mavros_earth/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool> ("mavros_earth/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode> ("mavros_earth/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0); //period 0.025 s
   

	nh.param("earth_node/mode", mode,0);
//	nh.param("wn", wn, 1.0);
//	nh.param("sun_x", sun_x, 0.0);
//	nh.param("sun_y", sun_y, 0.0);
//	nh.param("sun_z", sun_z, 0.0);


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
    pose.pose.position.x = earth_x;
    pose.pose.position.y = earth_y;
    pose.pose.position.z = 5;

    //send a few setpoints before starting
    for(int i = 60; ros::ok() && i > 0; --i){
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
                ROS_INFO("earth Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("earth armed");
                }
                last_request = ros::Time::now();
            }
        }

        nh.param("earth_node/mode", mode,0);
//	ROS_INFO("earth mode = %d", mode);
	
	//geting sun position
//	nh.param("sun_x", sun_x, 0.0);
//	nh.param("sun_y", sun_y, 0.0);
//	nh.param("sun_z", sun_z, 0.0);

	if(mode == 0){ //home
	pose.pose.position.x = 0;
    	pose.pose.position.y = 0;
    	pose.pose.position.z = 5;
        local_pos_pub.publish(pose);

	}

	else if(mode==1){ //round
		//get own position
		theta = wn*count*0.025;  //0.4rad/s if wn=0.4
		earth_x = sun_x + r*sin(theta);
		earth_y = sun_y + r*cos(theta);
		earth_z = sun_z + 2;

		//goto own position
	    	pose.pose.position.x = earth_x;
	    	pose.pose.position.y = earth_y;
	    	pose.pose.position.z = earth_z;
		local_pos_pub.publish(pose);

		//publish earth position to moon
		msg.earth_x = earth_x;
		msg.earth_y = earth_y;
		msg.earth_z = earth_z;
		ros_coordinate_pub.publish(msg);

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

