#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

float distance[8];
const float distance_avoid = 3.0;
const float k1 = 0.6;
const float k2 = 0.5;
float v = 0.0;
const float vmax = 4.0;
const float vmin = 0.1;
float vx = 0.0;
float vy = 0.0;
float theta = 0.0;

void callback_sensor1(const sensor_msgs::Range::ConstPtr& msg)
{	
	distance[0] = msg->range;
}

void callback_sensor2(const sensor_msgs::Range::ConstPtr& msg)
{	
	distance[1] = msg->range;
}

void callback_sensor3(const sensor_msgs::Range::ConstPtr& msg)
{	
	distance[2] = msg->range;
}

void callback_sensor4(const sensor_msgs::Range::ConstPtr& msg)
{	
	distance[3] = msg->range;
}

void callback_sensor5(const sensor_msgs::Range::ConstPtr& msg)
{	
	distance[4] = msg->range;
}

void callback_sensor6(const sensor_msgs::Range::ConstPtr& msg)
{	
	distance[5] = msg->range;
}

void callback_sensor7(const sensor_msgs::Range::ConstPtr& msg)
{	
	distance[6] = msg->range;
}

void callback_sensor8(const sensor_msgs::Range::ConstPtr& msg)
{	
	distance[7] = msg->range;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "obstacle_avoidance_ultrasonic_node");
	ros::NodeHandle n;
	
	ros::Subscriber sub_sensor1 = n.subscribe("/distance_sensor/1", 10, callback_sensor1);
    ros::Subscriber sub_sensor2 = n.subscribe("/distance_sensor/2", 10, callback_sensor2);
    ros::Subscriber sub_sensor3 = n.subscribe("/distance_sensor/3", 10, callback_sensor3);
    ros::Subscriber sub_sensor4 = n.subscribe("/distance_sensor/4", 10, callback_sensor4);
    ros::Subscriber sub_sensor5 = n.subscribe("/distance_sensor/5", 10, callback_sensor5);
    ros::Subscriber sub_sensor6 = n.subscribe("/distance_sensor/6", 10, callback_sensor6);
    ros::Subscriber sub_sensor7 = n.subscribe("/distance_sensor/7", 10, callback_sensor7);
    ros::Subscriber sub_sensor8 = n.subscribe("/distance_sensor/8", 10, callback_sensor8);
	
	ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("velocity_topic",10);
	
	ROS_INFO("Executing obstacle avoidance algorithm");
	ros::Rate rate(10);
	
	while(ros::ok())
	{
		vx = 0; vy = 0; theta = 0;
        for(int i=0;i<8;i++)
 		{
            if(distance[i] < distance_avoid)
            {
                v = -k1 * pow((distance[i] - (distance_avoid + k2)) ,2);
                theta = M_PI * (i / 4.0);
                vx += v * cos(theta);
                vy += v * sin(theta);
                ROS_INFO("Sensor %d , distance = %f , v = %f , vx = %f , vy = %f , theta = %f",i+1,distance[i],v,vx,vy,theta*180/M_PI);
            }
        }
        if(abs(vx) > vmax)
        {
            vx > 0 ? vx = vmax : vx = -vmax;
        }
        if(abs(vy) > vmax)
        {
            vy > 0 ? vy = vmax : vy = -vmax;
        }
        ROS_INFO("Final: vx = %f , vy = %f",vx,vy);

 		geometry_msgs::Twist msg;
 		msg.linear.x = vx;
        msg.linear.y = vy;
 		velocity_publisher.publish(msg);

		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
