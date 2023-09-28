#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <std_msgs/Int8.h>

float distance[8];
int sector;

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

void callback_best_sector(const std_msgs::Int8::ConstPtr& msg)
{
	sector = msg->data;
}

float* obstacle_avoidance_ultrasonic_sensors()
{
	const float distance_avoid = 2.0;
	const float k1 = 1.0, k2 = 0.5;
	float speed = 0.0;
	float theta = 0.0;
	const float vmin = 0.5, vmax = 4.0;
	float vx = 0, vy = 0;
	static float v[2];

	for(int i=0;i<8;i++)
	{
		if(distance[i] < distance_avoid)
		{
			speed = (-1) * k1 * pow((distance[i] - (distance_avoid + k2)), 2);
			theta = M_PI * (i / 4.0);
			vx += speed * cos(theta);
			vy += speed * sin(theta);
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
	v[0] = vx, v[1] = vy;
	return v;
}

float* obstacle_avoidance_stereo_camera()
{
	const int w = 640, h = 480;
	const int y_init = w/2, z_init = h/2;
	int y_goal, z_goal;
	float theta;
	const float speed = 0.5;
	float vy = 0, vz = 0;
	static float v[2];

	switch (sector)
	{
		case 1:
			y_goal = w/6; z_goal = h/6;
			break;
		case 2:
			y_goal = w/2; z_goal = h/6;
			break;
		case 3:
			y_goal = 5*w/6; z_goal = h/6;
			break;
		case 4:
			y_goal = w/6; z_goal = h/2;
			break;
		case 5:
			v[0] = 0, v[1] = 0;
			return v;
			break;
		case 6:
			y_goal = 5*w/6; z_goal = h/2;
			break;
		case 7:
			y_goal = w/6; z_goal = 5*h/6;
			break;
		case 8:
			y_goal = w/2; z_goal = 5*h/6;
			break;
		case 9:
			y_goal = 5*w/6; z_goal = 5*h/6;
			break;
		default:
			break;
	}
	theta = atan2(y_goal-y_init,z_goal-z_init);
	vy = speed * sin(theta);
	vz = speed * cos(theta);
	v[0] = vy, v[1] = vz;
	return v;
}

float* obstacle_avoidance()
{
	const float k1 = 1.0, k2 = 1.0;
	//v = [vx,vy,vz]
	static float v[3];
	float* v_ultrasonic = obstacle_avoidance_ultrasonic_sensors();
	float* v_stereo = obstacle_avoidance_stereo_camera();
	v[0] = k1 * v_ultrasonic[0];
	v[1] = (k1 * v_ultrasonic[1]) + (k2 * v_stereo[0]);
	v[2] = k2 * v_stereo[1];
	return v;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "obstacle_avoidance_node");
	ros::NodeHandle n;

	ros::Subscriber sub_sensor1 = n.subscribe("/distance_sensor/1", 10, callback_sensor1);
	ros::Subscriber sub_sensor2 = n.subscribe("/distance_sensor/2", 10, callback_sensor2);
	ros::Subscriber sub_sensor3 = n.subscribe("/distance_sensor/3", 10, callback_sensor3);
	ros::Subscriber sub_sensor4 = n.subscribe("/distance_sensor/4", 10, callback_sensor4);
	ros::Subscriber sub_sensor5 = n.subscribe("/distance_sensor/5", 10, callback_sensor5);
	ros::Subscriber sub_sensor6 = n.subscribe("/distance_sensor/6", 10, callback_sensor6);
	ros::Subscriber sub_sensor7 = n.subscribe("/distance_sensor/7", 10, callback_sensor7);
	ros::Subscriber sub_sensor8 = n.subscribe("/distance_sensor/8", 10, callback_sensor8);
	ros::Subscriber sub_best_sector = n.subscribe("/best_sector_topic", 10, callback_best_sector);

	ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("avoidance_velocity_topic",10);

	ROS_INFO("Executing obstacle avoidance algorithm");
	ros::Rate rate(10);

	while(ros::ok())
	{
		float* v = obstacle_avoidance();
		ROS_INFO("Avoidance velocity: vx = %f , vy = %f, vz = %f",v[0],v[1],v[2]);
 		geometry_msgs::Twist msg;
 		msg.linear.x = v[0];
		msg.linear.y = v[1];
		msg.linear.z = v[2];
 		velocity_publisher.publish(msg);
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
