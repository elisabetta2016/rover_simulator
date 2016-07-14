#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
//#include "libRover.h"
#include <sensor_msgs/Joy.h>
#include <math.h>
#include "donkey_rover/Scanner_Command.h"
#include "donkey_rover/Rover_Track_Speed.h"
#include "donkey_rover/Rover_Track_Bogie_Angle.h"
#include "donkey_rover/Rover_Scanner.h"
#include "donkey_rover/Rover_Power_Data.h" 
#include "donkey_rover/Speed_control.h"

#include <math.h>
#include <custom_msgs/gnssSample.h>
#include <sensor_msgs/Imu.h>

//Rover rover(false);
//EScannerState state;
float scannerRaw = 0;
float scannerCal = 0;
float VX = 0.0;
float VY = 0.0;
bool Low_Battery = false;
float radius = 6378137; // [m] earth radius 
const float R = 0.8;
class DonkeyRoverClass
{
	public:
		
		DonkeyRoverClass(ros::NodeHandle& node)
		{
			n_=node;

			//subscribers
			subFromJoystick_ 		= n_.subscribe("joy", 1, &DonkeyRoverClass::joyCallback,this);
			subFromCMDVEL_ 			= n_.subscribe("cmd_vel", 1, &DonkeyRoverClass::CMDVELLCommander,this);
			
			subFromRightLeftCommands_	= n_.subscribe("speedfollow", 5, &DonkeyRoverClass::RLcommander,this);
			subFromSpeedControl_		= n_.subscribe("speed_control", 5, &DonkeyRoverClass::SpeedControlCallback,this);
			

			// publishers
			odom_pub 	   	  = n_.advertise<nav_msgs::Odometry>("odom", 100);
			twist_pub 	  	  = n_.advertise<geometry_msgs::Twist>("twist", 100);
			Rover_Track_Speed_pub     = n_.advertise<donkey_rover::Rover_Track_Speed>("RoverTrackSpeed", 100);
			Rover_Track_Angles_pub    = n_.advertise<donkey_rover::Rover_Track_Bogie_Angle>("RoverTrackAngles", 100);	
			Rover_Scanner_Data_pub    = n_.advertise<donkey_rover::Rover_Scanner>("RoverScannerInfo", 100);
			Rover_Power_Data_pub      = n_.advertise<donkey_rover::Rover_Power_Data>("RoverPowerInfo", 100);
			GPS_SIM_pub		  = n_.advertise<custom_msgs::gnssSample>("/mti/sensor/gnssPvt", 100);
            		IMU_SIM_pub		  = n_.advertise<sensor_msgs::Imu>("/mti/sensor/imu", 100);
            		
            		
            		// initializer
            		Speed_control.RLC = true;
            		Speed_control.CMD = true;
            		Speed_control.JOY = true;
            		
            		v = 0;
		 	vth = 0;
 			VL = 0;
			VR = 0;
			
			v_cmd = 0;
		 	vth_cmd = 0;
 			VL_cmd = 0;
			VR_cmd = 0;
			cmd_count = 0;
			
			v_joy = 0;
		 	vth_joy = 0;
 			VL_joy = 0;
			VR_joy = 0;
			joy_count = 0;
			
			v_rlc = 0;
		 	vth_rlc = 0;
 			VL_rlc = 0;
			VR_rlc = 0;
			rlc_count = 0;
			
			rate = 100;
		}


		// Global Varriables of the Class

		bool is_equal(float a, float b,float precision)
		{
		   bool out = false;
		   if (fabs(a-b) < precision) out = true;
		   return out;
		}
		
		void SpeedControlCallback(const donkey_rover::Speed_control::ConstPtr& msg)
		{
			Speed_control = *msg;
			//ROS_INFO("New control speed received");
			//ROS_WARN("RLC Enabled %d", Speed_control.RLC);
		}
		
		void RLcommander(const geometry_msgs::Vector3::ConstPtr& s)
		{
  			if (Speed_control.RLC)
  			{
  				geometry_msgs::Vector3 speed = *s;
  			
  				VL_rlc = speed.y;
  				VR_rlc = speed.x;
			
				v_rlc   = (VL_rlc + VR_rlc)/2;
				vth_rlc = (VR_rlc - VL_rlc)/R;
				rlc_count ++;
			}
		}
		
		


		void CMDVELLCommander(const geometry_msgs::Twist::ConstPtr& vel)
		{
  			if (Speed_control.CMD)
  			{
  				geometry_msgs::Twist new_vel = *vel;
  				v_cmd = sqrt (new_vel.linear.x * new_vel.linear.x + new_vel.linear.y * new_vel.linear.y);
  				vth_cmd = new_vel.angular.z;
				VL_cmd = (R*vth_cmd+2*v)/2;
				VR_cmd = 2*v_cmd -VL_cmd;
				cmd_count ++;
			} 

		}

		void joyCallback(const sensor_msgs::JoyConstPtr& joy)
		{
			if (Speed_control.JOY)
  			{
  				v_joy=joy->axes[1];
  				vth_joy=joy->axes[2];
 		        	float a1=joy->buttons[4];
 		        	
  				v_joy= v_joy*(a1+1)/2;
				VL_joy = (R*vth_joy+2*v_joy)/2;
				VR_joy = 2*v_joy -VL_joy; 
				joy_count ++;  			
  			}
		}

		
		
		void RoverDataProvider()
		{
			//Time timestamp;
			float temp_Front_Left_Track_Speed;
			float temp_Front_Right_Track_Speed;
			float temp_Rear_Left_Track_Speed;
			float temp_Rear_Right_Track_Speed;
			
  			ros::Time current_time;
  			current_time = ros::Time::now();
			temp_Front_Left_Track_Speed = VL;
			temp_Rear_Left_Track_Speed = VL;
			temp_Front_Right_Track_Speed = VR;
			temp_Rear_Right_Track_Speed = VR;
						


			outputTrackSpeed.Front_Left_Track_Speed	  = temp_Front_Left_Track_Speed;
			outputTrackSpeed.Front_Right_Track_Speed  = temp_Front_Right_Track_Speed;
			outputTrackSpeed.Rear_Left_Track_Speed    = temp_Rear_Left_Track_Speed;
			outputTrackSpeed.Rear_Right_Track_Speed   = temp_Rear_Right_Track_Speed;
			outputTrackSpeed.header.stamp = current_time;
   	 		outputTrackSpeed.header.frame_id = "base_link";
			outputTrackSpeed.TimeStamp = current_time.toSec();

			outputBogieAngle.Front_Left_Track_Angle   = 0;
			outputBogieAngle.Front_Right_Track_Angle  = 0;
			outputBogieAngle.Rear_Left_Track_Angle    = 0;
			outputBogieAngle.Rear_Right_Track_Angle   = 0;
			outputBogieAngle.Rear_Bogie_Angle         = 0;
 			outputBogieAngle.header.stamp = current_time;
   	 		outputBogieAngle.header.frame_id = "base_link";
			outputBogieAngle.TimeStamp = current_time.toSec();
			
			outputScanner.Scanner_State = "Idle";
			outputScanner.Scanner_Period = 2;
			outputScanner.Scanner_adjustment_angle = 0;
			outputScanner.Scanner_angle = 0;
			outputScanner.Scanner_angle_encoder = 0;
			outputScanner.Scanner_angle_degree = 0;
			outputScanner.Scanner_angle = 0;
 			outputScanner.header.stamp = current_time;
   	 		outputScanner.header.frame_id = "base_link";
			outputScanner.TimeStamp = current_time.toSec();

			outputPower.Battery_Voltage = 54;
			outputPower.Front_Right_Track_Current = VR*0.3;
			outputPower.Front_Left_Track_Current  = VL*0.3;
			outputPower.Rear_Right_Track_Current  = VR*0.3;
			outputPower.Rear_Left_Track_Current   = VL*0.3;
 			outputPower.header.stamp = current_time;
   	 		outputPower.header.frame_id = "base_link";
			outputPower.TimeStamp = current_time.toSec();

      
			Rover_Track_Speed_pub.publish(outputTrackSpeed); 
			Rover_Track_Angles_pub.publish(outputBogieAngle);
			Rover_Scanner_Data_pub.publish(outputScanner);
			Rover_Power_Data_pub.publish(outputPower);

		}

		void GPS_From_XY()
		{
			//Calculate Lat and Long from global variables x and y
			gps.latitude =  14.141515 + x/radius*180.0/M_PI; // x = position = NORTH
			gps.longitude = 14.141515 - y/radius*180.0/M_PI;  //y = position = EAST
			gps.hEll = 0.00;//To be Done
			gps.hMsl= 0.00;//To be Done
			GPS_SIM_pub.publish(gps);
		
		}

        	void Odometry_Handle()
		{
			
			rate = 100;
  			x = 0.0;
  			y = 0.0;
  			th = 0.0;
  			float vx = 0.0;
  			float vy = -0.0;
  			int count = 0;
			// Start Scanner angle calcultion - Defining variables
			float delta_scanner = 0;
			float last_scanner_value = 0;
			float scanner_offsetVal = 0;
			tf::TransformBroadcaster broadcaster;
			// End Scanner angle calcultion - Defining variables
			ROS_INFO_ONCE("Rover Simulator Started");
  			ros::Time current_time, last_time;
  			current_time = ros::Time::now();
  			last_time = ros::Time::now();
			ros::Rate loop_rate(rate);
			
  			while(ros::ok()){

				// Start Scanner Angle Loop 
    		    		delta_scanner =scannerRaw - last_scanner_value;
				if (delta_scanner > 10000)       scanner_offsetVal = scanner_offsetVal - scannerRaw;
				else if(delta_scanner < -10000)  scanner_offsetVal = scanner_offsetVal + last_scanner_value;
				last_scanner_value = scannerRaw;
				scannerCal = scannerRaw + scanner_offsetVal;
    				tf::Quaternion scan_quart;
    				scan_quart.setRPY(((scannerCal/21687)*M_PI/2),0.0 , 0.0);
    				/*broadcaster.sendTransform(
      				tf::StampedTransform(

        			tf::Transform(scan_quart, tf::Vector3(0.0, 0.0, 0.0)), ros::Time::now(),"base_laser","base_scanner")
    				);*/
    				
    				//////// WATCH DOG ///////
				if (count == 1)   //Reset
				{
				//ROS_INFO("RESET");
					cmd_count = 0;
					joy_count = 0;
					rlc_count = 0;
						
				}
				if(count == 10)	//Check
				{
				//ROS_WARN("CHECK");
					if (rlc_count < 1 || !Speed_control.RLC ) 
					{
						v_rlc = 0.0;
						vth_rlc = 0.0;
						VL_rlc = 0.0;
						VR_rlc = 0.0;
					}
					if (cmd_count < 1 || !Speed_control.CMD) 
					{
						v_cmd = 0.0;
						vth_cmd = 0.0;
						VL_cmd = 0.0;
						VR_cmd = 0.0;
					}
					if ( !Speed_control.JOY) //joy_count < 1 ||
					{
					
						v_joy = 0.0;
						vth_joy = 0.0;
						VL_joy = 0.0;
						VR_joy = 0.0;
					}
					
					count = 0;
				}
				
				v = v_rlc + v_cmd + v_joy;
				vth = vth_rlc + vth_cmd + vth_joy;
				VL = VL_rlc + VL_cmd + VL_joy;
				VR = VR_rlc + VR_cmd + VR_joy;
				
				
				if ( !is_equal(v,v_cmd,0.001) && !is_equal(v,v_joy,0.001) && !is_equal(v,v_rlc,0.001) )
				{
					ROS_ERROR(" Conflicting speeds, The control is handed to Joy!");
					v = v_joy;
					vth = vth_joy;
					VL = VL_joy;
					VR = VR_joy;
				
				}
				
				
				///// WATCH DOG END //////
				
    				ros::spinOnce();               
    				current_time = ros::Time::now();
    				

    				// Correction applied
    				//vx = v * cos(th);    
    				//vy = v * sin(th);
    				vy = v * cos(th);
				vx = -v * sin(th);


    				//compute odometry in a typical way given the velocities of the robot
    				float dt = (current_time - last_time).toSec();
    				float delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    				float delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    				float delta_th = vth * dt;
				//DEBUG
    				delta_x = vx * dt;
    				delta_y = vy * dt;
				// DEBUG END
    				x += delta_x;
    				y += delta_y;
    				th += delta_th;
    				//since all odometry is 6DOF we'll need a quaternion created from yaw
    				geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    				//first, we'll publish the transform over tf
    				geometry_msgs::TransformStamped odom_trans;
    				odom_trans.header.stamp = current_time;
    				odom_trans.header.frame_id = "odom";
    				odom_trans.child_frame_id = "Imu_link";

    				odom_trans.transform.translation.x = x;
   				odom_trans.transform.translation.y = y;
    				odom_trans.transform.translation.z = 0.0;
    				odom_trans.transform.rotation = odom_quat;

    				//send the transform
    				odom_broadcaster.sendTransform(odom_trans);

    				//next, we'll publish the odometry message over ROS
    				//nav_msgs::Odometry odom;
    				odom.header.stamp = current_time;
   	 			    odom.header.frame_id = "odom";
    				//geometry_msgs::Twist tw;

    				//set the position
    				odom.pose.pose.position.x = x;
    				odom.pose.pose.position.y = y;
    				odom.pose.pose.position.z = 0.0;
    				odom.pose.pose.orientation = odom_quat;

    				//set the velocity
    				odom.child_frame_id = "Imu_link";
    				odom.twist.twist.linear.x = vx;
    				odom.twist.twist.linear.y = vy;
    				odom.twist.twist.angular.z = vth;
    				tw.linear.x = VX;
    				tw.linear.y = VY;
                    
                    		//Create the simulated Imu
                    		sensor_msgs::Imu sim_imu;
                    		geometry_msgs::Quaternion sim_quat;
                    		sim_quat.w = cos(th/2);  //ENU standard
                    		sim_quat.x = 0;
                    		sim_quat.y = 0;
                    		sim_quat.z = sin(th/2);
                    		sim_imu.orientation = sim_quat;//odom_quat;

    				//publish the message
    				odom_pub.publish(odom);
    				twist_pub.publish(tw);
                    		IMU_SIM_pub.publish(sim_imu);
				RoverDataProvider();
				GPS_From_XY();
    				last_time = current_time;
    				count ++;
				loop_rate.sleep();
  				}
		}
				
		

	protected:
		/*state here*/
		ros::NodeHandle n_;

		// Subscribers
		ros::Subscriber subFromJoystick_;
		ros::Subscriber subFromCMDVEL_;
		ros::Subscriber subFromScannerCommander_;
		//ros::Subscriber subFromIMUSpeed_;
	        ros::Subscriber subFromRightLeftCommands_;
	        ros::Subscriber subFromSpeedControl_;
		//ros::Subscriber subFromscannerdata_;
		//ros::Subscriber subFromscannercommands_;
		// Publishers
		ros::Publisher odom_pub;
		ros::Publisher twist_pub;
		ros::Publisher Rover_Track_Speed_pub;
		ros::Publisher Rover_Track_Angles_pub;
		ros::Publisher Rover_Scanner_Data_pub;
		ros::Publisher Rover_Power_Data_pub;
		ros::Publisher GPS_SIM_pub;
		ros::Publisher IMU_SIM_pub;

		nav_msgs::Odometry odom;
		tf::TransformBroadcaster odom_broadcaster;
		geometry_msgs::Twist tw;
		donkey_rover::Rover_Track_Speed outputTrackSpeed;
		donkey_rover::Rover_Scanner outputScanner;
		donkey_rover::Rover_Track_Bogie_Angle outputBogieAngle;
		donkey_rover::Rover_Power_Data outputPower;
		donkey_rover::Speed_control Speed_control;
		custom_msgs::gnssSample gps;
		bool speed_present;
		
		float v;
		float vth;
 		float VL;
		float VR;
		
		float v_joy;
		float vth_joy;
 		float VL_joy;
		float VR_joy;
		size_t joy_count;
		
		float v_cmd;
		float vth_cmd;
 		float VL_cmd;
		float VR_cmd;
		size_t cmd_count;
		
		float v_rlc;
		float vth_rlc;
 		float VL_rlc;
		float VR_rlc;
		size_t rlc_count;
		
  		float x;
  		float y;
  		float th;
		int rate; 

	private:
		//float temp_adjustment_angle = -100;
		//float temp_roll_angle = -100;
		//float temp_home_angle = -100;
		//float temp_scanner_period = -100;
		
		
		
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "donkey_rover");
	ros::NodeHandle node;

	DonkeyRoverClass DonkeyRoverNode(node);

	//DonkeyRoverNode.Rover_Handle();
	
	//DonkeyRoverNode.Scanner_Handle();
	//DonkeyRoverNode.RoverDataProvider();
	DonkeyRoverNode.Odometry_Handle();
	return 0;
}
