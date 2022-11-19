#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <string>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>

class Total
{
	public:
		float position_x;
		float position_y;
		float position_z;
		float orientation_x;
		float orientation_y;
		float orientation_z;
		float orientation_w;
		int counter;
		Total()
		{
			position_x = 0;
			position_y = 0;
			position_z = 0;
			orientation_x = 0;
			orientation_y = 0;
			orientation_z = 0;
			orientation_w = 0;
			counter = 0;
		}
};
Total total;

void default_worldframe_generator(std::string& camera_frame,std::string& world_frame,tf::TransformBroadcaster& br,std::string& which_team);
tf::Transform marktoworld_transfrom_generator(float x ,float y ,float z ,float roll ,float pitch ,float yaw );
bool worldframe_generator(std::string& camera_frame , tf::Transform& marktoworld_transform , tf::StampedTransform& markertocam, std::string& world_frame, tf::TransformBroadcaster& br);
bool camtomark_listener(tf::TransformListener& tfListener, std::string& refFrame, std::string& childFrame, std::string& world_frame, tf::TransformBroadcaster& br);
void print_trasform_detail(tf::Transform transform);

int main(int argc, char** argv)
{
	ros::init(argc, argv, "camera_pose_correction");
	ros::NodeHandle nh;
	std::string camera_frame = "stereo_gazebo_left_camera_optical_frame";
	std::string world_frame = "world_frame_cam";
	std::string marker_frame = "marker_frame_0";
	std::string which_team;
	ros::Rate rate(10.0);
	tf::TransformListener tfListener;
	tf::StampedTransform camtomarker;
	nh.param<std::string>("/camera_pose_correction/which_team",which_team,"orange");
	std::cout << which_team << std::endl;
	//std::cout << camera_frame << std::endl;
	ros::Duration(2).sleep();
	tf::TransformBroadcaster br;
	while (nh.ok())
	{
		if(camtomark_listener(tfListener , camera_frame , marker_frame , world_frame, br)==true)
		{
			rate.sleep();
		}
		else
		{
			//std::cout << "something wrong" << std::endl;
                        ROS_INFO("somthing wrong: check if camera open and make sure marker_0 in view");
                        ROS_INFO("using default transform");
			default_worldframe_generator(camera_frame,world_frame,br,which_team);
		}
	}
	return 0;
};

void default_worldframe_generator(std::string& camera_frame,std::string& world_frame,tf::TransformBroadcaster& br, std::string& which_team)
{
	double x;
        double y;
	double z;
	double qx;
	double qy;
	double qz;
	double qw;
	if (which_team == "orange")
	{
		x = -0.186827;
		y = 1.2741;
		z = 1.08594;
		qx = 0.629724;
		qy = -0.618618;
		qz = 0.317639;
		qw = -0.346215;
	}
	else if (which_team == "purple")
	{
		x = -0.171585;
		y = 1.63342;
		z = 1.09612;
		qx = 0.628024;
		qy = -0.62553;
		qz = 0.308475;
		qw = -0.345168;
	}
	tf::Transform transform_send;
	transform_send.setOrigin(tf::Vector3(x,y,z));
	transform_send.setRotation(tf::Quaternion(qx,qy,qz,qw));
	tf::StampedTransform stampedTransform_default_camtoworld(transform_send, ros::Time::now(), world_frame, camera_frame);
	br.sendTransform(stampedTransform_default_camtoworld);
}

tf::Transform marktoworld_transfrom_generator(float x = 1.25,float y = 1.5,float z = 0,float roll = 0,float pitch = 0,float yaw = 1.57)
{
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(x,y,z));
	tf::Quaternion q;
	q.setRPY(roll,pitch,yaw);
	transform.setRotation(q);
	return transform;
}

bool worldframe_generator(std::string& camera_frame , tf::Transform& marktoworld_transform , tf::StampedTransform& markertocam, std::string& world_frame, tf::TransformBroadcaster& br)
{
	tf::Transform transform_send;
	transform_send = marktoworld_transform * (static_cast<tf::Transform>(markertocam).inverse());

	//geometry_msgs::PoseStamped poseMsg;
	//tf::poseTFToMsg(transform,poseMsg.pose);
	//std::cout << poseMsg.pose << std::endl;

	//print_trasform_detail(static_cast<tf::Transform>(markertocam).inverse());
	//print_trasform_detail(transform_send);
	tf::StampedTransform stampedTransform_camtoworld(transform_send, ros::Time::now(), world_frame, camera_frame);
	//geometry_msgs_TransformStamped(transform_send, ros::Time::now(), world_frame, camera_frame);
	br.sendTransform(stampedTransform_camtoworld);
	//br.sendTransform(transform_send,ros::Time::now(),camera_frame,world_frame
	return true;
}

bool camtomark_listener(tf::TransformListener& tfListener, std::string& refFrame, std::string& childFrame, std::string& world_frame, tf::TransformBroadcaster& br)
{
  std::string errMsg;
  tf::StampedTransform stamp_transform_markertocam;
  tf::Transform transform_marker_world = marktoworld_transfrom_generator(1.25,1.5,0,0,0,1.57);

  if (!tfListener.waitForTransform(refFrame, childFrame, ros::Time(0), ros::Duration(0.5), ros::Duration(0.01), &errMsg))
  {
    ROS_ERROR_STREAM("Unable to get pose from TF: " << errMsg);
    return false;
  }
  else
  {
    try
    {
      tfListener.lookupTransform(refFrame, childFrame, ros::Time(0), stamp_transform_markertocam);// get latest available marker to camera, maker:child,camera:ref
      //print_trasform_detail(static_cast<tf::Transform>(stamp_transform_camera_marker));
      worldframe_generator(refFrame , transform_marker_world , stamp_transform_markertocam, world_frame, br);
    }
    catch (const tf::TransformException& e)
    {
      ROS_ERROR_STREAM("Error in lookupTransform of " << childFrame << " in " << refFrame);
      return false;
    }
  }
  return true;
}

void print_trasform_detail(tf::Transform transform)
{
  geometry_msgs::PoseStamped poseMsg;
  tf::poseTFToMsg(transform, poseMsg.pose);
  total.position_x = total.position_x + poseMsg.pose.position.x;
  total.position_y = total.position_y + poseMsg.pose.position.y;
  total.position_z = total.position_z + poseMsg.pose.position.z;
  total.orientation_x = total.orientation_x + poseMsg.pose.orientation.x;
  total.orientation_y = total.orientation_y + poseMsg.pose.orientation.y;
  total.orientation_z = total.orientation_z + poseMsg.pose.orientation.z;
  total.orientation_w = total.orientation_w + poseMsg.pose.orientation.w;
  total.counter++;

  std::cout << "position.x: " << total.position_x/total.counter << std::endl;
  std::cout << "position.y: " << total.position_y/total.counter << std::endl;
  std::cout << "position.z: " << total.position_z/total.counter << std::endl;
  std::cout << "orientation.x: " << total.orientation_x/total.counter << std::endl;
  std::cout << "orientation.y: " << total.orientation_y/total.counter << std::endl;
  std::cout << "orientation.z: " << total.orientation_z/total.counter << std::endl;
  std::cout << "orientation.w: " << total.orientation_w/total.counter << std::endl;
  tf::Quaternion trn_q(total.orientation_x/total.counter,total.orientation_y/total.counter,total.orientation_z/total.counter,total.orientation_w/total.counter);
  tf::Matrix3x3 trn_m(trn_q);
  double roll,pitch,yaw;
  trn_m.getRPY(roll,pitch,yaw);
  std::cout << "pitch(y): " << pitch*180/3.1415926 << std::endl;
  std::cout << "roll(x): " << roll*180/3.1415926 << std::endl;
  std::cout << "yaw(z): " << yaw*180/3.1415926 << std::endl;

  if (total.counter == 50)
  {
  	Total total;
  }
}
