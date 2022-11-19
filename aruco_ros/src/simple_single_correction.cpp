/*****************************
 Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are
 permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this list of
 conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice, this list
 of conditions and the following disclaimer in the documentation and/or other materials
 provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
 WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 The views and conclusions contained in the software and documentation are those of the
 authors and should not be interpreted as representing official policies, either expressed
 or implied, of Rafael Muñoz Salinas.
 ********************************/
/**
 * @file simple_single.cpp
 * @author Bence Magyar
 * @date June 2012
 * @version 0.1
 * @brief ROS version of the example named "simple" in the ArUco software package.
 */

#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <aruco_ros/aruco_ros_utils.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <dynamic_reconfigure/server.h>
#include <aruco_ros/ArucoThresholdConfig.h>

//en
#include <aruco_ros/custom_msgs.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class ArucoSimple
{
private:
  cv::Mat inImage;
  aruco::CameraParameters camParam;
  tf::StampedTransform rightToLeft;
  bool useRectifiedImages;
  aruco::MarkerDetector mDetector{"ARUCO_MIP_16h3",1};
  std::vector<aruco::Marker> markers;
  ros::Subscriber cam_info_sub;
  bool cam_info_received;
  image_transport::Publisher image_pub;
  image_transport::Publisher debug_pub;
  ros::Publisher pose_pub;
  ros::Publisher transform_pub;
  ros::Publisher position_pub;
  ros::Publisher marker_pub; // rviz visualization marker
  ros::Publisher pixel_pub;
  
  //en add publisher
  //ros::Publisher custom_pose_pub;
  image_transport::Publisher test_pub;
  
  std::string marker_frame;
  std::string camera_frame;
  std::string reference_frame;

  double marker_size;
  int marker_id;

  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub;

  tf::TransformListener _tfListener;

  dynamic_reconfigure::Server<aruco_ros::ArucoThresholdConfig> dyn_rec_server;

public:
  ArucoSimple() :
      cam_info_received(false), nh("~"), it(nh)
  {

    if (nh.hasParam("corner_refinement"))
      ROS_WARN(
          "Corner refinement options have been removed in ArUco 3.0.0, corner_refinement ROS parameter is deprecated");

    aruco::MarkerDetector::Params params = mDetector.getParameters();
    std::string thresh_method;
    switch (params._thresMethod)
    {
      case aruco::MarkerDetector::ThresMethod::THRES_ADAPTIVE:
        thresh_method = "THRESH_ADAPTIVE";
        break;
      case aruco::MarkerDetector::ThresMethod::THRES_AUTO_FIXED:
        thresh_method = "THRESH_AUTO_FIXED";
        break;
      default:
        thresh_method = "UNKNOWN";
        break;
    }

    // Print parameters of ArUco marker detector:
    ROS_INFO_STREAM("Threshold method: " << thresh_method);

    float min_marker_size; // percentage of image area
    nh.param<float>("min_marker_size", min_marker_size, 0.02);

    std::string detection_mode;
    nh.param<std::string>("detection_mode", detection_mode, "DM_FAST");
    if (detection_mode == "DM_FAST")
      mDetector.setDetectionMode(aruco::DM_FAST, min_marker_size);
    else if (detection_mode == "DM_VIDEO_FAST")
      mDetector.setDetectionMode(aruco::DM_VIDEO_FAST, min_marker_size);
    else
      // Aruco version 2 mode
      mDetector.setDetectionMode(aruco::DM_NORMAL, min_marker_size);

    ROS_INFO_STREAM("Marker size min: " << min_marker_size << "% of image area");
    ROS_INFO_STREAM("Detection mode: " << detection_mode);

    image_sub = it.subscribe("/image", 1, &ArucoSimple::image_callback, this);
    cam_info_sub = nh.subscribe("/camera_info", 1, &ArucoSimple::cam_info_callback, this);

    image_pub = it.advertise("result", 1);
    debug_pub = it.advertise("debug", 1);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 100);
    transform_pub = nh.advertise<geometry_msgs::TransformStamped>("transform", 100);
    position_pub = nh.advertise<geometry_msgs::Vector3Stamped>("position", 100);
    marker_pub = nh.advertise<visualization_msgs::Marker>("marker", 10);
    pixel_pub = nh.advertise<geometry_msgs::PointStamped>("pixel", 10);
    //en add advertise
    //custom_pose_pub = nh.advertise<aruco_ros::custom_msgs>("custom_publisher", 10);
    test_pub = it.advertise("test", 1);
    
    nh.param<double>("marker_size", marker_size, 0.05);
    nh.param<int>("marker_id", marker_id, 300);
    nh.param<std::string>("reference_frame", reference_frame, "");
    nh.param<std::string>("camera_frame", camera_frame, "");
    nh.param<std::string>("marker_frame", marker_frame, "");
    nh.param<bool>("image_is_rectified", useRectifiedImages, true);

    ROS_ASSERT(camera_frame != "" && marker_frame != "");

    if (reference_frame.empty())
      reference_frame = camera_frame;

    ROS_INFO("ArUco node started with marker size of %f m and marker id to track: %d", marker_size, marker_id);
    ROS_INFO("ArUco node will publish pose to TF with %s as parent and %s as child.", reference_frame.c_str(),
             marker_frame.c_str());

    dyn_rec_server.setCallback(boost::bind(&ArucoSimple::reconf_callback, this, _1, _2));
  }

  bool getTransform(const std::string& refFrame, const std::string& childFrame, tf::StampedTransform& transform)
  {
    std::string errMsg;

    if (!_tfListener.waitForTransform(refFrame, childFrame, ros::Time(0), ros::Duration(0.5), ros::Duration(0.01),
                                      &errMsg))
    {
      ROS_ERROR_STREAM("Unable to get pose from TF: " << errMsg);
      return false;
    }
    else
    {
      try
      {
        _tfListener.lookupTransform(refFrame, childFrame, ros::Time(0), // get latest available
                                    transform);
      }
      catch (const tf::TransformException& e)
      {
        ROS_ERROR_STREAM("Error in lookupTransform of " << childFrame << " in " << refFrame);
        return false;
      }

    }
    return true;
  }

  void image_callback(const sensor_msgs::ImageConstPtr& msg)
  {
    if ((image_pub.getNumSubscribers() == 0) && (debug_pub.getNumSubscribers() == 0)
        && (pose_pub.getNumSubscribers() == 0) && (transform_pub.getNumSubscribers() == 0)
        && (position_pub.getNumSubscribers() == 0) && (marker_pub.getNumSubscribers() == 0)
        && (pixel_pub.getNumSubscribers() == 0) && (test_pub.getNumSubscribers() == 0) && (0)) //(custom_pose_pub.getNumSubscribers() == 0)
    {
      std::cout << "no subscriber" << std::endl;
      ROS_DEBUG("No subscribers, not looking for ArUco markers");
      return;
    }

    static tf::TransformBroadcaster br;
    if (cam_info_received)
    {
      ros::Time curr_stamp = msg->header.stamp;
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        inImage = cv_ptr->image;
        // detection results will go into "markers"
        cv::Mat test;
        markers.clear();
        // ok, let's detect
        mDetector.detect(inImage, markers, camParam, test, marker_size, false, false);
        // for each marker, draw info and its boundaries in the image
        for (std::size_t i = 0; i < markers.size(); ++i)
        {
          //en add
          // only publishing the selected marker
          if (markers[i].id == marker_id)
          {
            //en add: this function is write in aruco_ros_utils.cpp
            tf::Transform transform = aruco_ros::arucoMarker2Tf(markers[i]);
            //en add: transform include tf2::Transform(tf_rot, tf_orig), where type(tf_rot)=tf2::Matrix3x3, type(tf_orig)=tf2::Vector3
            tf::StampedTransform cameraToReference;
            cameraToReference.setIdentity();

            //en add end
            if (markers[i].id == 0)
            {
            	tf::StampedTransform stampedTransform_marktocamera(transform, curr_stamp, camera_frame , "marker_frame_0");//frame_id,child_frame_id
            	br.sendTransform(stampedTransform_marktocamera);
            }

            if (reference_frame != camera_frame)
            {
              getTransform(reference_frame, camera_frame, cameraToReference);
              //getTransform("world_frame_cam", camera_frame, cameraToReference);
	    }
            transform = static_cast<tf::Transform>(cameraToReference) * static_cast<tf::Transform>(rightToLeft)
                * transform;

            tf::StampedTransform stampedTransform(transform, curr_stamp, reference_frame, marker_frame);
            br.sendTransform(stampedTransform);
            geometry_msgs::PoseStamped poseMsg;
            tf::poseTFToMsg(transform, poseMsg.pose);
            poseMsg.header.frame_id = reference_frame;
            poseMsg.header.stamp = curr_stamp;
            pose_pub.publish(poseMsg);
            //en add trans Q to E
            /*
            tf::Quaternion trn_q(poseMsg.pose.orientation.x,poseMsg.pose.orientation.y,poseMsg.pose.orientation.z,poseMsg.pose.orientation.w);
            tf::Matrix3x3 trn_m(trn_q);
            double roll,pitch,yaw,car_rot,mark_ground;
            trn_m.getRPY(roll,pitch,yaw);
            calculate_moving_angle(roll, pitch, yaw, car_rot, mark_ground);
            std::cout << "pitch(y): " << pitch*180/3.1415926 << std::endl;
            std::cout << "roll(x): " << roll*180/3.1415926 << std::endl;
            std::cout << "yaw(z): " << yaw*180/3.1415926 << std::endl;
            std::cout << "car_rot: " << car_rot << std::endl;
            std::cout << "mark_ground: " << mark_ground << std::endl;
            std::cout << std::endl;
            */
            //en
            /*
            aruco_ros::custom_msgs customMsg;
            customMsg.mark_ground_ms = mark_ground;
            customMsg.car_rot_ms = car_rot;
            customMsg.pitch_ms = pitch*180/3.1415926;
            customMsg.roll_ms = roll*180/3.1415926;
            customMsg.yaw_ms = yaw*180/3.1415926;
            customMsg.pose_custom = poseMsg;
            customMsg.id_custom = markers[i].id;
            custom_pose_pub.publish(customMsg);
            */
            //
            geometry_msgs::TransformStamped transformMsg;
            tf::transformStampedTFToMsg(stampedTransform, transformMsg);
            transform_pub.publish(transformMsg);

            geometry_msgs::Vector3Stamped positionMsg;
            positionMsg.header = transformMsg.header;
            positionMsg.vector = transformMsg.transform.translation;
            position_pub.publish(positionMsg);

            geometry_msgs::PointStamped pixelMsg;
            pixelMsg.header = transformMsg.header;
            pixelMsg.point.x = markers[i].getCenter().x;
            pixelMsg.point.y = markers[i].getCenter().y;
            pixelMsg.point.z = 0;
            pixel_pub.publish(pixelMsg);

            // publish rviz marker representing the ArUco marker patch
            visualization_msgs::Marker visMarker;
            visMarker.header = transformMsg.header;
            visMarker.id = 1;
            visMarker.type = visualization_msgs::Marker::CUBE;
            visMarker.action = visualization_msgs::Marker::ADD;
            visMarker.pose = poseMsg.pose;
            visMarker.scale.x = marker_size;
            visMarker.scale.y = marker_size;
            visMarker.scale.z = 0.001;
            visMarker.color.r = 1.0;
            visMarker.color.g = 0;
            visMarker.color.b = 0;
            visMarker.color.a = 1.0;
            visMarker.lifetime = ros::Duration(3.0);
            marker_pub.publish(visMarker);

          }
          // but drawing all the detected markers
          markers[i].draw(inImage, cv::Scalar(0, 0, 255), 2);
        }

        // draw a 3d cube in each marker if there is 3d info
        if (camParam.isValid() && marker_size != -1)
        {
          for (std::size_t i = 0; i < markers.size(); ++i)
          {
            aruco::CvDrawingUtils::draw3dAxis(inImage, markers[i], camParam);
          }
        }
        if (image_pub.getNumSubscribers() > 0)
        {
          // show input with augmented information
          cv_bridge::CvImage out_msg;
          out_msg.header.stamp = curr_stamp;
          out_msg.encoding = sensor_msgs::image_encodings::RGB8;
          out_msg.image = inImage;
          image_pub.publish(out_msg.toImageMsg());
        }

        if (debug_pub.getNumSubscribers() > 0)
        {
          // show also the internal image resulting from the threshold operation
          cv_bridge::CvImage debug_msg;
          debug_msg.header.stamp = curr_stamp;
          debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
          debug_msg.image = mDetector.getThresholdedImage();
          debug_pub.publish(debug_msg.toImageMsg());
        }
        
        //en add getnumsubscribers test
        if (test_pub.getNumSubscribers() > 0)
        {
          cv_bridge::CvImage test_msg;
          test_msg.header.stamp = curr_stamp;
          test_msg.encoding = sensor_msgs::image_encodings::RGB8;
          test_msg.image = test;
          if (test.empty()==false)
          {
            test_pub.publish(test_msg.toImageMsg());
          }
        }
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
    }
  }

  // wait for one camerainfo, then shut down that subscriber
  void cam_info_callback(const sensor_msgs::CameraInfo &msg)
  {
    camParam = aruco_ros::rosCameraInfo2ArucoCamParams(msg, useRectifiedImages);

    // handle cartesian offset between stereo pairs
    // see the sensor_msgs/CameraInfo documentation for details
    rightToLeft.setIdentity();
    rightToLeft.setOrigin(tf::Vector3(-msg.P[3] / msg.P[0], -msg.P[7] / msg.P[5], 0.0));

    cam_info_received = true;
    cam_info_sub.shutdown();
  }

  void reconf_callback(aruco_ros::ArucoThresholdConfig &config, uint32_t level)
  {
    mDetector.setDetectionMode(aruco::DetectionMode(config.detection_mode), config.min_image_size);
    if (config.normalizeImage)
    {
      ROS_WARN("normalizeImageIllumination is unimplemented!");
    }
  }
  
  //en add calculate_moving_angle
  void calculate_moving_angle(double x, double y, double z, double& car_rot, double& mark_ground)
  {
    double const PI=3.1415926; 
    double R[3][3];
    double R1[3][3];
    double z_axis[3] = {0,0,1};
    double ans[3][3];
    double matrix1[3][3] = {{cos(y),0,sin(y)},
                            {0,1,0},
                            {-sin(y),0,cos(y)}};
    double matrix2[3][3] = {{1,0,0},
                            {0,cos(x),-sin(x)},
                            {0,sin(x),cos(x)}};
    double matrix3[3][3] = {{cos(z),-sin(z),0},
                            {sin(z),cos(z),0},
                            {0,0,1}};  
    for(int i=0;i<3;i++)
    {    
        for(int j=0;j<3;j++)
        {    
            R[i][j]=0;
            for(int k=0;k<3;k++)
            {    
                R[i][j] += matrix1[i][k] * matrix2[k][j];    
            }    
        }    
    }
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            if(R[i][j]<0.0001 && R[i][j]>0)
            {
                R[i][j] = 0;
            }
            else if(R[i][j]>-0.0001 && R[i][j]<0)
            {
                R[i][j] = 0;
            }
        }
    }
    for(int i=0;i<3;i++)    
    {    
        for(int j=0;j<3;j++)    
        {       
            R1[i][j]=0;
            for(int k=0;k<3;k++)    
            {    
                R1[i][j] += matrix3[i][k] * R[k][j];    
            }    
        }    
    }
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            if(R1[i][j]<0.00001 && R1[i][j]>0)
            {
                R1[i][j] = 0;
            }
            else if(R1[i][j]>-0.00001 && R1[i][j]<0)
            {
                R1[i][j] = 0;
            }
        }
    }
    for(int i=0;i<3;i++)    
    {     
        ans[i][0]=0;    
        for(int k=0;k<3;k++)    
        {    
            ans[i][0] += R1[i][k] * z_axis[k];    
        }  
    }  
    car_rot = (double)acos(0*(-ans[0][0])+1*(-ans[1][0])/pow(pow(ans[0][0],2)+pow(ans[1][0],2),0.5))/PI*180;
    if(abs(ans[0][0])<0.15 && abs(ans[1][0])<0.15)
    	car_rot = 0;
    mark_ground = abs((double)acos(0*ans[0][0]+0*ans[1][0]+1*ans[2][0]/pow(pow(ans[0][0],2)+pow(ans[1][0],2)+pow(ans[2][0],2),0.5))/PI*180);
    if(ans[0][0] > 0)
    {
        car_rot = car_rot;
    }
    else
    {
        car_rot = -1*car_rot;
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aruco_simple");

  ArucoSimple node;
  
  ros::spin();
}
