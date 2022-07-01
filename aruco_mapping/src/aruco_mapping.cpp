/*********************************************************************************************//**
* @file aruco_mapping.cpp
*
* Copyright (c)
* Smart Robotic Systems
* March 2015
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

/* Author: Jan Bacik */

#ifndef ARUCO_MAPPING_CPP
#define ARUCO_MAPPING_CPP

#include <tf2/LinearMath/Quaternion.h>
#include <aruco_mapping.h>
#include "lidar_camera_calibration/marker_6dof.h"
#include <geometry_msgs/Pose.h>

namespace aruco_mapping
{

ArucoMapping::ArucoMapping(ros::NodeHandle *nh) :
  listener_ (new tf::TransformListener),  // Initialize TF Listener  
  num_of_markers_ (10),                   // Number of used markers
  marker_size_(0.1),                      // Marker size in m
  camera_frame_id_("camera_0"),
  calib_filename_("empty"),               // Calibration filepath
  space_type_ ("plane"),                  // Space type - 2D plane 
  roi_allowed_ (false),                   // ROI not allowed by default
  view_image_ (false),
  first_marker_detected_(false),          // First marker not detected by defualt
  lowest_marker_id_(-1),                  // Lowest marker ID
  marker_counter_(0),                     // Reset marker counter
  closest_camera_index_(0),                // Reset closest camera index 
  marker_id_1_(101),
  marker_id_2_(202),
  marker_id_3_(11),
  marker_id_4_(12),
  marker_id_5_(13),
  group_name_("ns0")
  
{
  double temp_marker_size;  
  
  //Parse params from launch file 
  nh->getParam("aruco_mapping/calibration_file", calib_filename_);
  nh->getParam("aruco_mapping/marker_size", temp_marker_size); 
  nh->getParam("aruco_mapping/num_of_markers", num_of_markers_);
  nh->getParam("aruco_mapping/space_type",space_type_);
  nh->getParam("aruco_mapping/roi_allowed",roi_allowed_);
  nh->getParam("aruco_mapping/roi_x",roi_x_);
  nh->getParam("aruco_mapping/roi_y",roi_y_);
  nh->getParam("aruco_mapping/roi_w",roi_w_);
  nh->getParam("aruco_mapping/roi_h",roi_h_);
  nh->getParam("aruco_mapping/view_image",view_image_);
  nh->getParam("aruco_mapping/marker_id_1", marker_id_1_);
  nh->getParam("aruco_mapping/marker_id_2", marker_id_2_);
  nh->getParam("aruco_mapping/marker_id_3", marker_id_3_);
  nh->getParam("aruco_mapping/marker_id_4", marker_id_4_);
  nh->getParam("aruco_mapping/marker_id_5", marker_id_5_);
  nh->getParam("aruco_mapping/camera_frame_id", camera_frame_id_);
  nh->getParam("robot_group", group_name_); 
     
  // Double to float conversion
  marker_size_ = float(temp_marker_size);
  
  if(calib_filename_ == "empty")
    ROS_WARN("Calibration filename empty! Check the launch file paths");
  else
  {
    ROS_INFO_STREAM("Calibration file path: " << calib_filename_ );
    ROS_INFO_STREAM("Number of markers: " << num_of_markers_);
    ROS_INFO_STREAM("Marker Size: " << marker_size_);
    ROS_INFO_STREAM("Type of space: " << space_type_);
    ROS_INFO_STREAM("ROI allowed: " << roi_allowed_);
    ROS_INFO_STREAM("ROI x-coor: " << roi_x_);
    ROS_INFO_STREAM("ROI y-coor: " << roi_x_);
    ROS_INFO_STREAM("ROI width: "  << roi_w_);
    ROS_INFO_STREAM("ROI height: " << roi_h_);      
    ROS_INFO_STREAM("VIEW image: " << view_image_);
    ROS_INFO_STREAM("Marker ID 1 : " <<marker_id_1_);
    ROS_INFO_STREAM("Marker ID 2 : " <<marker_id_2_);
    ROS_INFO_STREAM("Marker ID 3 : " <<marker_id_3_);
    ROS_INFO_STREAM("Marker ID 4 : " <<marker_id_4_);
    ROS_INFO_STREAM("Marker ID 5 : " <<marker_id_5_);
    ROS_INFO_STREAM("camera frame id: " << camera_frame_id_);
  }
    
  //ROS publishers
  marker_msg_pub_           = nh->advertise<aruco_mapping::ArucoMarker>("aruco_poses",1);
  marker_visualization_pub_ = nh->advertise<visualization_msgs::Marker>("aruco_markers",1);
  marker_msg_pub1_ = nh->advertise<geometry_msgs::Pose>("aruco_markers_pose1",1);
  marker_msg_pub2_ = nh->advertise<geometry_msgs::Pose>("aruco_markers_pose2",1);
  marker_msg_pub3_ = nh->advertise<geometry_msgs::Pose>("aruco_markers_pose3",1);
  marker_msg_pub4_ = nh->advertise<geometry_msgs::Pose>("aruco_markers_pose4",1);
  marker_msg_pub5_ = nh->advertise<geometry_msgs::Pose>("aruco_markers_pose5",1);
  lidar_camera_calibration_rt = nh->advertise< lidar_camera_calibration::marker_6dof >("lidar_camera_calibration_rt",1);

  //Parse data from calibration file
  parseCalibrationFile(calib_filename_);

  //Initialize OpenCV windowrosto
  cv::namedWindow("Mono8", cv::WINDOW_AUTOSIZE);       
      
  //Resize marker container
  markers_.resize(num_of_markers_);
  
  // Default markers_ initialization
  for(size_t i = 0; i < num_of_markers_;i++)
  {
    markers_[i].previous_marker_id = -1;
    markers_[i].visible = false;
    markers_[i].marker_id = -1;
  }
}

ArucoMapping::~ArucoMapping()
{
 delete listener_;
}

bool
ArucoMapping::parseCalibrationFile(std::string calib_filename)
{
  sensor_msgs::CameraInfo camera_calibration_data;
  std::string camera_name = "camera";

  ROS_INFO_STREAM("calib_filename: " << calib_filename);

  camera_calibration_parsers::readCalibrationIni(calib_filename, camera_name, camera_calibration_data);

  // Alocation of memory for calibration data
  cv::Mat  *intrinsics       = new(cv::Mat)(3, 3, CV_64F);
  cv::Mat  *distortion_coeff = new(cv::Mat)(5, 1, CV_64F);
  cv::Size *image_size       = new(cv::Size);

  image_size->width = camera_calibration_data.width;
  image_size->height = camera_calibration_data.height;

  for(size_t i = 0; i < 3; i++)
    for(size_t j = 0; j < 3; j++)
    intrinsics->at<double>(i,j) = camera_calibration_data.K.at(3*i+j);

  if(camera_calibration_data.D.size() == 5)
  {
    for(size_t i = 0; i < 5; i++) {
      ROS_INFO_STREAM("camera_calibration_data.D.at(i) :" << i);
      distortion_coeff->at<double>(i,0) = camera_calibration_data.D.at(i);
    }
  }
  else {
    for(size_t i = 0; i < 5; i++) {
       distortion_coeff->at<double>(i,0) = 0.0;
    }
  }


  ROS_INFO_STREAM("Image width: " << image_size->width);
  ROS_INFO_STREAM("Image height: " << image_size->height);
  ROS_INFO_STREAM("Intrinsics:" << std::endl << *intrinsics);
  ROS_INFO_STREAM("Distortion: " << *distortion_coeff);




  //Load parameters to aruco_calib_param_ for aruco detection
  aruco_calib_params_.setParams(*intrinsics, *distortion_coeff, *image_size);

  //Simple check if calibration data meets expected values
  if ((intrinsics->at<double>(2,2) == 1) && (distortion_coeff->at<double>(0,4) == 0))
  {
    ROS_INFO_STREAM("Calibration data loaded successfully");
    return true;
  }
  else
  {
    ROS_WARN("Wrong calibration data, check calibration file and filepath");
    return false;
  }
}

void
ArucoMapping::imageCallback(const sensor_msgs::ImageConstPtr &original_image)
{
  //Create cv_brigde instance
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr=cv_bridge::toCvCopy(original_image, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Not able to convert sensor_msgs::Image to OpenCV::Mat format %s", e.what());
    return;
  }
  
  // sensor_msgs::Image to OpenCV Mat structure
  cv::Mat I = cv_ptr->image;
  
  // region of interest
  if(roi_allowed_==true)
    I = cv_ptr->image(cv::Rect(roi_x_,roi_y_,roi_w_,roi_h_));

  //Marker detection
  processImage(I,I);
  
  // Show image
  if (view_image_)
  {
    cv::imshow("Mono8", I);
    cv::waitKey(10);
  }
  /*cv::waitKey(2000);
  ros::shutdown();*/
}

bool ArucoMapping::processImage(cv::Mat input_image,cv::Mat output_image)
{
  aruco::MarkerDetector Detector;
  std::vector<aruco::Marker> temp_markers;

  //Set visibility flag to false for all markers
  for(size_t i = 0; i < num_of_markers_; i++)
      markers_[i].visible = false;

  // Save previous marker count
  marker_counter_previous_ = marker_counter_;

  // Detect markers
  Detector.detect(input_image,temp_markers,aruco_calib_params_,marker_size_);
    
  // If no marker found, print statement
  if(temp_markers.size() == 0)
    ROS_DEBUG("No marker found!");
  else
    first_marker_detected_ = true;
    
  //------------------------------------------------------
  // FOR EVERY MARKER DO
  //------------------------------------------------------
  std::string pkg_loc = ros::package::getPath("lidar_camera_calibration") + "/conf/transform.txt";
  std::ofstream outfile(pkg_loc.c_str(), std::ios_base::trunc);

  lidar_camera_calibration::marker_6dof marker_r_and_t;

  marker_r_and_t.header.stamp = ros::Time::now();
  marker_r_and_t.dof.data.clear();

  marker_r_and_t.num_of_markers = temp_markers.size();
  
  for(size_t i = 0; i < temp_markers.size();i++)
  {
    int current_marker_id = temp_markers[i].id;
    //ROS_INFO_STREAM("current_marker_id: " << current_marker_id);

    //Draw marker convex, ID, cube and axis
    temp_markers[i].draw(output_image, cv::Scalar(0,0,255),2);
    aruco::CvDrawingUtils::draw3dCube(output_image,temp_markers[i], aruco_calib_params_);
    aruco::CvDrawingUtils::draw3dAxis(output_image,temp_markers[i], aruco_calib_params_);

    marker_r_and_t.dof.data.push_back( temp_markers[i].id ); 
    marker_r_and_t.dof.data.push_back( temp_markers[i].Tvec.ptr<float>(0)[0] );
    marker_r_and_t.dof.data.push_back( temp_markers[i].Tvec.ptr<float>(0)[1] );
    marker_r_and_t.dof.data.push_back( temp_markers[i].Tvec.ptr<float>(0)[2] );
    marker_r_and_t.dof.data.push_back( temp_markers[i].Rvec.ptr<float>(0)[0] );
    marker_r_and_t.dof.data.push_back( temp_markers[i].Rvec.ptr<float>(0)[1] );
    marker_r_and_t.dof.data.push_back( temp_markers[i].Rvec.ptr<float>(0)[2] );

    markers_[i].current_camera_tf=arucoMarker2Tf(temp_markers[i]);

    tf::Vector3 marker_origin = markers_[i].current_camera_tf.getOrigin();
    markers_[i].current_camera_pose.position.x = marker_origin.getX();
    markers_[i].current_camera_pose.position.y = marker_origin.getY();
    markers_[i].current_camera_pose.position.z = marker_origin.getZ();

    tf::Quaternion marker_quaternion = markers_[i].current_camera_tf.getRotation();
    markers_[i].current_camera_pose.orientation.x = marker_quaternion.getX();
    markers_[i].current_camera_pose.orientation.y = marker_quaternion.getY();
    markers_[i].current_camera_pose.orientation.z = marker_quaternion.getZ();
    markers_[i].current_camera_pose.orientation.w = marker_quaternion.getW();
    
    markers_[i].current_camera_tf = arucoMarker2Tf(temp_markers[i]);
    
    string marker_frame_id = "marker" + to_string(current_marker_id);
    
    geometry_msgs::Pose marker_pose_msg;

    tf::poseTFToMsg(markers_[i].current_camera_tf, marker_pose_msg);
    
    broadcaster_.sendTransform(tf::StampedTransform(markers_[i].current_camera_tf, ros::Time::now(), camera_frame_id_, marker_frame_id));

       
   if(marker_id_1_ == current_marker_id){
      //marker_msg_pub1_.publish(marker_pose_msg);

  
      tf::StampedTransform odom_to_marker;
      try{
          // listener_->lookupTransform("odom", "safe_link", ros::Time(0), odom_to_marker);
          listener_->lookupTransform("/map", marker_frame_id, ros::Time(0), odom_to_marker);

          geometry_msgs::Pose odom_to_marker_msg;
          tf::poseTFToMsg(odom_to_marker, odom_to_marker_msg);

          tf::Quaternion q(odom_to_marker_msg.orientation.x, odom_to_marker_msg.orientation.y, odom_to_marker_msg.orientation.z, odom_to_marker_msg.orientation.w);
          tf::Quaternion qy(0.0, 0.7071, 0.0, 0.7071);
          
          tf::Quaternion qq = q*qy;
          
          odom_to_marker_msg.orientation.x = qq[0];
          odom_to_marker_msg.orientation.y = qq[1];
          odom_to_marker_msg.orientation.z = qq[2];
          odom_to_marker_msg.orientation.w = qq[3];
          // Eigen::Isometry3d To2m;
          // To2m.linear() = qq;
          // To2m.translation() = Eigen::Vector3d(odom_to_marker_msg.position.x, odom_to_marker_msg.position.y, odom_to_marker_msg.position.z);          

          marker_msg_pub1_.publish(odom_to_marker_msg);
      }
      catch (tf::TransformException &ex) {
          continue;
      }
   }
   if(marker_id_2_ == current_marker_id){
      //  marker_msg_pub2_.publish(marker_pose_msg);
      tf::StampedTransform odom_to_marker;

      try{
          listener_->lookupTransform(group_name_ + "_odom", marker_frame_id, ros::Time(0), odom_to_marker);

          geometry_msgs::Pose odom_to_marker_msg;
          tf::poseTFToMsg(odom_to_marker, odom_to_marker_msg);
          marker_msg_pub2_.publish(odom_to_marker_msg);
      }
       catch (tf::TransformException &ex) {
          continue;
      }
   }
   if(marker_id_3_ == current_marker_id){
      //  marker_msg_pub3_.publish(marker_pose_msg);
      tf::StampedTransform odom_to_marker;

      try{
          listener_->lookupTransform(group_name_ + "_odom", marker_frame_id, ros::Time(0), odom_to_marker);

          geometry_msgs::Pose odom_to_marker_msg;
          tf::poseTFToMsg(odom_to_marker, odom_to_marker_msg);
          marker_msg_pub3_.publish(odom_to_marker_msg);
      }
       catch (tf::TransformException &ex) {
          continue;
      }
   }
   if(marker_id_4_ == current_marker_id){
      //  marker_msg_pub4_.publish(marker_pose_msg);
      tf::StampedTransform odom_to_marker;

      try{
          listener_->lookupTransform(group_name_ + "_odom", marker_frame_id, ros::Time(0), odom_to_marker);

          geometry_msgs::Pose odom_to_marker_msg;
          tf::poseTFToMsg(odom_to_marker, odom_to_marker_msg);
          marker_msg_pub4_.publish(odom_to_marker_msg);
      }
       catch (tf::TransformException &ex) {
          continue;
      }
   }
   if(marker_id_5_ == current_marker_id){
      //  marker_msg_pub5_.publish(marker_pose_msg);
      tf::StampedTransform odom_to_marker;

      try{
          listener_->lookupTransform(group_name_ + "_odom", marker_frame_id, ros::Time(0), odom_to_marker);

          geometry_msgs::Pose odom_to_marker_msg;
          tf::poseTFToMsg(odom_to_marker, odom_to_marker_msg);
          marker_msg_pub5_.publish(odom_to_marker_msg);
      }
       catch (tf::TransformException &ex) {
          continue;
      }
   }
    
    
  }

  marker_r_and_t.dof.data.clear();
}

void
ArucoMapping::publishTfs(bool world_option)
{
  for(int i = 0; i < marker_counter_; i++)
  {
    // Actual Marker
    std::stringstream marker_tf_id;
    marker_tf_id << "marker_" << i;
    // Older marker - or World
    std::stringstream marker_tf_id_old;
    if(i == 0)
      marker_tf_id_old << "world";
    else
      marker_tf_id_old << "marker_" << markers_[i].previous_marker_id;
    broadcaster_.sendTransform(tf::StampedTransform(markers_[i].tf_to_previous,ros::Time::now(),marker_tf_id_old.str(),marker_tf_id.str()));

    // Position of camera to its marker
    std::stringstream camera_tf_id;
    camera_tf_id << "camera_" << i;
    broadcaster_.sendTransform(tf::StampedTransform(markers_[i].current_camera_tf,ros::Time::now(),marker_tf_id.str(),camera_tf_id.str()));

    if(world_option == true)
    {
      // Global position of marker TF
      std::stringstream marker_globe;
      marker_globe << "marker_globe_" << i;
      broadcaster_.sendTransform(tf::StampedTransform(markers_[i].tf_to_world,ros::Time::now(),"world",marker_globe.str()));
    }

    // Cubes for RVIZ - markers
    publishMarker(markers_[i].geometry_msg_to_previous,markers_[i].marker_id,i);
  }

  // Global Position of object
  if(world_option == true)
    broadcaster_.sendTransform(tf::StampedTransform(world_position_transform_,ros::Time::now(),"world","camera_position"));
}

////////////////////////////////////////////////////////////////////////////////////////////////

void
ArucoMapping::publishMarker(geometry_msgs::Pose marker_pose, int marker_id, int index)
{
  visualization_msgs::Marker vis_marker;

  if(index == 0)
    vis_marker.header.frame_id = "map";
  else
  {
    std::stringstream marker_tf_id_old;
    marker_tf_id_old << "marker_" << markers_[index].previous_marker_id;
    vis_marker.header.frame_id = marker_tf_id_old.str();
  }

  vis_marker.header.stamp = ros::Time::now();
  vis_marker.ns = "basic_shapes";
  vis_marker.id = marker_id;
  vis_marker.type = visualization_msgs::Marker::CUBE;
  vis_marker.action = visualization_msgs::Marker::ADD;

  vis_marker.pose = marker_pose;
  vis_marker.scale.x = marker_size_;
  vis_marker.scale.y = marker_size_;
  vis_marker.scale.z = RVIZ_MARKER_HEIGHT;

  vis_marker.color.r = RVIZ_MARKER_COLOR_R;
  vis_marker.color.g = RVIZ_MARKER_COLOR_G;
  vis_marker.color.b = RVIZ_MARKER_COLOR_B;
  vis_marker.color.a = RVIZ_MARKER_COLOR_A;

  vis_marker.lifetime = ros::Duration(RVIZ_MARKER_LIFETIME);

  marker_visualization_pub_.publish(vis_marker);
}

////////////////////////////////////////////////////////////////////////////////////////////////

tf::Transform
ArucoMapping::arucoMarker2Tf(const aruco::Marker &marker)
{
  cv::Mat marker_rotation(3,3, CV_32FC1);
  cv::Rodrigues(marker.Rvec, marker_rotation);
  cv::Mat marker_translation = marker.Tvec;

  cv::Mat rotate_to_ros(3,3,CV_32FC1);
  rotate_to_ros.at<float>(0,0) = 0.0;
  rotate_to_ros.at<float>(0,1) = 1.0;
  rotate_to_ros.at<float>(0,2) = 0.0;
  rotate_to_ros.at<float>(1,0) = -1.0;
  rotate_to_ros.at<float>(1,1) = 0.0;
  rotate_to_ros.at<float>(1,2) = 0.0;
  rotate_to_ros.at<float>(2,0) = 0.0;
  rotate_to_ros.at<float>(2,1) = 0.0;
  rotate_to_ros.at<float>(2,2) = 1.0;
  
marker_rotation = marker_rotation * rotate_to_ros.t();

  // Origin solution
  tf::Matrix3x3 marker_tf_rot(marker_rotation.at<float>(0,0),marker_rotation.at<float>(0,1),marker_rotation.at<float>(0,2),
                              marker_rotation.at<float>(1,0),marker_rotation.at<float>(1,1),marker_rotation.at<float>(1,2),
                              marker_rotation.at<float>(2,0),marker_rotation.at<float>(2,1),marker_rotation.at<float>(2,2));

  tf::Vector3 marker_tf_tran(marker_translation.at<float>(0,0),
                             marker_translation.at<float>(1,0),
                             marker_translation.at<float>(2,0));

  /*ROS_INFO_STREAM("in aruco mapping: \n" <<  marker << "\n");
  ROS_INFO_STREAM("Edited: \n" <<  marker_tf_tran[0] << "\n");*/
  return tf::Transform(marker_tf_rot, marker_tf_tran);


}



} 

#endif  //aruco_mapping_CPP
