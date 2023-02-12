/**
* \file detection_node.cpp
* \brief Node for moving and detecting
* \author Davide Bruzzo, Carmine Recchiuto
* \version 0.1
* \date 29/01/2023
* \param [in] world_width Define the width of the discretized world.
*
* \details
*
* Subscribes to: <BR>
* ° /robot/camera1/image_raw
*
* Publishes to: <BR>
* ° /robot/joint1_position_controller/command
* ° /robot/joint2_position_controller/command
* ° /robot/joint3_position_controller/command 
* ° /info_room_sm
*
* Services : <BR>
* ° /room_info
*
* Description :
*
* This node moves the arm of the robot in order to make camera pointing the ArUco markers,
* Then implenting the client for the room_info server which is in the marker_server.cpp obtains the rooms' informations.
*/

// ROS headers
#include <ros/ros.h>
#include <ros/topic.h>
#include <stdlib.h>
#include<std_msgs/Float64.h>
#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <assignment2/RoomInformation.h>
#include <assignment2/InfoRoom.h>
#include <assignment2/RoomConnection.h>


std_msgs::Float64 position; //!< Value to publish on the joint controllers


ros::Publisher pub_joint_1; //!< Publisher for joint 1
ros::Publisher pub_joint_2; //!< Publisher for joint 2
ros::Publisher pub_joint_3; //!< Publisher for joint 3

ros::Publisher pub_info_room; //!< Publisher for room info topic

// room service client
ros::ServiceClient roomInfoClient;

/*!
 * \class MyArucoMarkerPublisher
 *
 * \brief A class for identifying markers in camera photos and publishing room information.
 *
 * This class detects the ArUco markers, takes room informations stored in them through `/room_info` of `marker_server`
 *  and publish those informations in the `/room_info` topic.
 *
 */

class MyArucoMarkerPublisher
{
private:
  // ArUco stuff
  aruco::MarkerDetector mDetector_;
  std::vector<aruco::Marker> markers_;
  aruco::CameraParameters camParam_;

  std::map<int, bool> detected_marker;   //!< Map to store if a marker has been detected or not
  std::vector<int> id_marker;           //!< Vector to store the ids of the detected markers
  int markers_number;

  assignment2::RoomInformation srv;


  // node params
  double marker_size_;
  bool useCamInfo_;

  // ROS pub-sub
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;


  cv::Mat inImage_;

public:
  /**
     * @brief Constructor
     * Initializes the objects and subscribes to the '/robot/camera1/image_raw' topic
     */
  MyArucoMarkerPublisher() :
      nh_("~"), it_(nh_), useCamInfo_(true)
  {
    image_sub_ = it_.subscribe("/robot/camera1/image_raw", 1, &MyArucoMarkerPublisher::image_callback, this);

    nh_.param<bool>("use_camera_info", useCamInfo_, false);
    camParam_ = aruco::CameraParameters();
  }

  void image_callback(const sensor_msgs::ImageConstPtr& msg)
  {
    /**
* \brief Image callback.
* \param msg sensor_msgs type
*
* This function is the callback of the camera that is called every time a marker is detected. Once called it creates the client for the marker_server.
* In this way it obtains the informations of the room related to that marker, and sends them to my_helper node.
*/

    ros::Time curr_stamp = msg->header.stamp;
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      inImage_ = cv_ptr->image;

      // clear out previous detection results
      markers_.clear();

      // ok, let's detect
      mDetector_.detect(inImage_, markers_, camParam_, marker_size_, false);


        for (std::size_t i = 0; i < markers_.size(); ++i)
        {


          int marker_id = markers_[i].id;

          if(!detected_marker[marker_id])
            {
                detected_marker[marker_id] = true;

                std::cout << "The id of the detected marker detected is: ";
          	std::cout << markers_.at(i).id << " ";
          	std::cout << std::endl;

                srv.request.id = markers_.at(i).id;

          	if(roomInfoClient.call(srv)){

          		      // print the response
                    printf("Room server response:\n Room: %s;\n Coord: (%f,%f);\n", srv.response.room.c_str(), srv.response.x, srv.response.y);
                    for (std::size_t j = 0; j < srv.response.connections.size(); ++j){
                        printf("Connected to room: %s Through door: %s;\n", srv.response.connections.at(j).connected_to.c_str(), srv.response.connections.at(j).through_door.c_str());
                    }

          		assignment2::InfoRoom room_info_msg;

          		room_info_msg.room = srv.response.room;
          		room_info_msg.x = srv.response.x;
          		room_info_msg.y = srv.response.y;

          		for(std::size_t j = 0; j < srv.response.connections.size(); j++){

          			room_info_msg.connections.push_back(srv.response.connections.at(j));

          		}
          		pub_info_room.publish(room_info_msg);
          	}

        }
       }



    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }
};




int main(int argc, char** argv)
{
  // Ilnit the ROS node
  ros::init(argc, argv, "detection_node");

  ROS_INFO("Starting moving arm ...");

  // Precondition: Valid clock
  ros::NodeHandle nh;

  pub_joint_1 = nh.advertise<std_msgs::Float64>("/robot/joint1_position_controller/command",100);
  pub_joint_2 = nh.advertise<std_msgs::Float64>("/robot/joint2_position_controller/command",100);
  pub_joint_3 = nh.advertise<std_msgs::Float64>("/robot/joint3_position_controller/command",100);

  pub_info_room = nh.advertise<assignment2::InfoRoom>("/info_room_sm",100);

  while(pub_info_room.getNumSubscribers() < 1)
  {
  	ros::Duration(0.1).sleep();
  }

  roomInfoClient = nh.serviceClient<assignment2::RoomInformation>("/room_info");
  roomInfoClient.waitForExistence();

  MyArucoMarkerPublisher node;

  int flagCycle = 0;


   while(ros::ok()){


   while(flagCycle!=-1){

	   	if(flagCycle < 30){
	   	position.data = 6.28;
	  	pub_joint_1.publish(position);
	   	//printf("Rotation\n");

	   	}

	   	if(flagCycle > 30 && flagCycle < 50){
	   	position.data = 0.0;
	  	pub_joint_1.publish(position);
	  	position.data = 0.4;
	  	pub_joint_3.publish(position);

	   	}
	   	if(flagCycle == 50){
	   		flagCycle = -1;
	   		printf("FULL ROTATION DONE\n");
	   		break;
	   	}

	   flagCycle++;
	   ros::Duration(1.0).sleep();
	   ros::spinOnce();


	}

   ros::spinOnce();

   }

  return 0;
 }
