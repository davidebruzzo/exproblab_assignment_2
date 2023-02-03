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


std_msgs::Float64 position;


ros::Publisher pub_joint_1;
ros::Publisher pub_joint_2;
ros::Publisher pub_joint_3;

ros::Publisher pub_info_room;

// room service client
ros::ServiceClient roomInfoClient;


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

  //????????????????????????????????????????????

 //

  cv::Mat inImage_;

public:
  MyArucoMarkerPublisher() :
      nh_("~"), it_(nh_), useCamInfo_(true)
  {
    image_sub_ = it_.subscribe("/robot/camera1/image_raw", 1, &MyArucoMarkerPublisher::image_callback, this);
    //image_pub_ = it_.advertise("result", 1);


    nh_.param<bool>("use_camera_info", useCamInfo_, false);
    camParam_ = aruco::CameraParameters();
  }

  void image_callback(const sensor_msgs::ImageConstPtr& msg)
  {
    //bool publishImage = image_pub_.getNumSubscribers() > 0;

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
          	//srv.request.id = markers_.at(i).id;
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
  // Init the ROS node
  ros::init(argc, argv, "move_prova");

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
	   	if(flagCycle > 50 && flagCycle < 60){
	   	position.data = 1.57;
	  	pub_joint_1.publish(position);
	  	position.data = 1.57;
	  	pub_joint_2.publish(position);
	  	position.data = 1.4;
	  	pub_joint_3.publish(position);
	   	}
	   	if(flagCycle == 60){
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
