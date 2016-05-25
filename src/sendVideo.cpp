/**

	Node for sending video images to the Trollserver over TCP. Can subscribe to the RGB images from the kinect
	or capture images from a local camera.

	Subscribes to:
		- /head/kinect2/rgb/image_color

*/


//ROS
#include "ros/ros.h"
#include <ros/callback_queue.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

//Misc
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <vector>

#include "opencv2/opencv.hpp"

//Sockets
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 


//ROS message files
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>


// NETWORK DETAILS
#define VIDEO_PORT 		8999 //Port used to send video to the Trollserver

//IP adress of Trollserver
#define DEFAULT_ADDRESS 	"129.241.154.37" 


//Namespace
using namespace cv;

//Ros topic names
std::string rgb_image_topic_name 		= "/head/kinect2/rgb/image_color";


//TCP variables for video
int 				videoSocket, videoPortNo;
bool 				videoTCPInit = false;
struct sockaddr_in 	videoServerAddr;
struct hostent 		*videoServer;



//Desired dimensions of the picture in pixels. The values are overwritten by those the Trollserver wants.
int camera_height = 0;
int camera_width = 0;

// For capturing images from local camera
VideoCapture *capturePointer = NULL;


// ================== INIT ================================
bool useLocalCamera = true; //true if capturing local camera and send to Trollserver. False if going to use kinect or no video.
float seconds_between_camera_capture = 0.1;





//Subscription function to the kinect. Resizes the images and sends them to the trollface.
void kinectImageHandler(const sensor_msgs::Image::ConstPtr & img)
{
	
	ROS_INFO("Received image from kinect");
	cv_bridge::CvImagePtr kinectPtr;
	kinectPtr = cv_bridge::toCvCopy(img,sensor_msgs::image_encodings::BGR8);
	
	
	
	//check if there have been any problems with the connection, and possibly reconnect.
	if ( (!(videoTCPInit)) ||  errno == ECONNRESET || errno == ETIMEDOUT || errno == EPIPE)
	{
		errno = 0;
		ROS_INFO("entered  connection loop Video");
		videoPortNo = VIDEO_PORT;
		videoSocket = socket(AF_INET, SOCK_STREAM, 0);

		if (videoSocket < 0)
			ROS_ERROR("ERROR opening socket");

		videoServer = gethostbyname(DEFAULT_ADDRESS);

		if (videoServer == NULL) {
			ROS_ERROR("ERROR, no such host\n");
			exit(0);
		}

		bzero((char *) &videoServerAddr, sizeof(videoServerAddr));

		videoServerAddr.sin_family = AF_INET;
		videoServerAddr.sin_port = htons(videoPortNo);


		bcopy((char *)videoServer->h_addr,
			 (char *)&videoServerAddr.sin_addr.s_addr,
			 videoServer->h_length);



		//Connecting to trollface over TCP
		if (connect(videoSocket,(struct sockaddr *) &videoServerAddr,sizeof(videoServerAddr)) < 0)
		{
			ROS_ERROR("ERROR connecting to Trollserver over video socket");
			close(videoSocket);
			videoTCPInit = false;
			

		}
		else
		{
			//Connection established, receiving desired camera capture properties from troll
			ROS_INFO("Created connection to Trollserver on video socket");
			
			//Receive desired height and width of camera.
			char buffer[256];
			bzero(buffer,256);
			

			int bytes = recv(videoSocket,buffer,255,0);
			

			if(bytes < 0)
			{
				ROS_ERROR("Could not receive camera properties");
			}
			else
			{
				ROS_INFO("Received camera properties: [%s]",buffer);
				videoTCPInit = true;
				
				std::stringstream ss(buffer);
				
				//first desired image width, then height;
				ss >> camera_width >> camera_height;
				
				ROS_INFO("Desired image dimensions are: Width = %d, Height = %d",camera_width, camera_height);
				

			}

		}
	}
	

	//transform subscribed image and send over TCP
	if(videoTCPInit)
	{

		Mat frame = Mat(camera_height,camera_width,CV_8UC1);
		
		
		ROS_INFO("Received image size: [%lu]", kinectPtr->image.total()*kinectPtr->image.elemSize());
		
		resize(kinectPtr->image,frame,frame.size(), 0,0,INTER_AREA);
		
		if(true) //check if transform is successfull later
		{

			int imgSize = frame.total() * frame.elemSize();
			int bytes = 0;
			

			IplImage iplimg = frame;
			
			cvShowImage("testwindow",&iplimg);
			waitKey(10);

			//send processed image
			if ((bytes = send(videoSocket, iplimg.imageData, imgSize, 0)) < 0)
			{
				 ROS_ERROR("Error writing to video socket, errorno: [%s]", strerror(errno));
			}
			else
			{
				ROS_INFO("Sent image with bytes = [%d] over TCP",bytes);
				frame.release();
			}
		}
		else
			ROS_ERROR("unable to transform kinect image");


	}
}



//Test function. Captures images from local camera and sends over TCP to troll.
void LocalCameraHandler(const ros::TimerEvent& event)
{
	//Only do  if flag for using the local camera is set
	if(useLocalCamera)
	{

		//check if there have been any problems with the connection, and possibly reconnect.
		if ( (!(videoTCPInit)) ||  errno == ECONNRESET || errno == ETIMEDOUT || errno == EPIPE)
		{
			errno = 0;
			ROS_INFO("entered  connection loop Video");
			videoPortNo = VIDEO_PORT;
			videoSocket = socket(AF_INET, SOCK_STREAM, 0);

			if (videoSocket < 0)
				ROS_ERROR("ERROR opening socket");

			videoServer = gethostbyname(DEFAULT_ADDRESS);

			if (videoServer == NULL) {
				ROS_ERROR("ERROR, no such host\n");
				exit(0);
			}

			bzero((char *) &videoServerAddr, sizeof(videoServerAddr));

			videoServerAddr.sin_family = AF_INET;
			videoServerAddr.sin_port = htons(videoPortNo);


			bcopy((char *)videoServer->h_addr,
				 (char *)&videoServerAddr.sin_addr.s_addr,
				 videoServer->h_length);



			//Connecting to trollface over TCP
			if (connect(videoSocket,(struct sockaddr *) &videoServerAddr,sizeof(videoServerAddr)) < 0)
			{
				ROS_ERROR("ERROR connecting to Trollserver over video socket");
				close(videoSocket);
				videoTCPInit = false;
				

			}
			else
			{
				//Connection established, receiving desired camera capture properties from troll
				ROS_INFO("Created connection to Trollserver on video socket");
				
				//Receive desired height and width of camera.
				char buffer[256];
				bzero(buffer,256);
				
				int bytes = recv(videoSocket,buffer,255,0);
				
				if(bytes < 0)
				{
					ROS_ERROR("Could not receive camera properties");
				}
				else
				{
					ROS_INFO("Received camera properties: [%s]",buffer);
					videoTCPInit = true;
					
					std::stringstream ss(buffer);
					
					//first desired image width, then height;
					ss >> camera_width >> camera_height;
					
					ROS_INFO("Desired image dimensions are: Width = %d, Height = %d",camera_width, camera_height);
					capturePointer->open(0);
					capturePointer->set(CV_CAP_PROP_FRAME_WIDTH , camera_width);
					capturePointer->set(CV_CAP_PROP_FRAME_HEIGHT , camera_height);
					
				}

			}
		}
		

		//Capture frame from camera and send over TCP
		if(videoTCPInit)
		{
			if(capturePointer->isOpened())
			{
				Mat frame;			
				
				if(capturePointer->read(frame))
				{
					int imgSize = frame.total() * frame.elemSize();
					int bytes = 0;
					
					IplImage iplimg = frame;
					
					cvShowImage("testwindow",&iplimg);
					waitKey(10);

					//send processed image
					if ((bytes = send(videoSocket, iplimg.imageData, imgSize, 0)) < 0)
					{
						 ROS_ERROR("Error writing to video socket, errorno: [%s]", strerror(errno));
					}
					else
						ROS_INFO("Sent local image over TCP, bytes = %d",bytes);
				}
				else
					ROS_ERROR("Unable to capture frame from camera");

			}
			else
			{
				ROS_ERROR("Camera is not opened");
			}
		}
	}
}




int main(int argc, char *argv[])
{


	ros::init(argc, argv, "sendVideo");
	ros::NodeHandle n;

	//Subscriber
	ros::Subscriber kinect_img  = n.subscribe(rgb_image_topic_name,10,kinectImageHandler); //For subscribing to the kinect
	
	ros::Timer local_camera = n.createTimer(ros::Duration(seconds_between_camera_capture),LocalCameraHandler); //Captures images from local camera. For testing image TCP and trollserver.
	
	if (useLocalCamera){
		ROS_INFO("Tries to capture camera");
	
		VideoCapture cap(0); // open the default camera

		if(cap.isOpened())
		{
			ROS_INFO("Captured the camera");
			capturePointer = &cap;
		}
		else
		{
			ROS_ERROR("Could not capture camera");
			capturePointer = NULL;
		}
	}
	
	ros::spin();
	
	return 0;

}
