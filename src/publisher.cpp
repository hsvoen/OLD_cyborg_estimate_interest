#include "ros/ros.h"
#include "std_msgs/String.h"
#include "trollnode/Expression.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sstream>


#define BUFFER_LENGTH 1024
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "publisher");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<trollnode::Expression>("trollTopic", 1000);

  ros::Rate loop_rate(0.1);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  int i = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    	
	trollnode::Expression expr_msg;
	//char ss[256];
    
	
	//bzero(ss,256);
    
	//printf("Please enter message to send");
	//gets(ss);

    //msg.data = ss;
	
	if (i == 0 )
		expr_msg.expression = "happy";
	else if (i == 1)
		expr_msg.expression = "angry";
	else if (i == 2)
		expr_msg.expression = "smile";
	else if (i == 3)
		expr_msg.expression = "sad";
	else if (i == 4)
		expr_msg.expression = "disgust";
	else if (i == 5)
		expr_msg.expression = "surprise";
	else if (i == 6)
		expr_msg.expression = "fear";
	else if (i == 7)
		expr_msg.expression = "suspicious";
	else if (i == 8)
		expr_msg.expression = "blink";
	else if (i == 9)
		expr_msg.expression = "pain";
	else if (i == 10)
		expr_msg.expression = "duckface";
	else
	{
		i = 0;
		expr_msg.expression = "happy";
	}
	
	i = i + 1;
	expr_msg.speech = "";
	expr_msg.peak_time = 1;
	expr_msg.peak_duration = 2;
	expr_msg.fade_time = 1;

    ROS_INFO("%s", expr_msg.expression.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(expr_msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
