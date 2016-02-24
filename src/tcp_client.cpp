#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <trollnode/Expression.h>
#include "trollnode/expression_templates.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <vector>


#define DEFAULT_PORT 8888
#define DEFAULT_ADRESS "itk-d913.win.ntnu.no"
#define BUFFER_LENGTH 1024

int sockfd, portno;
struct sockaddr_in serv_addr;
struct hostent *server;







std::string getAUName(int AU)
{
	switch(AU)
	{
		case 1: return "AU01innerbrowraiser";
			break;
		case 2: return "AU02outerbrowraiser";//more commands exist for this AU
			break;
		case 3: return "AU03procerus";
			break;
		case 4: return "AU04browlowerer";
			break;
		case 5: return "AU05upperlidraiser";
			break;
		case 6: return "AU06cheekraiser";
			break;
		case 7: return "AU07lidtightener";
			break;
		case 8: return "AU08liptowardeachother";
			break;
		case 9: return "AU09nosewrinkler";
			break;
		case 10: return "AU10upperlipraiser";
			break;
		case 11: return "AU11nasolabialfurrowdeepener";
			break;
		case 12: return "AU12lipcornerpuller";//more commands exist for this AU
			break;
		case 13: return "AU13cheekpuffer";
			break;
		case 14: return "AU14dimpler";
			break;
		case 15: return "AU15lipcornerdepressor";
			break;
		case 16: return "AU16lowerlipdepressor";
			break;
		case 17: return "AU17chinraiser";
			break;
		case 18: return "AU18lippuckerer";
			break;
		case 19: return "AU19tongueshow";
			break;
		case 20: return "AU20lipstretcher";
			break;
		case 21: return "AU21necktightener";
			break;
		case 22: return "AU22lipfunneler";
			break;
		case 23: return "AU23liptightener";
			break;
		case 24: return "AU24lippressor";
			break;
		case 25: return "AU25lipspart";
			break;
		case 26: return "AU26jawopen";
			break;
		case 27: return "AU27mouthstretch";
			break;
		case 28: return "AU28lipsuck";
			break;
		case 29: return "AU29jawthrust";//more commands exist for this AU
			break;
		case 30: return "AU30jawsidewaysL";//more commands exist for this AU
			break;
		case 31: return "AU31jawclencher";
			break;
		case 32: return "AU32bite";
			break;
		case 33: return "AU33blow";
			break;
		case 34: return "AU34puff";
			break;
		case 35: return "AU35cheeksuck";
			break;
		case 36: return "AU36tonguebuldgeL"; //more commands exist for this AU
			break;
		case 37: return "AU37lipwipeM"; //more commands exist for this AU
			break;
		case 38: return "AU38nostrildilate";
			break;
		case 39: return "AU39nostrilcompress";
			break;
		case 40: return "";
			break;
		case 41: return "AU41liddrop";
			break;
		case 42: return "AU42slit";
			break;
		case 43: return "AU43eyesclosed";
			break;
		case 44: return "AU44squint";
			break;
		case 45: return "AU45blink"; //more commands exist for this AU
			break;
		case 46: return "AU46winkR";
			break;
		case 47: return "AU47winkL";
			break;
		case 51: return "AU51turnleft";
			break;
		case 52: return "AU52turnright";
			break;
		case 53: return "AU53headup";
			break;
		case 54: return "AU54headdown";
			break;
		case 55: return "AU55tiltleft";
			break;
		case 56: return "AU56tiltright";
			break;
		case 57: return "AU57headforward";
			break;
		case 58: return "AU58headback";
			break;
		case 66: return "AU66crosseye";
			break;
		default: return "";
			break;
	}
}




std::string createExprMsg(const std::string & expression)
{
	std::string msgString;
	std::string tempString;
	float (*action_units)[2];
	int arraySize;
	
	//Select approperiate message
	if (expression.compare("angry") == 0)
	{
		arraySize = sizeof(angry)/sizeof(angry[0]);
		action_units = angry;
	}
	else if (expression.compare("smile") == 0)
	{
		arraySize = sizeof(smile)/sizeof(smile[0]);
		action_units = smile;
	}
	else if (expression.compare("happy") == 0)
	{
		arraySize = sizeof(happy)/sizeof(happy[0]);
		action_units = happy;
	}
	else if (expression.compare("sad") == 0)
	{
		arraySize = sizeof(sad)/sizeof(sad[0]);
		action_units = sad;
	}
	else if (expression.compare("blink") == 0)
	{
		arraySize = sizeof(blink)/sizeof(blink[0]);
		action_units = blink;
	}
	else if (expression.compare("surprise") == 0)
	{
		arraySize = sizeof(surprise)/sizeof(surprise[0]);
		action_units = surprise;
	}
	else if (expression.compare("suspicious") == 0)
	{
		arraySize = sizeof(suspicious)/sizeof(suspicious[0]);
		action_units = suspicious;
	}
	else if (expression.compare("disgust") == 0)
	{
		arraySize = sizeof(disgust)/sizeof(disgust[0]);
		action_units = disgust;
	}
	else if (expression.compare("fear") == 0)
	{
		arraySize = sizeof(fear)/sizeof(fear[0]);
		action_units = fear;
	}
	else if (expression.compare("duckface") == 0)
	{
		arraySize = sizeof(duckface)/sizeof(duckface[0]);
		action_units = duckface;
	}
	else if (expression.compare("pain") == 0)
	{
		arraySize = sizeof(pain)/sizeof(pain[0]);
		action_units = pain;
	}
	else //No recognized expression
	{
		ROS_ERROR("Did not recognize expression [%s], no msg created.",expression.c_str());
		return "";
	}
		
	
	std::stringstream ss;
	std::size_t pos;
	
	
	for(int i = 0; i < arraySize;i++)
	{
		tempString = msg_template;
		tempString.insert(1,getAUName(action_units[i][0]));
		msgString.append(tempString);
		
		
		ss.str("");
		ss << action_units[i][1];

		//adding intensity
		while( (pos = msgString.find("$i")) !=std::string::npos)
		{
			msgString.replace(pos,2,ss.str());
		}
		
	}
	
	return msgString;

}





std::string createAUString(int AU, float startInt, float stopInt, float stopTime)
{
	std::stringstream AUString;
	
	AUString << "[" << getAUName(AU) << "0," << startInt << "," << stopTime << "," << stopInt << "]";
	
	return AUString.str();
	
}




void sendMsg(const trollnode::Expression::ConstPtr& msg)
{
	int n;
	std::string msgString;


	if (msg->peak_time < 0 || msg->peak_time > 10 ||  msg->peak_duration < 0 || msg->peak_duration >10 || msg->fade_time < 0 || msg->fade_time > 10)
	{
		ROS_INFO("invalid content in expression msg.");
		return;
	}
	

	ROS_INFO("Received ROS message: [%s]", msg->expression.c_str());
	msgString = createExprMsg(msg->expression);

	
	std::size_t pos;
	std::stringstream ss;
	
	ss << msg->peak_time;


	// handle peak time
	while( (pos = msgString.find("$p")) !=std::string::npos)
	{
		msgString.replace(pos,2,ss.str());
	}
	

	// handle peak duration
	ss.str(""); //clearing stringstream
	ss << (msg->peak_duration + msg->peak_time);

	while( (pos = msgString.find("$d")) !=std::string::npos)
	{
		msgString.replace(pos,2,ss.str());

	}

	// handle fade time
	ss.str(""); //clearing stringstream
	ss << (msg->fade_time + msg->peak_duration + msg->peak_time);

	while( (pos = msgString.find("$f")) !=std::string::npos)
	{
		msgString.replace(pos,2,ss.str());

	}

	// insert speech
	msgString.insert(0,msg->speech.c_str());

	ROS_INFO("Message: [%s]", msgString.c_str());
	
	

	n = write(sockfd,msgString.c_str(),strlen(msgString.c_str()));
	if (n < 0)
	{
		ROS_ERROR("ERROR writing to socket");
	    
	}

	/*bzero(buffer,256);
	n = read(sockfd,buffer,255);
	if (n < 0) 
	     error("ERROR reading from socket");
	printf("%s\n",buffer);*/

}




int main(int argc, char *argv[])
{


	ros::init(argc, argv, "tcp_client");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("trollTopic",1000,sendMsg);
    char buffer[BUFFER_LENGTH];

    //portno = atoi(argv[2]);
	portno = DEFAULT_PORT;
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) 
        ROS_ERROR("ERROR opening socket");
    server = gethostbyname(DEFAULT_ADRESS);
    if (server == NULL) {
        ROS_ERROR("ERROR, no such host\n");
        exit(0);
    }
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, 
         (char *)&serv_addr.sin_addr.s_addr,
         server->h_length);
    serv_addr.sin_port = htons(portno);
    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) 
        ROS_ERROR("ERROR connecting");

	ros::spin();
	return 0;

}
