#include "qingColor.h"
//the test of time code
//#include <sys/time.h>
//#include <string.h>
///

// for thread and timer:
#include <iostream>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>


//for json communication
// for zmq:
#include <zmq.h>
//#include <zmq_api.h>
// for json:
#include <json/json.h>


//for serial communication


#include <stdio.h>     
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h> 
#include <fcntl.h>
#include <sstream>
#include <assert.h>

#include <termios.h> 
#include <errno.h>      

//The lib file that zmq requires
#include <cmath>

#define KEY_ACTUATOR "Actuator"

#define TIMER1_PERIOD 20
#define TIMER2_PERIOD 20
#define TIMER3_PERIOD 100
#define TIMER4_PERIOD 100

#define SERIAL_SENDING_SLEEP 1000

// -------------------- Changxy -----------------------------
struct DGPS{
    double utc_second;
	double latitude;
	char latdir;
	double longitude;
	char londir;
	double heading;
	int satNum;
	int status;
}gps;
int hex2dec(char ch){
    if('0'<=ch && ch<='9') return ch-'0';
    if('A'<=ch && ch<='F') return ch-'A'+10;
    return -1;
}
bool str2double(char ch[],int st,int ed,double &ans){
    int i=st;
    if(st>ed) return false;
    ans=0;
    if(ch[i]=='-') i++;
    while(i<=ed && ch[i]!='.'){
        if(ch[i]<'0' || '9'<ch[i]) return false;
        ans=ans*10+ch[i++]-'0';
    }
    if(i<=ed){
        // get decimal
        double t=0.1;
        i++;
        while(i<=ed){
            if(ch[i]<'0' || '9'<ch[i]) return false;
            ans=ans+(ch[i++]-'0')*t; t*=0.1;
        }
    }
    if(ch[st]=='-') ans=-ans;
    return true;
}
//---------------------End of Changxy-------------------------
  
  
int open_port(char* dev)
{
	int fd;
	fd = open(dev,O_RDWR);
	if(-1 == fd)
	{
		perror("Cannot open port.");
		return -1;
	}
	else
		return fd;
}

void set_speed(int fd, int speed)
{
	int speed_arr[] = { B115200,B38400, B19200, B9600, B4800, B1200, B300, B38400, B19200, B9600, B4800, B1200, B300};
	int name_arr[] = { 115200,38400, 19200, 9600, 4800, 1200, 300, 38400, 19200, 9600, 4800, 1200, 300};
	int i;
	int status;
	struct termios Opt;
	tcgetattr(fd, &Opt);
	for( i = 0; i < (sizeof(speed_arr) / sizeof(int)); i++)
	{
		if (speed == name_arr[i])
		{
			tcflush(fd, TCIOFLUSH);
			cfsetispeed(&Opt, speed_arr[i]);
			cfsetospeed(&Opt, speed_arr[i]);
			status = tcsetattr(fd, TCSANOW, &Opt);
			if (status != 0)
			{
				perror("tcsetattr fd");
				std::cout<<"set_speed err"<<std::endl;
				return;
			}
			tcflush(fd,TCIOFLUSH);
		}
	}
//	printf("speed is setted to %d.\n", speed);
}

int set_port(int fd,int databits,int stopbits,char parity)
{
	struct termios options;
	if  ( tcgetattr( fd,&options)  !=  0)
	{
	        perror("SetupSerial 1");    
	        return -1; 
	}
	options.c_cflag &= ~CSIZE;
	switch (databits)
	{  
		case 7:    
		        options.c_cflag |= CS7;
	        	break;
		case 8:    
        		options.c_cflag |= CS8;
	        	break; 
		default:
        		fprintf(stderr,"Unsupported data size\n"); 
			return 0; 
	}

	switch (parity)
	{  
		case 'n':
		case 'N':   
        		options.c_cflag &= ~PARENB;  
        		options.c_iflag &= ~INPCK;    
	        	break; 
		case 'o':  
		case 'O':    
        		options.c_cflag |= (PARODD | PARENB);
        		options.c_iflag |= INPCK;           
	        	break; 
		case 'e': 
		case 'E':  
        		options.c_cflag |= PARENB;    
        		options.c_cflag &= ~PARODD;    
        		options.c_iflag |= INPCK;            
			break;
		case 'S':
		case 's': 
			options.c_cflag &= ~PARENB;
			options.c_cflag &= ~CSTOPB;
			break; 
		default:  
			fprintf(stderr,"Unsupported parity\n");   
			return -1; 
    } 

	switch (stopbits)
	{  
		case 1:
			options.c_cflag &= ~CSTOPB; 
			break; 
		case 2:
			options.c_cflag |= CSTOPB; 
			break;
		default:   
			fprintf(stderr,"Unsupported stop bits\n"); 
			return -1;

	}

	if (parity != 'n')  
		options.c_iflag |= INPCK;

	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  // 141111: Binary mode to disable auto adding '0d' before '0a'
  options.c_oflag &= ~(INLCR|IGNCR|ICRNL);
  options.c_oflag &= ~(ONLCR|OCRNL);

	tcflush(fd,TCIFLUSH);
	options.c_cc[VTIME] = 150;
	options.c_cc[VMIN] = 0;
	if (tcsetattr(fd,TCSANOW,&options) != 0)  
	{
		perror("SetupSerial 3");  
		return -1; 
	}
	printf("The port is working at: %d, %c, %d.\n",databits, parity, stopbits);
	return 0; 
}



class Jobs
{
	public:
	explicit Jobs(boost::asio::io_service& io, int f, std::vector<void*>& receivingSocketList, std::vector<void*>& sendingSocketList) : strand1(io), strand2(io), strand3(io), strand4(io), timer1(io, boost::posix_time::milliseconds(5)), timer2(io, boost::posix_time::milliseconds(10)), timer3(io, boost::posix_time::milliseconds(13)), timer4(io, boost::posix_time::milliseconds(16)), fd(f), rSocketList(receivingSocketList), sSocketList(sendingSocketList) 
	{
	  timer1.async_wait(strand1.wrap(boost::bind(&Jobs::serialSubscriber, this)));
	  //timer2.async_wait(strand1.wrap(boost::bind(&Jobs::socketSubscriber, this)));
	  //timer3.async_wait(strand1.wrap(boost::bind(&Jobs::serialPublisher, this)));
	  timer4.async_wait(strand1.wrap(boost::bind(&Jobs::socketPublisher, this)));
	}

	~Jobs()
	{
	  ;
	}

	#define msgSize 300
	void serialSubscriber()
	{
		// read a valid message
		char startch,ch,msg[msgSize]="",header[20]="";
		int msglen,tmp,comma[20],commaNum,i;
		uint8_t checksum,nowsum;
		double dtmp,tmpdeg;

		while(true)
		{
			// if received a valid message then break
			// else continue
			// read a message
			read(fd,&ch,1);
printf("%c",ch);
		}
    	timer1.expires_at(timer1.expires_at() + boost::posix_time::milliseconds(TIMER1_PERIOD));
		timer1.async_wait(strand1.wrap(boost::bind(&Jobs::serialSubscriber, this)));
	}

		


	    void socketPublisher()
	    {
			Json::Value values;

			values["Gps"]["Latitude"] = gps.latitude;
			//values["Gps"]["Latdir"] = gps.latdir;
			values["Gps"]["Longitude"] = gps.longitude;
			//values["Gps"]["Londir"] = gps.londir;
			values["Gps"]["Status"] = gps.status;
			values["Gps"]["Heading"] = gps.heading;
			values["Gps"]["IsValid"] = true;
			values["Gps"]["UtcSecond"] = gps.utc_second;

			Json::FastWriter fastWriter;
			std::string jsonString = fastWriter.write(values);
			//jsonString.erase (std::remove(jsonString.begin(), jsonString.end(), '\n'), jsonString.end());
			zmq_send(sSocketList[0], jsonString.c_str(), strlen(jsonString.c_str()), 0);
			//      std::cout << "No." << ++counter <<std::endl<<"Sent: " << jsonString << std::endl;
			printf("UTC=%i:%i:%4.1lf  ",(int16_t)(gps.utc_second/3600),(int16_t)(fmod(gps.utc_second,3600)/60),fmod(gps.utc_second,60));
			printf("lat=%.8lf lon=%.8lf hdg=%.2lf status=%d\n",gps.latitude,gps.longitude,gps.heading,gps.status);
			timer4.expires_at(timer4.expires_at() + boost::posix_time::milliseconds(TIMER4_PERIOD));
			timer4.async_wait(strand4.wrap(boost::bind(&Jobs::socketPublisher, this)));
	    }

		private:
	    boost::asio::strand strand1;
	    boost::asio::strand strand2;
	    boost::asio::strand strand3;
	    boost::asio::strand strand4;
	    boost::asio::deadline_timer timer1;
	    boost::asio::deadline_timer timer2;
	    boost::asio::deadline_timer timer3;
	    boost::asio::deadline_timer timer4;

	    std::vector<void*> rSocketList;
	    std::vector<void*> sSocketList;
	    int fd;
	    int counter = 0;
};


int main()
{
	std::cout << "program started." << std::endl;
	int fd;
	fd = open("/dev/ttyTHS3", O_RDWR);
	if( -1 == fd)
	{
		printf("Error opening the port!");
	}
	else
	{
		printf("Serial port opened\n");
	}
	//  assert(-1 == fd);
	set_speed(fd, 115200);
	if(-1 == set_port(fd,8,1,'N'))
	{
		printf("Set port error.\n");
	}
	else;
	tcflush(fd, TCIOFLUSH);  
	// char ch;while(true)read(fd,&ch,1),printf("%c",ch);
	//zmq communication
	void* context = zmq_ctx_new ();

	// zmq receiving sockets
	void* commandSocket = zmq_socket (context, ZMQ_SUB);  
	zmq_connect (commandSocket, "tcp://192.168.1.31:6970");
	//  zmq_connect (commandSocket, "tcp://192.168.1.33:6970");
	//  zmq_connect (commandSocket, "tcp://192.168.1.217:6970");
	//  zmq_connect (commandSocket, "tcp://127.0.0.1:6970");
	zmq_setsockopt (commandSocket, ZMQ_SUBSCRIBE, "", 0);
	std::vector<void*> receivingSocketList;
	receivingSocketList.push_back(commandSocket);

	// zmq sending sockets
	void* broadcastingSocket = zmq_socket (context, ZMQ_PUB);
	zmq_bind(broadcastingSocket, "tcp://*:5557");  
	std::vector<void*> sendingSocketList;
	sendingSocketList.push_back(broadcastingSocket);

	// thread initial
	boost::asio::io_service io;
	Jobs jobs(io, fd, receivingSocketList, sendingSocketList);
	boost::thread t(boost::bind(&boost::asio::io_service::run, &io));
	io.run();
	t.join();

	//printf("Stop to send \n");
	//canetUdpS.closeUdpCanet();
	return 0;
}

