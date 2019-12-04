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
#include <string>
using namespace std;

#include <termios.h> 
#include <errno.h>      

//The lib file that zmq requires
#include <cmath>

//QXWZ
#include <time.h>
#include <pthread.h>

#include "qxwz_rtcm.h"

#undef QXLOGI
#define QXLOGI printf

qxwz_account_info *p_account_info = NULL;
qxwz_config config;
RTCM_data_trans data_trans;

#define KEY_ACTUATOR "Actuator"

#define TIMER1_PERIOD 50
#define TIMER2_PERIOD 100
#define TIMER3_PERIOD 100
#define TIMER4_PERIOD 100

#define SERIAL_SENDING_SLEEP 1000

//QX
void get_qxwz_sdk_account_info(void) {
    p_account_info = getqxwzAccount();
    if(p_account_info->appkey != NULL) {
        printf("appkey=%s\n",p_account_info->appkey);
    }
    if(p_account_info->deviceID != NULL) {
        printf("deviceID=%s\n",p_account_info->deviceID);
    }
    if(p_account_info->deviceType != NULL) {
        printf("deviceType=%s\n",p_account_info->deviceType);
    }

#if _ENABLE_CHISHUI
    if(p_account_info->dsk != NULL) {
        printf("dsk=%s\n",p_account_info->dsk);
    }
#else
    if(p_account_info->NtripUserName != NULL) {
        printf("dsk=%s\n",p_account_info->NtripUserName);
    }
#endif
    printf("expire_time=%ld\n",p_account_info->expire_time);
}

void qxwz_rtcm_response_callback(qxwz_rtcm data) {
	data_trans.length = data.length;
    //QXLOGI("QXWZ_RTCM_DATA_LEN:%ld\n",data.length);
    if (!data.buffer) {
        QXLOGI("no rtcm data\n");
        return;
    }
    
    int idx = 0;
    for (idx = 0; idx < data.length; idx++) {
    	data_trans.buffer[idx] = data.buffer[idx];
         //if (idx % 32 == 0)
             //QXLOGI("\n");
         //QXLOGI("%02x ", (unsigned char)(data.buffer[idx]));
    }
     //QXLOGI("\n");
}

#if _ACTIVE_ACCOUNT
void qxwz_active_account_cb(qxwz_rtcm_status code, const char* msg) {
    if (code == QXWZ_STATUS_ACCOUNT_IS_ACTIVATED) {
        QXLOGI("active account:%d, msg,%s\n",code, msg);
    } else if (code == QXWZ_STATUS_ACCOUNT_ACTIVATE_FAILURE) {
        QXLOGI("active accoun:%d, msg,%s\n",code, msg);
    }
}
#endif

void qxwz_status_response_callback(qxwz_rtcm_status code) {
    QXLOGI("QXWZ_RTCM_STATUS:%d\n",code);
    int ret = 0;
	//test account expire
	if(code == QXWZ_STATUS_OPENAPI_ACCOUNT_TOEXPIRE) {
		get_qxwz_sdk_account_info();
	}else if(code == QXWZ_STATUS_OPENAPI_ACCOUNT_EXPIRED) {
		get_qxwz_sdk_account_info();
    }
#if _ACTIVE_ACCOUNT
    else if(code == QXWZ_STATUS_OPENAPI_DISABLED_ACCOUNT) {
        ret = qxwz_activeAccount(qxwz_active_account_cb, config.deviceId, config.deviceType);
        QXLOGI("active account result:%d\n", ret);
    }
#endif
}

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
	explicit Jobs(boost::asio::io_service& io, int f,  int f_g, std::vector<void*>& receivingSocketList, std::vector<void*>& sendingSocketList) : strand1(io), strand2(io), strand3(io), strand4(io), timer1(io, boost::posix_time::milliseconds(5)), timer2(io, boost::posix_time::milliseconds(10)), timer3(io, boost::posix_time::milliseconds(13)), timer4(io, boost::posix_time::milliseconds(16)), fd(f) , fd_g(f_g), rSocketList(receivingSocketList), sSocketList(sendingSocketList) 
	{
	  timer1.async_wait(strand1.wrap(boost::bind(&Jobs::serialSubscriber, this)));
	  timer2.async_wait(strand1.wrap(boost::bind(&Jobs::DGPSSubscriber, this)));
	  // timer3.async_wait(strand1.wrap(boost::bind(&Jobs::DGPSPublisher, this)));
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
			while(ch!='$'&&ch!='#') read(fd,&ch,1);
			startch=ch;
			msglen=0; memset(msg, 0, sizeof(msg));
			read(fd,&ch,1);
			while(ch!='*')
			{
				//        if(ch=='$' || ch=='#') {
				//          printf("starter incide message : continue\n");
				//          break;
				//        }
        		if(msglen==msgSize-1) 
        		{
		          printf("Normal_msg out of size : continue\n");
		          break; // no valid msg
        		}
				msg[msglen++]=ch;
				read(fd,&ch,1);
			}
      		if(ch!='*') continue; // no valid message received
			msg[msglen]='\0';
			//printf("%s\n",msg);

			if(startch=='$')
			{
				// check sum
				read(fd,&ch,1); tmp=hex2dec(ch); if(tmp==-1) continue;
				checksum=tmp;
				read(fd,&ch,1); tmp=hex2dec(ch); if(tmp==-1) continue;
				checksum=checksum*16+tmp;
				nowsum=0;
				for(i=0; i<msglen; i++) nowsum^=msg[i];
				//printf("num=%d\n%s\n",msglen,msg);
				//printf("nowsum=%X checksum=%X\n",nowsum,checksum);
				if(nowsum!=checksum)
				{
					printf("Checksum Failed! nowsum=%X checksum=%X\n",nowsum,checksum);
					//continue;
				}
				// get header and commas
				memset(header, 0, sizeof(header));
				strncpy(header,msg,5);
				commaNum=0;
				for(i=5; i<msglen; i++) // get positions of all commas
					if(msg[i]==',') comma[commaNum++]=i;
				// for(i=0;i<commaNum;i++) printf("%d ",comma[i]); printf("\n");
				// decode
				if(strcmp(header,"GPGGA")==0)
				{
					if(commaNum!=14) continue;
          			// UTC
          			if(!str2double(msg,comma[0]+1,comma[1]-1,dtmp)) continue;
          				gps.utc_second=((int16_t)(dtmp/10000)+8)*3600+((int16_t)(dtmp/100))%100*60+fmod(dtmp,100);
					// Latitude
					if(!str2double(msg,comma[1]+1,comma[2]-1,dtmp)) continue;
						tmpdeg=(int16_t)dtmp/100; // get deg
						tmpdeg+=fmod(dtmp,100)/60.0; // get min
						gps.latitude=tmpdeg;
						ch=msg[comma[2]+1]; if(ch!='N' && ch!='S') continue;
						gps.latdir=ch;
					// longitude
					if(!str2double(msg,comma[3]+1,comma[4]-1,dtmp)) continue;
						tmpdeg=(int16_t)dtmp/100; // get deg
						tmpdeg+=fmod(dtmp,100)/60.0; // get min
						gps.longitude=tmpdeg;
						ch=msg[comma[4]+1]; if(ch!='E' && ch!='W') continue;
						gps.londir=ch;
					// QF
					ch=msg[comma[5]+1];
					if(ch<'0' || '5'<ch) continue;
		//printf()
				gps.status=ch-'0';
					// satNum
					if(!str2double(msg,comma[6]+1,comma[7]-1,dtmp)) continue;
						gps.satNum=dtmp;

					//printf("GPGGA\nlat = %f %c\nlon = %f %c\nstatus = %d\nsatNum = %d\n\n",gps.latitude,gps.latdir,gps.longitude,gps.londir,gps.status,gps.satNum);

					break; // valid message
				}
				else if(strcmp(header,"GPRMC")==0)
				{
					if(commaNum!=12) continue;
					// Latitude
					if(!str2double(msg,comma[2]+1,comma[3]-1,dtmp)) continue;
						tmpdeg=(int16_t)dtmp/100; // get deg
						tmpdeg+=fmod(dtmp,100)/60.0; // get min
						gps.latitude=tmpdeg;
						ch=msg[comma[3]+1]; if(ch!='N' && ch!='S') continue;
						gps.latdir=ch;
					// longitude
					if(!str2double(msg,comma[4]+1,comma[5]-1,dtmp)) continue;
						tmpdeg=(int16_t)dtmp/100; // get deg
						tmpdeg+=fmod(dtmp,100)/60.0; // get min
						gps.longitude=tmpdeg;
						ch=msg[comma[5]+1]; if(ch!='E' && ch!='W') continue;
						gps.londir=ch;
					// mode
					ch=msg[comma[11]+1];/*
					switch(ch)
					{
						case 'A':gps.status=1;break;
						case 'D':gps.status=4;break;
						case 'E':gps.status=5;break;
						case 'N':gps.status=0;break;
						default:gps.status=-1;
					}*/
					if(gps.status==-1) continue;

					//printf("GPRMC\nlat = %f %c\nlon = %f %c\nstatus = %d\n\n",gps.latitude,gps.latdir,gps.longitude,gps.londir,gps.status);

					break; // valid message
				}
			}
			else
			{ // startch=='#'
				for(i=0; i<8; i++) read(fd,&ch,1); // read CRC
				memset(header, 0, sizeof(header));
				strncpy(header,msg,13);
				//printf("%s\n",msg);
				if(strcmp(header,"HEADINGA,COM1")==0)
				{

					commaNum=0; // get positions of all commas
					for(i=5; i<msglen; i++){
						if(msg[i]==',') comma[commaNum++]=i;
						if(commaNum==13) break;
					}
					if(!str2double(msg,comma[11]+1,comma[12]-1,dtmp)) continue;
					gps.heading=dtmp;

//std::cout << "heading= " << gps.heading << std::endl; 
				//printf("HEADINGA\nheading = %f\n\n",gps.heading);
				}
				break;
			}
		}
    	timer1.expires_at(timer1.expires_at() + boost::posix_time::milliseconds(TIMER1_PERIOD));
		timer1.async_wait(strand1.wrap(boost::bind(&Jobs::serialSubscriber, this)));
	}

	
	void DGPSSubscriber()
	{	
		// read a valid message
//ccy
		char startch,ch,msg[msgSize]="",header[20]="";
		int msglen;

		//while(true)
		//{
			// if received a valid message then break
			// else continue
			// read a message
			read(fd_g,&ch,1);
			while(ch!='$'&&ch!='#') read(fd_g,&ch,1);
			startch=ch;
			msglen=0; memset(msg, 0, sizeof(msg));
			msg[0] = startch;
			read(fd_g,&ch,1);
			while(ch!='$'&&ch!='#')
			{

        		if(msglen==msgSize-1) 
        		{
		          printf("Diff_msg out of size : continue\n");
		          break; // no valid msg
        		}
				msglen = msglen + 1;
				msg[msglen]=ch;
				read(fd_g,&ch,1);
usleep(100);
			if(ch == '\0')
{break;}
			}

      		//if(ch!='*') continue; // no valid message received
			msg[msglen -1] = '\r';
			//msg[msglen++] = '\n';
			//msg[msglen++] = '\0';

			memset(header, 0, sizeof(header));
			strncpy(header,msg+1,5);
			if(startch=='$' && strcmp(header,"GPGGA")==0)
			{
printf("%s\n",msg);
				//设置appKey和appSecret
				//apapKey申请详细见说明文档
				config.appkey="751454";
				config.appSecret="80869e2424b3f1ada803c3f93f90febf63bc2a1e94ff44e00d97b07c032632ba";
				config.deviceId="wbd05";
				config.deviceType="wbd";

				//[1] Set sdk configs
				qxwz_setting(&config);
				//[2] Start rtcm sdk
				qxwz_rtcm_start(qxwz_rtcm_response_callback,qxwz_status_response_callback);
				qxwz_rtcm_sendGGAWithGGAString(msg);

for (int i = 1;i<=sizeof(msg);i++)
{
printf("%d ",msg[i]);
}
printf("\n");
//qxwz_rtcm_sendGGAWithGGAString("$GPGGA,090144.00,3939.26258416,N,11606.03324171,E,1,14,1.6,43.6772,M,-9.7060,M,,*49\r\n");
		        QXLOGI("Send GGA done\r\n");
			}
//break;
		//}
/*
		int idx = 0;
		for (idx = 0; idx < data_trans.length; idx++) 
		{
			if (idx % 32 == 0)
				QXLOGI("\n");
			QXLOGI("%02x ", (unsigned char)(data_trans.buffer[idx]));
		}
		QXLOGI("\n");*/

		int ret = write(fd_g,data_trans.buffer,data_trans.length);
printf("ret = %d\n",ret);
//printf("length = %d\n",data_trans.length);
		//tcflush(fd_g, TCIOFLUSH);
		usleep(60 * 1000);
		tcflush(fd_g, TCOFLUSH);

		timer2.expires_at(timer2.expires_at() + boost::posix_time::milliseconds(TIMER2_PERIOD));
		timer2.async_wait(strand2.wrap(boost::bind(&Jobs::DGPSSubscriber, this)));
	}

  //   void DGPSPublisher()
  //   {
		// timer4.expires_at(timer3.expires_at() + boost::posix_time::milliseconds(TIMER3_PERIOD));
		// timer4.async_wait(strand3.wrap(boost::bind(&Jobs::DGPSPublisher, this)));
  //   }


    void socketPublisher()
    {
		Json::Value values;

		values["Gps"]["Latitude"] = gps.latitude;
		//values["Gps"]["Latdir"] = gps.latdir;
		values["Gps"]["Longitude"] = gps.longitude;
		//values["Gps"]["Londir"] = gps.londir;
		values["Gps"]["Status"] = gps.status;
		values["Gps"]["Yaw"] = gps.heading;
//std::cout<<"publish yaw"<<values["Gps"]["Yaw"] << std::endl;
		values["Gps"]["IsValid"] = true;
		values["Gps"]["UtcSecond"] = gps.utc_second;

		Json::FastWriter fastWriter;
		std::string jsonString = fastWriter.write(values);
		//jsonString.erase (std::remove(jsonString.begin(), jsonString.end(), '\n'), jsonString.end());
		zmq_send(sSocketList[0], jsonString.c_str(), strlen(jsonString.c_str()), 0);
		//      std::cout << "No." << ++counter <<std::endl<<"Sent: " << jsonString << std::endl;
		//printf("UTC=%i:%i:%4.1lf  ",(int16_t)(gps.utc_second/3600),(int16_t)(fmod(gps.utc_second,3600)/60),fmod(gps.utc_second,60));
		//printf("lat=%.8lf lon=%.8lf hdg=%.2lf status=%d\n",gps.latitude,gps.longitude,gps.heading,gps.status);
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
    int fd_g;
    int counter = 0;
};


int main()
{
	std::cout << "program started." << std::endl;

	int fd;
	fd = open("/dev/ttyTHS1", O_RDWR);

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

	//DGPS
	std::cout << "DGPS started." << std::endl;
	int fd_g;
	fd_g = open("/dev/ttyTHS3", O_RDWR);
	if( -1 == fd_g)
	{
		printf("Error opening the DGPS port!");
	}
	else
	{
		printf("Serial DGPS port opened\n");
	}
	//  assert(-1 == fd);
	set_speed(fd_g, 115200);
	if(-1 == set_port(fd_g,8,1,'N'))
	{
		printf("Set DGPS port error.\n");
	}
	else;
	tcflush(fd_g, TCIOFLUSH);  


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
	Jobs jobs(io, fd, fd_g, receivingSocketList, sendingSocketList);
	boost::thread t(boost::bind(&boost::asio::io_service::run, &io));
	io.run();
	t.join();

	//printf("Stop to send \n");
	//canetUdpS.closeUdpCanet();
	return 0;
}

