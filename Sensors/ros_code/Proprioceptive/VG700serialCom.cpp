/* minicom.cpp 
 A simple demonstration minicom client with Boost asio 
 Process function implemented to decode VG700 serial data frames
 
 Parameters: 
 baud rate 
 serial port (eg /dev/ttyS0 or COM1) 
 
 To end the application, send Ctrl-C on standard input 
 */ 

#include <vector>
#include <iostream> 
#include <boost/bind.hpp> 
#include <boost/asio.hpp> 
#include <boost/asio/serial_port.hpp> 
#include <boost/thread.hpp> 
#include <boost/lexical_cast.hpp> 
#include <boost/foreach.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp> 

#include "VG700decoder.hpp"

#ifdef POSIX 
#include <termios.h> 
#endif 
#ifdef WIN32
#include <conio.h>
#endif

using namespace std;

class minicom_client 
    { 
    public: 
        minicom_client(boost::asio::io_service& io_service, unsigned int baud, const string& device) 
        : active_(true), 
        io_service_(io_service), 
        serialPort(io_service, device) ,
        yaw_(0.0), prev_time_(0)
        { 
            if (!serialPort.is_open()) 
            { 
                cerr << "Failed to open serial port\n"; 
                return; 
            } 
            boost::asio::serial_port_base::baud_rate baud_option(baud); 
            serialPort.set_option(baud_option); // set the baud rate after the port has been opened 
            read_start(); 
        } 
        
        void write(const char msg) // pass the write data to the do_write function via the io service in the other thread 
        { 
            io_service_.post(boost::bind(&minicom_client::do_write, this, msg)); 
        } 
        
        void close() // call the do_close function via the io service in the other thread 
        { 
            io_service_.post(boost::bind(&minicom_client::do_close, this, boost::system::error_code())); 
        } 
        
        bool active() // return true if the socket is still active 
        { 
            return active_; 
        } 
        
    private: 
        
        static const int max_read_length = 128; // maximum amount of data to read in one operation 
        
        void read_start(void) 
        { // Start an asynchronous read and call read_complete when it completes or fails 
            serialPort.async_read_some(boost::asio::buffer(read_msg_, max_read_length), 
                                       boost::bind(&minicom_client::read_complete, 
                                                   this, 
                                                   boost::asio::placeholders::error, 
                                                   boost::asio::placeholders::bytes_transferred)); 
        } 
        
        void read_complete(const boost::system::error_code& error, size_t bytes_transferred) 
        { // the asynchronous read operation has now completed or failed and returned an error 
            if (!error) 
            { // read completed, so process the data 
				process(read_msg_, bytes_transferred);
                //cout.write(read_msg_, bytes_transferred); // echo to standard output 
                read_start(); // start waiting for another asynchronous read again 
            } 
            else 
                do_close(error); 
        } 
        
        void do_write(const char msg) 
        { // callback to handle write call from outside this class 
            bool write_in_progress = !write_msgs_.empty(); // is there anything currently being written? 
            write_msgs_.push_back(msg); // store in write buffer 
            if (!write_in_progress) // if nothing is currently being written, then start 
                write_start(); 
        } 
        
        void write_start(void) 
        { // Start an asynchronous write and call write_complete when it completes or fails 
            boost::asio::async_write(serialPort, 
                                     boost::asio::buffer(&write_msgs_.front(), 1), 
                                     boost::bind(&minicom_client::write_complete, 
                                                 this, 
                                                 boost::asio::placeholders::error)); 
        } 
        
        void write_complete(const boost::system::error_code& error) 
        { // the asynchronous read operation has now completed or failed and returned an error 
            if (!error) 
            { // write completed, so send next write data 
                write_msgs_.pop_front(); // remove the completed data 
                if (!write_msgs_.empty()) // if there is anthing left to be written 
                    write_start(); // then start sending the next item in the buffer 
            } 
            else 
                do_close(error); 
        } 
        
        void do_close(const boost::system::error_code& error) 
        { // something has gone wrong, so close the socket & make this object inactive 
            if (error == boost::asio::error::operation_aborted) // if this call is the result of a timer cancel() 
                return; // ignore it because the connection cancelled the timer 
            if (error) 
                cerr << "Error: " << error.message() << endl; // show the error message 
            else 
                cout << "Error: Connection did not succeed.\n"; 
            cout << "Press Enter to exit\n"; 
            serialPort.close(); 
            active_ = false; 
        } 
        
        // Process data
        // Here we deal with VG700 frames, assuming we are in VG mode
        void process(char *msg, size_t msg_len)
        {
            vector<VG700dataframe> frames = vg_decoder_.decode(msg, msg_len, 0.0 /* should be road_time */);
            BOOST_FOREACH( VG700dataframe f, frames )
            {
                // Computation of data output period from the time flag
                unsigned short usperiod = prev_time_ - f.embedded_time;
                double period = 0.79e-6 * usperiod;
                // Just to estimate the drift, yaw computation by integration of the yaw_rate
                yaw_ += period * f.yawRate;
                
                prev_time_ = f.embedded_time;

                // Display decoded data
                cout.setf(ios::fixed);
                cout.precision(3);
                cout.width(8);
                cout<< "r=" <<f.rollAngle
                    <<" p=" <<f.pitchAngle
                    //<<" rr="<<rollRate
                    //<<" pr="<<pitchRate
                    //<<" yr="<<yawRate
                    //<<" ax="<<accelX
                    //<<" ay="<<accelY
                    //<<" az="<<accelZ
                    //<<" t="<<temp
                    <<" yaw=" << yaw_
                    <<" f=" << 1. / period
                    <<endl;
            }
        }
        
    private: 
        bool active_; // remains true while this object is still operating 
        boost::asio::io_service& io_service_; // the main IO service that runs this connection 
        boost::asio::serial_port serialPort; // the serial port this instance is connected to 
        char read_msg_[max_read_length]; // data read from the socket 
        deque<char> write_msgs_; // buffered write data 
        
        VG700decoder vg_decoder_;
        
        double yaw_;
        unsigned short prev_time_;
    }; 

int main(int argc, char* argv[]) 
{ 
    // on Unix POSIX based systems, turn off line buffering of input, so cin.get() returns after every keypress 
    // On other systems, you'll need to look for an equivalent 
#ifdef POSIX 
    termios stored_settings; 
    tcgetattr(0, &stored_settings); 
    termios new_settings = stored_settings; 
    new_settings.c_lflag &= (~ICANON); 
    new_settings.c_lflag &= (~ISIG); // don't automatically handle control-C 
    tcsetattr(0, TCSANOW, &new_settings); 
#endif 
    try 
    { 
        if (argc != 3) 
        { 
            cerr << "Usage: minicom <baud> <device>\n"; 
            return 1; 
        } 
        boost::asio::io_service io_service; 
        // define an instance of the main class of this program 
        minicom_client c(io_service, boost::lexical_cast<unsigned int>(argv[1]), argv[2]); 
        // run the IO service as a separate thread, so the main thread can block on standard input 
        boost::thread t(boost::bind(&boost::asio::io_service::run, &io_service)); 
        while (c.active()) // check the internal state of the connection to make sure it's still running 
        { 
			char ch;
#ifdef WIN32
			ch = getch();
#else
            cin.get(ch); // blocking wait for standard input
#endif
			if (ch == 3) // ctrl-C to end program 
                break;
            c.write(ch); 
        } 
        c.close(); // close the minicom client connection 
        t.join(); // wait for the IO service thread to close 
    } 
    catch (exception& e) 
    { 
        cerr << "Exception: " << e.what() << "\n"; 
    } 
#ifdef POSIX // restore default buffering of standard input 
    tcsetattr(0, TCSANOW, &stored_settings); 
#endif 
    return 0; 
}