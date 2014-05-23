#include <serialcomm/serialcomm.h>
#include <termios.h>
#include <poll.h>
#include <fcntl.h>
#include <fstream>
#include <string>


#define BUFFER_MAX_LENGTH 128
#define ERROR_BUFFER_LENGTH 1000


SerialComm::SerialComm(): fd(-1), read_stream_thread(NULL) {
  buffer = new unsigned char[BUFFER_MAX_LENGTH];
  errorbuffer = new char[ERROR_BUFFER_LENGTH];
}


SerialComm::~SerialComm() {
  delete[] buffer;
  delete[] errorbuffer;
}


void SerialComm::open(const char *port_name, int baud_rate) {
  
  fd = ::open(port_name, O_RDWR | O_NONBLOCK | O_NOCTTY);
  if(fd == -1) {

    std::string extra_msg;
    switch(errno) {
      case EACCES:
	extra_msg = "You probably don't have premission to open the port for reading and writing.";
	break;

      case ENOENT:
	extra_msg = "The requested port does not exist. Is the device connected? Was the port name misspelled?";
	break;
    }
    snprintf(errorbuffer, ERROR_BUFFER_LENGTH, "[in function %s]::Failed to open port: %s. %s (errno = %d). %s", __FUNCTION__, port_name, strerror(errno), errno, extra_msg.c_str());
    throw std::runtime_error(errorbuffer);
  }
      
  try {
    struct flock fl;
    fl.l_type = F_WRLCK;
    fl.l_whence = SEEK_SET;
    fl.l_start = 0;
    fl.l_len = 0;
    fl.l_pid = getpid();

    if(fcntl(fd, F_SETLK, &fl) != 0) {
      snprintf(errorbuffer, ERROR_BUFFER_LENGTH, "[in function %s]::Device %s is already locked. Try 'lsof | grep %s' to find other processes that currently have the port open.", __FUNCTION__, port_name, port_name);
      throw std::runtime_error(errorbuffer);
    }

    // Settings for USB?
    struct termios newtio;
    tcgetattr(fd, &newtio);
    memset (&newtio.c_cc, 0, sizeof (newtio.c_cc));
    newtio.c_cflag = CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    cfsetspeed(&newtio, baud_rate);
    baud = baud_rate;

    // Activate new settings
    tcflush(fd, TCIFLUSH);
    if(tcsetattr(fd, TCSANOW, &newtio) < 0) {
      snprintf(errorbuffer, ERROR_BUFFER_LENGTH, "[in function %s]::Unable to set serial port attributes. The port you specified (%s) may not be a serial port.", __FUNCTION__, port_name);
      throw std::runtime_error(errorbuffer);  /// @todo tcsetattr returns true if at least one attribute was set. Hence, we might not have set everything on success.
    }

    usleep (200000);

  } catch(std::runtime_error& e) {
    // These exceptions mean something failed on open and we should close
    if(fd != -1) ::close(fd);
    fd = -1;
    throw e;
  }
}


void SerialComm::close() {
  int retval = 0;
  retval = ::close(fd);
  fd = -1;

  if(retval != 0) {
    snprintf(errorbuffer, ERROR_BUFFER_LENGTH, "[in function %s]::Failed to close port properly -- error = %d: %s", __FUNCTION__, errno, strerror(errno));
    throw std::runtime_error(errorbuffer);
  }
}


bool SerialComm::startReadStream() {

  if(read_stream_thread != NULL) return false;
  read_stream_stopped = false;
  read_stream_paused = false;
  read_stream_thread = new boost::thread(boost::bind(&SerialComm::readThread, this));
  return true;
}


void SerialComm::readThread() {

  struct pollfd ufd[1];
  ufd[0].fd = fd;
  ufd[0].events = POLLIN;
  while(!read_stream_stopped) {
    if(!read_stream_paused) {
      if(poll(ufd, 1, 10) > 0) {
	if(!(ufd[0].revents & POLLERR)) {
	  int ret = ::read(fd, buffer, BUFFER_MAX_LENGTH);
	  if(ret>0) { onRead(buffer, ret); }
	} 
      }
    } 
  }

}


void SerialComm::stopReadStream() {

  read_stream_stopped = true;
  read_stream_thread->join();

  delete read_stream_thread;
  read_stream_thread = NULL;
}


void SerialComm::pauseReadStream() { read_stream_paused = true; }


void SerialComm::resumeReadStream() { read_stream_paused = false; }


int SerialComm::write(const unsigned char *data, int length) {
  
  // IO is currently non-blocking. This is what we want for the more cerealon read case.
  int origflags = fcntl(fd, F_GETFL, 0);
  fcntl(fd, F_SETFL, origflags & ~O_NONBLOCK); // TODO: @todo can we make this all work in non-blocking?
  int retval = ::write(fd, data, length);
  fcntl(fd, F_SETFL, origflags | O_NONBLOCK);

  if(retval == length) {
    return retval;
  } else {
    snprintf(errorbuffer, ERROR_BUFFER_LENGTH, "[in function %s]::Write failed!", __FUNCTION__);
    throw std::runtime_error(errorbuffer);
  }
}
