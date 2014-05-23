#ifndef SERIALCOMM_H
#define SERIALCOMM_H

#include <boost/thread/thread.hpp>

class SerialComm {
public:
  
  SerialComm();
  ~SerialComm();
  
  void open(const char *port_name, int baud_rate);
  void close();

  bool startReadStream();
  void stopReadStream();
  void pauseReadStream();
  void resumeReadStream();
  
  int write(const unsigned char *data, int length);

protected:

  //! On read callback, implemented in dervided class
  virtual void onRead(const unsigned char *data, unsigned int len) = 0;

  //! Read thread
  void readThread();

  //! File descriptor
  int fd;
  //! Baud rate
  int baud;
  //! Read Stream thread
  boost::thread* read_stream_thread;
  //! Whether read streaming is paused or not
  bool read_stream_paused;
  //! Whether read streaming is stopped or not
  bool read_stream_stopped;
  //! Data buffer
  unsigned char *buffer;
  
  //! Exception buffer
  char *errorbuffer;
};

#endif  // SERIALCOMM_H

