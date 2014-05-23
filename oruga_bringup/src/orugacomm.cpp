#include <oruga_bringup/orugacomm.h>

#define FRAME_HEADER 0xFE
#define FRAME_TAIL   0xFD
#define TX_FRAME_MAX_LENGTH 64

OrugaComm::OrugaComm(std::string toOrugaTopic, std::string portName, int baudRate) {
  txBuffer = new unsigned char[TX_FRAME_MAX_LENGTH];
  memset(txBuffer, 0 , TX_FRAME_MAX_LENGTH);

  open(portName.c_str(), baudRate);

  sub = nh.subscribe(toOrugaTopic, 1, &OrugaComm::toOrugaCallback, this);
}

OrugaComm::~OrugaComm() {
  close();
  delete[] txBuffer;
}

void OrugaComm::writeToOrugaData() {
  if(toOrugaCs.hasNewData()) {
    toOrugaCs.get(&toOrugaData);
    size_t txFrameLength = buildFrameFromData(txBuffer, &toOrugaData);
    write(txBuffer, txFrameLength);
    /*for(unsigned int i = 0; i < txFrameLength; i++) {  // Debug!
      write(&txBuffer[i], 1);
      sleep(1);
    }*/
  }
}

size_t OrugaComm::buildFrameFromData(unsigned char *frame, oruga_msgs::ToOrugaData *data) {
  size_t valueSize = data->value.size();
  frame[0] = FRAME_HEADER;
  frame[1] = data->code;
  for(size_t i = 0; i < valueSize; i++) {
    frame[i+2] = data->value[i];
  }
  frame[valueSize+2] = FRAME_TAIL;

  return (3+valueSize);
}

