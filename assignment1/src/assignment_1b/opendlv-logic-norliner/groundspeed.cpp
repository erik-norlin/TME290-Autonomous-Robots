#include <chrono>
#include <iostream>

#include "cluon-complete.hpp"
#include "messages.hpp"

int32_t main(int32_t, char **) {
  uint16_t groundSpeedReadingID = 1046;
  uint16_t groundSpeedRequestID = 1091;
  float multiplier = 2.0f;
  
  cluon::OD4Session od4(132, 
    [&od4, &groundSpeedRequestID, &groundSpeedReadingID, &multiplier]
      (cluon::data::Envelope &&envelope) noexcept {
      if (envelope.dataType() == groundSpeedRequestID) {
        
        opendlv::proxy::GroundSpeedRequest receivedMsg = 
          cluon::extractMessage<opendlv::proxy::GroundSpeedRequest>(std::move(envelope));
        opendlv::proxy::GroundSpeedReading msg;
        msg.groundSpeed(multiplier * receivedMsg.groundSpeed());
        
        od4.send(msg);
      }
      if (envelope.dataType() == groundSpeedReadingID) {
        opendlv::proxy::GroundSpeedReading receivedMsg =
          cluon::extractMessage<opendlv::proxy::GroundSpeedReading>(std::move(envelope));   
        std::cout << "GroundSpeedReading: " << receivedMsg.groundSpeed() << std::endl;   
      }
      
    });
           
  while (od4.isRunning()) {
    float request;
    std::cout << "Enter GroundSpeedRequest: " << std::endl;
    std::cin >> request;
    
    opendlv::proxy::GroundSpeedRequest msg;
    msg.groundSpeed(request);
    od4.send(msg);
    
    std::this_thread::sleep_for(std::chrono::duration<double>(0.1));
  }
    
  return 0;
}
