#include <chrono>
#include <iostream>
#include <random>

#include "cluon-complete.hpp"
#include "prime-checker.hpp"
#include "messages.hpp"

/**
int32_t main(int32_t, char **) {
  PrimeChecker pc;
  std::cout << "Hello world = " << pc.isPrime(43) << std::endl;
  
  cluon::OD4Session od4(111, 
    [](cluon::data::Envelope &&envelope) noexcept {
      if (envelope.dataType() == 2001) {
        MyTestMessage1 receivedMsg =
          cluon::extractMessage<MyTestMessage1> 
          (std::move(envelope));
        
        PrimeChecker pc;
        std::cout << receivedMsg.myValue() << " is" 
          << (pc.isPrime(receivedMsg.myValue()) ? " " : " not ")
          << "a prime." << std::endl;
      }
    });
    
  uint32_t const maxnum{1000};
  std::random_device r;
  std::mt19937 rg(r());
  std::uniform_int_distribution<uint16_t> dist(1, maxnum);
           
  while (od4.isRunning()) {
    uint16_t value = dist(rg);
    
    MyTestMessage1 msg;
    msg.myValue(value);
    
    od4.send(msg);
    
    std::this_thread::sleep_for(std::chrono::duration<double>(3.0));
  }
    
  return 0;
}*/

int32_t main(int32_t, char **) {
  PrimeChecker pc;
  std::cout << "Hello world = " << pc.isPrime(43) << std::endl;
  
  cluon::OD4Session od4(111,
    [](cluon::data::Envelope &&envelope) noexcept {
      if (envelope.dataType() == 2001) {
        MyTestMessage1 receivedMsg = cluon::extractMessage<MyTestMessage1>(std::move(envelope));

        PrimeChecker pc;
        std::cout << receivedMsg.myValue() << " is" 
          << (pc.isPrime(receivedMsg.myValue()) ? " " : " not ")
          << "a prime." << std::endl;
      }
    });

  while (od4.isRunning()) {
    uint16_t value;
    std::cout << "Enter a number to check:" << std::endl;
    std::cin >> value;
    MyTestMessage1 msg;
    msg.myValue(value);

    od4.send(msg);
  }
 
  return 0;
}
