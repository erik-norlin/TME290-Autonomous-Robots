/*
 * Copyright (C) 2018 Ola Benderius
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include "behavior.hpp"

int32_t main(int32_t argc, char **argv) {
  int32_t retCode{0};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  // Check that the mandatory command line arguments are present.
  if (0 == commandlineArguments.count("cid") || 0 == commandlineArguments.count("freq")) {
    
    std::cerr 
    << argv[0] 
    << " tests the Kiwi platform by sending actuation " 
    << "commands and reacting to sensor input." 
    << std::endl;
    
    std::cerr << "Usage:   " 
    << argv[0] 
    << " --freq=<Integration frequency> " 
    << "--cid=<OpenDaVINCI session> [--verbose]" 
    << std::endl;
    
    std::cerr << "Example: " 
    << argv[0] 
    << " --freq=10 --cid=111" 
    << std::endl;
    retCode = 1;
    
  } else {
    // Extract the command line arguments.
    bool const VERBOSE{commandlineArguments.count("verbose") != 0};
    uint16_t const CID = std::stoi(commandlineArguments["cid"]);
    float const FREQ = std::stof(commandlineArguments["freq"]);

    // The behaviour is put in its own class, and an object is created here.
    Behavior behavior;

    // The OD4 data trigger lambda functions.
    auto onDistanceReading{[&behavior](cluon::data::Envelope &&envelope)
      {
        auto distanceReading = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(envelope));
        uint32_t const senderStamp = envelope.senderStamp();
        if (senderStamp == 0) {
          behavior.setFrontUltrasonic(distanceReading);
        } else if (senderStamp == 1) {
          behavior.setRearUltrasonic(distanceReading);
        } else if (senderStamp == 2) {
          behavior.setLeftIr(distanceReading);
        } else if (senderStamp == 3) {
          behavior.setRightIr(distanceReading);
        }
      }};

    // Creating the OD4 session.
    cluon::OD4Session od4{CID};
    od4.dataTrigger(opendlv::proxy::DistanceReading::ID(), onDistanceReading);
    
    float const dt = 1.0f / FREQ;
    float t{0};
    float r{0.04f};

    // Lambda function to run at a specified frequency.
    auto atFrequency{[&VERBOSE, &behavior, &od4, &dt, &t, &r]() -> bool
      {
        behavior.step();
        auto wL = behavior.get_wL();
        auto wR = behavior.get_wR();

        cluon::data::TimeStamp sampleTime = cluon::time::now();
        od4.send(wL, sampleTime, 0); // left = 0, right = 1
        od4.send(wR, sampleTime, 1);
        
        if (VERBOSE) {
          std::cout << "vL: "
            << r*wL.axleAngularVelocity()
            << "	vR: " 
            << r*wR.axleAngularVelocity()
            << "	t: "
            << t
            << std::endl;
        }
        
	t += dt;
        return true;
      }};

    // This will block until Ctrl+C is pressed.
    od4.timeTrigger(FREQ, atFrequency);
  }
  return retCode;
}
