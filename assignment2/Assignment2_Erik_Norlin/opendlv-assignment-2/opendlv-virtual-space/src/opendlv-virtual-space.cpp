/*
 * Copyright (C) 2020 Ola Benderius
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
#include "world-object.hpp"

// Slight change from the cluon version. Patch submitted
std::vector<std::string> split(const std::string &str,
                                      const char &delimiter) noexcept {
  std::vector<std::string> retVal{};
  std::string::size_type prev{0};
  for (std::string::size_type i{str.find_first_of(delimiter, prev)};
       i != std::string::npos;
       prev = i + 1, i = str.find_first_of(delimiter, prev)) {
    if (i != prev) {
      retVal.emplace_back(str.substr(prev, i - prev));
    }
    else {
      retVal.emplace_back("");
    }
  }
  if (prev < str.size()) {
    retVal.emplace_back(str.substr(prev, str.size() - prev));
  }
  else if (prev > 0) {
    retVal.emplace_back("");
  }
  return retVal;
}

int32_t main(int32_t argc, char **argv) {
  int32_t retCode{0};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if (0 == commandlineArguments.count("cid") 
      || 0 == commandlineArguments.count("freq") 
      || 0 == commandlineArguments.count("frame-id")) {
    std::cerr << argv[0] << " integrates the kinematic state of a body "
      << "into its global position." << std::endl
      << "Usage:   " << argv[0] << " --frame-id=<ID of frame to integrate> "
      << "--freq=<Integration frequency> --cid=<od4 session> "
      << "[--timemod=<Time scale modifier for simulation speed. Default: 1.0>] "
      << "[--extra-cid-out=<Additional conferences for output, as "
      << "'cid1:senderstamp,cid2:senderstamp'>] "
      << "[--x=<Initial X position] [--y=<Initial Y position] "
      << "[--z=<Initial Z position] [--roll=<Initial roll angle (around X)] "
      << "[--pitch=<Initial pitch angle (around Y)] "
      << "[--yaw=<Initial yaw angle (around Z)] [--verbose]" << std::endl
      << "Example: " << argv[0] << " --frame-id=0 --freq=100 --cid=111" 
      << std::endl;
    retCode = 1;
  } else {
    float const X{(commandlineArguments["x"].size() != 0) 
      ? static_cast<float>(std::stof(commandlineArguments["x"])) : 0.0f};
    float const Y{(commandlineArguments["y"].size() != 0) 
      ? static_cast<float>(std::stof(commandlineArguments["y"])) : 0.0f};
    float const Z{(commandlineArguments["z"].size() != 0) 
      ? static_cast<float>(std::stof(commandlineArguments["z"])) : 0.0f};
    float const ROLL{(commandlineArguments["roll"].size() != 0) 
      ? static_cast<float>(std::stof(commandlineArguments["roll"])) : 0.0f};
    float const PITCH{(commandlineArguments["pitch"].size() != 0) 
      ? static_cast<float>(std::stof(commandlineArguments["pitch"])) : 0.0f};
    float const YAW{(commandlineArguments["yaw"].size() != 0) 
      ? static_cast<float>(std::stof(commandlineArguments["yaw"])) : 0.0f};
    
    float const TIMEMOD{(commandlineArguments["timemod"].size() != 0) 
      ? static_cast<float>(std::stof(commandlineArguments["timemod"])) : 1.0f};

    uint32_t const FRAME_ID = std::stoi(commandlineArguments["frame-id"]);
    bool const VERBOSE{commandlineArguments.count("verbose") != 0};
    uint16_t const CID = std::stoi(commandlineArguments["cid"]);
    float const FREQ = std::stof(commandlineArguments["freq"]);
    double const DT = 1.0 / FREQ;

    std::vector<std::pair<std::unique_ptr<cluon::OD4Session>, uint32_t>> 
      extraOd4s;

    if (commandlineArguments.count("extra-cid-out") != 0) {
      auto cids = split(commandlineArguments["extra-cid-out"], ',');
      for (auto &cid : cids) {
        auto c = split(cid, ':');
        if (c.size() == 2) {
          extraOd4s.emplace_back(new cluon::OD4Session(std::stoi(c[0])), 
              std::stoi(c[1]));
        }
      }
    }

    WorldObject worldObject{X, Y, Z, ROLL, PITCH, YAW};

    auto onKinematicState{[&FRAME_ID, &worldObject](
        cluon::data::Envelope &&envelope)
      {
        uint32_t const senderStamp = envelope.senderStamp();
        if (FRAME_ID == senderStamp) {
          auto kinematicState = 
            cluon::extractMessage<opendlv::sim::KinematicState>(
                std::move(envelope));
          worldObject.setKinematicState(kinematicState);
        }
      }};
    // 2023-03-10 10:52:10 bb | Allow of reset of the system
    auto const onCluonRecordCommand{[&worldObject](cluon::data::Envelope &&a_env)->void{
        // auto const senderstamp{a_env.senderStamp()};
        // auto const timestamp{a_env.sampleTimeStamp()};
        // 2023-02-15 18:37:10 bb | 1 = start, 2 = stop, 3 = reset
        auto cluonRecordCommand = cluon::extractMessage<cluon::data::RecorderCommand>(std::move(a_env));
        // bool reset;
        if(cluonRecordCommand.command() != 3){
          return;
        }
        worldObject.resetState();
        
    }};

    cluon::OD4Session od4{CID};
    od4.dataTrigger(opendlv::sim::KinematicState::ID(), onKinematicState);
    od4.dataTrigger(cluon::data::RecorderCommand::ID(), onCluonRecordCommand);

    auto atFrequency{[&FRAME_ID, &VERBOSE, &DT, &worldObject, &od4, 
      &extraOd4s]() -> bool
      {
        opendlv::sim::Frame frame = worldObject.step(DT);

        cluon::data::TimeStamp sampleTime;
        od4.send(frame, sampleTime, FRAME_ID);
        
        for (auto &eod4 : extraOd4s) {
          eod4.first->send(frame, sampleTime, eod4.second);
        }
        
        if (VERBOSE) {
          std::cout << "Frame  with id " << FRAME_ID
            << " is at [x=" << frame.x() << ", y=" << frame.y() << ", z="
            << frame.z() << "] with the rotation [roll=" << frame.roll() 
            << ", pitch=" << frame.pitch() << ", yaw=" << frame.yaw() << "]." 
            << std::endl;
        }

        return true;
      }};


    od4.timeTrigger(TIMEMOD * FREQ, atFrequency);
  }
  return retCode;
}
