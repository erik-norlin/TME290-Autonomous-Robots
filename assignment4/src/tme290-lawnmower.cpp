/*
 * Copyright (C) 2019 Ola Benderius
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

#include <iostream>
#include <random>
#include <tuple>
#include <fstream>

#include "cluon-complete.hpp"
#include "tme290-sim-grass-msg.hpp"


std::tuple<uint8_t,bool> returnHome(tme290::grass::Sensors msg, bool chargeBatteryBool)
{
  uint8_t const WALL_I = 17;
  uint8_t const WALL_J = 21;
  uint8_t cmd = 0;
  uint8_t i = msg.j();
  uint8_t j = msg.i();

  // Move randomly left/up/diagonal to the charging station
  if (i <= WALL_I)
  {
    if (i > 0 && j > 0)
    {
      std::random_device dev;
      std::mt19937 rng(dev());
      std::uniform_int_distribution<std::mt19937::result_type> dist(1,3);
      cmd = dist(rng);
      if (cmd == 3)
      {
        cmd = 8;
      }
    }
    else if (i == 0 && j > 0)
    {
      cmd = 8;
    }
    else if (i > 0 && j == 0)
    {
      cmd = 2;
    }
    else if (i == 0 && j == 0)
    {
      cmd = 0;
      chargeBatteryBool = true;
    }
  }

  // Avoiding the wall
  else
  {
    if (i > WALL_I + 1 && j < WALL_J + 1)
    {
      std::random_device dev;
      std::mt19937 rng(dev());
      std::uniform_int_distribution<std::mt19937::result_type> dist(2,4);
      cmd = dist(rng);
    }
    else if (i == WALL_I + 1 && j < WALL_J + 1)
    {
      cmd = 4;
    }
    else if (j >= WALL_J + 1)
    {
      std::random_device dev;
      std::mt19937 rng(dev());
      std::uniform_int_distribution<std::mt19937::result_type> dist(1,2);
      cmd = dist(rng);
    }
    else if (i == WALL_I + 1 && j == WALL_J + 1)
    {
      cmd = 2;
    }
  }
  std::tuple<uint8_t,bool> act = std::make_tuple(cmd, chargeBatteryBool);
  return act;
}


std::tuple<uint8_t,bool> goToRow(tme290::grass::Sensors msg, bool goToRowBool, uint8_t row)
{
  uint8_t const YARD_LENGTH = 40;
  uint8_t const WALL_I = 17;
  uint8_t const WALL_J = 21;
  uint8_t cmd = 0;  
  uint8_t i = msg.j();
  uint8_t j = msg.i();

  // Moving randomly right/down/diagonal to the intended row
  if (i < row)
  {
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_int_distribution<std::mt19937::result_type> dist(4,6);
    cmd = dist(rng);
    if (i == WALL_I-1 && j <= WALL_J)
    {
      cmd = 4;
    }
  }
  else if (i == row)
  {
    cmd = 4;
    if (j+1 == YARD_LENGTH)
    {
      goToRowBool = false;
    }
  }

  std::tuple<uint8_t,bool> act = std::make_tuple(cmd, goToRowBool);
  return act;
}


uint8_t cutGrass(tme290::grass::Sensors msg, uint8_t row)
{
  uint8_t const YARD_LENGTH = 40;
  uint8_t const WALL_I = 17;
  uint8_t const WALL_J = 21;
  uint8_t cmd = 0;
  uint8_t i = msg.j();
  uint8_t j = msg.i();
  
  // Sweeping the grass going downwards
  if ((i-row) % 2 == 0)
  {
    cmd = 8;
    if (j == 0)
    {
      cmd = 6;
    }
  }
  else if ((i-row) % 2 != 0)
  {
    cmd = 4;
    if (j+1 == YARD_LENGTH)
    {
      cmd = 6;
    }
  }

  // Avoiding the wall
  if (i == WALL_I && j == WALL_J+1)
  {
    cmd = 6;
  }

  // Stopping every other min
  if (msg.time() % 2 == 0)
  {
    cmd = 0;
  }

  return cmd;
}


std::tuple<uint8_t,bool,bool,uint8_t> chargeBattery(tme290::grass::Sensors msg, bool chargeBatteryBool, bool goToRowBool, uint8_t row, uint8_t ROW_RESTART)
{
  uint8_t YARD_LENGTH = 40;
  uint8_t cmd = 0;
  
  if (msg.battery() > 0.9999f)
  {
    chargeBatteryBool = false;
    goToRowBool = true;

    if (row+1 == YARD_LENGTH)
    {
      row = ROW_RESTART;
    }
    else 
    {
      row += 1;
    }
  }
  std::tuple<uint8_t,bool,bool,uint8_t> act = std::make_tuple(cmd, chargeBatteryBool, goToRowBool, row);
  return act;
}


float getBatteryLevel(tme290::grass::Sensors msg)
{
  float lowBattery = 0.0f;

  // Semi-dynamic threshold for low battery
  if (msg.j() < 17)
  {
    lowBattery = 0.25f;
  }
  else if (msg.j() < 40)
  {
    lowBattery = 0.33f;
  }

  return lowBattery;
}


int32_t main(int32_t argc, char **argv) 
{
  int32_t retCode{0};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);

  std::ofstream file;
  file.open("status.csv");
  file << "t,mean,max" << std::endl;
  
  if (0 == commandlineArguments.count("cid")) 
  {
    std::cerr << argv[0] << " is a lawn mower control algorithm." << std::endl;
    std::cerr << "Usage:   " << argv[0] << " --cid=<OpenDLV session>" << "[--verbose]" << std::endl;
    std::cerr << "Example: " << argv[0] << " --cid=111 --verbose" << std::endl;
    retCode = 1;
  } 
  else 
  {
    bool const verbose{commandlineArguments.count("verbose") != 0};
    uint16_t const cid = std::stoi(commandlineArguments["cid"]);

    uint8_t const ROW_RESTART = 0;
    bool chargeBatteryBool = false;
    bool goToRowBool = true;
    uint8_t row = ROW_RESTART;
    cluon::OD4Session od4{cid};

    auto onSensors{[&od4, &chargeBatteryBool, &goToRowBool, &row, &ROW_RESTART](cluon::data::Envelope &&envelope)
      { 
        uint8_t cmd;
        auto msg = cluon::extractMessage<tme290::grass::Sensors>(std::move(envelope));
        float lowBattery = getBatteryLevel(msg);
        bool returnHomeBool = lowBattery > msg.battery();


        if (returnHomeBool) 
        {
          std::tuple<uint8_t,bool> act = returnHome(msg, chargeBatteryBool);
          cmd = std::get<0>(act);
          chargeBatteryBool = std::get<1>(act);
        }
        else if (goToRowBool)
        {
          std::tuple<uint8_t,bool> act = goToRow(msg, goToRowBool, row);
          cmd = std::get<0>(act);
          goToRowBool = std::get<1>(act);
        }
        else
        {
          cmd = cutGrass(msg, row);
        }
        if (chargeBatteryBool) 
        {
          std::tuple<uint8_t,bool,bool,uint8_t> act = chargeBattery(msg, chargeBatteryBool, goToRowBool, row, ROW_RESTART);
          cmd = std::get<0>(act);
          chargeBatteryBool = std::get<1>(act);
          goToRowBool = std::get<2>(act);
          row = std::get<3>(act);
        }  

        tme290::grass::Control control;
        od4.send(control.command(cmd));
      }};

    auto onStatus{[&verbose, &file](cluon::data::Envelope &&envelope)
      {
        auto msg = cluon::extractMessage<tme290::grass::Status>(std::move(envelope));
        if (verbose) 
        {
          std::cout << "Status at time " << msg.time() << ": " << msg.grassMean() << " / " << msg.grassMax() << std::endl;
        }
        if (msg.time() % 50 == 0 || msg.time() == 1)
        {
          file << msg.time() << "," << msg.grassMean() << "," << msg.grassMax() << std::endl;;
        }
      }};

    od4.dataTrigger(tme290::grass::Sensors::ID(), onSensors);
    od4.dataTrigger(tme290::grass::Status::ID(), onStatus);

    if (verbose) 
    {
      std::cout << "All systems ready, let's cut some grass!" << std::endl;
    }

    tme290::grass::Control control;
    control.command(0);
    od4.send(control);

    while (od4.isRunning()) 
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    retCode = 0;
  }
  return retCode;
}