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

#include "behavior.hpp"

Behavior::Behavior() noexcept:
  m_frontUltrasonicReading{},
  m_rearUltrasonicReading{},
  m_leftIrReading{},
  m_rightIrReading{},
  
  m_frontUltrasonicReadingMutex{},
  m_rearUltrasonicReadingMutex{},
  m_leftIrReadingMutex{},
  m_rightIrReadingMutex{},
  
  m_wL{},
  m_wR{},
  m_wLMutex{},
  m_wRMutex{}
{
}

opendlv::proxy::AxleAngularVelocityRequest Behavior::get_wL() noexcept
{
  std::lock_guard<std::mutex> lock(m_wLMutex);
  return m_wL;
}

opendlv::proxy::AxleAngularVelocityRequest Behavior::get_wR() noexcept
{
  std::lock_guard<std::mutex> lock(m_wRMutex);
  return m_wR;
}

void Behavior::setFrontUltrasonic(opendlv::proxy::DistanceReading const &frontUltrasonicReading) noexcept
{
  std::lock_guard<std::mutex> lock(m_frontUltrasonicReadingMutex);
  m_frontUltrasonicReading = frontUltrasonicReading;
}

void Behavior::setRearUltrasonic(opendlv::proxy::DistanceReading const &rearUltrasonicReading) noexcept
{
  std::lock_guard<std::mutex> lock(m_rearUltrasonicReadingMutex);
  m_rearUltrasonicReading = rearUltrasonicReading;
}

void Behavior::setLeftIr(opendlv::proxy::DistanceReading const &leftIrReading) noexcept
{
  std::lock_guard<std::mutex> lock(m_leftIrReadingMutex);
  m_leftIrReading = leftIrReading;
}

void Behavior::setRightIr(opendlv::proxy::DistanceReading const &rightIrReading) noexcept
{
  std::lock_guard<std::mutex> lock(m_rightIrReadingMutex);
  m_rightIrReading = rightIrReading;
}


void Behavior::step(float t) noexcept
{
  opendlv::proxy::DistanceReading frontUltrasonicReading;
  opendlv::proxy::DistanceReading rearUltrasonicReading;
  opendlv::proxy::DistanceReading leftIrReading;
  opendlv::proxy::DistanceReading rightIrReading;
  {
    std::lock_guard<std::mutex> lock1(m_frontUltrasonicReadingMutex);
    std::lock_guard<std::mutex> lock2(m_rearUltrasonicReadingMutex);
    std::lock_guard<std::mutex> lock3(m_leftIrReadingMutex);
    std::lock_guard<std::mutex> lock4(m_rightIrReadingMutex);

    frontUltrasonicReading = m_frontUltrasonicReading;
    rearUltrasonicReading = m_rearUltrasonicReading;
    leftIrReading = m_leftIrReading;
    rightIrReading = m_rightIrReading;
  }
   
  float const r{0.04f};
  float const v0{0.5f};
  uint8_t const t1{3};
  uint8_t const t2{10};
  float vL{};
  float vR{};
  
  if (0 <= t && t <= t1) {
    vL = 0;
    vR = v0*t/t1;
  }
  else if (t1 < t && t <= t2) {
    vL = v0*(t-t1)/t2;
    vR = v0;
  }
  else {
    vL = 0;
    vR = 0;
  }
  
  float wLVal = vL/r;
  float wRVal = vR/r;
  
  {
    std::lock_guard<std::mutex> lock1(m_wLMutex);
    std::lock_guard<std::mutex> lock2(m_wRMutex);

    opendlv::proxy::AxleAngularVelocityRequest wL;
    wL.axleAngularVelocity(wLVal);
    m_wL = wL;

    opendlv::proxy::AxleAngularVelocityRequest wR;
    wR.axleAngularVelocity(wRVal);
    m_wR = wR;
  }
}
