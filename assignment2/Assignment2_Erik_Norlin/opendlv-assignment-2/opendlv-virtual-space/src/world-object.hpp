/*
 * Copyright (C) 2023 Ola Benderius, Björnborg Nguyen
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

#ifndef WORLD_OBJECT
#define WORLD_OBJECT

#include <mutex>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "opendlv-standard-message-set.hpp"

class WorldObject {
 private:
  WorldObject(WorldObject const &) = delete;
  WorldObject(WorldObject &&) = delete;
  WorldObject &operator=(WorldObject const &) = delete;
  WorldObject &operator=(WorldObject &&) = delete;

 public:
  WorldObject(float, float, float, float, float, float) noexcept;
  ~WorldObject() = default;

 public:
  void setKinematicState(opendlv::sim::KinematicState const &) noexcept;
  opendlv::sim::Frame step(double) noexcept;
  void resetState() noexcept;
 private:
  opendlv::sim::Frame m_frame;
  opendlv::sim::KinematicState m_kinematicState;
  std::mutex m_kinematicStateMutex;
  Eigen::Affine3f m_initialState;
};

#endif
