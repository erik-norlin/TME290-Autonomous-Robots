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

#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include "catch.hpp"

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include "world-object.hpp"

TEST_CASE("Test world object, zero input should give zero output.") {
  WorldObject wo{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

  opendlv::sim::KinematicState ks;
  wo.setKinematicState(ks);

  opendlv::sim::Frame const f = wo.step(0.1);
  float const sum = f.x() + f.y() + f.z() + f.roll() + f.pitch() + f.yaw(); 
  REQUIRE(sum == Approx(0.0f));
}

TEST_CASE("Test world object, non-zero input should give non-zero output.") {
  WorldObject wo{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

  opendlv::sim::KinematicState ks;
  ks.vx(1).vy(1).vz(1).rollRate(1).pitchRate(1).yawRate(1);
  wo.setKinematicState(ks);

  opendlv::sim::Frame const f = wo.step(0.1);
  float const sum = f.x() + f.y() + f.z() + f.roll() + f.pitch() + f.yaw(); 
  REQUIRE(std::abs(sum) == Approx(0.6f).margin(1e-5));
}

TEST_CASE("Test world object, resetting the object should return to initial state.") {
  WorldObject wo{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

  opendlv::sim::KinematicState ks;
  ks.vx(1).vy(1).vz(1).rollRate(1).pitchRate(1).yawRate(1);
  wo.setKinematicState(ks);

  wo.step(1000);
  wo.resetState();
  ks.vx(0).vy(0).vz(0).rollRate(0).pitchRate(0).yawRate(0);
  wo.setKinematicState(ks);
  auto const f = wo.step(1000);
  float  const sum = f.x() + f.y() + f.z() + f.roll() + f.pitch() + f.yaw();  
  REQUIRE(sum == Approx(0.0f));
}
