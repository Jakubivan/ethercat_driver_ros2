// Copyright 2023 ICUBE Laboratory, University of Strasbourg
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TEST_GENERIC_EC_SENSO_DRIVE_HPP_
#define TEST_GENERIC_EC_SENSO_DRIVE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

#include "gmock/gmock.h"

#include "ethercat_generic_plugins/generic_ec_senso_drive.hpp"

// subclassing and friending so we can access member variables
class FriendEcSensoDrive : public ethercat_generic_plugins::EcSensoDrive
{
  FRIEND_TEST(EcSensoDriveTest, SlaveSetupDriveFromConfig);
  FRIEND_TEST(EcSensoDriveTest, SlaveSetupPdoChannels);
  FRIEND_TEST(EcSensoDriveTest, SlaveSetupSyncs);
  FRIEND_TEST(EcSensoDriveTest, SlaveSetupDomains);
  FRIEND_TEST(EcSensoDriveTest, EcReadTPDOToStateInterface);
  FRIEND_TEST(EcSensoDriveTest, EcWriteRPDOFromCommandInterface);
  FRIEND_TEST(EcSensoDriveTest, EcWriteRPDODefaultValue);
  // FRIEND_TEST(EcSensoDriveTest, FaultReset);
  FRIEND_TEST(EcSensoDriveTest, SwitchModeOfOperation);
  FRIEND_TEST(EcSensoDriveTest, EcWriteDefaultTargetPosition);
};

class EcSensoDriveTest : public ::testing::Test
{
public:
  void SetUp();
  void TearDown();

protected:
  std::unique_ptr<FriendEcSensoDrive> plugin_;
};

#endif  // TEST_GENERIC_EC_SENSO_DRIVE_HPP_
