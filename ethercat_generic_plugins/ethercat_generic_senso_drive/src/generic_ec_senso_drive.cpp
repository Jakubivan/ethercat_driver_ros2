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
//
// Author: Maciej Bednarczyk (macbednarczyk@gmail.com)

#include <numeric>

#include "ethercat_generic_plugins/generic_ec_senso_drive.hpp"

namespace ethercat_generic_plugins
{

EcSensoDrive::EcSensoDrive()
: GenericEcSlave() {}
EcSensoDrive::~EcSensoDrive() {}

bool EcSensoDrive::initialized() const {return initialized_;}

void EcSensoDrive::processData(size_t index, uint8_t * domain_address)
{

  // Special case: ControlWord
  if (pdo_channels_info_[index].index == SENSOD_RPDO_CONTROLWORD) {
    if (is_operational_) {
      if (fault_reset_command_interface_index_ >= 0) {
        if (command_interface_ptr_->at(fault_reset_command_interface_index_) == 0) {
          last_fault_reset_command_ = false;
        }
        if (last_fault_reset_command_ == false &&
          command_interface_ptr_->at(fault_reset_command_interface_index_) != 0 &&
          !std::isnan(command_interface_ptr_->at(fault_reset_command_interface_index_)))
        {
          last_fault_reset_command_ = true;
          fault_reset_ = true;
        }
      }

      if (auto_state_transitions_) {
        pdo_channels_info_[index].default_value = transition(
          state_,
          pdo_channels_info_[index].ec_read(domain_address));
      }
    }
  }

 if (pdo_channels_info_[index].index == SENSOD_RPDO_POSITION || pdo_channels_info_[index].index == SENSOD_RPDO_POSITION_ADVANCED) {
    if ((mode_of_operation_display_ != ModeOfOperation::MODE_NO_MODE)
    && ((pdo_channels_info_[index].is_default_position_set_ && std::abs(command_interface_ptr_->at(1)) > 1e-2) || (!pdo_channels_info_[index].is_default_position_set_))){ 
      
      pdo_channels_info_[index].default_value = pdo_channels_info_[index].factor * last_position_ + pdo_channels_info_[index].offset;

      // std::cout << "Overriding default position: " << pdo_channels_info_[index].default_value << " for index: " << index << std::endl;
      
      if (!pdo_channels_info_[index].is_default_position_set_) {

        // if counter overflows or previous default position is not even set or difference between previous and current default position is bigger than 999
        /* "difference between previous and current default position is bigger than 999" this happens because of a race condition where the default position (this part of a code) is called but the encoder does not report its value yet. 
        We implemented counter which ensures several cycles of reading before setting the default value even if no home offset is set/changed. Should be safe, can be improved though.*/
        if ((pdo_channels_info_[index].counter_default_position_>1000) || ((!std::isnan(pdo_channels_info_[index].previous_default_position_)) && (std::abs(pdo_channels_info_[index].previous_default_position_ - pdo_channels_info_[index].default_value) > 999))) {
          pdo_channels_info_[index].is_default_position_set_ = true;
          std::cout << "..\nDefault position set to " << pdo_channels_info_[index].default_value << " with " << std::dec << pdo_channels_info_[index].counter_default_position_ - 1 << " iterations\n.." << std::endl;
        } else {
          pdo_channels_info_[index].counter_default_position_++;
        }
      }

      pdo_channels_info_[index].previous_default_position_ = pdo_channels_info_[index].default_value;

    }
    pdo_channels_info_[index].override_command =
      (mode_of_operation_display_ != ModeOfOperation::MODE_CYCLIC_SYNC_POSITION && mode_of_operation_display_ != ModeOfOperation::MODE_CYCLIC_SYNC_POSITION_ADVANCED) ? true : false;
  }

  // setup mode of operation
  if (pdo_channels_info_[index].index == SENSOD_RPDO_MODE_OF_OPERATION) {
    if ((mode_of_operation_ >= 0 && mode_of_operation_ <= 10) || mode_of_operation_ == -108) {
      pdo_channels_info_[index].default_value = mode_of_operation_;
    }
  }

  pdo_channels_info_[index].ec_update(domain_address);

  // get mode_of_operation_display_
  if (pdo_channels_info_[index].index == SENSOD_TPDO_MODE_OF_OPERATION_DISPLAY) {
    mode_of_operation_display_ = pdo_channels_info_[index].last_value;
  }

  if (pdo_channels_info_[index].index == SENSOD_TPDO_POSITION) {
    last_position_ = pdo_channels_info_[index].last_value;
  }

  // Special case: StatusWord
  if (pdo_channels_info_[index].index == SENSOD_TPDO_STATUSWORD) {
    status_word_ = pdo_channels_info_[index].last_value;
  }


  // CHECK FOR STATE CHANGE
  if (index == all_channels_.size() - 1) {  // if last entry  in domain
    if (status_word_ != last_status_word_) {
      state_ = deviceState(status_word_);
      if (state_ != last_state_) {
        std::cout << "STATE: " << DEVICE_STATE_STR.at(state_)
                  << " with status word :" << status_word_ << std::endl;
      }
    }
    initialized_ = (state_ == STATE_OPERATION_ENABLED) && (last_state_ == STATE_OPERATION_ENABLED);

    last_status_word_ = status_word_;
    last_state_ = state_;
    counter_++;
  }
}

void EcSensoDrive::set_position_offset_ptrs(uint32_t product_id_) {
  int num_set_ptrs = 0;
  long unsigned int index = 0;

  // Set position offset pointers for TPDO and RPDO
  while (index  < pdo_channels_info_.size()) {
    try {
      if (is_tpdo_position_channel(index) || is_rpdo_position_channel(index)) {
        pdo_channels_info_[index].setPositionOffsetPtr(&position_offset_);
        num_set_ptrs++;
      }
    }
    catch (...) {
      std::cerr << "Error in 'set_position_offset_ptrs()' function." << std::endl;
    }

    index++;

    // TPDO + RPDO
    if (num_set_ptrs == 2) break;
  }

  if (num_set_ptrs != 2) {
    std::cerr << "SensoJoint product_id_: " << product_id_ << ", failed to update position offset PTRs for all relevant PDOs" << std::endl;
  }
}

bool EcSensoDrive::is_tpdo_position_channel(size_t index)
{
  return ((pdo_channels_info_[index].pdo_type == ethercat_interface::TPDO) &&
          (pdo_channels_info_[index].interface_name.compare("position") == 0));
}

bool EcSensoDrive::is_rpdo_position_channel(size_t index)
{
  return ((pdo_channels_info_[index].pdo_type == ethercat_interface::RPDO) &&
          (pdo_channels_info_[index].interface_name.compare("position") == 0));
}

bool EcSensoDrive::setupSlave(
  std::unordered_map<std::string, std::string> slave_paramters,
  std::vector<double> * state_interface,
  std::vector<double> * command_interface)
{
  state_interface_ptr_ = state_interface;
  command_interface_ptr_ = command_interface;
  paramters_ = slave_paramters;

  if (paramters_.find("slave_config") != paramters_.end()) {
    if (!setup_from_config_file(paramters_["slave_config"])) {
      return false;
    }
  } else {
    std::cerr << "EcSensoDrive: failed to find 'slave_config' tag in URDF." << std::endl;
    return false;
  }

  setup_interface_mapping();
  setup_syncs();

  if (paramters_.find("mode_of_operation") != paramters_.end()) {
    mode_of_operation_ = std::stod(paramters_["mode_of_operation"]);
  }

  if (paramters_.find("command_interface/reset_fault") != paramters_.end()) {
    fault_reset_command_interface_index_ = std::stoi(paramters_["command_interface/reset_fault"]);
  }

  if (paramters_.find("position_offset") != paramters_.end()) {
    position_offset_ = std::stof(paramters_["position_offset"]);
  }

  return true;
}

bool EcSensoDrive::setup_from_config(YAML::Node drive_config)
{
  if (!GenericEcSlave::setup_from_config(drive_config)) {return false;}
  // additional configuration parameters for SENSO Drives
  if (drive_config["auto_fault_reset"]) {
    auto_fault_reset_ = drive_config["auto_fault_reset"].as<bool>();
  }
  if (drive_config["auto_state_transitions"]) {
    auto_state_transitions_ = drive_config["auto_state_transitions"].as<bool>();
  }
  return true;
}

bool EcSensoDrive::setup_from_config_file(std::string config_file)
{
  // Read drive configuration from YAML file
  try {
    slave_config_ = YAML::LoadFile(config_file);
  } catch (const YAML::ParserException & ex) {
    std::cerr << "EcSensoDrive: failed to load drive configuration: " << ex.what() << std::endl;
    return false;
  } catch (const YAML::BadFile & ex) {
    std::cerr << "EcSensoDrive: failed to load drive configuration: " << ex.what() << std::endl;
    return false;
  }
  if (!setup_from_config(slave_config_)) {
    return false;
  }
  return true;
}

/** returns device state based upon the status_word */
DeviceState EcSensoDrive::deviceState(uint16_t status_word)
{
  if ((status_word & 0b01001111) == 0b00000000) {
    return STATE_NOT_READY_TO_SWITCH_ON;
  } else if ((status_word & 0b01001111) == 0b01000000) {
    return STATE_SWITCH_ON_DISABLED;
  } else if ((status_word & 0b01101111) == 0b00100001) {
    return STATE_READY_TO_SWITCH_ON;
  } else if ((status_word & 0b01101111) == 0b00100011) {
    return STATE_SWITCH_ON;
  } else if ((status_word & 0b01101111) == 0b00100111) {
    return STATE_OPERATION_ENABLED;
  } else if ((status_word & 0b01101111) == 0b00000111) {
    return STATE_QUICK_STOP_ACTIVE;
  } else if ((status_word & 0b01001111) == 0b00001111) {
    return STATE_FAULT_REACTION_ACTIVE;
  } else if ((status_word & 0b01001111) == 0b00001000) {
    return STATE_FAULT;
  }
  return STATE_UNDEFINED;
}

/** returns the control word that will take device from state to next desired state */
uint16_t EcSensoDrive::transition(DeviceState state, uint16_t control_word)
{
  switch (state) {
    case STATE_START:                     // -> STATE_NOT_READY_TO_SWITCH_ON (automatic)
      return control_word;
    case STATE_NOT_READY_TO_SWITCH_ON:    // -> STATE_SWITCH_ON_DISABLED (automatic)
      return control_word;
    case STATE_SWITCH_ON_DISABLED:        // -> STATE_READY_TO_SWITCH_ON
      return (control_word & 0b01111110) | 0b00000110;
    case STATE_READY_TO_SWITCH_ON:        // -> STATE_SWITCH_ON
      return (control_word & 0b01110111) | 0b00000111;
    case STATE_SWITCH_ON:                 // -> STATE_OPERATION_ENABLED
      return (control_word & 0b01111111) | 0b00001111;
    case STATE_OPERATION_ENABLED:         // -> GOOD
      return control_word;
    case STATE_QUICK_STOP_ACTIVE:         // -> STATE_OPERATION_ENABLED
      return (control_word & 0b01111111) | 0b00001111;
    case STATE_FAULT_REACTION_ACTIVE:     // -> STATE_FAULT (automatic)
      return control_word;
    case STATE_FAULT:                     // -> STATE_SWITCH_ON_DISABLED
      if (auto_fault_reset_ || fault_reset_) {
        fault_reset_ = false;
        return (control_word & 0b11111111) | 0b10000000;     // automatic reset
      } else {
        return control_word;
      }
    default:
      break;
  }
  return control_word;
}

}  // namespace ethercat_generic_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ethercat_generic_plugins::EcSensoDrive, ethercat_interface::EcSlave)
