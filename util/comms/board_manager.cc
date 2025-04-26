#include "util/comms/board_manager.h"
#include <algorithm> // For std::find
#include <chrono>    // For durations
#include <iostream>
#include <stdexcept> // For invalid_argument
#include <string>
#include <thread> // For std::this_thread::sleep_for
#include <vector>

// Helper function to pack multiple floats (implementation assumes
// little-endian)
static ByteVector PackFloats(const std::vector<float> &values) {
  ByteVector packed_data;
  packed_data.reserve(values.size() * sizeof(float));
  for (float val : values) {
    ByteVector temp = BLDCControllerClient::PackF32(val);
    packed_data.insert(packed_data.end(), temp.begin(), temp.end());
  }
  return packed_data;
}

BoardManager::BoardManager(const std::string &serial_port,
                           unsigned int baud_rate,
                           const std::vector<uint8_t> &target_board_ids)
    : target_board_ids_(target_board_ids) {
  client_ = std::make_unique<BLDCControllerClient>(serial_port, baud_rate);
  std::cout << "Board Manager created for port: " << serial_port << std::endl;
}

bool BoardManager::InitializeBoards(int max_retries_per_board) {
  if (!client_) {
    std::cerr << "Error: BLDC Client not initialized." << std::endl;
    return false;
  }

  std::cout << "Starting board initialization..." << std::endl;
  initialized_board_ids_.clear();

  try {
    // Resetting system with ID 0 might put all boards into bootloader
    std::cout << "Resetting system (ID 0) to enter bootloader mode..."
              << std::endl;
    client_->EnterBootloader(0); // Resets and adds delay
    client_->ResetInputBuffer(); // Clear any initial garbage

    for (uint8_t target_id : target_board_ids_) {
      std::cout << "--- Initializing Board ID: " << static_cast<int>(target_id)
                << " ---" << std::endl;
      bool enumerated = false;
      bool confirmed = false;

      // 1. Enumerate Board
      for (int attempt = 1; attempt <= max_retries_per_board && !enumerated;
           ++attempt) {
        std::cout << "  Attempt " << attempt << ": Enumerating..." << std::endl;
        try {
          uint8_t response_id = client_->EnumerateBoard(target_id);
          if (response_id == target_id) {
            std::cout << "  Enumeration successful (received ID: "
                      << static_cast<int>(response_id) << ")" << std::endl;
            enumerated = true;
          } else {
            // This case should ideally be caught by exceptions within
            // EnumerateBoard
            std::cerr << "  Enumeration response mismatch (Expected "
                      << static_cast<int>(target_id)
                      << ", Got: " << static_cast<int>(response_id) << ")"
                      << std::endl;
          }
        } catch (const TimeoutError &e) {
          std::cerr << "  Enumeration attempt " << attempt
                    << " timed out: " << e.what() << std::endl;
          client_->ResetInputBuffer(); // Clear buffer on timeout
        } catch (const CommunicationError &e) {
          std::cerr << "  Enumeration attempt " << attempt
                    << " failed: " << e.what() << std::endl;
          client_->ResetInputBuffer(); // Clear buffer on comms error
        }
        if (!enumerated) {
          std::this_thread::sleep_for(
              std::chrono::milliseconds(100)); // Wait before retrying
        }
      }

      if (!enumerated) {
        std::cerr << "Failed to enumerate Board ID: "
                  << static_cast<int>(target_id) << " after "
                  << max_retries_per_board << " attempts." << std::endl;
        continue; // Try next board
      }

      // 2. Confirm Board
      for (int attempt = 1; attempt <= max_retries_per_board && !confirmed;
           ++attempt) {
        std::cout << "  Attempt " << attempt << ": Confirming..." << std::endl;
        try {
          if (client_->ConfirmBoard(target_id)) {
            std::cout << "  Confirmation successful." << std::endl;
            confirmed = true;
          } else {
            // ConfirmBoard should throw on error, so this else might not be
            // reached
            std::cerr << "  Confirmation attempt " << attempt
                      << " reported failure." << std::endl;
          }
        } catch (const TimeoutError &e) {
          std::cerr << "  Confirmation attempt " << attempt
                    << " timed out: " << e.what() << std::endl;
          client_->ResetInputBuffer();
        } catch (const CommunicationError &e) {
          std::cerr << "  Confirmation attempt " << attempt
                    << " failed: " << e.what() << std::endl;
          client_->ResetInputBuffer();
        }
        if (!confirmed) {
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
      }

      if (confirmed) {
        std::cout << "--- Board ID: " << static_cast<int>(target_id)
                  << " successfully initialized. ---" << std::endl;
        initialized_board_ids_.push_back(target_id);
      } else {
        std::cerr << "Failed to confirm Board ID: "
                  << static_cast<int>(target_id) << " after "
                  << max_retries_per_board << " attempts." << std::endl;
      }
      std::this_thread::sleep_for(
          std::chrono::milliseconds(50)); // Small delay between boards
    } // End loop through target_board_ids

    // 3. Leave Bootloader (for initialized boards)
    if (!initialized_board_ids_.empty()) {
      std::cout << "Leaving bootloader for initialized boards: ";
      for (uint8_t id : initialized_board_ids_)
        std::cout << static_cast<int>(id) << " ";
      std::cout << std::endl;
      // Send jump command to all initialized boards simultaneously
      // Note: Python code leaves bootloader one by one. Sending all at once
      // might be okay. If issues arise, loop and call LeaveBootloader
      // individually.
      for (uint8_t id : initialized_board_ids_) {
        client_->LeaveBootloader(id); // Jumps and adds delay
      }
      std::cout << "Jump commands sent." << std::endl;
      // Extra delay after all jumps initiated
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      client_->ResetInputBuffer(); // Clear buffer after jumps
    } else {
      std::cout << "No boards were successfully initialized." << std::endl;
    }

  } catch (const CommunicationError &e) {
    std::cerr << "Communication error during board initialization: " << e.what()
              << std::endl;
    return false;
  } catch (const std::exception &e) {
    std::cerr << "Unexpected error during board initialization: " << e.what()
              << std::endl;
    return false;
  }

  std::cout << "Board initialization finished. Initialized IDs: ";
  if (initialized_board_ids_.empty()) {
    std::cout << "None";
  } else {
    for (size_t i = 0; i < initialized_board_ids_.size(); ++i) {
      std::cout << static_cast<int>(initialized_board_ids_[i])
                << (i == initialized_board_ids_.size() - 1 ? "" : ", ");
    }
  }
  std::cout << std::endl;

  // Return true only if all target boards were initialized
  return initialized_board_ids_.size() == target_board_ids_.size();
}

bool BoardManager::LoadCalibration(uint8_t board_id,
                                   const CalibrationData &calib_data) {
  if (!client_)
    return false;

  // Check if board_id is among the initialized ones (optional but good
  // practice)
  if (std::find(initialized_board_ids_.begin(), initialized_board_ids_.end(),
                board_id) == initialized_board_ids_.end()) {
    std::cerr << "Warning: Attempting to load calibration on board ID "
              << static_cast<int>(board_id)
              << " which was not successfully initialized." << std::endl;
    // Decide whether to proceed or return false
    // return false;
  }

  std::cout << "Loading calibration for Board ID: "
            << static_cast<int>(board_id) << std::endl;

  try {
    // Set parameters based on CalibrationData struct
    // Note: The Python code uses single-element lists. C++ methods take single
    // values.

    client_->WriteRegisters(
        board_id, 0x1000,
        BLDCControllerClient::PackU16(calib_data.angle)); // Zero Angle
    client_->WriteRegisters(
        board_id, 0x1002,
        BLDCControllerClient::PackU8(calib_data.inv ? 1 : 0)); // Invert Phases
    client_->WriteRegisters(
        board_id, 0x1001,
        BLDCControllerClient::PackU8(calib_data.epm)); // E-Revs per M-Rev
    client_->WriteRegisters(
        board_id, 0x1022,
        BLDCControllerClient::PackF32(calib_data.torque)); // Torque Constant
    client_->WriteRegisters(
        board_id, 0x1015,
        BLDCControllerClient::PackF32(calib_data.zero)); // Position Offset

    // Current Offsets (ia, ib, ic are consecutive registers 0x1050, 0x1051,
    // 0x1052)
    ByteVector offset_data =
        PackFloats({calib_data.ia_off, calib_data.ib_off, calib_data.ic_off});
    client_->WriteRegisters(board_id, 0x1050,
                            offset_data); // Write all 3 offsets

    // Optional EAC parameters
    if (calib_data.eac_scale.has_value()) {
      std::cout << "  Writing EAC scale..." << std::endl;
      client_->WriteRegisters(
          board_id, 0x1100,
          BLDCControllerClient::PackF32(calib_data.eac_scale.value()));
    }
    if (calib_data.eac_offset.has_value()) {
      std::cout << "  Writing EAC offset..." << std::endl;
      client_->WriteRegisters(
          board_id, 0x1101,
          BLDCControllerClient::PackF32(calib_data.eac_offset.value()));
    }
    if (calib_data.eac_table.has_value()) {
      std::cout << "  Writing EAC table..." << std::endl;
      const auto &table = calib_data.eac_table.value();
      size_t table_len = table.size();
      const size_t slice_len =
          64; // Max write size per transaction (adjust if needed)

      for (size_t i = 0; i < table_len; i += slice_len) {
        size_t current_slice_size = std::min(slice_len, table_len - i);
        ByteVector table_slice_bytes;
        table_slice_bytes.reserve(current_slice_size);
        for (size_t j = 0; j < current_slice_size; ++j) {
          // Pack int8_t - needs explicit cast to uint8_t for vector
          table_slice_bytes.push_back(static_cast<uint8_t>(table[i + j]));
        }
        // Address starts at 0x1200
        client_->WriteRegisters(board_id, static_cast<uint16_t>(0x1200 + i),
                                table_slice_bytes);
      }
      std::cout << "  EAC table written." << std::endl;
    }

    // Set default control mode after calibration (e.g., current control)
    // client_->WriteRegisters(board_id, 0x2000,
    // BLDCControllerClient::PackU8(kControlModes.at("current")));

    std::cout << "Calibration loaded successfully for Board ID: "
              << static_cast<int>(board_id) << std::endl;
    return true;

  } catch (const ProtocolError &e) {
    // Catch specific protocol errors, e.g., if EAC registers don't exist
    std::cerr << "Protocol error loading calibration for Board ID "
              << static_cast<int>(board_id) << ": " << e.what()
              << " (Error Flags: " << e.GetErrorFlags() << ")" << std::endl;
    if (e.GetErrorFlags() & CommConstants::COMM_ERRORS_INVALID_ARGS ||
        e.GetErrorFlags() & CommConstants::COMM_ERRORS_INVALID_FC) {
      std::cerr << "  This might indicate unsupported calibration features "
                   "(like EAC) on this firmware."
                << std::endl;
    }
  } catch (const CommunicationError &e) {
    std::cerr << "Communication error loading calibration for Board ID "
              << static_cast<int>(board_id) << ": " << e.what() << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "Unexpected error loading calibration for Board ID "
              << static_cast<int>(board_id) << ": " << e.what() << std::endl;
  }
  return false; // Return false if any error occurred
}

bool BoardManager::InitializeMotorParameters(
    const std::vector<uint8_t> &board_ids) {
  if (!client_)
    return false;

  const std::vector<uint8_t> &ids_to_init =
      board_ids.empty() ? initialized_board_ids_ : board_ids;

  if (ids_to_init.empty()) {
    std::cout << "No boards specified or initialized to set parameters for."
              << std::endl;
    return true; // Nothing to do
  }

  std::cout << "Initializing motor parameters for Board IDs: ";
  for (size_t i = 0; i < ids_to_init.size(); ++i) {
    std::cout << static_cast<int>(ids_to_init[i])
              << (i == ids_to_init.size() - 1 ? "" : ", ");
  }
  std::cout << std::endl;

  bool all_success = true;
  for (uint8_t board_id : ids_to_init) {
    // Check if board_id is among the initialized ones
    if (std::find(initialized_board_ids_.begin(), initialized_board_ids_.end(),
                  board_id) == initialized_board_ids_.end()) {
      std::cerr
          << "Skipping parameter initialization for uninitialized Board ID: "
          << static_cast<int>(board_id) << std::endl;
      all_success = false;
      continue;
    }

    std::cout << "  Setting parameters for Board ID: "
              << static_cast<int>(board_id) << std::endl;
    bool success_this_board = false;
    int retry_count = 0;
    const int max_retries = 3;

    while (!success_this_board && retry_count < max_retries) {
      try {

        // Set Watchdog (Register 0x1030, type uint16_t)
        client_->WriteRegisters(
            board_id, 0x1030,
            BLDCControllerClient::PackU16(1000)); // 1000ms timeout

        // Set Gains (Registers 0x1003-0x1006, type float)
        // Note: Writing individually for simplicity, could potentially combine
        // if registers are contiguous and WriteRegisters handles multi-register
        // writes correctly based on byte count.
        client_->WriteRegisters(
            board_id, 0x1003,
            BLDCControllerClient::PackF32(0.5f)); // Direct Current Kp
        client_->WriteRegisters(
            board_id, 0x1004,
            BLDCControllerClient::PackF32(0.1f)); // Direct Current Ki
        client_->WriteRegisters(
            board_id, 0x1005,
            BLDCControllerClient::PackF32(1.0f)); // Quadrature Current Kp
        client_->WriteRegisters(
            board_id, 0x1006,
            BLDCControllerClient::PackF32(0.2f)); // Quadrature Current Ki

        success_this_board =
            true; // All writes succeeded for this board on this attempt

      } catch (const CommunicationError &e) {
        retry_count++;
        std::cerr << "  Attempt " << retry_count << " failed for Board ID "
                  << static_cast<int>(board_id) << ": " << e.what()
                  << std::endl;
        if (retry_count >= max_retries) {
          all_success = false;
          std::cerr << "  Failed to set parameters for Board ID "
                    << static_cast<int>(board_id) << " after " << max_retries
                    << " attempts." << std::endl;
        } else {
          std::this_thread::sleep_for(
              std::chrono::milliseconds(100)); // Wait before retry
        }
      } catch (const std::exception &e) {
        std::cerr << "  Unexpected error setting parameters for Board ID "
                  << static_cast<int>(board_id) << ": " << e.what()
                  << std::endl;
        all_success = false;
        break; // Don't retry on unexpected errors
      }
    } // End retry loop
  } // End loop through board IDs

  std::cout << "Finished setting motor parameters." << std::endl;
  return all_success;
}

bool BoardManager::DriveMotor(uint8_t board_id,

                              const std::string &mode,
                              const std::vector<float> &actuation_values) {
  // Call the multi-board version with single elements
  return DriveMotor({board_id}, mode, {actuation_values});
}

bool BoardManager::DriveMotor(
    const std::vector<uint8_t> &board_ids, const std::string &mode,
    const std::vector<std::vector<float>> &actuation_values) {
  if (!client_)
    return false;

  if (board_ids.size() != actuation_values.size()) {
    throw std::invalid_argument(
        "Number of board IDs must match number of actuation value sets.");
  }

  if (kControlModes.find(mode) == kControlModes.end()) {
    throw std::invalid_argument("Invalid control mode specified: " + mode);
  }
  uint8_t control_mode_id = kControlModes.at(mode);

  bool all_success = true;
  for (size_t i = 0; i < board_ids.size(); ++i) {
    uint8_t board_id = board_ids[i];
    const auto &actuation = actuation_values[i];

    // Check if board_id is among the initialized ones
    if (std::find(initialized_board_ids_.begin(), initialized_board_ids_.end(),
                  board_id) == initialized_board_ids_.end()) {
      std::cerr << "Skipping drive command for uninitialized Board ID: "
                << static_cast<int>(board_id) << std::endl;
      all_success = false;
      continue;
    }

    std::cout << "Driving motor ID: " << static_cast<int>(board_id)
              << " Mode: " << mode << std::endl;

    try {
      // 1. Set Control Mode (Register 0x2000, type uint8_t)
      client_->WriteRegisters(board_id, 0x2000,
                              BLDCControllerClient::PackU8(control_mode_id));

      // 2. Write Actuation Values based on mode
      //    Register addresses based on Python example comments
      ByteVector packed_actuation;
      uint16_t actuation_reg_addr = 0;

      switch (control_mode_id) {
      case 0: // current (Id, Iq) - Registers 0x2001, 0x2002 (2 floats)
        if (actuation.size() != 2)
          throw std::invalid_argument(
              "Mode 'current' requires 2 actuation values (Id, Iq).");
        actuation_reg_addr = 0x2001;
        packed_actuation = PackFloats(actuation);
        break;
      case 1: // phase (Va, Vb, Vc) - Registers 0x2003, 0x2004, 0x2005 (3
              // floats) - **NOT FULLY IMPLEMENTED IN PYTHON EXAMPLE**
        if (actuation.size() != 3)
          throw std::invalid_argument(
              "Mode 'phase' requires 3 actuation values (Va, Vb, Vc).");
        actuation_reg_addr = 0x2003;
        packed_actuation = PackFloats(actuation);
        break;
      case 2: // torque (N*m) - Register 0x2006 (1 float)
        if (actuation.size() != 1)
          throw std::invalid_argument(
              "Mode 'torque' requires 1 actuation value.");
        actuation_reg_addr = 0x2006;
        packed_actuation = PackFloats(actuation);
        break;
      case 3: // velocity (rad/s) - Register 0x2007 (1 float)
        if (actuation.size() != 1)
          throw std::invalid_argument(
              "Mode 'velocity' requires 1 actuation value.");
        actuation_reg_addr = 0x2007;
        packed_actuation = PackFloats(actuation);
        break;
      case 4: // position (rad) - Register 0x2008 (1 float)
        if (actuation.size() != 1)
          throw std::invalid_argument(
              "Mode 'position' requires 1 actuation value.");
        actuation_reg_addr = 0x2008;
        packed_actuation = PackFloats(actuation);
        break;
      case 5: // pos_vel (rad, rad/s) - Registers 0x2008, 0x2009 (2 floats) -
              // **NOT FULLY IMPLEMENTED IN PYTHON EXAMPLE**
        if (actuation.size() != 2)
          throw std::invalid_argument(
              "Mode 'pos_vel' requires 2 actuation values (pos, vel).");
        actuation_reg_addr = 0x2008;
        packed_actuation = PackFloats(actuation);
        break;
      case 6: // pos_ff (rad, ff[A]) - Registers 0x2008, 0x2009 (2 floats) -
              // Python uses 0x2008/9
        if (actuation.size() != 2)
          throw std::invalid_argument(
              "Mode 'pos_ff' requires 2 actuation values (pos, ff).");
        actuation_reg_addr = 0x2008;
        packed_actuation = PackFloats(actuation);
        break;
      case 7: // pwm (dc) - Register 0x200A (1 float)
        if (actuation.size() != 1)
          throw std::invalid_argument("Mode 'pwm' requires 1 actuation value.");
        actuation_reg_addr = 0x200A;
        packed_actuation = PackFloats(actuation);
        break;
      default:
        throw std::logic_error(
            "Unhandled control mode ID in switch statement."); // Should not
                                                               // happen
      }

      // Write the packed actuation data
      if (!packed_actuation.empty()) {
        client_->WriteRegisters(board_id, actuation_reg_addr, packed_actuation);
      } else {
        std::cerr << "Warning: No actuation data generated for mode " << mode
                  << std::endl;
      }

    } catch (const CommunicationError &e) {
      std::cerr << "Communication error driving motor ID "
                << static_cast<int>(board_id) << ": " << e.what() << std::endl;
      all_success = false;
    } catch (const std::invalid_argument &e) {
      std::cerr << "Invalid argument driving motor ID "
                << static_cast<int>(board_id) << ": " << e.what() << std::endl;
      all_success = false; // Error in arguments for this board
    } catch (const std::exception &e) {
      std::cerr << "Unexpected error driving motor ID "
                << static_cast<int>(board_id) << ": " << e.what() << std::endl;
      all_success = false;
    }
  } // End loop through boards

  return all_success;
}

BLDCControllerClient *BoardManager::GetClient() {
  return client_.get(); // Return raw pointer from unique_ptr
}

const std::vector<uint8_t> &BoardManager::GetManagedBoardIDs() const {
  return initialized_board_ids_; // Return successfully initialized boards
}
