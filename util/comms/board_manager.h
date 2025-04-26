#ifndef BOARD_MANAGER_H
#define BOARD_MANAGER_H

#include "util/comms/client.h" // Include the client definition
#include <cstdint>
#include <map>
#include <memory> // For std::unique_ptr
#include <optional>
#include <string>
#include <vector>

// Forward declaration if CalibrationData becomes complex or defined elsewhere
// struct CalibrationData;

// Simple structure for calibration data (mirroring Python example)
struct CalibrationData {
  // Required fields from Python example
  uint16_t angle = 0;  // Zero angle register value (0x1000)
  bool inv = false;    // Invert phases (0x1002)
  uint8_t epm = 0;     // E-revs per M-rev (0x1001)
  float torque = 0.0f; // Torque constant (0x1022)
  float zero = 0.0f; // Position offset (0x1015) - Assuming this maps to 0x1015

  // Current sensor offsets
  float ia_off = 0.0f; // (0x1050)
  float ib_off = 0.0f; // (0x1051)
  float ic_off = 0.0f; // (0x1052)

  // Optional EAC (Encoder Angle Compensation) fields
  std::optional<float> eac_scale;               // (0x1100)
  std::optional<float> eac_offset;              // (0x1101)
  std::optional<std::vector<int8_t>> eac_table; // (0x1200 onwards)
};

// Map for control mode names to IDs
const std::map<std::string, uint8_t> kControlModes = {
    {"current", 0},
    {"phase", 1}, // Note: Python example doesn't show phase
                  // actuation logic
    {"torque", 2},
    {"velocity", 3},
    {"position", 4},
    {"pos_vel", 5}, // Note: Python example doesn't show pos_vel actuation logic
    {"pos_ff", 6},

    {"pwm", 7}};

class BoardManager {
public:
  // Constructor: Creates the client and stores target IDs
  BoardManager(const std::string &serial_port, unsigned int baud_rate,
               const std::vector<uint8_t> &target_board_ids);

  // Destructor (default is likely sufficient if using unique_ptr)
  ~BoardManager() = default;

  // Initializes boards by enumerating and confirming them.
  // Returns true if all target boards are successfully initialized.
  bool InitializeBoards(int max_retries_per_board = 5);

  // Loads calibration data onto a specific board.
  bool LoadCalibration(uint8_t board_id, const CalibrationData &calib_data);

  // Sets default motor parameters (watchdog, gains) for specified boards.
  // If board_ids is empty, applies to all managed boards.
  bool InitializeMotorParameters(const std::vector<uint8_t> &board_ids = {});

  // Drives one or more motors.
  // actuation_values should contain the necessary float(s) for the chosen mode.
  bool DriveMotor(const std::vector<uint8_t> &board_ids,
                  const std::string &mode,
                  const std::vector<std::vector<float>> &actuation_values);

  // Drives a single motor.
  bool DriveMotor(uint8_t board_id, const std::string &mode,
                  const std::vector<float> &actuation_values);

  // Provides access to the underlying client if needed for direct calls.
  BLDCControllerClient *GetClient();
  const std::vector<uint8_t> &GetManagedBoardIDs() const;

  // Deleted Copy Operations
  BoardManager(const BoardManager &) = delete;
  BoardManager &operator=(const BoardManager &) = delete;

private:
  std::unique_ptr<BLDCControllerClient> client_;
  std::vector<uint8_t> target_board_ids_;
  std::vector<uint8_t>
      initialized_board_ids_; // Boards successfully enumerated/confirmed
};

#endif // BOARD_MANAGER_H
