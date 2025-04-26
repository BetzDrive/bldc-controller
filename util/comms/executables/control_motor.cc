#include "util/comms/board_manager.h"
#include <atomic> // ** ADDED for std::atomic used in signal handler **
#include <boost/program_options.hpp>
#include <chrono>
#include <csignal> // For signal handling (Ctrl+C)
#include <cstdint>
#include <cstdlib> // ** ADDED for std::exit **
#include <exception>
#include <iomanip> // ** ADDED for std::setprecision and std::fixed **
#include <iostream>
#include <sstream>
#include <string>
#include <thread> // For std::this_thread::sleep_for
#include <vector>

namespace po = boost::program_options;

// --- Global flag for Ctrl+C handling ---
volatile sig_atomic_t g_signal_status = 0;

void SignalHandler(int signal) {
  // Check if already handling a signal to prevent recursive issues
  static std::atomic<bool> is_exiting{false};
  if (is_exiting.exchange(true)) {
    // Already trying to exit, force exit on second signal
    std::cerr << "\nCaught second signal " << signal << ", forcing exit."
              << std::endl;
    std::exit(2); // Use std::exit from <cstdlib>
  }
  g_signal_status = signal;
  std::cerr << "\nCaught signal " << signal
            << ", attempting graceful shutdown..." << std::endl;
}

// --- Helper Function to Parse Comma-Separated Strings ---
template <typename T> std::vector<T> ParseCommaSeparated(const std::string &s) {
  std::vector<T> result;
  std::stringstream ss(s);
  std::string item_str;
  while (std::getline(ss, item_str, ',')) {
    // Trim whitespace
    item_str.erase(0, item_str.find_first_not_of(" \t\n\r\f\v"));
    item_str.erase(item_str.find_last_not_of(" \t\n\r\f\v") + 1);
    if (item_str.empty())
      continue; // Skip empty items

    std::stringstream item_ss(item_str);
    T item_val;
    // Check for complete consumption of the item string
    if (!(item_ss >> item_val) || item_ss.rdbuf()->in_avail() != 0) {
      throw std::runtime_error("Failed to parse value: '" + item_str + "'");
    }
    result.push_back(item_val);
  }
  return result;
}

int main(int argc, char *argv[]) {
  // --- Argument Parsing Setup ---
  po::options_description desc("Options for driving motors in torque mode");
  std::string serial_port_str;
  unsigned int baud_rate;
  std::string board_ids_str;
  std::string torque_values_str;
  int num_iters;

  // Default baud rate - ensure CommConstants is accessible or define locally
  // Using a fixed default if CommConstants isn't directly included/accessible
  // here.
  const unsigned int default_baud = 1000000;

  desc.add_options()("help,h", "Show help message")(
      "serial,s", po::value<std::string>(&serial_port_str)->required(),
      "Serial port (e.g., /dev/ttyACM0, COM3)")(
      "baud_rate,b",
      po::value<unsigned int>(&baud_rate)->default_value(default_baud),
      "Serial baud rate")(
      "board_ids,i", po::value<std::string>(&board_ids_str)->required(),
      "Comma-separated list of board IDs (e.g., \"1,2,3\")")(
      "torques,t", po::value<std::string>(&torque_values_str)->required(),
      "Comma-separated list of torque values (N*m) - one per board ID (e.g., "
      "\"0.1,-0.1,0.0\")")("num_iters,n",
                           po::value<int>(&num_iters)->default_value(0),
                           "Number of iterations (0 for infinite loop)");

  po::variables_map vm;
  try {
    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help")) {
      std::cout << desc << std::endl;
      return 0;
    }

    po::notify(vm); // Check for required options and apply defaults

  } catch (const po::error &e) {
    std::cerr << "Error parsing arguments: " << e.what() << std::endl;
    std::cerr << desc << std::endl;
    return 1;
  } catch (const std::exception &e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  // --- Parse String Arguments ---
  std::vector<uint8_t> board_ids;
  std::vector<float> torque_values;
  try {
    // Parse IDs using uint16_t first for range checking before casting
    std::vector<uint16_t> board_ids_u16 =
        ParseCommaSeparated<uint16_t>(board_ids_str);
    board_ids.reserve(board_ids_u16.size()); // Reserve space
    for (uint16_t id : board_ids_u16) {
      if (id > 255)
        throw std::runtime_error("Board ID exceeds 255: " + std::to_string(id));
      board_ids.push_back(static_cast<uint8_t>(id));
    }

    torque_values = ParseCommaSeparated<float>(torque_values_str);

    if (board_ids.empty()) {
      throw std::runtime_error("Board IDs list cannot be empty.");
    }
    if (torque_values.empty()) {
      throw std::runtime_error("Torque values list cannot be empty.");
    }
    if (board_ids.size() != torque_values.size()) {
      throw std::runtime_error("Number of board IDs (" +
                               std::to_string(board_ids.size()) +
                               ") must match number of torque values (" +
                               std::to_string(torque_values.size()) + ").");
    }

  } catch (const std::exception &e) {
    std::cerr << "Error parsing board IDs or torque values: " << e.what()
              << std::endl;
    return 1;
  }

  // --- Setup Signal Handler ---
  std::signal(SIGINT, SignalHandler);  // Catch Ctrl+C
  std::signal(SIGTERM, SignalHandler); // Catch kill/termination signals

  // --- Board Manager Initialization ---
  std::unique_ptr<BoardManager> manager;
  try {
    std::cout << "Creating Board Manager for port: " << serial_port_str
              << " at " << baud_rate << " baud." << std::endl;
    manager =
        std::make_unique<BoardManager>(serial_port_str, baud_rate, board_ids);

    std::cout << "Initializing boards..." << std::endl;
    // InitializeBoards handles bootloader entry/exit and delays internally
    if (!manager->InitializeBoards()) {
      // BoardManager logs details, just exit here
      std::cerr << "Failed to initialize all target boards. Exiting."
                << std::endl;
      return 1;
    }
    // Check if the successfully initialized boards match the requested ones
    const auto &initialized_ids = manager->GetManagedBoardIDs();
    if (initialized_ids.size() != board_ids.size()) {
      std::cerr
          << "Warning: Not all requested boards were initialized successfully."
          << std::endl;
      // Potentially filter board_ids and torque_values to only include
      // initialized ones? For now, we proceed, but DriveMotor will skip
      // uninitialized boards.
    } else {
      std::cout << "All target boards initialized successfully." << std::endl;
    }

    std::cout << "Initializing motor parameters..." << std::endl;
    // Initialize parameters only for the boards that were successfully
    // initialized by BoardManager
    if (!manager->InitializeMotorParameters(initialized_ids)) {
      std::cerr << "Warning: Failed to initialize parameters on some boards."
                << std::endl;
      // Decide if this is critical - continuing for now
    } else {
      std::cout << "Motor parameters initialized successfully." << std::endl;
    }

  } catch (const CommunicationError &e) {
    std::cerr << "Initialization Error: " << e.what() << std::endl;
    return 1;
  } catch (const std::exception &e) {
    std::cerr << "Initialization Error: " << e.what() << std::endl;
    return 1;
  }

  // --- Main Control Loop ---
  long long iteration_count = 0;
  const long long print_interval = 1000; // Print status every N iterations

  // Prepare actuation data structure for DriveMotor
  std::vector<std::vector<float>> actuation_data;
  actuation_data.reserve(torque_values.size());
  for (float torque : torque_values) {
    actuation_data.push_back({torque}); // Each board gets a vector containing
                                        // its single torque value
  }

  std::cout << "\nStarting torque control loop"
            << (num_iters > 0
                    ? " for " + std::to_string(num_iters) + " iterations"
                    : "")
            << ". Press Ctrl+C to stop." << std::endl;

  auto loop_start_time = std::chrono::high_resolution_clock::now();

  while (g_signal_status == 0 &&
         (num_iters <= 0 || iteration_count < num_iters)) {
    // sleep for 5ms.
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    try {
      // Send torque commands to all specified boards. DriveMotor uses
      // DoTransaction internally, which waits for responses or timeouts for
      // each board write.
      if (!manager->DriveMotor(board_ids, "torque", actuation_data)) {
        // DriveMotor logs errors internally. We log a summary warning here.
        if (g_signal_status ==
            0) { // Avoid logging extra errors during shutdown
          std::cerr << "Warning: DriveMotor reported failure for one or more "
                       "boards on iteration "
                    << iteration_count << std::endl;
        }
        // No sleep here, rely on DoTransaction timeouts if issue persists
      }

      iteration_count++;

      // Print status periodically
      if (iteration_count % print_interval == 0 && g_signal_status == 0) {
        auto now = std::chrono::high_resolution_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                              now - loop_start_time)
                              .count();
        // Avoid division by zero if loop is extremely fast
        double rate =
            (elapsed_ms > 0)
                ? (static_cast<double>(print_interval) * 1000.0 / elapsed_ms)
                : 0.0;
        std::cout << "Iteration: " << iteration_count
                  << " (Rate: " << std::fixed << std::setprecision(1) << rate
                  << " Hz)" << std::endl;
        loop_start_time = now; // Reset timer for next interval
      }

      // No explicit delay - loop runs as fast as communication allows

    } catch (const CommunicationError &e) {
      if (g_signal_status == 0) { // Avoid logging extra errors during shutdown
        std::cerr << "Communication Error during loop (Iteration "
                  << iteration_count << "): " << e.what() << std::endl;
        // Option: break the loop on communication errors?
        // break;
        std::cerr << "Attempting to continue..." << std::endl;
        // No sleep, just continue to the next iteration attempt
      }
    } catch (const std::exception &e) {
      if (g_signal_status == 0) {
        std::cerr << "Unexpected Error during loop (Iteration "
                  << iteration_count << "): " << e.what() << std::endl;
      }
      break; // Stop on unexpected errors
    }
  } // End while loop

  // --- Cleanup ---
  if (g_signal_status != 0) {
    std::cout << "Loop interrupted by signal." << std::endl;
  } else if (num_iters > 0) {
    std::cout << "Loop finished after " << iteration_count << " iterations."
              << std::endl;
  } else {
    // This case means the loop exited without signal and without reaching
    // num_iters (if num_iters > 0) which implies an error broke the loop.
    std::cout << "Loop exited prematurely." << std::endl;
  }

  std::cout << "Setting torques to zero..." << std::endl;
  try {
    // Create zero torque data for all originally requested boards
    std::vector<std::vector<float>> zero_actuation_data(board_ids.size(),
                                                        {0.0f});
    // Attempt to drive even uninitialized boards to zero, DriveMotor will skip
    // if needed
    if (!manager->DriveMotor(board_ids, "torque", zero_actuation_data)) {
      std::cerr << "Warning: Failed to set zero torque on one or more boards "
                   "during shutdown."
                << std::endl;
    }
    // No sleep needed, DriveMotor waits for responses/timeouts
  } catch (const std::exception
               &e) { // Catch potential exceptions during shutdown drive
    std::cerr << "Error setting torques to zero during shutdown: " << e.what()
              << std::endl;
  } catch (...) {
    std::cerr << "Unknown error setting torques to zero during shutdown."
              << std::endl;
  }

  std::cout << "Exiting." << std::endl;
  // BoardManager destructor will handle client shutdown and thread joining.
  manager.reset(); // Explicitly destroy manager before returning (optional)
  return 0;
}
