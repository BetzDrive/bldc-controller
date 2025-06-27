#ifndef CLIENT_H // Use the actual filename
#define CLIENT_H

// C++ Standard Library Headers
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <optional> // requires C++17
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

// Other Library Headers (Boost)
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/circular_buffer.hpp>

// --- Constants ---
namespace CommConstants {
const uint8_t COMM_VERSION = 0xFE;
const uint8_t START_BYTE = 0xFF;

// Error Flags
const uint16_t COMM_ERRORS_NONE = 0;
const uint16_t COMM_ERRORS_OP_FAILED = 1;
const uint16_t COMM_ERRORS_MALFORMED = 2;
const uint16_t COMM_ERRORS_INVALID_FC = 4;
const uint16_t COMM_ERRORS_INVALID_ARGS = 8;
const uint16_t COMM_ERRORS_BUF_LEN_MISMATCH = 16;

// Function Codes
const uint8_t COMM_FC_NOP = 0x00;
const uint8_t COMM_FC_REG_READ = 0x01;
const uint8_t COMM_FC_REG_WRITE = 0x02;
const uint8_t COMM_FC_REG_READ_WRITE = 0x03;
const uint8_t COMM_FC_CLEAR_IWDGRST = 0x10;
const uint8_t COMM_FC_SYSTEM_RESET = 0x80;
const uint8_t COMM_FC_JUMP_TO_ADDR = 0x81;
const uint8_t COMM_FC_FLASH_SECTOR_COUNT = 0x82;
const uint8_t COMM_FC_FLASH_SECTOR_START = 0x83;
const uint8_t COMM_FC_FLASH_SECTOR_SIZE = 0x84;
const uint8_t COMM_FC_FLASH_SECTOR_ERASE = 0x85;
const uint8_t COMM_FC_FLASH_PROGRAM = 0x86;
const uint8_t COMM_FC_FLASH_READ = 0x87;
const uint8_t COMM_FC_FLASH_VERIFY = 0x88;
const uint8_t COMM_FC_FLASH_VERIFY_ERASED = 0x89;
const uint8_t COMM_FC_CONFIRM_ID = 0xFE;
const uint8_t COMM_FC_ENUMERATE = 0xFF;

// Flags
const uint8_t COMM_FLAG_SEND = 0x00;
const uint8_t COMM_FLAG_CRASH =
    0x02; // Indicates a crash occurred on the device

// Timeouts
const std::chrono::milliseconds DEFAULT_RESPONSE_TIMEOUT{10};
const std::chrono::milliseconds ENUMERATE_RESPONSE_TIMEOUT{
    1000}; // Longer timeout for enumerate

// Addresses (Add others as needed)
const uint32_t COMM_FIRMWARE_OFFSET =
    0x08010000; // Default firmware start address

} // namespace CommConstants

// --- Custom Exception Classes ---
class CommunicationError : public std::runtime_error {
public:
  explicit CommunicationError(const std::string &message)
      : std::runtime_error(message) {}
};

class ProtocolError : public CommunicationError {
public:
  ProtocolError(const std::string &message, uint16_t error_flags = 0)
      : CommunicationError(message), error_flags_(error_flags) {}

  uint16_t GetErrorFlags() const { return error_flags_; }

private:
  uint16_t error_flags_;
};

class MalformedPacketError : public CommunicationError {
public:
  explicit MalformedPacketError(const std::string &message)
      : CommunicationError(message) {}
};

class TimeoutError : public CommunicationError {
public:
  explicit TimeoutError(const std::string &message)
      : CommunicationError(message) {}
};

// --- Type Definitions ---
using ByteVector = std::vector<uint8_t>;

// --- Sub-Message Structure (for building requests) ---
struct SubMessage {
  uint8_t server_id;
  uint8_t func_code;
  ByteVector data;
};

// --- Packet Structure (for received data) ---
struct ReceivedPacket {
  // --- Header Fields ---
  uint8_t flags; // Raw flags byte from the response header
  uint16_t packet_length; // Packet length from the response header
  // --- Sub-Message Fields (assuming one per response packet) ---
  uint16_t sub_len;       // Sub-message length from the response body
  uint8_t server_id;     // Board ID from the response body
  uint8_t function_code; // Function code from the response body
  uint16_t errors;       // Error flags from the response body
  ByteVector data;       // Payload data from the response body
  // --- Derived Flags ---
  bool crash_flag; // Derived from the header flags byte
};

// --- More Type Definitions ---
using ResponseFuture = std::future<ReceivedPacket>;
using ResponsePromise = std::promise<ReceivedPacket>;
using ResponseMapKey = std::pair<uint8_t, uint8_t>; // server_id, function_code

// --- CRC Calculation ---
// CRC-16-IBM: Poly=0x8005, Init=0x0000, RefIn=true, RefOut=true, XorOut=0x0000
uint16_t ComputeCRC16(const uint8_t *data, size_t length);
uint16_t ComputeCRC16(const ByteVector &data);

// --- Main Client Class ---
class BLDCControllerClient {
public:
  BLDCControllerClient(const std::string &port_name, unsigned int baud_rate);
  ~BLDCControllerClient();

  // --- Public API Methods ---
  ByteVector ReadRegisters(uint8_t server_id, uint16_t start_addr,
                           uint8_t count);
  // ** UPDATED: Takes register_count explicitly **
  bool WriteRegisters(uint8_t server_id, uint16_t start_addr,
                      uint8_t register_count, const ByteVector &data);
  // ** UPDATED: Takes write_register_count explicitly **
  ByteVector ReadWriteRegisters(uint8_t server_id, uint16_t read_start_addr,
                                uint8_t read_count, uint16_t write_start_addr,
                                uint8_t write_register_count,
                                const ByteVector &write_data);

  bool ResetSystem(uint8_t server_id);         // Sends reset command
  void EnterBootloader(uint8_t server_id = 0); // Often targets ID 0 or all
  void
  LeaveBootloader(uint8_t server_id,
                  uint32_t jump_addr = CommConstants::COMM_FIRMWARE_OFFSET);
  bool JumpToAddress(uint8_t server_id, uint32_t jump_addr);

  // --- Bootloader Specific ---
  uint8_t EnumerateBoard(uint8_t target_id,
                         std::chrono::milliseconds timeout =
                             CommConstants::ENUMERATE_RESPONSE_TIMEOUT);
  bool ConfirmBoard(uint8_t board_id,
                    std::chrono::milliseconds timeout =
                        CommConstants::DEFAULT_RESPONSE_TIMEOUT);

  // --- Utility ---
  void ResetInputBuffer();

  // ... Add other public API methods as needed (Flash operations, etc.) ...

  // --- Lower Level Communication ---
  void WriteRequest(uint8_t server_id, uint8_t func_code,
                    const ByteVector &data = {});
  // Sends a packet containing potentially multiple sub-messages.
  // Does not wait for or handle responses directly. Suitable for
  // fire-and-forget commands or broadcasts.
  void WriteMultipleRequests(const std::vector<SubMessage> &sub_messages);
  ResponseFuture DoTransaction(uint8_t server_id, uint8_t func_code,
                               const ByteVector &data = {},
                               std::chrono::milliseconds timeout =
                                   CommConstants::DEFAULT_RESPONSE_TIMEOUT);

  // --- Deleted Copy Operations ---
  BLDCControllerClient(const BLDCControllerClient &) = delete;
  BLDCControllerClient &operator=(const BLDCControllerClient &) = delete;

private:
  // --- Asio and Threading Members ---
  boost::asio::io_context io_context_;
  boost::asio::serial_port serial_port_;
  std::thread io_thread_;
  std::thread processing_thread_;
  boost::asio::executor_work_guard<boost::asio::io_context::executor_type>
      work_guard_;

  // --- Buffering and State ---
  static constexpr size_t kReadBufferSize = 1024;

  std::array<uint8_t, kReadBufferSize> raw_read_buffer_;
  boost::circular_buffer<uint8_t> incoming_data_buffer_;
  std::mutex buffer_mutex_;

  std::atomic<bool> stop_threads_;

  // --- Request/Response Matching ---
  std::mutex response_map_mutex_;
  std::map<ResponseMapKey, ResponsePromise> pending_responses_;

  // --- Private Helper Methods ---
  void StartReceive();
  void HandleReceive(const boost::system::error_code &error,
                     size_t bytes_transferred);
  void DoWrite(const ByteVector &data);
  void HandleWrite(const boost::system::error_code &error,
                   size_t bytes_transferred);
  void ProcessIncomingData();
  void ClosePort();

  // Helper to build the full packet bytes for sending
  ByteVector BuildPacket(const std::vector<SubMessage> &sub_messages);

public:
  // --- Static Packing/Unpacking Helpers (Little-Endian) ---
  static ByteVector PackU8(uint8_t val);
  static ByteVector PackU16(uint16_t val);
  static ByteVector PackU32(uint32_t val);
  static ByteVector PackF32(float val);
  // ... add more pack helpers as needed ...

  static uint8_t UnpackU8(const ByteVector &data, size_t offset = 0);
  static uint16_t UnpackU16(const ByteVector &data, size_t offset = 0);
  static uint32_t UnpackU32(const ByteVector &data, size_t offset = 0);
  static float UnpackF32(const ByteVector &data, size_t offset = 0);
  // ... add more unpack helpers as needed ...

}; // class BLDCControllerClient

#endif // CLIENT_H
