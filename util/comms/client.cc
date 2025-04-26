#include "util/comm/client.h"

#include <cstring> // For memcpy
#include <iostream> // For basic debugging output (consider replacing with a logging library)
#include <stdexcept>    // For standard exceptions
#include <system_error> // For std::error_code comparison

// --- CRC-16 Implementation (CRC-16-IBM / CRC-16) ---
// Poly: 0x8005 (reflected 0xA001), Init: 0x0000, RefIn: true, RefOut: true,
// XorOut: 0x0000
uint16_t ComputeCRC16(const uint8_t *data, size_t length) {
  uint16_t crc = 0x0000; // Initial value
  while (length--) {
    crc ^= *data++;
    for (int i = 0; i < 8; ++i) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001; // 0xA001 is the reflection of 0x8005
      } else {
        crc >>= 1;
      }
    }
  }
  return crc; // No final XOR
}

uint16_t ComputeCRC16(const ByteVector &data) {
  return ComputeCRC16(data.data(), data.size());
}

// --- Packing/Unpacking Helpers (Little-Endian) ---
// Assumes little-endian, matching Python struct '<' format specifier.
// Add checks for buffer sizes in unpack functions for safety.

ByteVector BLDCControllerClient::PackU8(uint8_t val) { return {val}; }
ByteVector BLDCControllerClient::PackU16(uint16_t val) {
  ByteVector bytes(2);
  bytes[0] = static_cast<uint8_t>(val & 0xFF);
  bytes[1] = static_cast<uint8_t>((val >> 8) & 0xFF);
  return bytes;
}
ByteVector BLDCControllerClient::PackU32(uint32_t val) {
  ByteVector bytes(4);
  bytes[0] = static_cast<uint8_t>(val & 0xFF);
  bytes[1] = static_cast<uint8_t>((val >> 8) & 0xFF);
  bytes[2] = static_cast<uint8_t>((val >> 16) & 0xFF);
  bytes[3] = static_cast<uint8_t>((val >> 24) & 0xFF);
  return bytes;
}
ByteVector BLDCControllerClient::PackF32(float val) {
  ByteVector bytes(4);
  static_assert(sizeof(float) == 4, "Float size must be 4 bytes");
  // Use memcpy for type-punning safely
  std::memcpy(bytes.data(), &val, 4);
  return bytes;
}

uint8_t BLDCControllerClient::UnpackU8(const ByteVector &data, size_t offset) {
  if (offset >= data.size()) {

    throw std::out_of_range("UnpackU8 offset out of range");
  }
  return data[offset];
}

uint16_t BLDCControllerClient::UnpackU16(const ByteVector &data,
                                         size_t offset) {
  if (offset + 1 >= data.size()) {
    throw std::out_of_range("UnpackU16 offset out of range");
  }
  return static_cast<uint16_t>(data[offset]) |
         (static_cast<uint16_t>(data[offset + 1]) << 8);
}
uint32_t BLDCControllerClient::UnpackU32(const ByteVector &data,
                                         size_t offset) {
  if (offset + 3 >= data.size()) {
    throw std::out_of_range("UnpackU32 offset out of range");
  }
  return static_cast<uint32_t>(data[offset]) |
         (static_cast<uint32_t>(data[offset + 1]) << 8) |
         (static_cast<uint32_t>(data[offset + 2]) << 16) |
         (static_cast<uint32_t>(data[offset + 3]) << 24);
}
float BLDCControllerClient::UnpackF32(const ByteVector &data, size_t offset) {
  if (offset + 3 >= data.size()) {
    throw std::out_of_range("UnpackF32 offset out of range");
  }
  static_assert(sizeof(float) == 4, "Float size must be 4 bytes");
  float val;
  // Use memcpy for type-punning safely
  std::memcpy(&val, data.data() + offset, 4);
  return val;
}

// --- BLDCControllerClient Implementation ---

BLDCControllerClient::BLDCControllerClient(const std::string &port_name,
                                           unsigned int baud_rate)
    : io_context_(), serial_port_(io_context_),
      work_guard_(
          boost::asio::make_work_guard(io_context_)), // Keep io_context alive
      incoming_data_buffer_(2 *
                            kReadBufferSize), // Set circular buffer capacity
      stop_threads_(false) {
  try {
    serial_port_.open(port_name);
    serial_port_.set_option(
        boost::asio::serial_port_base::baud_rate(baud_rate));
    serial_port_.set_option(boost::asio::serial_port_base::character_size(8));
    serial_port_.set_option(boost::asio::serial_port_base::parity(
        boost::asio::serial_port_base::parity::none));
    serial_port_.set_option(boost::asio::serial_port_base::stop_bits(
        boost::asio::serial_port_base::stop_bits::one));
    serial_port_.set_option(boost::asio::serial_port_base::flow_control(
        boost::asio::serial_port_base::flow_control::none));

    std::cout << "Serial port " << port_name << " opened successfully."
              << std::endl;

    // Start the I/O thread using std::thread and a lambda
    io_thread_ = std::thread([this]() {
      std::cout << "IO thread started." << std::endl;
      this->io_context_.run(); // Blocks until stopped or out of work
      std::cout << "IO thread finished." << std::endl;
    });

    // Start the background data processing thread
    processing_thread_ =
        std::thread(&BLDCControllerClient::ProcessIncomingData, this);

    // Start the first asynchronous read
    StartReceive();

  } catch (const std::system_error &e) { // Catch specific Asio/system errors
    std::cerr << "Error opening or configuring serial port " << port_name
              << ": " << e.what() << std::endl;
    // Ensure threads aren't left running if port opening fails
    stop_threads_ = true; // Signal threads (though they might not have started)
    if (io_thread_.joinable())
      io_thread_.join();
    if (processing_thread_.joinable())
      processing_thread_.join();
    throw CommunicationError("Failed to initialize serial port: " +
                             std::string(e.what()));
  }
}

BLDCControllerClient::~BLDCControllerClient() {
  std::cout << "Shutting down BLDCControllerClient..." << std::endl;
  stop_threads_ = true; // Signal threads to stop

  // Post task to io_context to safely stop it and close the port from its own
  // thread.
  boost::asio::post(io_context_, [this]() {
    this->work_guard_
        .reset();      // Allow io_context::run() to exit when work is done
    this->ClosePort(); // Close the port safely
  });

  // Wait for threads to finish
  if (io_thread_.joinable()) {
    io_thread_.join();
    std::cout << "IO thread joined." << std::endl;
  }

  // No explicit interrupt needed for std::thread, joining handles it if
  // possible. If ProcessIncomingData blocks indefinitely on buffer access, it
  // might need a condition variable.
  if (processing_thread_.joinable()) {
    processing_thread_.join();
    std::cout << "Processing thread joined." << std::endl;
  }

  // Propagate exceptions for any promises that were not fulfilled due to
  // shutdown
  {
    std::lock_guard<std::mutex> lock(response_map_mutex_);
    for (auto &pair : pending_responses_) {
      try {
        pair.second.set_exception(std::make_exception_ptr(CommunicationError(
            "Client shutting down before response received.")));
      } catch (...) { /* Ignore if promise already set or future destroyed */
      }
    }
    pending_responses_.clear();
  }

  std::cout << "BLDCControllerClient shutdown complete." << std::endl;
}

void BLDCControllerClient::ClosePort() {
  // This should be called from the io_context thread via post()
  if (serial_port_.is_open()) {
    std::error_code ec;      // Use std::error_code
    serial_port_.cancel(ec); // Cancel pending async operations
    if (ec) {
      std::cerr << "Warning: Error cancelling serial port operations: "
                << ec.message() << std::endl;
    }
    serial_port_.close(ec);
    if (ec) {
      std::cerr << "Error closing serial port: " << ec.message() << std::endl;
    } else {
      std::cout << "Serial port closed." << std::endl;
    }
  }
}

void BLDCControllerClient::StartReceive() {
  // Asynchronously read data into the raw buffer
  // Use a lambda for the completion handler for cleaner syntax
  serial_port_.async_read_some(
      boost::asio::buffer(raw_read_buffer_),
      [this](const boost::system::error_code &error, size_t bytes_transferred) {
        this->HandleReceive(error, bytes_transferred);
      });
}

void BLDCControllerClient::HandleReceive(const boost::system::error_code &error,
                                         size_t bytes_transferred) {
  if (stop_threads_)
    return; // Stop if requested

  if (!error) {
    { // Lock scope for buffer access
      std::lock_guard<std::mutex> lock(buffer_mutex_);
      // Add received data to the circular buffer
      incoming_data_buffer_.insert(
          incoming_data_buffer_.end(), raw_read_buffer_.begin(),
          raw_read_buffer_.begin() + bytes_transferred);
    } // Mutex released

    // std::cout << "Received " << bytes_transferred << " bytes. Buffer size: "
    // << incoming_data_buffer_.size() << std::endl; // Debug

    // Start the next read operation
    StartReceive();
  } else if (error != boost::asio::error::operation_aborted) {
    std::cerr << "Serial read error: " << error.message() << std::endl;
    // Close port and notify pending futures about the error
    ClosePort(); // Close on error (safe to call again if already closed)
    {
      std::lock_guard<std::mutex> lock(response_map_mutex_);
      for (auto &pair : pending_responses_) {
        try {
          pair.second.set_exception(std::make_exception_ptr(CommunicationError(
              "Serial port read error: " + error.message())));
        } catch (...) { /* Ignore */
        }
      }
      pending_responses_.clear();
    }
  }
  // If operation_aborted, it means we're shutting down, so do nothing.
}

void BLDCControllerClient::DoWrite(const ByteVector &data) {
  // Post the write operation to the io_context to ensure thread safety
  boost::asio::post(io_context_, [this, data]() {
    // Use lambda for completion handler
    boost::asio::async_write(serial_port_, boost::asio::buffer(data),
                             [this](const boost::system::error_code &error,
                                    size_t bytes_transferred) {
                               this->HandleWrite(error, bytes_transferred);
                             });
  });
}

void BLDCControllerClient::HandleWrite(const boost::system::error_code &error,
                                       size_t bytes_transferred) {
  if (stop_threads_)
    return;

  if (error && error != boost::asio::error::operation_aborted) {
    std::cerr << "Serial write error: " << error.message() << std::endl;
    ClosePort(); // Close on error
    // Notify pending futures about the write error
    {
      std::lock_guard<std::mutex> lock(response_map_mutex_);
      for (auto &pair : pending_responses_) {
        try {
          pair.second.set_exception(std::make_exception_ptr(CommunicationError(
              "Serial port write error: " + error.message())));
        } catch (...) { /* Ignore */
        }
      }
      pending_responses_.clear();
    }
  }
  // else: Write successful or aborted during shutdown
}

// This function runs in its own thread (processing_thread_)
void BLDCControllerClient::ProcessIncomingData() {
  std::cout << "Processing thread started." << std::endl;
  ByteVector
      current_packet_buffer; // Temporarily store bytes for potential packet
  enum class State {
    SYNC,
    VERSION,
    FLAGS,
    LEN_H,
    LEN_L,
    MESSAGE,
    CRC_H,
    CRC_L
  };
  State state = State::SYNC;
  uint16_t expected_len = 0;
  uint16_t received_crc = 0;

  while (!stop_threads_) {
    try {
      uint8_t byte;
      bool byte_read = false;
      { // Lock scope for reading from circular buffer
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        if (!incoming_data_buffer_.empty()) {
          byte = incoming_data_buffer_.front();
          incoming_data_buffer_.pop_front();
          byte_read = true;
        }
      } // Mutex released

      if (!byte_read) {
        // No data, wait briefly before checking again to avoid busy-waiting
        // Consider using a condition variable for more efficient waiting if CPU
        // usage is high.
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        continue;
      }

      // --- State Machine for Packet Parsing ---
      switch (state) {
      case State::SYNC:
        if (byte == CommConstants::START_BYTE) {
          current_packet_buffer.clear();
          current_packet_buffer.push_back(byte); // Start byte
          state = State::VERSION;
        } // else: Keep searching for SYNC
        break;

      case State::VERSION:
        if (byte == CommConstants::COMM_VERSION) {
          current_packet_buffer.push_back(byte); // Version byte
          state = State::FLAGS;
        } else {
          std::cerr << "Malformed Packet: Invalid version "
                    << static_cast<int>(byte) << std::endl;
          state = State::SYNC; // Reset
        }
        break;

      case State::FLAGS:
        current_packet_buffer.push_back(byte); // Flags byte
        state = State::LEN_L; // Read low byte of length first (little-endian)
        break;

      case State::LEN_L:
        current_packet_buffer.push_back(byte); // Length low byte
        expected_len = byte;                   // Store low byte
        state = State::LEN_H;
        break;

      case State::LEN_H:
        current_packet_buffer.push_back(byte); // Length high byte
        expected_len |=
            (static_cast<uint16_t>(byte) << 8); // Combine with high byte
        // Sanity check length - adjust max length as needed
        if (expected_len == 0 || expected_len > kReadBufferSize * 4) {
          std::cerr << "Malformed Packet: Invalid length " << expected_len
                    << std::endl;
          state = State::SYNC; // Reset
        } else {
          state = State::MESSAGE;
          // Reserve space for efficiency if message sizes are predictable
          // current_packet_buffer.reserve(expected_len + 5 + 2); // Header +
          // Message + CRC
        }
        break;

      case State::MESSAGE:
        current_packet_buffer.push_back(byte);
        // Check if we have received the full message payload
        // Header size = SYNC(1) + VER(1) + FLAGS(1) + LEN(2) = 5
        if (current_packet_buffer.size() == (expected_len + 5)) {
          state = State::CRC_L; // Read low byte of CRC first
        }
        break;

      case State::CRC_L:
        received_crc = byte; // Store low byte
        state = State::CRC_H;
        break;

      case State::CRC_H: { // New scope for processing complete packet
        received_crc |=
            (static_cast<uint16_t>(byte) << 8); // Combine with high byte

        // --- Packet Complete - Verify CRC and Process ---
        // Extract the message part for CRC calculation (from Flags up to end of
        // message data) Message part starts at index 2 (Flags) and has length
        // expected_len Ensure expected_len doesn't cause out-of-bounds access
        if (2 + expected_len > current_packet_buffer.size()) {
          std::cerr << "Malformed Packet: Internal length mismatch during CRC "
                       "calculation."
                    << std::endl;
          state = State::SYNC; // Reset
          break;               // Break from case CRC_H
        }
        ByteVector message_part(current_packet_buffer.begin() + 2,
                                current_packet_buffer.begin() + 2 +
                                    expected_len);
        uint16_t calculated_crc = ComputeCRC16(message_part);

        if (calculated_crc == received_crc) {
          // CRC OK - Parse the message content
          try {
            // Message Header: MsgLen(2), ServerID(1), FuncCode(1), Errors(2) =
            // 6 bytes minimum Indices are relative to the start of message_part
            if (message_part.size() < 6) {
              throw MalformedPacketError(
                  "Message part too short (less than 6 bytes)");
            }

            ReceivedPacket packet;
            packet.server_id = UnpackU8(message_part, 2);
            packet.function_code = UnpackU8(message_part, 3);
            packet.errors = UnpackU16(message_part, 4);
            packet.crash_flag = (UnpackU8(current_packet_buffer, 2) &
                                 CommConstants::COMM_FLAG_CRASH) !=
                                0; // Flags from overall header
            if (message_part.size() > 6) {
              packet.data.assign(message_part.begin() + 6, message_part.end());
            }

            // --- Match response to a waiting promise ---
            ResponseMapKey key = {packet.server_id, packet.function_code};
            ResponsePromise
                promise; // Use a temporary to hold the promise outside the lock
            bool promise_found = false;
            { // Lock scope for response map
              std::lock_guard<std::mutex> lock(response_map_mutex_);
              auto it = pending_responses_.find(key);
              if (it != pending_responses_.end()) {
                promise = std::move(it->second); // Move promise out
                pending_responses_.erase(it);    // Remove from map
                promise_found = true;
              } else {
                // Handle unsolicited messages if necessary (e.g., log them)
                std::cout << "Warning: Received unsolicited packet for ID="
                          << static_cast<int>(key.first)
                          << ", FC=" << static_cast<int>(key.second)
                          << std::endl;
              }
            } // Mutex released

            if (promise_found) {
              try {
                promise.set_value(std::move(
                    packet)); // Fulfill the promise (move packet data)
              } catch (const std::future_error &e) {
                // Future was likely destroyed (e.g., timeout before response
                // arrived)
                std::cerr << "Warning: Could not set promise value (maybe "
                             "already timed out?): "
                          << e.what() << std::endl;
              }
            }

          } catch (const std::out_of_range &e) {
            std::cerr << "Error parsing message content (out_of_range): "
                      << e.what() << std::endl;
          } catch (const MalformedPacketError &e) {
            std::cerr << "Error parsing message content (malformed): "
                      << e.what() << std::endl;
          } catch (const std::exception &e) {
            std::cerr << "Error parsing message content (general): " << e.what()
                      << std::endl;
          }

        } else {
          std::cerr << "Malformed Packet: CRC mismatch. Expected "
                    << calculated_crc << ", Got " << received_crc << std::endl;
          // Reset state machine as the packet framing is likely wrong
          state = State::SYNC;
          break; // Break from case CRC_H
        }

        // Packet processed (successfully or not), reset state machine for next
        // packet
        state = State::SYNC;
      } // End scope for processing complete packet
      break;

      } // End switch(state)

    } catch (const std::exception &e) {
      // Catch potential exceptions from buffer access or state machine logic
      std::cerr << "Exception in processing thread: " << e.what() << std::endl;
      state = State::SYNC; // Reset state on general exceptions
    } catch (...) {
      std::cerr << "Unknown exception in processing thread." << std::endl;
      state = State::SYNC; // Reset state
    }
  } // End while(!stop_threads_)
  std::cout << "Processing thread finished." << std::endl;
}

// --- Public API Method Implementations ---

void BLDCControllerClient::WriteRequest(uint8_t server_id, uint8_t func_code,
                                        const ByteVector &data) {
  // Wire format based on Python code analysis:
  // Header: START(1), VER(1), FLAGS(1), TotalMsgLen(2)
  // Message: [SubMsgLen(2), SubMsgData(ID(1), FC(1), Data(...))] (Only one
  // sub-message in this implementation) Footer: CRC(2) (Calculated on Message
  // part)

  // 1. Create Sub-Message Data
  ByteVector sub_message_data;
  sub_message_data.push_back(server_id);
  sub_message_data.push_back(func_code);
  sub_message_data.insert(sub_message_data.end(), data.begin(), data.end());

  // 2. Create Message Part (SubMsgLen + SubMsgData)
  ByteVector message;
  ByteVector sub_msg_len_bytes =
      PackU16(static_cast<uint16_t>(sub_message_data.size()));
  message.insert(message.end(), sub_msg_len_bytes.begin(),
                 sub_msg_len_bytes.end());
  message.insert(message.end(), sub_message_data.begin(),
                 sub_message_data.end());

  // 3. Create Full Packet
  ByteVector packet;
  packet.push_back(CommConstants::START_BYTE);     // Sync
  packet.push_back(CommConstants::COMM_VERSION);   // Version
  packet.push_back(CommConstants::COMM_FLAG_SEND); // Flags
  // Add total message length (length of 'message' part)
  ByteVector total_msg_len_bytes =
      PackU16(static_cast<uint16_t>(message.size()));
  packet.insert(packet.end(), total_msg_len_bytes.begin(),
                total_msg_len_bytes.end());
  // Add the message itself
  packet.insert(packet.end(), message.begin(), message.end());

  // 4. Calculate CRC on the 'message' part
  uint16_t crc = ComputeCRC16(message);
  ByteVector crc_bytes = PackU16(crc);
  packet.insert(packet.end(), crc_bytes.begin(), crc_bytes.end());

  // 5. Send the packet asynchronously
  DoWrite(packet);
}

ResponseFuture
BLDCControllerClient::DoTransaction(uint8_t server_id, uint8_t func_code,
                                    const ByteVector &data,
                                    std::chrono::milliseconds timeout) {
  ResponseMapKey key = {server_id, func_code};
  ResponsePromise promise;
  ResponseFuture future = promise.get_future();

  { // Lock scope for response map access
    std::lock_guard<std::mutex> lock(response_map_mutex_);
    // Check if a request for this ID/FC is already pending
    auto [it, inserted] =
        pending_responses_.try_emplace(key, std::move(promise));
    if (!inserted) {
      // Handle error: Already waiting for a response for this key.
      // Option: Throw immediately.
      throw ProtocolError(
          "Transaction already pending for ID=" + std::to_string(server_id) +
          ", FC=" + std::to_string(func_code));
      // Option 2: Replace the old promise (caller of old future might hang or
      // timeout) it->second = std::move(promise); // Overwrite existing promise
    }
  } // Mutex released

  // Send the request packet
  WriteRequest(server_id, func_code, data);

  // Return the future. The caller will wait on this future.
  // Timeout handling happens when the caller waits on the future.
  return future;
}

// --- Example Public Method Implementations ---

ByteVector BLDCControllerClient::ReadRegisters(uint8_t server_id,
                                               uint16_t start_addr,
                                               uint8_t count) {
  // Pack arguments: start_addr (uint16_t), count (uint8_t)
  ByteVector args;
  ByteVector addr_bytes = PackU16(start_addr);
  ByteVector count_byte = PackU8(count);
  args.insert(args.end(), addr_bytes.begin(), addr_bytes.end());
  args.insert(args.end(), count_byte.begin(), count_byte.end());

  ResponseFuture future =
      DoTransaction(server_id, CommConstants::COMM_FC_REG_READ, args);

  // Wait for the response with timeout
  std::future_status status =
      future.wait_for(CommConstants::DEFAULT_RESPONSE_TIMEOUT);

  if (status == std::future_status::timeout) {
    // Timeout occurred - attempt to remove the pending promise to prevent late
    // fulfillment
    ResponseMapKey key = {server_id, CommConstants::COMM_FC_REG_READ};
    bool erased = false;
    {
      std::lock_guard<std::mutex> lock(response_map_mutex_);
      if (pending_responses_.count(key)) {
        // Check if the promise is still there before erasing
        // (it might have been fulfilled just after the timeout check)

        pending_responses_.erase(key);
        erased = true;
      }
    }
    if (erased) {
      std::cerr << "Timeout: Erased pending promise for ID="
                << static_cast<int>(server_id)
                << ", FC=" << static_cast<int>(CommConstants::COMM_FC_REG_READ)
                << std::endl;
    }
    throw TimeoutError("Timeout waiting for ReadRegisters response from ID=" +
                       std::to_string(server_id));
  } else if (status == std::future_status::deferred) {
    // Should not happen with std::promise unless async policy was used
    // differently
    throw CommunicationError("ReadRegisters future was deferred.");
  }
  // else: status == std::future_status::ready

  // Get the result (or exception if one occurred during processing/IO)
  ReceivedPacket response = future.get();

  // Check for device-reported errors
  if (response.errors != CommConstants::COMM_ERRORS_NONE) {

    // You could create more specific error messages based on flags
    throw ProtocolError("Device reported error during ReadRegisters (ID=" +
                            std::to_string(server_id) + ")",
                        response.errors);
  }
  if (response.crash_flag) {
    // Log or handle crash flag if needed
    std::cerr << "Warning: Crash flag set in response from ID="
              << static_cast<int>(server_id) << std::endl;
  }

  return response.data;
}

bool BLDCControllerClient::WriteRegisters(uint8_t server_id,
                                          uint16_t start_addr,
                                          const ByteVector &data) {
  // Pack arguments: start_addr (uint16_t), count (uint8_t), data
  if (data.size() > 255) {
    throw std::invalid_argument(
        "WriteRegisters data size exceeds maximum (255 bytes)");
  }
  ByteVector args;
  ByteVector addr_bytes = PackU16(start_addr);
  // The count here refers to the number of registers, matching Python's
  // `struct.pack("<HB", addr, ct)` Assuming the device protocol expects the
  // *number* of registers being written, not byte length. If it expects byte
  // length, change this. Let's assume register count for now.
  // **CRITICAL**: Verify the protocol spec for COMM_FC_REG_WRITE arguments!

  // If `count` should be byte length: PackU8(static_cast<uint8_t>(data.size()))
  // If `count` should be register count (assuming fixed size regs): Need
  // register size info. Let's *assume* the Python code's `ct` meant register
  // count and the device figures out bytes. This is ambiguous without the
  // device spec. Using byte count seems safer if unsure.
  ByteVector count_byte = PackU8(static_cast<uint8_t>(
      data.size())); // **ASSUMPTION: Count is byte length**

  args.insert(args.end(), addr_bytes.begin(), addr_bytes.end());
  args.insert(args.end(), count_byte.begin(),
              count_byte.end());                     // Add count byte
  args.insert(args.end(), data.begin(), data.end()); // Add actual data

  ResponseFuture future =
      DoTransaction(server_id, CommConstants::COMM_FC_REG_WRITE, args);

  std::future_status status =
      future.wait_for(CommConstants::DEFAULT_RESPONSE_TIMEOUT);

  if (status == std::future_status::timeout) {
    ResponseMapKey key = {server_id, CommConstants::COMM_FC_REG_WRITE};
    bool erased = false;
    {
      std::lock_guard<std::mutex> lock(response_map_mutex_);
      if (pending_responses_.count(key)) {
        pending_responses_.erase(key);
        erased = true;
      }
    }
    if (erased) {
      std::cerr << "Timeout: Erased pending promise for ID="
                << static_cast<int>(server_id)
                << ", FC=" << static_cast<int>(CommConstants::COMM_FC_REG_WRITE)
                << std::endl;
    }
    throw TimeoutError("Timeout waiting for WriteRegisters response from ID=" +
                       std::to_string(server_id));
  } else if (status == std::future_status::deferred) {
    throw CommunicationError("WriteRegisters future was deferred.");
  }

  ReceivedPacket response = future.get();

  if (response.errors != CommConstants::COMM_ERRORS_NONE) {
    throw ProtocolError("Device reported error during WriteRegisters (ID=" +
                            std::to_string(server_id) + ")",
                        response.errors);
  }
  if (response.crash_flag) {
    std::cerr << "Warning: Crash flag set in response from ID="
              << static_cast<int>(server_id) << std::endl;
  }

  // Success is indicated by no errors (response.data might be empty or contain
  // status)
  return true;
}

ByteVector BLDCControllerClient::ReadWriteRegisters(
    uint8_t server_id, uint16_t read_start_addr, uint8_t read_count,
    uint16_t write_start_addr, const ByteVector &write_data) {
  if (write_data.size() > 255) {
    throw std::invalid_argument(
        "ReadWriteRegisters write_data size exceeds maximum (255 bytes)");
  }
  // Pack arguments: read_start(u16), read_count(u8), write_start(u16),
  // write_count(u8), write_data(...) Again, assuming 'count' refers to byte
  // length for write_count. **VERIFY PROTOCOL**
  ByteVector args;
  args.insert(args.end(), PackU16(read_start_addr).begin(),
              PackU16(read_start_addr).end());
  args.insert(args.end(), PackU8(read_count).begin(),
              PackU8(read_count).end()); // Read count (likely register count)
  args.insert(args.end(), PackU16(write_start_addr).begin(),
              PackU16(write_start_addr).end());
  args.insert(args.end(),
              PackU8(static_cast<uint8_t>(write_data.size())).begin(),
              PackU8(static_cast<uint8_t>(write_data.size()))
                  .end()); // Write count (assuming bytes)
  args.insert(args.end(), write_data.begin(), write_data.end());

  ResponseFuture future =
      DoTransaction(server_id, CommConstants::COMM_FC_REG_READ_WRITE, args);

  std::future_status status =
      future.wait_for(CommConstants::DEFAULT_RESPONSE_TIMEOUT);

  if (status == std::future_status::timeout) {
    ResponseMapKey key = {server_id, CommConstants::COMM_FC_REG_READ_WRITE};
    bool erased = false;
    {
      std::lock_guard<std::mutex> lock(response_map_mutex_);
      if (pending_responses_.count(key)) {
        pending_responses_.erase(key);
        erased = true;
      }
    }
    if (erased) {
      std::cerr << "Timeout: Erased pending promise for ID="
                << static_cast<int>(server_id) << ", FC="
                << static_cast<int>(CommConstants::COMM_FC_REG_READ_WRITE)
                << std::endl;
    }
    throw TimeoutError(
        "Timeout waiting for ReadWriteRegisters response from ID=" +
        std::to_string(server_id));
  } else if (status == std::future_status::deferred) {
    throw CommunicationError("ReadWriteRegisters future was deferred.");
  }

  ReceivedPacket response = future.get();

  if (response.errors != CommConstants::COMM_ERRORS_NONE) {
    throw ProtocolError("Device reported error during ReadWriteRegisters (ID=" +
                            std::to_string(server_id) + ")",
                        response.errors);
  }
  if (response.crash_flag) {
    std::cerr << "Warning: Crash flag set in response from ID="
              << static_cast<int>(server_id) << std::endl;
  }

  return response.data; // Return the read data
}

bool BLDCControllerClient::ResetSystem(uint8_t server_id) {
  // This function likely doesn't expect a response as the device resets.
  // Use WriteRequest for fire-and-forget.
  WriteRequest(server_id, CommConstants::COMM_FC_SYSTEM_RESET);
  // Add a small delay to allow the device to potentially start resetting.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  return true; // Assume success unless an error occurs during write itself
               // (handled in HandleWrite)
}

bool BLDCControllerClient::JumpToAddress(uint8_t server_id,
                                         uint32_t jump_addr) {
  // Similar to reset, might not get a response.
  ByteVector args = PackU32(jump_addr);
  WriteRequest(server_id, CommConstants::COMM_FC_JUMP_TO_ADDR, args);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  return true;
}

// --- Implement other public methods here ---
// (e.g., GetFlashSectorCount, EraseFlashSector, ProgramFlash, etc.)
// Each will involve:
// 1. Packing arguments into a ByteVector according to the device protocol.
// 2. Calling DoTransaction (if a response is expected) or WriteRequest.
// 3. Waiting on the future (if using DoTransaction) and handling
// timeouts/errors.
// 4. Unpacking the response data (if any) or checking for errors.
