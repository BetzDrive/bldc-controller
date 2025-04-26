#include "util/comms/client.h" // Use the actual filename

#include <chrono>  // For durations
#include <cstring> // For memcpy
#include <iomanip> // For std::setprecision in debug output
#include <iostream> // For basic debugging output (consider replacing with a logging library)
#include <stdexcept>    // For standard exceptions
#include <system_error> // For std::error_code comparison
#include <thread>       // For std::this_thread::sleep_for

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
  std::memcpy(&val, data.data() + offset, 4);
  return val;
}

// --- BLDCControllerClient Implementation ---

BLDCControllerClient::BLDCControllerClient(const std::string &port_name,
                                           unsigned int baud_rate)
    : io_context_(), serial_port_(io_context_),

      work_guard_(boost::asio::make_work_guard(io_context_)),
      incoming_data_buffer_(2 * kReadBufferSize), stop_threads_(false) {
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

    io_thread_ = std::thread([this]() {
      std::cout << "IO thread started." << std::endl;
      try {
        this->io_context_.run(); // Blocks until stopped or out of work
      } catch (const std::exception &e) {
        std::cerr << "Exception in IO thread: " << e.what() << std::endl;
      }
      std::cout << "IO thread finished." << std::endl;
    });

    processing_thread_ =
        std::thread(&BLDCControllerClient::ProcessIncomingData, this);

    StartReceive();

  } catch (const std::system_error &e) {
    std::cerr << "Error opening or configuring serial port " << port_name
              << ": " << e.what() << std::endl;
    stop_threads_ = true;
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
  if (!stop_threads_.exchange(true)) { // Ensure stop signal is sent only once

    boost::asio::post(io_context_, [this]() {
      this->work_guard_.reset();
      this->ClosePort();
    });

    if (io_thread_.joinable()) {
      io_thread_.join();
      std::cout << "IO thread joined." << std::endl;
    }

    if (processing_thread_.joinable()) {
      processing_thread_.join();
      std::cout << "Processing thread joined." << std::endl;
    }

    {
      std::lock_guard<std::mutex> lock(response_map_mutex_);
      for (auto &pair : pending_responses_) {
        try {
          pair.second.set_exception(std::make_exception_ptr(CommunicationError(
              "Client shutting down before response received.")));
        } catch (...) { /* Ignore */
        }
      }
      pending_responses_.clear();
    }
    std::cout << "BLDCControllerClient shutdown complete." << std::endl;
  } else {
    std::cout << "BLDCControllerClient already shutting down." << std::endl;
  }
}

void BLDCControllerClient::ClosePort() {
  if (serial_port_.is_open()) {
    boost::system::error_code ec;
    serial_port_.cancel(ec);
    serial_port_.close(ec);
    if (ec) {
      std::cerr << "Error closing serial port: " << ec.message() << std::endl;
    } else {
      std::cout << "Serial port closed." << std::endl;
    }
  }
}

void BLDCControllerClient::StartReceive() {
  if (stop_threads_ || !serial_port_.is_open())
    return;

  serial_port_.async_read_some(
      boost::asio::buffer(raw_read_buffer_),
      [this](const boost::system::error_code &error, size_t bytes_transferred) {
        this->HandleReceive(error, bytes_transferred);
      });
}

void BLDCControllerClient::HandleReceive(const boost::system::error_code &error,
                                         size_t bytes_transferred) {
  if (stop_threads_)
    return;

  if (!error) {
    {
      std::lock_guard<std::mutex> lock(buffer_mutex_);
      incoming_data_buffer_.insert(
          incoming_data_buffer_.end(), raw_read_buffer_.begin(),
          raw_read_buffer_.begin() + bytes_transferred);
    }
    StartReceive();
  } else if (error != boost::asio::error::operation_aborted &&
             error != boost::asio::error::eof) {
    std::cerr << "Serial read error: " << error.message() << std::endl;
    ClosePort();
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
  } else if (error == boost::asio::error::eof) {
    std::cerr << "Serial port connection closed (EOF)." << std::endl;
    ClosePort();
  }
}

void BLDCControllerClient::DoWrite(const ByteVector &data) {
  if (stop_threads_ || !serial_port_.is_open()) {
    std::cerr
        << "Warning: Write attempted while client stopping or port closed."
        << std::endl;
    return;
  }
  boost::asio::post(io_context_, [this, data]() {
    if (stop_threads_ || !serial_port_.is_open())
      return;
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
    ClosePort();
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
}

// --- ProcessIncomingData (Index-Based Robust Parsing Logic) ---
void BLDCControllerClient::ProcessIncomingData() {
  std::cout << "Processing thread started (Index-Based Robust Parsing)."
            << std::endl;

  // State persists across iterations of the outer loop
  enum class State {
    SEEK_SYNC,
    CHECK_VERSION,
    READ_FLAGS,
    READ_LEN_L,
    READ_LEN_H,
    READ_MESSAGE,
    READ_CRC_L,
    READ_CRC_H
  };
  State current_state = State::SEEK_SYNC;
  uint16_t expected_payload_len = 0;
  uint16_t received_crc = 0;
  size_t working_index =
      0; // Current byte being examined relative to buffer start (index 0)

  while (!stop_threads_) {
    try {
      size_t buffer_size;
      { // Lock scope to check buffer size
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        buffer_size = incoming_data_buffer_.size();
      } // Lock released

      if (buffer_size == 0) {
        // No data, wait briefly
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        continue; // Go back to checking buffer size
      }

      // --- Attempt to parse one packet from the start of the buffer ---
      current_state = State::SEEK_SYNC;
      working_index = 0; // Start check from the beginning of the current buffer
      bool packet_found = false;
      bool parse_error = false;
      size_t packet_len = 0; // Length of successfully parsed packet

      // Inner loop: Process bytes using working_index as long as data is
      // available
      while (working_index < buffer_size) {
        uint8_t current_byte;
        { // Lock scope to read byte at working_index
          std::lock_guard<std::mutex> lock(buffer_mutex_);
          // Re-check size in case buffer changed or was popped by error
          // handling
          buffer_size = incoming_data_buffer_.size();
          if (working_index >= buffer_size) {
            break; // Not enough data currently available for this index
          }
          current_byte = incoming_data_buffer_[working_index];
        } // Lock released

        // --- State Machine ---
        switch (current_state) {
        case State::SEEK_SYNC:
          if (working_index != 0) {
            std::cerr << "Parser Internal Error: SEEK_SYNC state at index > 0. "
                         "Resetting."
                      << std::endl;
            parse_error = true;
            goto handle_parse_result;
          }
          if (current_byte == CommConstants::START_BYTE) {
            current_state = State::CHECK_VERSION;
          } else {
            parse_error = true;
            // Don't log every skipped byte unless debugging is enabled
            // std::cerr << "Parser: Skipping non-sync byte 0x" << std::hex <<
            // static_cast<int>(current_byte) << std::dec << " at buffer start."
            // << std::endl;
          }
          break; // End SEEK_SYNC

        case State::CHECK_VERSION:
          if (current_byte == CommConstants::COMM_VERSION) {
            current_state = State::READ_FLAGS;
          } else {
            // std::cerr << "Parser: Invalid version byte 0x" << std::hex
            //           << static_cast<int>(current_byte) << ". Resetting."
            //           << std::dec << std::endl;
            parse_error = true;
          }
          break; // End CHECK_VERSION

        case State::READ_FLAGS:
          current_state = State::READ_LEN_L;
          break; // End READ_FLAGS

        case State::READ_LEN_L:
          expected_payload_len = current_byte;
          current_state = State::READ_LEN_H;
          break; // End READ_LEN_L

        case State::READ_LEN_H:
          expected_payload_len |= (static_cast<uint16_t>(current_byte) << 8);
          if (expected_payload_len == 0 ||
              expected_payload_len >
                  kReadBufferSize * 4) { // Sanity check length
            std::cerr << "Parser: Invalid payload length "
                      << expected_payload_len << ". Resetting." << std::endl;
            parse_error = true;
          } else {
            current_state = State::READ_MESSAGE;
          }
          break; // End READ_LEN_H

        case State::READ_MESSAGE:
          // Header=5 bytes. Payload starts at index 5. Last payload byte is
          // index 5 + len - 1.
          if (working_index == (5 + expected_payload_len - 1)) {
            current_state = State::READ_CRC_L;
          } else if (working_index > (5 + expected_payload_len - 1)) {
            std::cerr << "Parser: Overshot expected message length. Resetting."
                      << std::endl;
            parse_error = true;
          }
          break; // End READ_MESSAGE

        case State::READ_CRC_L:
          received_crc = current_byte;
          current_state = State::READ_CRC_H;
          break; // End READ_CRC_L

        case State::READ_CRC_H: { // Scope for CRC check and final processing
          received_crc |= (static_cast<uint16_t>(current_byte) << 8);
          packet_len =
              working_index + 1; // Total bytes from SYNC to CRC_H inclusive

          ByteVector received_packet_bytes;
          { // Lock scope to copy packet bytes
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            buffer_size = incoming_data_buffer_.size();
            if (packet_len > buffer_size) {
              std::cerr << "Parser: Buffer size (" << buffer_size
                        << ") changed during CRC check, less than packet len ("
                        << packet_len << "). Resetting." << std::endl;
              parse_error = true;
              goto handle_parse_result;
            }
            received_packet_bytes.reserve(packet_len);
            for (size_t i = 0; i < packet_len; ++i) {
              received_packet_bytes.push_back(incoming_data_buffer_[i]);
            }
          }

          const size_t inner_msg_start_index = 5;
          const size_t crc_calc_start_index = inner_msg_start_index;
          const size_t crc_calc_len = expected_payload_len;

          if (crc_calc_start_index + crc_calc_len >
              received_packet_bytes.size() - 2) { // -2 for CRC bytes
            std::cerr
                << "Parser: Internal length/CRC bounds mismatch. Resetting."
                << std::endl;
            parse_error = true;
            goto handle_parse_result;
          }

          ByteVector message_part_for_crc(
              received_packet_bytes.begin() + crc_calc_start_index,
              received_packet_bytes.begin() + crc_calc_start_index +
                  crc_calc_len);
          uint16_t calculated_crc = ComputeCRC16(message_part_for_crc);

          if (calculated_crc == received_crc) {
            try {
              if (message_part_for_crc.size() < 6) {
                throw MalformedPacketError(
                    "Message part too short (less than 6 bytes)");
              }
              ReceivedPacket packet;
              packet.server_id = UnpackU8(message_part_for_crc, 2);
              packet.function_code = UnpackU8(message_part_for_crc, 3);
              packet.errors = UnpackU16(message_part_for_crc, 4);

              packet.crash_flag = (UnpackU8(received_packet_bytes, 2) &
                                   CommConstants::COMM_FLAG_CRASH) != 0;

              if (message_part_for_crc.size() > 6) {
                packet.data.assign(message_part_for_crc.begin() + 6,
                                   message_part_for_crc.end());
              }

              ResponseMapKey key = {packet.server_id, packet.function_code};
              ResponsePromise promise;
              bool promise_found = false;
              {
                std::lock_guard<std::mutex> lock(response_map_mutex_);
                auto it = pending_responses_.find(key);
                if (it != pending_responses_.end()) {
                  promise = std::move(it->second);
                  pending_responses_.erase(it);
                  promise_found = true;
                }
              }
              if (promise_found) {
                try {
                  promise.set_value(std::move(packet));
                } catch (const std::future_error &) {
                }
              } else {
                std::cout << "Parser: Received unsolicited packet ID="
                          << static_cast<int>(key.first)
                          << ", FC=" << static_cast<int>(key.second)
                          << std::endl;
                std::cout << "Outstanding packets: " << std::endl;
                for (const auto &entry : pending_responses_) {
                  std::cout << "  ID=" << static_cast<int>(entry.first.first)
                            << ", FC=" << static_cast<int>(entry.first.second)
                            << std::endl;
                }
              }

            } catch (const std::exception &e) {
              std::cerr << "Parser: Error parsing valid packet content: "
                        << e.what() << ". Treating as error." << std::endl;
              parse_error = true;
              goto handle_parse_result;
            }
            packet_found = true; // Signal success

          } else {
            std::cerr << "Parser: CRC mismatch. Expected 0x" << std::hex
                      << calculated_crc << ", Got 0x" << received_crc
                      << ". Resetting." << std::dec << std::endl;
            parse_error = true; // Signal error
          }
        }
          goto handle_parse_result; // Exit switch and inner loop after CRC
                                    // state
          break;                    // End READ_CRC_H

        } // End switch(current_state)

        if (parse_error) {
          goto handle_parse_result; // Exit inner loop if error detected
        }

        working_index++; // Move to next byte for next iteration

      } // End inner while loop (parsing attempt)

    handle_parse_result:; // Label to jump to after finishing or erroring in
                          // inner loop

      { // Lock scope for buffer modification
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        if (packet_found) {
          // Pop the successfully parsed packet
          size_t current_buf_size = incoming_data_buffer_.size();
          size_t pop_count = std::min(packet_len, current_buf_size);
          if (pop_count < packet_len) {

            std::cerr
                << "Parser Warning: Buffer size (" << current_buf_size
                << ") shrunk unexpectedly before popping packet of length "
                << packet_len << ". Popping available bytes." << std::endl;
          }
          for (size_t i = 0; i < pop_count; ++i) {
            incoming_data_buffer_.pop_front();
          }
        } else if (parse_error) {
          // Pop only the first byte that started the failed parse attempt
          if (!incoming_data_buffer_.empty()) {
            incoming_data_buffer_.pop_front();
          }
        }
        // If neither: ran out of data mid-packet, don't pop anything.
      } // Lock released

    } catch (const std::exception &e) {
      std::cerr << "Exception in processing thread outer loop: " << e.what()
                << std::endl;
    } catch (...) {
      std::cerr << "Unknown exception in processing thread outer loop."
                << std::endl;
    }
    // Loop continues to check buffer size again
  } // End outer while(!stop_threads_)
  std::cout << "Processing thread finished." << std::endl;
}

// --- Public API Method Implementations ---

void BLDCControllerClient::WriteRequest(uint8_t server_id, uint8_t func_code,
                                        const ByteVector &data) {
  ByteVector sub_message_data;
  sub_message_data.push_back(server_id);
  sub_message_data.push_back(func_code);
  sub_message_data.insert(sub_message_data.end(), data.begin(), data.end());

  ByteVector message;
  ByteVector sub_msg_len_bytes =
      PackU16(static_cast<uint16_t>(sub_message_data.size()));
  message.insert(message.end(), sub_msg_len_bytes.begin(),
                 sub_msg_len_bytes.end());
  message.insert(message.end(), sub_message_data.begin(),
                 sub_message_data.end());

  ByteVector packet;
  packet.push_back(CommConstants::START_BYTE);
  packet.push_back(CommConstants::COMM_VERSION);
  packet.push_back(CommConstants::COMM_FLAG_SEND);
  ByteVector total_msg_len_bytes =
      PackU16(static_cast<uint16_t>(message.size()));
  packet.insert(packet.end(), total_msg_len_bytes.begin(),
                total_msg_len_bytes.end());
  packet.insert(packet.end(), message.begin(), message.end());

  uint16_t crc = ComputeCRC16(message);
  ByteVector crc_bytes = PackU16(crc);
  packet.insert(packet.end(), crc_bytes.begin(), crc_bytes.end());

  DoWrite(packet);
}

ResponseFuture
BLDCControllerClient::DoTransaction(uint8_t server_id, uint8_t func_code,
                                    const ByteVector &data,
                                    std::chrono::milliseconds timeout) {
  ResponseMapKey key = {server_id, func_code};
  ResponsePromise promise;
  ResponseFuture future = promise.get_future();

  {
    std::lock_guard<std::mutex> lock(response_map_mutex_);
    auto [it, inserted] =
        pending_responses_.try_emplace(key, std::move(promise));
    if (!inserted) {
      throw ProtocolError(
          "Transaction already pending for ID=" + std::to_string(server_id) +
          ", FC=" + std::to_string(func_code));
    }
  }

  WriteRequest(server_id, func_code, data);

  return future;
}

ByteVector BLDCControllerClient::ReadRegisters(uint8_t server_id,
                                               uint16_t start_addr,
                                               uint8_t count) {
  ByteVector args;
  args.insert(args.end(), PackU16(start_addr).begin(),
              PackU16(start_addr).end());
  args.insert(args.end(), PackU8(count).begin(),
              PackU8(count).end()); // Pass register count directly

  ResponseFuture future =
      DoTransaction(server_id, CommConstants::COMM_FC_REG_READ, args,
                    CommConstants::DEFAULT_RESPONSE_TIMEOUT);
  std::future_status status =
      future.wait_for(CommConstants::DEFAULT_RESPONSE_TIMEOUT);

  if (status == std::future_status::timeout) {
    ResponseMapKey key = {server_id, CommConstants::COMM_FC_REG_READ};
    {
      std::lock_guard<std::mutex> lock(response_map_mutex_);
      pending_responses_.erase(key);
    }

    throw TimeoutError("Timeout waiting for ReadRegisters response from ID=" +
                       std::to_string(server_id));
  } else if (status == std::future_status::deferred) {
    throw CommunicationError("ReadRegisters future was deferred.");
  }

  ReceivedPacket response = future.get();

  if (response.errors != CommConstants::COMM_ERRORS_NONE) {
    // ** UPDATED: Include error code in exception message **
    throw ProtocolError("Device reported error during ReadRegisters (ID=" +
                            std::to_string(server_id) +
                            ", EC=" + std::to_string(response.errors) + ")",
                        response.errors);
  }
  if (response.crash_flag) {
    std::cerr << "Warning: Crash flag set in ReadRegisters response from ID="
              << static_cast<int>(server_id) << std::endl;
  }
  return response.data;
}

// ** Updated WriteRegisters **
bool BLDCControllerClient::WriteRegisters(uint8_t server_id,
                                          uint16_t start_addr,
                                          uint8_t register_count,
                                          const ByteVector &data) {
  // Caller is responsible for ensuring data size matches register_count *
  // size_of_register

  ByteVector args;
  try {
    ByteVector addr_bytes = PackU16(start_addr);
    // ** UPDATED: Use passed register_count **
    ByteVector count_byte = PackU8(register_count);

    args.insert(args.end(), addr_bytes.begin(), addr_bytes.end());
    args.insert(args.end(), count_byte.begin(), count_byte.end());
    args.insert(args.end(), data.begin(), data.end());

  } catch (const std::exception &e) {
    std::cerr << "!!! Exception caught during args.insert in WriteRegisters !!!"
              << std::endl;
    std::cerr << "    Error: " << e.what() << std::endl;
    std::cerr << "    Input data size: " << data.size() << std::endl;
    throw; // Re-throw
  }

  ResponseFuture future =
      DoTransaction(server_id, CommConstants::COMM_FC_REG_WRITE, args,
                    CommConstants::DEFAULT_RESPONSE_TIMEOUT);
  std::future_status status =
      future.wait_for(CommConstants::DEFAULT_RESPONSE_TIMEOUT);

  if (status == std::future_status::timeout) {
    ResponseMapKey key = {server_id, CommConstants::COMM_FC_REG_WRITE};
    {
      std::lock_guard<std::mutex> lock(response_map_mutex_);
      pending_responses_.erase(key);
    }
    throw TimeoutError("Timeout waiting for WriteRegisters response from ID=" +
                       std::to_string(server_id));
  } else if (status == std::future_status::deferred) {
    throw CommunicationError("WriteRegisters future was deferred.");
  }

  ReceivedPacket response = future.get();

  if (response.errors != CommConstants::COMM_ERRORS_NONE) {
    // ** UPDATED: Include error code in exception message **
    throw ProtocolError("Device reported error during WriteRegisters (ID=" +
                            std::to_string(server_id) +
                            ", EC=" + std::to_string(response.errors) + ")",
                        response.errors);
  }
  if (response.crash_flag) {
    std::cerr << "Warning: Crash flag set in WriteRegisters response from ID="
              << static_cast<int>(server_id) << std::endl;
  }
  return true;
}

// ** Updated ReadWriteRegisters **
ByteVector BLDCControllerClient::ReadWriteRegisters(
    uint8_t server_id, uint16_t read_start_addr, uint8_t read_count,
    uint16_t write_start_addr, uint8_t write_register_count,
    const ByteVector &write_data) {
  // Caller is responsible for ensuring write_data size matches
  // write_register_count * size_of_register

  ByteVector args;
  args.insert(args.end(), PackU16(read_start_addr).begin(),
              PackU16(read_start_addr).end());
  args.insert(args.end(), PackU8(read_count).begin(),
              PackU8(read_count).end()); // Read register count
  args.insert(args.end(), PackU16(write_start_addr).begin(),
              PackU16(write_start_addr).end());
  // ** UPDATED: Use passed write_register_count **
  args.insert(args.end(), PackU8(write_register_count).begin(),
              PackU8(write_register_count).end());
  args.insert(args.end(), write_data.begin(), write_data.end());

  ResponseFuture future =
      DoTransaction(server_id, CommConstants::COMM_FC_REG_READ_WRITE, args,
                    CommConstants::DEFAULT_RESPONSE_TIMEOUT);
  std::future_status status =
      future.wait_for(CommConstants::DEFAULT_RESPONSE_TIMEOUT);

  if (status == std::future_status::timeout) {
    ResponseMapKey key = {server_id, CommConstants::COMM_FC_REG_READ_WRITE};
    {
      std::lock_guard<std::mutex> lock(response_map_mutex_);
      pending_responses_.erase(key);
    }
    throw TimeoutError(
        "Timeout waiting for ReadWriteRegisters response from ID=" +
        std::to_string(server_id));
  } else if (status == std::future_status::deferred) {
    throw CommunicationError("ReadWriteRegisters future was deferred.");
  }

  ReceivedPacket response = future.get();

  if (response.errors != CommConstants::COMM_ERRORS_NONE) {
    // ** UPDATED: Include error code in exception message **
    throw ProtocolError("Device reported error during ReadWriteRegisters (ID=" +
                            std::to_string(server_id) +
                            ", EC=" + std::to_string(response.errors) + ")",
                        response.errors);
  }
  if (response.crash_flag) {
    std::cerr
        << "Warning: Crash flag set in ReadWriteRegisters response from ID="
        << static_cast<int>(server_id) << std::endl;
  }
  return response.data;
}

bool BLDCControllerClient::ResetSystem(uint8_t server_id) {
  WriteRequest(server_id, CommConstants::COMM_FC_SYSTEM_RESET);
  return true;
}

void BLDCControllerClient::EnterBootloader(uint8_t server_id) {
  ResetSystem(server_id);
  std::this_thread::sleep_for(
      std::chrono::milliseconds(100)); // Allow time to reset
}

void BLDCControllerClient::LeaveBootloader(uint8_t server_id,
                                           uint32_t jump_addr) {
  std::cout << "Sending Jump command to ID: " << static_cast<int>(server_id)
            << std::endl;
  JumpToAddress(server_id, jump_addr);

  std::cout << "Clearing buffer after jump command for 200ms..." << std::endl;
  auto start_time = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start_time <
         std::chrono::milliseconds(200)) {
    ResetInputBuffer();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ResetInputBuffer();
  std::cout << "Buffer clearing finished." << std::endl;
}

bool BLDCControllerClient::JumpToAddress(uint8_t server_id,
                                         uint32_t jump_addr) {
  ByteVector args = PackU32(jump_addr);
  WriteRequest(server_id, CommConstants::COMM_FC_JUMP_TO_ADDR, args);
  return true; // Command is fire-and-forget
}

uint8_t
BLDCControllerClient::EnumerateBoard(uint8_t target_id,
                                     std::chrono::milliseconds timeout) {
  ByteVector args = PackU8(target_id);
  ResponseMapKey response_key = {0, CommConstants::COMM_FC_ENUMERATE};

  ResponsePromise promise;
  ResponseFuture future = promise.get_future();

  {
    std::lock_guard<std::mutex> lock(response_map_mutex_);
    auto [it, inserted] =
        pending_responses_.try_emplace(response_key, std::move(promise));
    if (!inserted) {
      throw ProtocolError("Enumeration already pending for target ID=" +
                          std::to_string(target_id));
    }
  }

  WriteRequest(0, CommConstants::COMM_FC_ENUMERATE, args); // Send TO ID 0

  std::future_status status = future.wait_for(timeout);

  if (status == std::future_status::timeout) {
    {
      std::lock_guard<std::mutex> lock(response_map_mutex_);
      pending_responses_.erase(response_key);
    }
    throw TimeoutError(
        "Timeout waiting for Enumerate response from target ID=" +
        std::to_string(target_id));
  } else if (status == std::future_status::deferred) {
    throw CommunicationError("EnumerateBoard future was deferred.");
  }

  ReceivedPacket response = future.get();

  if (response.errors != CommConstants::COMM_ERRORS_NONE) {
    // ** UPDATED: Include error code in exception message **
    throw ProtocolError("Device reported error during Enumerate (target ID=" +
                            std::to_string(target_id) +
                            ", EC=" + std::to_string(response.errors) + ")",
                        response.errors);
  }
  if (response.crash_flag) {
    std::cerr << "Warning: Crash flag set in Enumerate response from ID="
              << static_cast<int>(response.server_id) << std::endl;
  }

  if (response.data.empty()) {
    throw ProtocolError("Enumerate response from target ID=" +
                        std::to_string(target_id) + " has no data.");
  }
  uint8_t response_payload_id = UnpackU8(response.data, 0);
  if (response_payload_id != target_id) {
    throw ProtocolError("Enumerate response payload ID mismatch. Expected " +
                        std::to_string(target_id) + ", Got " +
                        std::to_string(response_payload_id));
  }
  if (response.server_id != target_id) {
    std::cerr << "Warning: Enumerate response header ID ("
              << static_cast<int>(response.server_id)
              << ") differs from target ID (" << static_cast<int>(target_id)
              << ")" << std::endl;
  }

  return response_payload_id;
}

bool BLDCControllerClient::ConfirmBoard(uint8_t board_id,
                                        std::chrono::milliseconds timeout) {
  ResponseFuture future =
      DoTransaction(board_id, CommConstants::COMM_FC_CONFIRM_ID, {}, timeout);
  std::future_status status = future.wait_for(timeout);

  if (status == std::future_status::timeout) {
    ResponseMapKey key = {board_id, CommConstants::COMM_FC_CONFIRM_ID};
    {
      std::lock_guard<std::mutex> lock(response_map_mutex_);
      pending_responses_.erase(key);
    }
    throw TimeoutError("Timeout waiting for Confirm response from ID=" +
                       std::to_string(board_id));
  } else if (status == std::future_status::deferred) {
    throw CommunicationError("ConfirmBoard future was deferred.");
  }

  ReceivedPacket response = future.get();

  if (response.errors != CommConstants::COMM_ERRORS_NONE) {
    // ** UPDATED: Include error code in exception message **
    throw ProtocolError(
        "Device reported error during Confirm (ID=" + std::to_string(board_id) +
            ", EC=" + std::to_string(response.errors) + ")",
        response.errors);
  }
  if (response.crash_flag) {
    std::cerr << "Warning: Crash flag set in Confirm response from ID="
              << static_cast<int>(board_id) << std::endl;
  }
  return true;
}

void BLDCControllerClient::ResetInputBuffer() {
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  incoming_data_buffer_.clear();
}
