#include "util/comms/client.h"

#include <chrono>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <numeric> // For std::accumulate
#include <stdexcept>
#include <system_error>
#include <thread>

// #define DEBUG

// --- CRC-16 Implementation (remain the same) ---
// ...
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
// --- Packing/Unpacking Helpers (remain the same) ---
// ...
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

// ... (Constructor remains the same) ...
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
// ... (Destructor remains the same) ...
BLDCControllerClient::~BLDCControllerClient() {
  std::cout << "Shutting down BLDCControllerClient..." << std::endl;
  if (!stop_threads_.exchange(true)) { // Ensure stop signal is sent only once

    boost::asio::post(io_context_, [this]() {
      this->work_guard_.reset();
      this->ClosePort();
    });

    // Ensure threads are signaled to stop *before* joining
    // (stop_threads_ is atomic, so this is safe)
    // No need for extra signaling here as work_guard_.reset() stops io_context

    if (processing_thread_.joinable()) {
      // No explicit signal needed, relies on stop_threads_
      processing_thread_.join();
      std::cout << "Processing thread joined." << std::endl;
    } else {
      std::cout << "Processing thread not joinable on shutdown." << std::endl;
    }

    if (io_thread_.joinable()) {
      // io_context_.stop(); // Alternative way to stop if work_guard isn't
      // enough
      io_thread_.join();
      std::cout << "IO thread joined." << std::endl;
    } else {
      std::cout << "IO thread not joinable on shutdown." << std::endl;
    }

    {
      std::lock_guard<std::mutex> lock(response_map_mutex_);
      for (auto &pair : pending_responses_) {
        try {
          pair.second.set_exception(std::make_exception_ptr(CommunicationError(
              "Client shutting down before response received.")));
        } catch (
            ...) { /* Ignore std::future_error if promise already satisfied */
        }
      }
      pending_responses_.clear();
    }
    std::cout << "BLDCControllerClient shutdown complete." << std::endl;
  } else {
    std::cout << "BLDCControllerClient already shutting down." << std::endl;
  }
}
// ... (ClosePort remains the same) ...
void BLDCControllerClient::ClosePort() {
  if (serial_port_.is_open()) {
    boost::system::error_code ec;
    // Cancel any pending asynchronous operations
    serial_port_.cancel(ec);
    if (ec) {
      // Log non-critical error
      // std::cerr << "Warning: Error cancelling serial port operations: " <<
      // ec.message() << std::endl;
    }
    // Close the serial port
    serial_port_.close(ec);
    if (ec) {
      std::cerr << "Error closing serial port: " << ec.message() << std::endl;
    } else {
      std::cout << "Serial port closed." << std::endl;
    }
  }
}
// ... (StartReceive remains the same) ...
void BLDCControllerClient::StartReceive() {
  if (stop_threads_ || !serial_port_.is_open())
    return;

  serial_port_.async_read_some(
      boost::asio::buffer(raw_read_buffer_),
      [this](const boost::system::error_code &error, size_t bytes_transferred) {
        this->HandleReceive(error, bytes_transferred);
      });
}
// ... (HandleReceive remains the same) ...
void BLDCControllerClient::HandleReceive(const boost::system::error_code &error,
                                         size_t bytes_transferred) {
  if (stop_threads_)
    return;

  if (!error) {
    {
      std::lock_guard<std::mutex> lock(buffer_mutex_);
      // Check if buffer has enough space? Boost circular buffer handles
      // overwrite.
      incoming_data_buffer_.insert(
          incoming_data_buffer_.end(), raw_read_buffer_.begin(),
          raw_read_buffer_.begin() + bytes_transferred);
    }
    // Notify processing thread? Could use condition variable if
    // ProcessIncomingData sleeps longer.
    StartReceive(); // Continue reading
  } else if (error == boost::asio::error::operation_aborted) {
    // Operation cancelled, likely during shutdown. Normal.
    std::cout << "Serial read operation aborted." << std::endl;
  } else if (error == boost::asio::error::eof) {
    std::cerr << "Serial port connection closed by peer (EOF)." << std::endl;
    ClosePort(); // Close our end
                 // Notify pending futures about the error
    {
      std::lock_guard<std::mutex> lock(response_map_mutex_);
      for (auto &pair : pending_responses_) {
        try {
          pair.second.set_exception(std::make_exception_ptr(
              CommunicationError("Serial port connection closed (EOF)")));
        } catch (...) { /* Ignore */
        }
      }
      pending_responses_.clear();
    }
  } else {
    // Other read error
    std::cerr << "Serial read error: " << error.message() << std::endl;
    ClosePort(); // Close port on error
    // Notify pending futures about the error
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
    // Consider attempting to reconnect or signaling a higher-level component.
  }
}
// ... (DoWrite remains the same) ...
void BLDCControllerClient::DoWrite(const ByteVector &data) {
  if (stop_threads_ || !serial_port_.is_open()) {
    // It might be better to throw an exception here if writes are critical
    // Or return a bool/error code
    std::cerr
        << "Warning: Write attempted while client stopping or port closed."
        << std::endl;
    // Consider throwing CommunicationError("Attempted write on closed/stopping
    // client");
    return;
  }
  // Copy data to ensure lifetime if the original vector goes out of scope
  // before the async operation completes.
  auto shared_data = std::make_shared<ByteVector>(data);

  boost::asio::post(io_context_, [this, shared_data]() {
    if (stop_threads_ || !serial_port_.is_open()) {
      // Log or handle the case where the state changed between post and
      // execution
      return;
    }
    boost::asio::async_write(
        serial_port_, boost::asio::buffer(*shared_data),
        [this, shared_data](const boost::system::error_code &error,
                            size_t bytes_transferred) {
          // shared_data is kept alive until handler completes
          this->HandleWrite(error, bytes_transferred);
        });
  });
}
// ... (HandleWrite remains the same) ...
void BLDCControllerClient::HandleWrite(const boost::system::error_code &error,
                                       size_t bytes_transferred) {
  // Note: bytes_transferred is useful for logging/debugging but async_write
  // guarantees all bytes are written or an error occurs.
  if (stop_threads_)
    return; // Ignore callbacks during shutdown

  if (error && error != boost::asio::error::operation_aborted) {
    std::cerr << "Serial write error: " << error.message() << std::endl;
    ClosePort(); // Close port on write error
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
      pending_responses_.clear(); // Clear map as subsequent writes will fail
    }
    // Consider attempting to reconnect or signaling a higher-level component.
  } else if (error == boost::asio::error::operation_aborted) {
    std::cout << "Serial write operation aborted." << std::endl;
  }
  // else: Write successful (or aborted during shutdown)
}

/*
 * | Sync Flag (`0xFF`) | Protocol Version (`0xFE`) | Flag | Packet Length | Sub
 * Len | | Board ID | Function | Errors | Payload | CRC |
 */
void BLDCControllerClient::ProcessIncomingData() {
  std::cout << "Processing thread started (Robust Parsing - "
               "Consume-on-Success/Error)."
            << std::endl;

  enum class State {
    SEEK_SYNC,
    CHECK_VERSION,
    READ_FLAGS,
    READ_LEN_L,
    READ_LEN_H,
    READ_SUB_LEN_L,
    READ_SUB_LEN_H,
    READ_BOARD_ID,
    READ_FUNCTION,
    READ_ERRORS_L,
    READ_ERRORS_H,
    READ_PAYLOAD,
    READ_CRC_L,
    READ_CRC_H
  };
  State current_state = State::SEEK_SYNC;
  uint16_t current_packet_len = 0; // Packet Length field from header
  uint16_t current_sub_len = 0;    // Sub Len field from body
  uint16_t received_crc = 0;
  uint16_t payload_bytes_to_read = 0;
  uint16_t parse_idx = 0;
  ByteVector current_packet_buffer; // Stores bytes of the packet being parsed
                                    // *in this attempt*

  while (!stop_threads_) {
    // --- Start Parsing Attempt ---
    try {
      // We hold the data buffer for the whole parse duration.
      std::lock_guard<std::mutex> lock(buffer_mutex_);

      // Re-check size in case it changed between unlock/lock (very unlikely but
      // safe)
      size_t current_buffer_size = incoming_data_buffer_.size();

      if (current_buffer_size <= parse_idx) {
        continue;
      }

#ifdef DEBUG
      std::cout << "Processing buffer (" << incoming_data_buffer_.size()
                << " bytes): ";
      for (uint8_t b : incoming_data_buffer_) {
        std::cout << std::hex << std::setw(2) << std::setfill('0')
                  << static_cast<int>(b) << " ";
      }
      std::cout << std::dec << std::endl;
#endif

      uint8_t next_byte = incoming_data_buffer_[parse_idx];

#ifdef DEBUG
      // Optional: Print state changes or byte being processed
      std::cout << "  Idx=" << parse_idx << ", Byte=0x" << std::hex
                << static_cast<int>(next_byte) << std::dec
                << ", State=" << static_cast<int>(current_state) << std::endl;
#endif

      // If the state is not updated, that means we had an error and should
      // restart.
      State next_state = current_state;
      // --- State Machine ---
      switch (current_state) {
      case State::SEEK_SYNC:
        if (next_byte == CommConstants::START_BYTE) {
          next_state = State::CHECK_VERSION;
        }
        break;

      case State::CHECK_VERSION:
        if (next_byte == CommConstants::COMM_VERSION) {
          next_state = State::READ_FLAGS;
        }
        break;

      case State::READ_FLAGS:
        // We can assume this should have a one in the lower bit as traffic
        // should only come from a non-host.
        if (next_byte == 0x01 || next_byte == 0x03) {
          next_state = State::READ_LEN_L;
        }
        break;

      case State::READ_LEN_L:
        current_packet_len = next_byte;
        next_state = State::READ_LEN_H;
        break;

      case State::READ_LEN_H:
        current_packet_len |= (static_cast<uint16_t>(next_byte) << 8);
        if (current_packet_len > 256) {
          std::cerr << "Parser: Invalid response length " << current_packet_len
                    << "." << std::endl;
        } else {
          next_state = State::READ_SUB_LEN_L;
        }
        break;

      case State::READ_SUB_LEN_L:
        current_sub_len = next_byte;
        next_state = State::READ_SUB_LEN_H;
        break;

      case State::READ_SUB_LEN_H:
        current_sub_len |= (static_cast<uint16_t>(next_byte) << 8);
        // SubLen is always 2 less than the full length.
        if (current_sub_len != current_packet_len - 2) {
          std::cerr << "Parser: Invalid response sub-length " << current_sub_len
                    << ". Expected " << current_packet_len - 2 << "."
                    << std::endl;
        } else {
          next_state = State::READ_BOARD_ID;
        }
        break;

      case State::READ_BOARD_ID:
        next_state = State::READ_FUNCTION;
        break;

      case State::READ_FUNCTION:
        next_state = State::READ_ERRORS_L;
        break;

      case State::READ_ERRORS_L:
        next_state = State::READ_ERRORS_H;
        break;

      case State::READ_ERRORS_H:
        // Check if there's no payload.
        if (current_sub_len == 4) {
          next_state = State::READ_CRC_L;
        } else {
          next_state = State::READ_PAYLOAD;
          payload_bytes_to_read = current_sub_len - 4;
        }
        break;

      case State::READ_PAYLOAD:
        // Keep consuming bytes until we reach 0.
        if (!--payload_bytes_to_read) {
          next_state = State::READ_CRC_L;
        }
        break;

      case State::READ_CRC_L:
        received_crc = next_byte;
        next_state = State::READ_CRC_H;
        break;

      case State::READ_CRC_H:
        received_crc |= (static_cast<uint16_t>(next_byte) << 8);

        ByteVector data_for_crc(incoming_data_buffer_.begin() + 5,
                                incoming_data_buffer_.begin() + 5 +
                                    current_packet_len);

        // Print the incoming data buffer and then the segment for CRC
        for (auto b : incoming_data_buffer_) {
          std::cout << "0x" << std::hex << static_cast<int>(b) << " ";
        }
        std::cout << std::dec << std::endl;
        for (auto b : data_for_crc) {
          std::cout << "0x" << std::hex << static_cast<int>(b) << " ";
        }
        std::cout << std::dec << std::endl;

        uint16_t calculated_crc = ComputeCRC16(data_for_crc);

        if (calculated_crc == received_crc) {
          // CRC OK! Process the packet
          try {
            ReceivedPacket packet;
            // Parse fields using indices relative to current_packet_buffer or
            // data_for_crc
            packet.flags = incoming_data_buffer_[2];
            ByteVector packet_len(incoming_data_buffer_.begin() + 3,
                                  incoming_data_buffer_.begin() + 5);
            packet.packet_length =
                UnpackU16(packet_len, 0); // == current_packet_len
            // Use data_for_crc for body parts (indices relative to
            // data_for_crc start)
            packet.sub_len = UnpackU16(data_for_crc, 0); // == current_sub_len
            packet.server_id = UnpackU8(data_for_crc, 2);
            packet.function_code = UnpackU8(data_for_crc, 3);
            packet.errors = UnpackU16(data_for_crc, 4);
            packet.crash_flag =
                (packet.flags & CommConstants::COMM_FLAG_CRASH) != 0;

            // Extract payload (starts after SubLen(2)+ID(1)+Func(1)+Errors(2)
            // = 6 bytes in data_for_crc)
            packet.data.assign(data_for_crc.begin() + 6, data_for_crc.end());

            // --- Match response to promise ---
            ResponseMapKey key = {packet.server_id, packet.function_code};
            // Special case for Enumerate: Response comes FROM target_id but
            // key uses ID 0
            if (packet.function_code == CommConstants::COMM_FC_ENUMERATE) {
              key.first = 0; // Match against the key used in EnumerateBoard
            }

            ResponsePromise promise;
            bool promise_found = false;
            { // Lock response map separately
              std::lock_guard<std::mutex> resp_lock(response_map_mutex_);
              auto it = pending_responses_.find(key);
              if (it != pending_responses_.end()) {
                promise = std::move(it->second); // Move promise out
                pending_responses_.erase(it);    // Erase entry from map
                promise_found = true;
              }
            } // Release response map lock

            if (promise_found) {
              try {
                promise.set_value(std::move(packet));
              } catch (const std::future_error &fe) {
                // Not critical: Future already gone (e.g., timeout)
                std::cerr << "Parser: Future error setting promise value "
                             "(likely already timed out): "
                          << fe.what() << std::endl;
              }
            } else {
              // Unsolicited packet or response after timeout/client shutdown
              std::cout << "Parser: Received packet with no matching pending "
                           "request. ID="
                        << static_cast<int>(packet.server_id)
                        << ", FC=" << static_cast<int>(packet.function_code)
                        << ", Err=" << packet.errors << std::endl;
            }
            next_state = State::SEEK_SYNC;
          } catch (const std::out_of_range &oor) {
            std::cerr << "Parser: Error parsing packet content (out_of_range): "
                      << oor.what() << ". Packet ignored." << std::endl;
          } catch (const std::exception &e) {
            std::cerr << "Parser: Error processing valid packet content: "
                      << e.what() << ". Packet ignored." << std::endl;
          }
        } else {
          // CRC Mismatch
          std::cerr << "Parser: CRC mismatch. Expected 0x" << std::hex
                    << calculated_crc << ", Got 0x" << received_crc
                    << ". Discarding packet attempt." << std::dec << std::endl;
#ifdef DEBUG
          // Log the failed packet bytes for debugging
          std::cerr << "Failed packet bytes (" << current_packet_buffer.size()
                    << "): ";
          for (uint8_t b : current_packet_buffer) {
            std::cerr << std::hex << static_cast<int>(b) << " ";
          }
          std::cerr << std::dec << std::endl;
#endif
        }
        break; // End case READ_CRC_H

      } // End switch

      // Move to the next byte in the incoming buffer for this attempt
      parse_idx++;

      // --- Modify Buffer Based on Outcome ---
      if (current_state == next_state && payload_bytes_to_read == 0) {
        // This means we did not make progress. Pop a byte and try again.
        incoming_data_buffer_.pop_front();
        parse_idx = 0;
        next_state = State::SEEK_SYNC;
        if (current_state != State::SEEK_SYNC &&
            current_state != State::CHECK_VERSION) {
          // Print the hex value of the byte we're dropping.
          std::cerr << "Parser: Got stuck in state " << (int)current_state
                    << ". Popping byte (" << std::hex << std::setw(2)
                    << std::setfill('0') << next_byte << ") and trying again."
                    << std::dec << std::endl;
        }
      } else if (current_state == State::READ_CRC_H &&
                 next_state == State::SEEK_SYNC) {
        // We have consumed an entire packet! Pop the number of bytes used.
        // header (5) + packet length + CRC (2)
        uint16_t packet_size = current_packet_len + 7;
#ifdef DEBUG
        std::cout << "Parser: Parsed packet of length " << packet_size
                  << " bytes popping from buffer of size "
                  << incoming_data_buffer_.size() << std::endl;
#endif
        for (size_t i = 0; i < packet_size; ++i) {
          incoming_data_buffer_.pop_front();
        }
        parse_idx = 0;
        payload_bytes_to_read = 0;
      }
      current_state = next_state;
    } catch (...) {
      // Catch unknown exceptions during parsing attempt
      std::cerr << "Unknown exception during packet parsing attempt."
                << std::endl;
      // Consume one byte
      if (!incoming_data_buffer_.empty()) {
        incoming_data_buffer_.pop_front();
        std::cerr << "  Consumed 1 byte after unknown exception." << std::endl;
      }
    }
  } // End outer while (!stop_threads_)
  std::cout << "Processing thread finished." << std::endl;
}

// --- Packet Building Helper ---
ByteVector
BLDCControllerClient::BuildPacket(const std::vector<SubMessage> &sub_messages) {
  ByteVector packet_data;
  ByteVector all_sub_message_bytes;

  if (sub_messages.empty()) {
    // Or throw? Sending an empty packet seems wrong.
    std::cerr << "Warning: Attempting to build packet with no sub-messages."
              << std::endl;
    // Maybe send a NOP? For now, return empty.
    return {};
  }

  // 1. Construct all sub-message byte sequences
  for (const auto &sub_msg : sub_messages) {
    // Sub Message Format: Length (2), Board ID (1), Function (1), Payload (n)
    size_t payload_len = sub_msg.data.size();
    uint16_t sub_message_len = 1 + 1 + payload_len; // ID + Func + Payload

    ByteVector sub_len_bytes = PackU16(sub_message_len);
    all_sub_message_bytes.insert(all_sub_message_bytes.end(),
                                 sub_len_bytes.begin(), sub_len_bytes.end());
    all_sub_message_bytes.push_back(sub_msg.server_id);
    all_sub_message_bytes.push_back(sub_msg.func_code);
    all_sub_message_bytes.insert(all_sub_message_bytes.end(),
                                 sub_msg.data.begin(), sub_msg.data.end());
  }

  // 2. Calculate total packet length (length of all sub-messages)
  // NOTE: Protocol doc says Packet Length = length of Sub-Messages + CRC.
  // This seems wrong. Let's assume Packet Length = length of Sub-Messages only.
  // The receiver logic seems to interpret Packet Length differently anyway.
  // Let's follow the *request* format description: Packet Length = length of
  // Sub-Messages field.
  uint16_t total_sub_messages_len =
      static_cast<uint16_t>(all_sub_message_bytes.size());
  if (all_sub_message_bytes.size() > std::numeric_limits<uint16_t>::max()) {
    throw std::length_error(
        "Total size of sub-messages exceeds maximum packet length.");
  }

  // 3. Construct the part of the packet used for CRC calculation
  ByteVector data_for_crc;
  data_for_crc.push_back(CommConstants::START_BYTE);
  data_for_crc.push_back(CommConstants::COMM_VERSION);
  data_for_crc.push_back(
      CommConstants::COMM_FLAG_SEND); // Assuming always sending from client
  ByteVector packet_len_bytes = PackU16(total_sub_messages_len);
  data_for_crc.insert(data_for_crc.end(), packet_len_bytes.begin(),
                      packet_len_bytes.end());
  data_for_crc.insert(data_for_crc.end(), all_sub_message_bytes.begin(),
                      all_sub_message_bytes.end());

  // 4. Calculate CRC
  // Protocol: "CRC is computed over the entire packet excluding the CRC
  // itself."
  uint16_t crc = ComputeCRC16(all_sub_message_bytes);
  ByteVector crc_bytes = PackU16(crc);

  // 5. Assemble final packet
  packet_data = data_for_crc; // Start with the data used for CRC
  packet_data.insert(packet_data.end(), crc_bytes.begin(),
                     crc_bytes.end()); // Append CRC

#ifdef DEBUG
  // Print out the packet for debug
  std::cout << "Sending packet: ";
  for (uint8_t b : packet_data) {
    std::cout << std::hex << static_cast<int>(b) << " ";
  }
  std::cout << std::dec << std::endl;
#endif
  return packet_data;
}

// --- Public API Method Implementations ---

// ** NEW METHOD **
void BLDCControllerClient::WriteMultipleRequests(
    const std::vector<SubMessage> &sub_messages) {
  if (sub_messages.empty()) {
    std::cerr << "WriteMultipleRequests called with empty message list."
              << std::endl;
    return; // Or throw?
  }
  ByteVector packet = BuildPacket(sub_messages);
  if (!packet.empty()) {
    DoWrite(packet);
  }
}

// ** Keep original WriteRequest signature for backward compatibility? **
// ** Or remove it and force users to use DoTransaction or
// WriteMultipleRequests? **
// ** Let's keep it for now, acting as a single-message write **
void BLDCControllerClient::WriteRequest(uint8_t server_id, uint8_t func_code,
                                        const ByteVector &data) {
  WriteMultipleRequests({{server_id, func_code, data}});
}

// ** MODIFIED DoTransaction **
ResponseFuture
BLDCControllerClient::DoTransaction(uint8_t server_id, uint8_t func_code,
                                    const ByteVector &data,
                                    std::chrono::milliseconds timeout) {
  // DoTransaction is designed for a single logical request expecting a single
  // response. We map this to a packet with one sub-message.
  SubMessage single_message = {server_id, func_code, data};
  ResponseMapKey key = {server_id,
                        func_code}; // Key based on the single request

  // Special case for Enumerate: Request goes TO ID 0, but response comes FROM
  // target_id. The key must match what the parser will look for.
  if (func_code == CommConstants::COMM_FC_ENUMERATE) {
    key.first = 0; // Expect response with server_id=0 in the map key
  }

  ResponsePromise promise;
  ResponseFuture future = promise.get_future();

  {
    std::lock_guard<std::mutex> lock(response_map_mutex_);
    // Use try_emplace to avoid overwriting if key exists (though it shouldn't
    // ideally)
    auto [it, inserted] =
        pending_responses_.try_emplace(key, std::move(promise));
    if (!inserted) {
      // Clean up the promise we didn't insert
      // promise is moved-from, but let's be explicit if needed.
      // If a request for the same ID/FC is already pending, this is likely an
      // error.
      throw ProtocolError(
          "Transaction already pending for ID=" + std::to_string(key.first) +
          ", FC=" + std::to_string(key.second));
    }
  }

  // Build and send the packet containing the single sub-message
  try {
    ByteVector packet = BuildPacket({single_message});
    if (!packet.empty()) {
      DoWrite(packet);
    } else {
      // Should not happen if BuildPacket handles empty sub_messages
      // Need to reject the promise if we don't send
      throw CommunicationError("Failed to build packet for transaction.");
    }
  } catch (...) {
    // If building or writing fails, remove the pending promise and rethrow
    {
      std::lock_guard<std::mutex> lock(response_map_mutex_);
      pending_responses_.erase(key);
    }
    // Rethrow the exception caught from BuildPacket or DoWrite
    throw;
  }

  return future; // Return the future associated with the promise
}

// ... (ReadRegisters remains the same, uses DoTransaction) ...
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

  // --- Timeout Handling ---
  std::future_status status =
      future.wait_for(CommConstants::DEFAULT_RESPONSE_TIMEOUT);

  if (status == std::future_status::timeout) {
    ResponseMapKey key = {server_id, CommConstants::COMM_FC_REG_READ};
    // Attempt to remove the timed-out promise
    {
      std::lock_guard<std::mutex> lock(response_map_mutex_);
      // Only erase if it's still there (it might have been fulfilled just after
      // timeout check)
      pending_responses_.erase(key);
    }
    throw TimeoutError("Timeout waiting for ReadRegisters response from ID=" +
                       std::to_string(server_id));
  } else if (status == std::future_status::deferred) {
    // Should not happen with std::promise unless async was used differently
    throw CommunicationError("ReadRegisters future was deferred.");
  }

  // --- Process Response ---
  ReceivedPacket response =
      future.get(); // Can throw if promise holds exception

  if (response.errors != CommConstants::COMM_ERRORS_NONE) {
    throw ProtocolError(
        "Device reported error during ReadRegisters (ID=" +
            std::to_string(server_id) + ", FC=" +
            std::to_string(response.function_code) + // Use FC from response
            ", EC=" + std::to_string(response.errors) + ")",
        response.errors);
  }
  if (response.crash_flag) {
    std::cerr << "Warning: Crash flag set in ReadRegisters response from ID="
              << static_cast<int>(server_id) << std::endl;
    // Decide if this should be an error or just a warning
  }
  // TODO: Add check: response.function_code == CommConstants::COMM_FC_REG_READ?

  return response.data;
}

// ... (WriteRegisters remains the same, uses DoTransaction) ...
bool BLDCControllerClient::WriteRegisters(uint8_t server_id,
                                          uint16_t start_addr,
                                          uint8_t register_count,
                                          const ByteVector &data) {

  ByteVector args;
  // Basic validation: Does data size make sense for register count?
  // This assumes registers are uniform size, which might not be true.
  // A better approach involves a register map defining sizes.
  // if (data.size() % register_count != 0) { // Example check if all regs were
  // same size
  //    throw std::invalid_argument("WriteRegisters data size inconsistent with
  //    register count");
  // }

  try {
    ByteVector addr_bytes = PackU16(start_addr);
    ByteVector count_byte = PackU8(register_count);

    args.reserve(addr_bytes.size() + count_byte.size() +
                 data.size()); // Pre-allocate
    args.insert(args.end(), addr_bytes.begin(), addr_bytes.end());
    args.insert(args.end(), count_byte.begin(), count_byte.end());
    args.insert(args.end(), data.begin(), data.end());

  } catch (const std::exception &e) {
    // This catch block seems unlikely to be hit by std::vector::insert unless
    // allocation fails
    std::cerr << "!!! Exception caught during args assembly in WriteRegisters: "
              << e.what() << std::endl;
    throw; // Re-throw
  }

  ResponseFuture future =
      DoTransaction(server_id, CommConstants::COMM_FC_REG_WRITE, args,
                    CommConstants::DEFAULT_RESPONSE_TIMEOUT);

  // --- Timeout Handling ---
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

  // --- Process Response ---
  ReceivedPacket response = future.get();

  if (response.errors != CommConstants::COMM_ERRORS_NONE) {
    throw ProtocolError("Device reported error during WriteRegisters (ID=" +
                            std::to_string(server_id) +
                            ", FC=" + std::to_string(response.function_code) +
                            ", EC=" + std::to_string(response.errors) + ")",
                        response.errors);
  }
  if (response.crash_flag) {
    std::cerr << "Warning: Crash flag set in WriteRegisters response from ID="
              << static_cast<int>(server_id) << std::endl;
  }
  // TODO: Add check: response.function_code ==
  // CommConstants::COMM_FC_REG_WRITE?
  // TODO: Check response.data size? Should be empty for this command.

  return true; // Indicate success
}

// ... (ReadWriteRegisters remains the same, uses DoTransaction) ...
ByteVector BLDCControllerClient::ReadWriteRegisters(
    uint8_t server_id, uint16_t read_start_addr, uint8_t read_count,
    uint16_t write_start_addr, uint8_t write_register_count,
    const ByteVector &write_data) {

  ByteVector args;
  // Basic validation similar to WriteRegisters could be added for write_data
  // size

  try {
    args.reserve(2 + 1 + 2 + 1 + write_data.size()); // Pre-allocate
    args.insert(args.end(), PackU16(read_start_addr).begin(),
                PackU16(read_start_addr).end());
    args.insert(args.end(), PackU8(read_count).begin(),
                PackU8(read_count).end());
    args.insert(args.end(), PackU16(write_start_addr).begin(),
                PackU16(write_start_addr).end());
    args.insert(args.end(), PackU8(write_register_count).begin(),
                PackU8(write_register_count).end());
    args.insert(args.end(), write_data.begin(), write_data.end());
  } catch (const std::exception &e) {
    std::cerr
        << "!!! Exception caught during args assembly in ReadWriteRegisters: "
        << e.what() << std::endl;
    throw;
  }

  ResponseFuture future =
      DoTransaction(server_id, CommConstants::COMM_FC_REG_READ_WRITE, args,
                    CommConstants::DEFAULT_RESPONSE_TIMEOUT);

  // --- Timeout Handling ---
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

  // --- Process Response ---
  ReceivedPacket response = future.get();

  if (response.errors != CommConstants::COMM_ERRORS_NONE) {
    throw ProtocolError("Device reported error during ReadWriteRegisters (ID=" +
                            std::to_string(server_id) +
                            ", FC=" + std::to_string(response.function_code) +
                            ", EC=" + std::to_string(response.errors) + ")",
                        response.errors);
  }
  if (response.crash_flag) {
    std::cerr
        << "Warning: Crash flag set in ReadWriteRegisters response from ID="
        << static_cast<int>(server_id) << std::endl;
  }
  // TODO: Add check: response.function_code ==
  // CommConstants::COMM_FC_REG_READ_WRITE?
  // TODO: Check response.data size? Should correspond to read_count.

  return response.data;
}

// ** ResetSystem now uses WriteMultipleRequests (fire-and-forget) **
bool BLDCControllerClient::ResetSystem(uint8_t server_id) {
  WriteMultipleRequests({{server_id, CommConstants::COMM_FC_SYSTEM_RESET, {}}});
  return true; // Command sent, no response expected/handled here
}

// ... (EnterBootloader remains the same, uses ResetSystem) ...
void BLDCControllerClient::EnterBootloader(uint8_t server_id) {
  try {
    ResetSystem(server_id);
    // Delay to allow the device to enter bootloader mode
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // Optionally clear input buffer after delay?
    // ResetInputBuffer();
  } catch (const std::exception &e) {
    std::cerr << "Error sending reset command in EnterBootloader: " << e.what()
              << std::endl;
    // Re-throw or handle as appropriate
    throw;
  }
}

// ... (LeaveBootloader remains the same, uses JumpToAddress) ...
void BLDCControllerClient::LeaveBootloader(uint8_t server_id,
                                           uint32_t jump_addr) {
  std::cout << "Sending Jump command to ID: " << static_cast<int>(server_id)
            << " Addr: 0x" << std::hex << jump_addr << std::dec << std::endl;
  try {
    JumpToAddress(server_id, jump_addr);

    // Clear buffer after jump command. This is often needed as the device
    // might send garbage during/after the jump before the application starts
    // cleanly.
    std::cout << "Clearing input buffer after jump command for 200ms..."
              << std::endl;
    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time <
           std::chrono::milliseconds(200)) {
      ResetInputBuffer(); // Clear any received data
      std::this_thread::sleep_for(std::chrono::milliseconds(20)); // Short sleep
    }
    ResetInputBuffer(); // Final clear
    std::cout << "Buffer clearing finished." << std::endl;

  } catch (const std::exception &e) {
    std::cerr << "Error sending jump command in LeaveBootloader: " << e.what()
              << std::endl;
    // Re-throw or handle as appropriate
    throw;
  }
}

// ** JumpToAddress now uses WriteMultipleRequests (fire-and-forget) **
bool BLDCControllerClient::JumpToAddress(uint8_t server_id,
                                         uint32_t jump_addr) {
  ByteVector args = PackU32(jump_addr);
  WriteMultipleRequests(
      {{server_id, CommConstants::COMM_FC_JUMP_TO_ADDR, args}});
  return true; // Command sent
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

// ... (ConfirmBoard remains the same, uses DoTransaction) ...
bool BLDCControllerClient::ConfirmBoard(uint8_t board_id,
                                        std::chrono::milliseconds timeout) {
  ResponseFuture future =
      DoTransaction(board_id, CommConstants::COMM_FC_CONFIRM_ID, {}, timeout);

  // --- Timeout Handling ---
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

  // --- Process Response ---
  ReceivedPacket response = future.get();

  // Check response came from the expected board ID
  if (response.server_id != board_id) {
    throw ProtocolError("Confirm response header ID mismatch. Expected " +
                        std::to_string(board_id) + ", Got " +
                        std::to_string(response.server_id));
  }

  if (response.errors != CommConstants::COMM_ERRORS_NONE) {
    throw ProtocolError(
        "Device reported error during Confirm (ID=" + std::to_string(board_id) +
            ", FC=" + std::to_string(response.function_code) +
            ", EC=" + std::to_string(response.errors) + ")",
        response.errors);
  }
  if (response.crash_flag) {
    std::cerr << "Warning: Crash flag set in Confirm response from ID="
              << static_cast<int>(board_id) << std::endl;
  }
  if (response.function_code != CommConstants::COMM_FC_CONFIRM_ID) {
    throw ProtocolError("Confirm response function code mismatch. Expected " +
                        std::to_string(CommConstants::COMM_FC_CONFIRM_ID) +
                        ", Got " + std::to_string(response.function_code));
  }
  // Confirm response payload is typically empty. Check?
  // if (!response.data.empty()) { ... }

  return true;
}

// ... (ResetInputBuffer remains the same) ...
void BLDCControllerClient::ResetInputBuffer() {
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  incoming_data_buffer_.clear();
  // Also potentially reset parser state?
  // If called externally, the parser might be mid-packet.
  // However, the parser is designed to resync on errors or completion.
  // Clearing the buffer should effectively force a resync.
}
