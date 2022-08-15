#include "state.hpp"

#include <cstring>

#include "pb_decode.h"
#include "pb_encode.h"
#include "flash.h"
#include "helper.h"

namespace motor_driver {
namespace state {

Results results;

Calibration calibration;
motor_calibration_t calibration_pb;

Parameters parameters;

Recorder recorder;

volatile bool should_copy_results = false;

volatile bool should_copy_parameters = false;

void copyCalibrationToPb() {
  calibration_pb.erev_start = calibration.erev_start;
  calibration_pb.erevs_per_mrev = calibration.erevs_per_mrev;
  calibration_pb.flip_phases = calibration.flip_phases;
  calibration_pb.foc_kp_d = calibration.foc_kp_d;
  calibration_pb.foc_ki_d = calibration.foc_ki_d;
  calibration_pb.foc_kp_q = calibration.foc_kp_q;
  calibration_pb.foc_ki_q = calibration.foc_ki_q;
  calibration_pb.velocity_kp = calibration.velocity_kp;
  calibration_pb.velocity_kd = calibration.velocity_kd;
  calibration_pb.position_kp = calibration.position_kp;
  calibration_pb.position_kd = calibration.position_kd;
  calibration_pb.current_limit = calibration.current_limit;
  calibration_pb.torque_limit = calibration.torque_limit;
  calibration_pb.velocity_limit = calibration.velocity_limit;
  calibration_pb.position_lower_limit = calibration.position_lower_limit;
  calibration_pb.position_upper_limit = calibration.position_upper_limit;
  calibration_pb.motor_resistance = calibration.motor_resistance;
  calibration_pb.motor_inductance = calibration.motor_inductance;
  calibration_pb.motor_torque_const = calibration.motor_torque_const;
  calibration_pb.control_timeout = calibration.control_timeout;
  calibration_pb.hf_velocity_filter_param =
      calibration.hf_velocity_filter_param;
  calibration_pb.lf_velocity_filter_param =
      calibration.lf_velocity_filter_param;
  calibration_pb.position_offset = calibration.position_offset;
  calibration_pb.ia_offset = calibration.ia_offset;
  calibration_pb.ib_offset = calibration.ib_offset;
  calibration_pb.ic_offset = calibration.ic_offset;
  calibration_pb.enc_ang_corr_scale = calibration.enc_ang_corr_scale;
  calibration_pb.enc_ang_corr_offset = calibration.enc_ang_corr_offset;
  for (int i = 0; i < calibration_pb.enc_ang_corr_table_values.size; i++) {
    calibration_pb.enc_ang_corr_table_values.bytes[i] =
        calibration.enc_ang_corr_table_values[i];
  }
}

void initState() {
  calibration_pb.erev_start = 0;
  calibration_pb.erevs_per_mrev = 1;
  calibration_pb.flip_phases = false;
  calibration_pb.foc_kp_d = 0.5f;
  calibration_pb.foc_ki_d = 0.1f;
  calibration_pb.foc_kp_q = 1.0f;
  calibration_pb.foc_ki_q = 0.2f;
  calibration_pb.velocity_kp = 0.1f;
  calibration_pb.velocity_kd = 1e-3f;
  calibration_pb.position_kp = 5.0f;
  calibration_pb.position_kd = 0.0f;
  calibration_pb.current_limit = 2.0f;
  calibration_pb.torque_limit = 3.0f;
  calibration_pb.velocity_limit = 10.0f;
  calibration_pb.position_lower_limit = 0.0f;
  calibration_pb.position_upper_limit = 0.0f;
  calibration_pb.motor_resistance = 17.8f;
  calibration_pb.motor_inductance = 0.0f;
  calibration_pb.motor_torque_const = 0.0f;
  calibration_pb.control_timeout = 0;
  calibration_pb.hf_velocity_filter_param = 0.01f;
  calibration_pb.lf_velocity_filter_param = (1.0 - .9975);
  calibration_pb.position_offset = 0.0f;
  calibration_pb.ia_offset = 0.0f;
  calibration_pb.ib_offset = 0.0f;
  calibration_pb.ic_offset = 0.0f;
  calibration_pb.enc_ang_corr_scale = 0.0f;
  calibration_pb.enc_ang_corr_offset = 0.0f;
}

uint8_t calibration_buffer[1024];
struct calibration_header_t {
  uint16_t start_sequence;
  uint16_t length;
};

void storeCalibration() {
  // TODO(greg): Verify length of calibration is less than reserved space in
  // linker file.
  uint32_t addr = reinterpret_cast<uintptr_t>(consts::calibration_ptr);

  uint8_t offset = sizeof(calibration_header_t);
  pb_ostream_t stream = pb_ostream_from_buffer(
      calibration_buffer + offset, sizeof(calibration_buffer) - offset);
  pb_encode(&stream, motor_calibration_t_fields, &calibration_pb);

  calibration_header_t header;
  header.start_sequence = consts::calib_ss_pb;
  header.length = stream.bytes_written;

  memcpy(calibration_buffer, &header, sizeof(header));

  struct IWDG_Values save = pauseIWDG();
  flashErase(addr, stream.bytes_written + offset);
  resumeIWDG(save);

  flashWrite(addr, reinterpret_cast<char *>(calibration_buffer),
             stream.bytes_written + offset);
}

void loadCalibration() {
  uint32_t addr = reinterpret_cast<uintptr_t>(consts::calibration_ptr);
  calibration_header_t header;
  // Retry loading flash until successful load. This is critical to read.
  while (!(flashRead(addr, reinterpret_cast<char *>(&header), sizeof(header)) ==
           FLASH_RETURN_SUCCESS)) {
  }
  if (header.start_sequence == consts::calib_ss_struct) {
    // NOTE: The start sequence indicates the current calibration is stored in
    // the old format (struct). We read as normal.
    while (!(flashRead(addr, reinterpret_cast<char *>(&state::calibration),
                       sizeof(state::Calibration)) == FLASH_RETURN_SUCCESS)) {
    }
    copyCalibrationToPb();
  } else if (header.start_sequence == consts::calib_ss_pb) {
    while (!(flashRead(addr, reinterpret_cast<char *>(calibration_buffer),
                       header.length + sizeof(calibration_header_t)) ==
             FLASH_RETURN_SUCCESS)) {
    }
    pb_istream_t stream = pb_istream_from_buffer(
        calibration_buffer + sizeof(calibration_header_t), header.length);
    pb_decode(&stream, motor_calibration_t_fields, &calibration_pb);
  }
}

void clearCalibration() {
  // Copy default values into calibration.
  initState();
}

} // namespace state
} // namespace motor_driver
