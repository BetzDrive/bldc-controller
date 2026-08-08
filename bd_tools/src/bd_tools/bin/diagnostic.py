#!/usr/bin/env python3
"""Peripheral I/O diagnostic: verifies encoder, ADC, I2C, and motor PWM."""

import argparse
import math
import struct
import time

import serial

from bd_tools import boards, comms


def parser_args():
    parser = argparse.ArgumentParser(
        description="Run peripheral I/O diagnostics on a board."
    )
    boards.addBoardArgs(parser)
    parser.add_argument(
        "--sweep_amplitude",
        type=float,
        default=0.15,
        help="Open-loop sweep duty amplitude (default 0.15)",
    )
    parser.add_argument(
        "--sweep_duration",
        type=float,
        default=2.0,
        help="Sweep duration in seconds (default 2.0)",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Enable verbose serial debugging",
    )
    parser.add_argument(
        "--monitor",
        action="store_true",
        help="Continuous state monitor: detects IWDG resets, comms timeouts, nFAULT glitches",
    )
    parser.add_argument(
        "--monitor_interval",
        type=float,
        default=0.2,
        help="Monitor poll interval in seconds (default 0.2)",
    )
    parser.set_defaults(
        baud_rate=comms.COMM_DEFAULT_BAUD_RATE,
    )
    return parser.parse_args()


def read_float(client, board_id, addr):
    data = client.readRegisters([board_id], [addr], [1])
    return struct.unpack("<f", data[0])[0]


def read_uint16(client, board_id, addr):
    data = client.readRegisters([board_id], [addr], [1])
    return struct.unpack("<H", data[0])[0]


def read_uint32(client, board_id, addr):
    data = client.readRegisters([board_id], [addr], [1])
    return struct.unpack("<I", data[0])[0]


def read_uint8(client, board_id, addr):
    data = client.readRegisters([board_id], [addr], [1])
    return struct.unpack("<B", data[0])[0]


def read_int32x3(client, board_id, addr):
    data = client.readRegisters([board_id], [addr], [3])
    return struct.unpack("<iii", data[0])


def set_phase_pwm(client, board_id, p0, p1, p2):
    """Set control mode to raw_phase_pwm and apply duty cycles."""
    client.writeRegisters(
        [board_id], [0x2000], [1], [struct.pack("<B", 1)]
    )
    client.writeRegisters(
        [board_id], [0x2003], [3], [struct.pack("<fff", p0, p1, p2)]
    )


def brake(client, board_id):
    """Set raw_phase_pwm mode with zero duty cycles (true brake)."""
    client.writeRegisters(
        [board_id], [0x2003], [3], [struct.pack("<fff", 0.0, 0.0, 0.0)]
    )
    client.writeRegisters(
        [board_id], [0x2000], [1], [struct.pack("<B", 1)]
    )


def test_system(client, board_id):
    """Test 1: System check - board responds and time is advancing."""
    t1 = read_float(client, board_id, 0x0006)
    time.sleep(0.05)
    t2 = read_float(client, board_id, 0x0006)

    if t2 <= t1:
        print(
            f"[FAIL] System: time not advancing "
            f"(t1={t1:.3f}s, t2={t2:.3f}s)"
        )
        return False

    print(f"[PASS] System: board_id={board_id}, time={t2:.3f}s")
    return True


def test_bus_voltage(client, board_id):
    """Test 2: Bus voltage ADC - reasonable and stable."""
    readings = []
    for _ in range(10):
        readings.append(read_float(client, board_id, 0x3004))
        time.sleep(0.01)

    avg = sum(readings) / len(readings)
    variance = sum((r - avg) ** 2 for r in readings) / len(readings)
    std = variance ** 0.5

    if avg < 5.0 or avg > 60.0:
        print(f"[FAIL] Bus Voltage: {avg:.1f}V out of range [5-60V]")
        return False

    print(f"[PASS] Bus Voltage: {avg:.1f}V (sigma={std:.2f})")
    return True


def test_current_sense(client, board_id):
    """Test 3: Current sensing - read d/q currents and vin individually."""
    brake(client, board_id)
    time.sleep(0.1)

    id_curr = read_float(client, board_id, 0x3002)
    iq_curr = read_float(client, board_id, 0x3003)
    supply_v = read_float(client, board_id, 0x3004)

    print(
        f"[INFO] Current Sense: id={id_curr:.3f}A, iq={iq_curr:.3f}A, "
        f"vin={supply_v:.1f}V"
    )

    if supply_v < 1.0:
        print(
            f"[WARN] Current Sense: vin={supply_v:.1f}V - "
            f"no bus power? ADC may not be validating."
        )

    magnitude = (id_curr ** 2 + iq_curr ** 2) ** 0.5
    if magnitude > 5.0:
        print(
            f"[FAIL] Current Sense: magnitude={magnitude:.2f}A too high "
            f"(motor should be off)"
        )
        return False

    print(f"[PASS] Current Sense: magnitude={magnitude:.2f}A (motor off)")
    return True


def test_encoder(client, board_id):
    """Test 4: Encoder SPI - valid readings and control loop running."""
    raw = read_uint16(client, board_id, 0x3010)
    pos = read_float(client, board_id, 0x3000)
    loops1 = read_uint32(client, board_id, 0x3040)
    time.sleep(0.05)
    loops2 = read_uint32(client, board_id, 0x3040)

    if raw > 16383:
        print(f"[FAIL] Encoder: raw={raw} out of 14-bit range")
        return False

    if loops2 <= loops1:
        print(
            f"[FAIL] Encoder: estimation_loops not incrementing "
            f"({loops1} -> {loops2})"
        )
        return False

    print(
        f"[PASS] Encoder: raw={raw}, pos={pos:.3f}rad, "
        f"loops={loops2} (incrementing)"
    )
    return True


def test_i2c_sensors(client, board_id):
    """Test 5: I2C sensors - temperature and accelerometer."""
    temp = read_float(client, board_id, 0x3005)
    ax, ay, az = read_int32x3(client, board_id, 0x3006)

    # Check temperature range
    temp_ok = -10.0 < temp < 80.0

    # Check accelerometer shows gravity (~1g on at least one axis)
    # Raw values depend on scale; just check they're not all zero
    accel_ok = (ax != 0 or ay != 0 or az != 0)

    if not temp_ok:
        print(f"[FAIL] I2C Sensors: temp={temp:.1f}C out of range [-10,80]")
        return False

    if not accel_ok:
        print(f"[FAIL] I2C Sensors: accel all zeros ({ax}, {ay}, {az})")
        return False

    print(
        f"[PASS] I2C Sensors: temp={temp:.1f}C, "
        f"accel=({ax}, {ay}, {az})"
    )
    return True


def test_motor_sweep(client, board_id, amplitude, duration):
    """Test 6: Open-loop motor sweep - encoder tracks rotation."""
    gate_active = read_uint8(client, board_id, 0x200B)
    if not gate_active:
        print("[FAIL] Motor Sweep: gate driver not active (DRV8312 nFAULT asserted?)")
        return False

    start_pos = read_float(client, board_id, 0x3000)
    start_raw = read_uint16(client, board_id, 0x3010)

    steps = 100
    dt = duration / steps
    sample_interval = steps // 5  # sample current ~5 times during sweep

    positions = [start_pos]
    peak_current = 0.0

    try:
        for i in range(steps + 1):
            theta = 2.0 * math.pi * i / steps
            p0 = 0.5 + amplitude * math.sin(theta)
            p1 = 0.5 + amplitude * math.sin(theta - 2.0 * math.pi / 3.0)
            p2 = 0.5 + amplitude * math.sin(theta - 4.0 * math.pi / 3.0)
            set_phase_pwm(client, board_id, p0, p1, p2)
            time.sleep(dt)

            pos = read_float(client, board_id, 0x3000)
            positions.append(pos)

            if i % sample_interval == 0:
                ia = read_float(client, board_id, 0x3060)
                ib = read_float(client, board_id, 0x3061)
                peak_current = max(peak_current, math.sqrt(ia**2 + ib**2))
    finally:
        brake(client, board_id)

    end_pos = positions[-1]
    end_raw = read_uint16(client, board_id, 0x3010)
    total_displacement = abs(end_pos - start_pos)

    if total_displacement < 0.01:
        if peak_current < 0.05:
            print(
                f"[FAIL] Motor Sweep: no current (peak={peak_current:.3f}A) - "
                f"gate driver fault or motor phases disconnected"
            )
        else:
            print(
                f"[FAIL] Motor Sweep: current flowing (peak={peak_current:.3f}A) "
                f"but motor stuck - check calibration or mechanical load"
            )
        return False

    print(
        f"[PASS] Motor Sweep: encoder tracked {total_displacement:.3f}rad "
        f"over 1 erev sweep "
        f"(raw: {start_raw}->{end_raw}, peak_current={peak_current:.3f}A)"
    )
    return True


def test_calibration(client, board_id):
    """Test 7: Calibration loaded - key parameters non-default."""
    erev_start = read_uint16(client, board_id, 0x1000)
    erevs_per_mrev = read_uint8(client, board_id, 0x1001)
    flip_phases = read_uint8(client, board_id, 0x1002)
    foc_kp_d = read_float(client, board_id, 0x1003)
    foc_ki_d = read_float(client, board_id, 0x1004)
    foc_kp_q = read_float(client, board_id, 0x1005)
    foc_ki_q = read_float(client, board_id, 0x1006)
    current_limit = read_float(client, board_id, 0x1010)
    motor_resistance = read_float(client, board_id, 0x1020)
    motor_inductance = read_float(client, board_id, 0x1021)
    motor_torque_const = read_float(client, board_id, 0x1022)
    ia_offset = read_float(client, board_id, 0x1050)
    ib_offset = read_float(client, board_id, 0x1051)
    ic_offset = read_float(client, board_id, 0x1052)

    print(
        f"[INFO] Calibration:"
        f"\n       erev_start={erev_start}, erevs_per_mrev={erevs_per_mrev}, "
        f"flip_phases={flip_phases}"
        f"\n       foc_kp_d={foc_kp_d:.3f}, foc_ki_d={foc_ki_d:.3f}, "
        f"foc_kp_q={foc_kp_q:.3f}, foc_ki_q={foc_ki_q:.3f}"
        f"\n       current_limit={current_limit:.1f}A, "
        f"motor_R={motor_resistance:.2f}ohm, "
        f"motor_L={motor_inductance:.6f}H, "
        f"motor_Kt={motor_torque_const:.4f}Nm/A"
        f"\n       ia_offset={ia_offset:.4f}, ib_offset={ib_offset:.4f}, "
        f"ic_offset={ic_offset:.4f}"
    )

    ok = True
    if motor_torque_const == 0.0:
        print(
            "[FAIL] Calibration: motor_torque_const=0 "
            "(torque mode will not work)"
        )
        ok = False

    if erev_start == 0 and erevs_per_mrev <= 1:
        print(
            "[WARN] Calibration: erev_start=0, erevs_per_mrev=1 "
            "(may not be calibrated)"
        )

    if ok:
        print("[PASS] Calibration: loaded from flash")
    return ok


CONTROL_MODE_NAMES = {
    0: "foc_current",
    1: "raw_phase_pwm",
    2: "torque",
    3: "velocity",
    4: "position",
    5: "pos_vel",
    6: "pos_ff",
    7: "pwm_drive",
}


def safe_mode_name(mode):
    if mode in CONTROL_MODE_NAMES:
        return CONTROL_MODE_NAMES[mode]
    return f"INVALID(0x{mode:02X})"


def monitor_board(client, board_id, interval):
    """Continuous monitor: flags IWDG resets, comms timeouts, nFAULT glitches."""
    print(f"\n=== Live Monitor (Board {board_id}) — Ctrl-C to stop ===")

    try:
        ctrl_timeout = read_uint16(client, board_id, 0x1030)
        print(f"[INFO] control_timeout in flash: {ctrl_timeout} ms "
              f"({'disabled' if ctrl_timeout == 0 else 'active'})")
    except Exception:
        print("[WARN] Could not read control_timeout")

    print(f"{'TIME':>8}  {'MODE':<18}  {'GATE':>4}  {'FAULT':>6}  FLAGS")
    print("-" * 72)

    prev_time = None
    prev_mode = None
    prev_gate_active = None
    prev_gate_fault = None
    iwdg_resets = 0
    timeouts = 0
    fault_transitions = 0
    bad_reads = 0

    while True:
        flags = []
        try:
            cur_time = read_float(client, board_id, 0x0006)
            mode = read_uint8(client, board_id, 0x2000)
            gate_active = read_uint8(client, board_id, 0x200B)
            gate_fault = read_uint8(client, board_id, 0x200C)
        except (comms.ProtocolError, comms.MalformedPacketError, struct.error) as e:
            print(f"  [COMMS ERROR] {type(e).__name__}: {e}")
            time.sleep(interval)
            continue

        # Sanity check: mode must be 0-7. Anything else = corrupted read.
        if mode > 7:
            bad_reads += 1
            print(
                f"{cur_time:>8.2f}  {'?':<18}  "
                f"{'?':>4}  {'?':>6}  *** CORRUPT READ "
                f"(mode=0x{mode:02X} gate=0x{gate_active:02X} "
                f"fault=0x{gate_fault:02X}) total={bad_reads}"
            )
            time.sleep(interval)
            continue

        # IWDG reset: time went backward
        if prev_time is not None and cur_time < prev_time - 0.5:
            iwdg_resets += 1
            flags.append(f"*** IWDG RESET #{iwdg_resets}")

        # Comms timeout: mode silently became raw_phase_pwm
        if (prev_mode is not None and prev_mode != 1 and mode == 1):
            timeouts += 1
            flags.append(f"*** COMMS TIMEOUT #{timeouts} (mode→raw_phase_pwm)")

        # Any other mode change
        elif prev_mode is not None and mode != prev_mode:
            flags.append(f"mode {safe_mode_name(prev_mode)}→{safe_mode_name(mode)}")

        # Gate transitions
        if prev_gate_active is not None:
            if gate_active and not prev_gate_active:
                flags.append("gate ENABLED")
            elif not gate_active and prev_gate_active:
                flags.append("gate DISABLED")

        # gate_fault is LATCHED in firmware (set on first nFAULT, never cleared).
        # Only flag NEW transitions to fault, not the steady-state latched value.
        if prev_gate_fault is not None and gate_fault and not prev_gate_fault:
            fault_transitions += 1
            flags.append(f"*** NEW nFAULT #{fault_transitions}")

        flag_str = "  ".join(flags) if flags else ""
        fault_label = "LATCH" if gate_fault else "ok"
        print(
            f"{cur_time:>8.2f}  {safe_mode_name(mode):<18}  "
            f"{'ON' if gate_active else 'OFF':>4}  {fault_label:>6}  {flag_str}"
        )

        prev_time = cur_time
        prev_mode = mode
        prev_gate_active = gate_active
        prev_gate_fault = gate_fault
        time.sleep(interval)


def flush_and_report(ser, label=""):
    """Flush serial input buffer, report any stale bytes."""
    waiting = ser.in_waiting
    if waiting > 0:
        stale = ser.read(waiting)
        print(
            f"  [DEBUG] {label}: flushed {len(stale)} stale bytes: "
            f"{stale[:32].hex()}"
            f"{'...' if len(stale) > 32 else ''}"
        )
    return waiting


def action(args):
    board_ids = [int(bid) for bid in args.board_ids.split(",")]

    ser = serial.Serial(
        port=args.serial, baudrate=args.baud_rate, timeout=0.04
    )
    client = comms.BLDCControllerClient(ser)

    if args.debug:
        comms.DEBUG = True

    print("Initializing boards...")
    boards.initBoards(client, board_ids)
    client.leaveBootloader(board_ids)
    client.resetInputBuffer()

    # Give firmware time to start up peripherals
    time.sleep(0.5)

    # Set watchdog timeout so motor shuts off if comms stop
    for bid in board_ids:
        client.setWatchdogTimeout([bid], [1000])

    if args.monitor:
        if len(board_ids) > 1:
            print("Monitor only supports one board at a time; using first board.")
        try:
            monitor_board(client, board_ids[0], args.monitor_interval)
        except KeyboardInterrupt:
            print("\nMonitor stopped.")
        ser.close()
        return

    for board_id in board_ids:
        print(f"\n=== Peripheral I/O Diagnostic (Board {board_id}) ===")

        passed = 0
        total = 7

        tests = [
            ("System", lambda: test_system(client, board_id)),
            ("Calibration", lambda: test_calibration(client, board_id)),
            ("Bus Voltage", lambda: test_bus_voltage(client, board_id)),
            (
                "Current Sense",
                lambda: test_current_sense(client, board_id),
            ),
            ("Encoder", lambda: test_encoder(client, board_id)),
            ("I2C Sensors", lambda: test_i2c_sensors(client, board_id)),
            (
                "Motor Sweep",
                lambda: test_motor_sweep(
                    client,
                    board_id,
                    args.sweep_amplitude,
                    args.sweep_duration,
                ),
            ),
        ]

        for name, test_fn in tests:
            flush_and_report(ser, f"before {name}")
            try:
                if test_fn():
                    passed += 1
            except (
                comms.ProtocolError,
                comms.MalformedPacketError,
                struct.error,
                Exception,
            ) as e:
                print(f"[FAIL] {name}: {type(e).__name__} - {e}")
                flush_and_report(ser, f"after {name} failure")

        print(f"=== {passed}/{total} PASSED ===")

    ser.close()


if __name__ == "__main__":
    action(parser_args())
