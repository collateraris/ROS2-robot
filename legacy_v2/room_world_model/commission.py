"""One deliberately triggered manual motor pulse for physical commissioning.

Dry run is the default. This local, line-of-sight tool bypasses camera/model
gates because its purpose is measuring the bounds those gates later require.
It never changes accepted/calibration flags and never repeats a pulse itself.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import queue
import threading
import time

from .actuator import MOTOR_PROTOCOL_VERSION, MotorProtocolError, PulseSerial, unmix_pwm, validate_motor_interval
from .protocol import ACTUATION_SCHEMA_VERSION
from .configuration import load_config


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="config/robot.yaml")
    parser.add_argument("--left-pwm", type=int, required=True)
    parser.add_argument("--right-pwm", type=int, required=True)
    parser.add_argument("--duration-ms", type=int, default=50)
    parser.add_argument("--execute", action="store_true", help="Open the real serial port; otherwise only print a plan")
    area = parser.add_mutually_exclusive_group()
    area.add_argument("--wheels-raised", action="store_true", help="Wheels cannot touch the floor; first commissioning stage")
    area.add_argument("--clear-test-lane", action="store_true", help="Manually supervised free test lane with motor power stop available")
    parser.add_argument("--record", type=Path, help="Optional JSON record of this one test")
    parser.add_argument("--manual-measurement", action="store_true", help="After motors are locked, ask for observed travel and stopping time")
    return parser


def validate_args(parser: argparse.ArgumentParser, args) -> None:
    if not 0 <= args.left_pwm <= 140 or not 0 <= args.right_pwm <= 140 or not (args.left_pwm or args.right_pwm):
        parser.error("PWM must be 0..140 and at least one wheel group must be nonzero")
    if not 1 <= args.duration_ms <= 100:
        parser.error("Duration must be 1..100 ms")
    if args.execute and not (args.wheels_raised or args.clear_test_lane):
        parser.error("Execution requires --wheels-raised or --clear-test-lane")


def wait_for(adapter: PulseSerial, predicate, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        adapter.poll()
        if adapter._faulted:
            raise MotorProtocolError(adapter.fault)
        if predicate():
            return
        time.sleep(0.01)
    raise MotorProtocolError("Commissioning step timed out")


def local_trigger(adapter: PulseSerial, timeout_s: float = 120.0) -> bool:
    """Keep zero-motion firmware heartbeats running while waiting for local input."""
    answers: queue.Queue = queue.Queue(maxsize=1)

    def read():
        try:
            answers.put(input("Один импульс. Введи ровно PULSE для запуска или Enter для отмены: "))
        except (EOFError, OSError):
            answers.put("")

    threading.Thread(target=read, daemon=True, name="commission-local-trigger").start()
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        adapter.poll()
        if adapter._faulted or not adapter.ready:
            raise MotorProtocolError(adapter.fault or "Drive no longer ready")
        try:
            return answers.get_nowait().strip() == "PULSE"
        except queue.Empty:
            time.sleep(0.01)
    return False


def optional_number(prompt: str) -> float | None:
    value = input(prompt).strip()
    if not value:
        return None
    result = float(value.replace(",", "."))
    if not math.isfinite(result) or result < 0:
        raise ValueError("External measurement must be a finite nonnegative number")
    return result


def main(argv=None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    validate_args(parser, args)
    config = load_config(args.config)
    robot = config["robot"]
    nominal_v, nominal_w = unmix_pwm(args.left_pwm, args.right_pwm, robot["track_width_m"],
                                    robot["wheel_command_scale_mps"], robot["pwm_limit"])
    report = {
        "kind": "manual_motor_commissioning", "execute": args.execute,
        "schema_version": ACTUATION_SCHEMA_VERSION, "motor_serial_protocol": MOTOR_PROTOCOL_VERSION,
        "area": "wheels_raised" if args.wheels_raised else "clear_test_lane" if args.clear_test_lane else "unspecified",
        "left_pwm": args.left_pwm, "right_pwm": args.right_pwm, "requested_duration_ms": args.duration_ms,
        "nominal_velocity_from_unverified_mapping": [nominal_v, nominal_w],
        "measured_distance_to_full_stop_mm": None,
        "measured_total_motion_time_s": None,
        "matched_mcu_completion": False, "normal_pulse_completion": False,
        "confirmed_mcu_elapsed_ms": None, "confirmed_stop_reason": None,
        "pwm_off_is_not_physical_rest": True,
        "events": [],
    }
    print(json.dumps({k: v for k, v in report.items() if k != "events"}, indent=2, ensure_ascii=False))
    print("Номинальная скорость выше вычислена по настройкам PWM; это НЕ измеренная скорость.")
    if not args.execute:
        print("DRY RUN: последовательный порт не открывался, команды двигателям не отправлялись.")
        return 0
    print("Ручной калибровочный тест: первый запуск только на подставке. Держи доступ к отключению питания двигателей.")
    print("Этот запуск выполнит максимум один импульс; для следующего нужен отдельный запуск и новый ввод PULSE.")
    adapter = None
    completed = False
    try:
        import serial

        port = serial.Serial(robot["serial_port"], robot["baud"], timeout=0, write_timeout=0, exclusive=True)
        # Opening Uno/Nano commonly resets the board. Wait at zero before any
        # protocol probe; no motor command is sent during this boot interval.
        time.sleep(2.0)
        port.reset_input_buffer()

        def event(value):
            report["events"].append(value)
            if value.get("type") == "motor_interval":
                validate_motor_interval(value)
                report.update(matched_mcu_completion=True,
                    normal_pulse_completion=bool(value["normal_completion"] and not value["duration_mismatch"]),
                    confirmed_mcu_elapsed_ms=value["mcu_elapsed_ms"],
                    confirmed_stop_reason=value["stop_reason"])
            print(json.dumps(value, ensure_ascii=False, allow_nan=False))

        adapter = PulseSerial(port, on_event=event,
                              clock_rate_bound_ppm=robot.get("mcu_clock_rate_bound_ppm", 20000.0))
        wait_for(adapter, lambda: adapter.ready, 5.0)
        if not local_trigger(adapter):
            print("Отменено: импульс не отправлялся.")
            return 0
        adapter.send_pulse(1, args.left_pwm, args.right_pwm, args.duration_ms / 1000.0)
        wait_for(adapter, lambda: adapter.pending is None, 2.0)
        completed = report["normal_pulse_completion"]
        if not completed:
            raise MotorProtocolError("No matched MCU completion; the interval is not valid calibration data")
    except KeyboardInterrupt:
        print("Прервано: отправляется остановка; повторного импульса не будет.")
        return 130
    except (OSError, ValueError, ImportError) as exc:
        print(f"Тест остановлен: {exc}")
        return 1
    finally:
        if adapter is not None:
            adapter.close()
        # If setup failed before the adapter was constructed, still close a
        # successfully opened port. No pulse could have been sent in this case.
        elif "port" in locals():
            port.close()
        # An aborted pulse can still have an exact, matched START/STOP duration.
        # Preserve that evidence even when wait_for() exited on a latched fault.
        report["normal_pulse_completion"] = completed
        if args.record:
            args.record.parent.mkdir(parents=True, exist_ok=True)
            args.record.write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")
    if completed and args.manual_measurement:
        print("Двигатели заблокированы. Внешние измерения вводятся отдельно от PWM/меток MCU.")
        try:
            report["measured_distance_to_full_stop_mm"] = optional_number("Полный путь до остановки, мм (Enter = не измерял): ")
            report["measured_total_motion_time_s"] = optional_number("Время от начала движения до остановки, с (Enter = не измерял): ")
        except (ValueError, EOFError, KeyboardInterrupt) as exc:
            print(f"Внешнее измерение не записано полностью: {exc}")
        print("Путь/время дают лишь среднюю скорость. Для speed_bound нужен максимум по внешнему видео с запасом.")
        if args.record:
            args.record.write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")
    print("Один импульс завершён. Флаги калибровки и разрешения автономности не изменялись.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
