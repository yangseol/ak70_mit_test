from __future__ import annotations

from calibration import apply_software_offset, load_motor_calibration
from mit_packet import analyze_feedback_candidate


# Offline analysis only. This script must never open a CAN bus or transmit frames.
CAPTURED_RAW_HEX = [
    "0A884F8007FED500",
    "0A90FE7FB7FED600",
    "0A8AB28017FFD700",
    "0A7D4E7FA7FED400",
    "0A855A7FE7FFD400",
    "FFFFFFFFFFFFFFFC",
]


def _fmt_uint(value: int, width: int) -> str:
    return f"0x{value:0{width}X}"


def _fmt_float(value: float) -> str:
    return f"{value:.3f}"


def _format_row(idx: int, raw_hex: str, result: dict | None, calibration: dict) -> list[str]:
    if result is None:
        return [
            str(idx),
            raw_hex,
            "-",
            "-",
            "-",
            "-",
            "-",
            "-",
            "-",
            "-",
            "-",
            "-",
            "[SKIP] command/echo frame",
        ]

    motor_id = result["motor_id"]
    raw_position_rad = result["candidate_position_rad"]
    joint_position_rad = apply_software_offset(raw_position_rad, motor_id, calibration)

    return [
        str(idx),
        raw_hex,
        _fmt_uint(motor_id, 2),
        _fmt_uint(result["p_uint"], 4),
        _fmt_float(raw_position_rad),
        _fmt_float(joint_position_rad),
        _fmt_uint(result["v_uint"], 3),
        _fmt_float(result["candidate_velocity_rads"]),
        _fmt_uint(result["t_uint"], 3),
        _fmt_float(result["candidate_effort_from_torque_limit"]),
        _fmt_uint(result["status_byte6"], 2),
        _fmt_uint(result["status_byte7"], 2),
        "candidate only",
    ]


def _print_table(rows: list[list[str]]) -> None:
    headers = [
        "idx",
        "raw_hex",
        "motor_id",
        "p_uint",
        "pos_rad",
        "joint_rad",
        "v_uint",
        "vel_rad_s",
        "t_uint",
        "effort",
        "status_byte6",
        "status_byte7",
        "note",
    ]
    widths = [len(header) for header in headers]
    for row in rows:
        widths = [max(width, len(cell)) for width, cell in zip(widths, row)]

    header_line = " | ".join(header.ljust(width) for header, width in zip(headers, widths))
    separator = "-+-".join("-" * width for width in widths)
    print(header_line)
    print(separator)
    for row in rows:
        print(" | ".join(cell.ljust(width) for cell, width in zip(row, widths)))


def main() -> None:
    calibration = load_motor_calibration()
    entries = []
    for idx, raw_hex in enumerate(CAPTURED_RAW_HEX, start=1):
        packet = bytes.fromhex(raw_hex)
        result = analyze_feedback_candidate(packet)
        entries.append((idx, raw_hex, result, _format_row(idx, raw_hex, result, calibration)))

    print("Captured order")
    _print_table([entry[3] for entry in entries])

    sorted_entries = sorted(
        (entry for entry in entries if entry[2] is not None),
        key=lambda entry: entry[2]["candidate_position_rad"],
    )

    print()
    print("Sorted by pos_rad")
    _print_table([entry[3] for entry in sorted_entries])


if __name__ == "__main__":
    main()
