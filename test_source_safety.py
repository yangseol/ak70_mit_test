from pathlib import Path


def test_no_forbidden_hardware_origin_patterns_in_runtime_sources():
    root = Path(__file__).resolve().parent
    forbidden = ["FFFFFFFFFFFFFFFE", "set_zero_position", "send_hardware_zero"]
    for path in root.glob("*.py"):
        if path.name.startswith("test_"):
            continue
        text = path.read_text(encoding="utf-8")
        for pattern in forbidden:
            assert pattern not in text, f"{pattern} found in {path.name}"

