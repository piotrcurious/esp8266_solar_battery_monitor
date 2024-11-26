import re

# Input source file containing pin definitions and device mappings
INPUT_FILE = "pins.h"

# Map of pin names to physical locations (left/right columns)
PIN_LAYOUT = [
    ("PIN_3V3", "PIN_RST"),
    ("PIN_D0", "PIN_A0"),
    ("PIN_D1", "PIN_G"),
    ("PIN_D2", "PIN_5V"),
    ("PIN_D3", "PIN_D8"),
    ("PIN_D4", "PIN_D7"),
    ("PIN_D5", "PIN_D6"),
    ("PIN_GND", "PIN_3V3B"),
]

# Regex patterns to extract pin mappings
PIN_REGEX = re.compile(r"#define\s+(PIN_[A-Z0-9]+)\s+(-?\d+|A0)")
DEVICE_REGEX = re.compile(r"#define\s+([A-Z0-9_]+)\s+(PIN_[A-Z0-9]+)")

def parse_mappings(file_path):
    """Parse pin and device mappings from the input file."""
    pin_map = {}
    device_map = {}

    with open(file_path, "r") as f:
        for line in f:
            # Match pin definitions
            pin_match = PIN_REGEX.match(line)
            if pin_match:
                pin_map[pin_match.group(1)] = pin_match.group(2)

            # Match device mappings
            device_match = DEVICE_REGEX.match(line)
            if device_match:
                device_map[device_match.group(2)] = device_match.group(1)

    return pin_map, device_map

def generate_ascii_art(pin_layout, pin_map, device_map):
    """Generate ASCII art based on pin and device mappings."""
    ascii_art = [
        "  +----------------------------------+",
        "  | Left Column         Right Column |",
        "  +----------------------------------+",
    ]

    for left_pin, right_pin in pin_layout:
        left_device = device_map.get(left_pin, "FREE").ljust(12)
        right_device = device_map.get(right_pin, "FREE").ljust(12)

        ascii_art.append(f"  | {left_device}   {right_device} |")

    ascii_art.append("  +----------------------------------+")
    return "\n".join(ascii_art)

def main():
    # Parse mappings from the input file
    pin_map, device_map = parse_mappings(INPUT_FILE)

    # Generate ASCII art
    ascii_art = generate_ascii_art(PIN_LAYOUT, pin_map, device_map)
    print(ascii_art)

if __name__ == "__main__":
    main()
