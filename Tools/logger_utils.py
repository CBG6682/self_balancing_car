#!/usr/bin/env python3
"""
Shared Logging and Terminal Formatting Utility for Self-Balancing Robot Tools.
Features:
- Standard ANSI colors (zero external dependencies).
- Windows VT100 ANSI activation.
- Formatted Step Headers with bold cyan borders and clean spacing.
- Semantic log levels ([INFO], [SUCCESS], [PASS], [WARN], [FALLBACK], [ERROR], [FAIL]).
- Universal table formatter with auto-column width and alignment.
- Hardware serial port filter for USB-UART devices.
"""

import os
import sys

# Windows VT100 ANSI support activation
if sys.platform == "win32":
    try:
        import ctypes
        kernel32 = ctypes.windll.kernel32
        kernel32.SetConsoleMode(kernel32.GetStdHandle(-11), 7)
    except Exception:
        pass

# ANSI Escape Codes
RESET = "\033[0m"
BOLD = "\033[1m"
DIM = "\033[2m"

RED = "\033[31m"
GREEN = "\033[32m"
YELLOW = "\033[33m"
BLUE = "\033[34m"
MAGENTA = "\033[35m"
CYAN = "\033[36m"
WHITE = "\033[37m"

BOLD_RED = "\033[1;31m"
BOLD_GREEN = "\033[1;32m"
BOLD_YELLOW = "\033[1;33m"
BOLD_BLUE = "\033[1;34m"
BOLD_MAGENTA = "\033[1;35m"
BOLD_CYAN = "\033[1;36m"
BOLD_WHITE = "\033[1;37m"


def step_header(title: str, step_num: int = None, width: int = 70):
    """Prints a prominent bold cyan header box with clean spacing above and below."""
    full_title = title
    padding = width - 4
    if len(full_title) > padding:
        full_title = full_title[:padding - 3] + "..."
    centered_title = full_title.center(padding)

    print("\n" + BOLD_CYAN + "╔" + "═" * (width - 2) + "╗")
    print(f"║ {centered_title} ║")
    print("╚" + "═" * (width - 2) + "╝" + RESET + "\n")


def sub_header(title: str, width: int = 70):
    """Prints a clean section divider."""
    prefix = f"── {title} "
    fill_len = max(0, width - len(prefix))
    print("\n" + BOLD_CYAN + prefix + ("─" * fill_len) + RESET)


def info(msg: str):
    """Prints an info log line in blue/dim."""
    print(f"{BOLD_BLUE}[INFO]{RESET} {msg}")


def success(msg: str):
    """Prints a success log line in bold green."""
    print(f"{BOLD_GREEN}[SUCCESS]{RESET} {msg}")


def pass_msg(msg: str):
    """Prints a PASS log line in bold green."""
    print(f"{BOLD_GREEN}[PASS]{RESET} {msg}")


def cache_hit(msg: str):
    """Prints a CACHE HIT log line in bold green."""
    print(f"{BOLD_GREEN}[CACHE HIT]{RESET} {msg}")


def warn(msg: str):
    """Prints a warning log line in bold yellow."""
    print(f"{BOLD_YELLOW}[WARN]{RESET} {msg}")


def fallback(msg: str):
    """Prints a fallback log line in bold yellow."""
    print(f"{BOLD_YELLOW}[FALLBACK]{RESET} {msg}")


def error(msg: str):
    """Prints an error log line in bold red."""
    print(f"{BOLD_RED}[ERROR]{RESET} {msg}")


def fail_msg(msg: str):
    """Prints a FAIL log line in bold red."""
    print(f"{BOLD_RED}[FAIL]{RESET} {msg}")


def box_banner(msg: str, is_pass: bool = True, width: int = 70):
    """Prints a prominent outcome banner."""
    color = BOLD_GREEN if is_pass else BOLD_RED
    padding = width - 4
    centered = msg.center(padding)
    print("\n" + color + "═" * width)
    print(f"  {centered}")
    print("═" * width + RESET + "\n")


def print_table(headers: list, rows: list, col_align: list = None):
    """
    Prints a beautifully formatted Unicode box table.
    headers: list of column title strings
    rows: list of rows, where each row is a list of cell values
    col_align: list of 'left', 'right', or 'center' for each column
    """
    n_cols = len(headers)
    if n_cols == 0:
        return

    # Calculate column widths
    str_rows = [[str(cell) for cell in row] for row in rows]
    col_widths = [len(str(h)) for h in headers]
    for row in str_rows:
        for i in range(min(n_cols, len(row))):
            col_widths[i] = max(col_widths[i], len(row[i]))

    # Add 2 padding spaces
    col_widths = [w + 2 for w in col_widths]

    if not col_align:
        col_align = ["left"] * n_cols

    def format_cell(text, width, align):
        content_width = width - 2
        if align == "right":
            s = text.rjust(content_width)
        elif align == "center":
            s = text.center(content_width)
        else:
            s = text.ljust(content_width)
        return f" {s} "

    # Table borders
    top_border = "┌" + "┬".join("─" * w for w in col_widths) + "┐"
    mid_border = "├" + "┼".join("─" * w for w in col_widths) + "┤"
    bot_border = "└" + "┴".join("─" * w for w in col_widths) + "┘"

    # Header line
    hdr_cells = [format_cell(headers[i], col_widths[i], "center") for i in range(n_cols)]
    hdr_line = "│" + "│".join(hdr_cells) + "│"

    print(DIM + top_border + RESET)
    print(BOLD_WHITE + hdr_line + RESET)
    print(DIM + mid_border + RESET)

    for row in str_rows:
        row_cells = []
        for i in range(n_cols):
            val = row[i] if i < len(row) else ""
            align = col_align[i] if i < len(col_align) else "left"
            row_cells.append(format_cell(val, col_widths[i], align))
        print("│" + "│".join(row_cells) + "│")

    print(DIM + bot_border + RESET)


def filter_physical_ports(ports: list) -> list:
    """
    Filters out legacy dummy Linux serial ports (/dev/ttyS0..31 with 'n/a' description).
    Keeps true USB-UART adapters, ESP32-S3 DevKitC (CDC/UART), FTDI, CH340/CH343, CP2102, ST-Link, COM ports, /dev/ttyUSB*, /dev/ttyACM*.
    """
    physical = []
    for p in ports:
        dev = p.device
        desc = (p.description or "").lower()

        # Filter out legacy PC serial ports on Linux with no hardware attached
        if dev.startswith("/dev/ttyS") and ("n/a" in desc or p.vid is None):
            continue

        # Valid physical devices
        if any(dev.startswith(prefix) for prefix in ["/dev/ttyUSB", "/dev/ttyACM", "/dev/cu.usb", "/dev/tty.usb"]):
            physical.append(p)
        elif dev.startswith("COM") and "n/a" not in desc:
            physical.append(p)
        elif p.vid is not None:
            physical.append(p)

    return physical
