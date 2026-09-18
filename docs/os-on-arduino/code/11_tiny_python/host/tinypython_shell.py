#!/usr/bin/env python3
"""
TinyPython Shell - IPython-like interface for Arduino TinyPython

Usage:
    python tinypython_shell.py [--port /dev/cu.usbmodemXXXX] [--baud 115200]

Features:
    - Syntax highlighting (Python)
    - Tab completion for builtins and modules
    - Command history (up/down arrows)
    - Numbered In[n]/Out[n] prompts
    - _ for last result
    - ? for help (e.g., print?)
    - %magic commands (%who, %timeit, %mem, %reset, %upload)
    - Multi-line block editing (if/for/while/def)
    - ANSI color output
"""

import argparse
import sys
import time
import re

import serial
from prompt_toolkit import PromptSession
from prompt_toolkit.history import InMemoryHistory
from prompt_toolkit.lexers import PygmentsLexer
from prompt_toolkit.completion import Completer, Completion
from prompt_toolkit.formatted_text import HTML
from prompt_toolkit.styles import Style
from pygments.lexers.python import Python3Lexer


# ---- Constants ----

BUILTINS = [
    "print", "type", "len", "abs", "min", "max", "int", "str", "range",
    "pin_mode", "digital_write", "digital_read", "analog_read",
    "delay", "millis",
    "True", "False", "None",
    "if", "elif", "else", "while", "for", "in", "def", "return",
    "and", "or", "not", "import",
]

MODULES = {
    "led": ["blink", "on", "off"],
    "matrix": ["pixel", "show", "clear", "fill", "row"],
    "gpio": ["pin_mode", "write", "read", "analog"],
}

HELP_TEXTS = {
    "print": "print(*args) - Print values to serial output",
    "type": "type(x) - Return the type of x as a string",
    "len": "len(s) - Return the length of string s",
    "abs": "abs(n) - Return the absolute value of n",
    "min": "min(a, b) - Return the smaller of a and b",
    "max": "max(a, b) - Return the larger of a and b",
    "int": "int(x) - Convert x to integer",
    "str": "str(x) - Convert x to string",
    "range": "range(stop) or range(start, stop[, step]) - Generate sequence for for-loop",
    "pin_mode": "pin_mode(pin, mode) - Set pin mode (0=INPUT, 1=OUTPUT)",
    "digital_write": "digital_write(pin, val) - Write HIGH(1) or LOW(0) to pin",
    "digital_read": "digital_read(pin) - Read digital value from pin",
    "analog_read": "analog_read(pin) - Read analog value (0-1023) from pin",
    "delay": "delay(ms) - Wait for ms milliseconds",
    "millis": "millis() - Return milliseconds since boot",
    "led": "Module: led - LED_BUILTIN control\n  led.blink(n, ms)  - Blink n times\n  led.on()           - Turn on\n  led.off()          - Turn off",
    "matrix": "Module: matrix - 12x8 LED matrix\n  matrix.pixel(r,c,v) - Set pixel\n  matrix.show()       - Render buffer\n  matrix.clear()      - Clear all\n  matrix.fill(v)      - Fill all\n  matrix.row(r, bits) - Set row pattern",
    "gpio": "Module: gpio - GPIO control\n  gpio.pin_mode(p,m) - Set mode\n  gpio.write(p,v)    - Digital write\n  gpio.read(p)       - Digital read\n  gpio.analog(p)     - Analog read",
}

STYLE = Style.from_dict({
    "prompt": "#00aa00 bold",
    "prompt.dots": "#888888",
    "continuation": "#888888",
})

BLOCK_STARTERS = ("if ", "elif ", "else:", "while ", "for ", "def ")


# ---- Serial Communication ----

class TinyPythonConnection:
    """Manages serial communication with TinyPython on Arduino."""

    def __init__(self, port, baud=115200, timeout=1):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        self.ser.dtr = True
        time.sleep(0.1)

    def _has_prompt(self, buf):
        """Check if buffer ends with a TinyPython prompt."""
        return buf.endswith(">>> ") or buf.endswith("... ")

    def wait_for_prompt(self, timeout=5):
        """Read until we see >>> or ... prompt."""
        buf = ""
        start = time.time()
        while time.time() - start < timeout:
            if self.ser.in_waiting:
                chunk = self.ser.read(self.ser.in_waiting).decode("utf-8", errors="replace")
                buf += chunk
                if self._has_prompt(buf):
                    return buf
            time.sleep(0.01)
        return buf

    def send_line(self, line):
        """Send a line and read the response."""
        self.ser.write((line + "\r").encode("utf-8"))

        # Read response until next prompt
        buf = ""
        deadline = time.time() + 10
        idle_since = time.time()
        while time.time() < deadline:
            if self.ser.in_waiting:
                chunk = self.ser.read(self.ser.in_waiting).decode("utf-8", errors="replace")
                buf += chunk
                idle_since = time.time()
                if self._has_prompt(buf):
                    break
            else:
                # No data: short sleep, but bail after 0.5s of silence
                # (if we already have some data)
                if buf and time.time() - idle_since > 0.5:
                    break
                time.sleep(0.005)

        return buf

    def close(self):
        self.ser.close()


def parse_response(raw, line_sent):
    """Parse Arduino response, removing echo and prompt."""
    lines = raw.replace("\r\n", "\n").replace("\r", "\n").split("\n")

    # Remove echo of sent line
    output_lines = []
    skip_echo = True
    for line in lines:
        stripped = line.strip()
        if skip_echo and stripped == line_sent.strip():
            skip_echo = False
            continue
        # Remove prompt lines
        if stripped in (">>> ", "... ", ">>>", "..."):
            continue
        if stripped.startswith(">>> ") or stripped.startswith("... "):
            continue
        output_lines.append(line)

    # Clean up
    result = "\n".join(output_lines).strip()
    return result


# ---- Magic Commands ----

def handle_magic(cmd, conn, exec_count):
    """Handle %magic commands. Returns (output_string, handled)."""

    if cmd == "%who":
        # List variables by evaluating a trick
        # We can't directly list vars, but we can tell the user
        return ("\033[33mNote: %who is not yet supported on device.\n"
                "Use variable names directly to check if they exist.\033[0m", True)

    if cmd.startswith("%timeit "):
        expr = cmd[8:]
        conn.send_line("__t0 = millis()")
        for _ in range(100):
            conn.send_line(expr)
        raw = conn.send_line("print(millis() - __t0)")
        result = parse_response(raw, "print(millis() - __t0)")
        try:
            total_ms = int(result)
            per_call = total_ms / 100
            return (f"\033[36m100 loops, {total_ms} ms total, "
                    f"{per_call:.1f} ms per loop\033[0m", True)
        except ValueError:
            return (f"\033[31mTimeit error: {result}\033[0m", True)

    if cmd == "%reset":
        conn.ser.dtr = False
        time.sleep(0.1)
        conn.ser.dtr = True
        time.sleep(2)
        banner = conn.wait_for_prompt(timeout=5)
        return ("\033[33mBoard reset. Waiting for TinyPython...\033[0m\n" + banner, True)

    if cmd.startswith("%upload "):
        filename = cmd[8:].strip()
        try:
            with open(filename, "r") as f:
                script_lines = f.readlines()
        except FileNotFoundError:
            return (f"\033[31mFile not found: {filename}\033[0m", True)

        print(f"\033[36mUploading {filename} ({len(script_lines)} lines)...\033[0m")
        for line in script_lines:
            line = line.rstrip("\n")
            if line.strip() == "" or line.strip().startswith("#"):
                continue
            raw = conn.send_line(line)
            output = parse_response(raw, line)
            if output:
                print(output)
        return (f"\033[32mUpload complete.\033[0m", True)

    if cmd == "%modules":
        module_list = "\n".join(
            f"  \033[1m{name}\033[0m: {', '.join(funcs)}"
            for name, funcs in MODULES.items()
        )
        return (f"\033[36mAvailable modules:\033[0m\n{module_list}", True)

    if cmd == "%help":
        return ("\033[36mMagic commands:\033[0m\n"
                "  %who       - List variables (limited)\n"
                "  %timeit X  - Time expression X (100 iterations)\n"
                "  %reset     - Reset Arduino board\n"
                "  %upload F  - Upload and run Python file F\n"
                "  %modules   - List available modules\n"
                "  %help      - Show this help\n"
                "\n"
                "\033[36mSpecial syntax:\033[0m\n"
                "  name?      - Show help for name\n"
                "  _          - Last result value", True)

    return (f"\033[31mUnknown magic: {cmd}\033[0m", True)


# ---- Completer ----

class TinyPythonCompleter(Completer):
    """Context-aware completer for TinyPython.

    - After 'module.', complete with module functions
    - Otherwise, complete with builtins, keywords, and module names
    """

    def __init__(self):
        self.global_words = list(BUILTINS)
        for mod_name in MODULES:
            self.global_words.append(mod_name)

    def get_completions(self, document, complete_event):
        text = document.text_before_cursor
        # Find the current word being typed
        # Handle module.func pattern
        word = ""
        i = len(text) - 1
        while i >= 0 and (text[i].isalnum() or text[i] in "_."):
            word = text[i] + word
            i -= 1

        if not word:
            return

        # module.func completion
        if "." in word:
            parts = word.split(".", 1)
            mod_name = parts[0]
            partial = parts[1] if len(parts) > 1 else ""
            if mod_name in MODULES:
                for func in MODULES[mod_name]:
                    full = f"{mod_name}.{func}"
                    if func.startswith(partial):
                        yield Completion(
                            full,
                            start_position=-len(word),
                            display_meta="",
                        )
            return

        # Global completion
        for w in self.global_words:
            if w.startswith(word) and w != word:
                yield Completion(w, start_position=-len(word))


def build_completer():
    return TinyPythonCompleter()


# ---- Main REPL ----

def main():
    parser = argparse.ArgumentParser(description="TinyPython Shell")
    parser.add_argument("--port", "-p", default=None,
                        help="Serial port (auto-detect if not specified)")
    parser.add_argument("--baud", "-b", type=int, default=115200)
    args = parser.parse_args()

    # Auto-detect port
    port = args.port
    if port is None:
        import serial.tools.list_ports
        ports = [p.device for p in serial.tools.list_ports.comports()
                 if "usbmodem" in p.device or "ttyACM" in p.device or "ttyUSB" in p.device]
        if not ports:
            print("\033[31mNo Arduino found. Specify --port.\033[0m")
            sys.exit(1)
        port = ports[0]
        print(f"\033[33mAuto-detected port: {port}\033[0m")

    # Connect
    print(f"\033[36mConnecting to {port} at {args.baud} baud...\033[0m")
    try:
        conn = TinyPythonConnection(port, args.baud)
    except serial.SerialException as e:
        print(f"\033[31mConnection failed: {e}\033[0m")
        sys.exit(1)

    # Wait for TinyPython banner
    print("\033[33mWaiting for TinyPython (press reset if needed)...\033[0m")
    banner = conn.wait_for_prompt(timeout=10)
    if banner:
        # Print banner without the >>> prompt
        for line in banner.split("\n"):
            if line.strip() not in (">>>", ">>> "):
                print(line)

    print()
    print("\033[36m  IPython-like shell for TinyPython\033[0m")
    print("\033[36m  Type %help for magic commands, name? for help\033[0m")
    print()

    # Setup prompt_toolkit
    session = PromptSession(
        history=InMemoryHistory(),
        lexer=PygmentsLexer(Python3Lexer),
        completer=build_completer(),
        complete_style="READLINE_LIKE",
        complete_while_typing=False,
        style=STYLE,
    )

    exec_count = 1
    last_result = None
    in_block = False
    block_lines = []

    try:
        while True:
            # Prompt
            if in_block:
                prompt_text = [("class:prompt.dots", f"...[{exec_count}]: ")]
            else:
                prompt_text = [("class:prompt", f"In [{exec_count}]: ")]

            try:
                line = session.prompt(prompt_text)
            except KeyboardInterrupt:
                if in_block:
                    in_block = False
                    block_lines = []
                    print("\n\033[33mKeyboardInterrupt\033[0m")
                    continue
                print("\n\033[33mKeyboardInterrupt\033[0m")
                continue
            except EOFError:
                break

            # Handle empty line in block mode
            if in_block and line.strip() == "":
                # Send all block lines
                for bl in block_lines:
                    conn.send_line(bl)
                    time.sleep(0.02)
                raw = conn.send_line("")  # Empty line to end block
                output = parse_response(raw, "")

                # Clean output from intermediate prompts
                clean_lines = []
                for ol in output.split("\n"):
                    ol_stripped = ol.strip()
                    if ol_stripped.startswith("... "):
                        continue
                    if ol_stripped == "...":
                        continue
                    clean_lines.append(ol)
                output = "\n".join(clean_lines).strip()

                if output:
                    print(f"\033[31m{output}\033[0m" if "Error" in output
                          else output)
                in_block = False
                block_lines = []
                exec_count += 1
                continue

            if in_block:
                block_lines.append(line)
                continue

            # Skip empty lines
            if not line.strip():
                continue

            # Handle ? suffix (help)
            if line.strip().endswith("?") and not line.strip().endswith("??"):
                name = line.strip()[:-1]
                if name in HELP_TEXTS:
                    print(f"\033[36m{HELP_TEXTS[name]}\033[0m")
                else:
                    print(f"\033[33mNo help available for '{name}'\033[0m")
                continue

            # Handle % magic commands
            if line.strip().startswith("%"):
                output, handled = handle_magic(line.strip(), conn, exec_count)
                if handled:
                    print(output)
                    exec_count += 1
                continue

            # Check if this starts a block
            stripped = line.strip()
            if any(stripped.startswith(s) for s in BLOCK_STARTERS) and stripped.endswith(":"):
                in_block = True
                block_lines = [line]
                # Send the first line
                conn.send_line(line)
                time.sleep(0.05)
                # Read the ... prompt
                conn.wait_for_prompt(timeout=2)
                continue

            # Regular single-line command
            raw = conn.send_line(line)
            output = parse_response(raw, line)

            if output:
                # Color errors red
                if any(err in output for err in
                       ["Error:", "NameError", "TypeError", "ZeroDivision",
                        "SyntaxError", "ModuleNotFound"]):
                    print(f"\033[31m{output}\033[0m")
                else:
                    last_result = output
                    print(f"\033[34mOut[{exec_count}]:\033[0m {output}")

            exec_count += 1

    except Exception as e:
        print(f"\n\033[31mError: {e}\033[0m")
    finally:
        conn.close()
        print("\n\033[33mDisconnected.\033[0m")


if __name__ == "__main__":
    main()
