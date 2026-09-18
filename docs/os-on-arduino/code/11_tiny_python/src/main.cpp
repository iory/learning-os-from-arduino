/**
 * Chapter 11: TinyPython - A Minimal Python Interpreter
 *
 * A Python subset interpreter running on Arduino Uno R4 WiFi.
 * Supports: variables, expressions, if/elif/else, while, for,
 * def functions, print(), and Arduino GPIO control.
 *
 * This demonstrates how MicroPython works at a fundamental level.
 */

#include <Arduino.h>
#include "Arduino_LED_Matrix.h"
#include "tp_config.h"
#include "tp_value.h"
#include "tp_env.h"
#include "tp_eval.h"
#include "tp_module.h"
#include <string.h>

// LED Matrix globals
ArduinoLEDMatrix g_matrix;
uint8_t g_matrix_frame[8][12] = {0};

// Wrapper for renderBitmap (calling via extern doesn't work reliably for C++ objects)
extern "C" void matrix_render(void)
{
    g_matrix.renderBitmap(g_matrix_frame, 8, 12);
}

// ---- REPL State ----
static char g_line_buf[TP_LINE_LEN];
static int g_line_pos = 0;
static char g_last_char = 0;  // for \r\n handling

// Block accumulation mode (for if/while/for/def)
static int g_in_block = 0;
static int g_block_start = -1;
static int g_block_line_count = 0;
static int g_block_indent = 0;
static char g_block_type[8]; // "if", "while", "for", "def"

// For def: store function metadata
static char g_def_name[16];
static char g_def_params[TP_MAX_PARAMS][16];
static int g_def_param_count = 0;

static void show_banner(void)
{
    Serial.println();
    Serial.println("  _____  _             ____        _   _");
    Serial.println(" |_   _|(_)_ __  _   _|  _ \\ _   _| |_| |__   ___  _ __");
    Serial.println("   | |  | | '_ \\| | | | |_) | | | | __| '_ \\ / _ \\| '_ \\");
    Serial.println("   | |  | | | | | |_| |  __/| |_| | |_| | | | (_) | | | |");
    Serial.println("   |_|  |_|_| |_|\\__, |_|    \\__, |\\__|_| |_|\\___/|_| |_|");
    Serial.println("                  |___/       |___/");
    Serial.println();
    Serial.println("  TinyPython 0.1 on Arduino UNO R4 WiFi");
    Serial.print("  SRAM: 32KB, Flash: 256KB");
    Serial.println();
    Serial.println("  Type Python code. Try: print(1 + 2)");
    Serial.println();
    Serial.println("  Built-ins:");
    Serial.println("    Python:  print, type, len, abs, min, max, int, str, range");
    Serial.println("    GPIO:    pin_mode, digital_write, digital_read, analog_read");
    Serial.println("    Timing:  delay, millis");
    Serial.println("    LED:     led.blink, led.on, led.off");
    Serial.println("    Matrix:  matrix.pixel, matrix.show, matrix.clear,");
    Serial.println("             matrix.fill, matrix.row");
    Serial.println();
}

static void prompt(void)
{
    if (g_in_block) {
        Serial.print("... ");
    } else {
        Serial.print(">>> ");
    }
}

// Parse def header: "def name(a, b, c):"
static void parse_def_header(const char *line)
{
    // Skip "def "
    line += 4;
    while (*line == ' ') line++;

    // Read function name
    int i = 0;
    while (*line && *line != '(' && *line != ' ' && i < 15) {
        g_def_name[i++] = *line++;
    }
    g_def_name[i] = '\0';

    // Read parameters
    g_def_param_count = 0;
    if (*line == '(') {
        line++;
        while (*line && *line != ')') {
            while (*line == ' ' || *line == ',') line++;
            if (*line == ')') break;
            int j = 0;
            while (*line && *line != ',' && *line != ')' && *line != ' ' && j < 15) {
                g_def_params[g_def_param_count][j++] = *line++;
            }
            g_def_params[g_def_param_count][j] = '\0';
            g_def_param_count++;
            if (g_def_param_count >= TP_MAX_PARAMS) break;
        }
    }
}

static void process_line(const char *line)
{
    // Count indentation
    int indent = 0;
    while (line[indent] == ' ') indent++;
    const char *trimmed = line + indent;

    // If we're in block accumulation mode
    if (g_in_block) {
        // Empty line or line with less/equal indent than block start = end of block
        if (trimmed[0] == '\0') {
            // End of block - execute it
            if (strcmp(g_block_type, "def") == 0) {
                // Register function
                TpFunc *fn = tp_func_new(g_def_name);
                if (fn) {
                    fn->body_start = g_block_start;
                    fn->body_count = g_block_line_count;
                    fn->param_count = g_def_param_count;
                    for (int i = 0; i < g_def_param_count; i++) {
                        strncpy(fn->params[i], g_def_params[i], 15);
                        fn->params[i][15] = '\0';
                    }
                }
            } else {
                // Execute block (if/while/for)
                tp_clear_error();
                tp_eval_lines(g_block_start, g_block_line_count, 0);
            }
            g_in_block = 0;
            return;
        }
        // Accumulate line into block buffer
        tp_block_store_line(line);
        g_block_line_count++;
        return;
    }

    // Skip empty lines
    if (trimmed[0] == '\0') return;

    // Check for block-starting keywords
    // if ...:   while ...:   for ...:   def ...:
    int is_block = 0;
    if (strncmp(trimmed, "if ", 3) == 0 && strchr(trimmed, ':'))
        { is_block = 1; strncpy(g_block_type, "if", 3); }
    else if (strncmp(trimmed, "while ", 6) == 0 && strchr(trimmed, ':'))
        { is_block = 1; strncpy(g_block_type, "while", 6); }
    else if (strncmp(trimmed, "for ", 4) == 0 && strchr(trimmed, ':'))
        { is_block = 1; strncpy(g_block_type, "for", 4); }
    else if (strncmp(trimmed, "def ", 4) == 0 && strchr(trimmed, ':'))
        { is_block = 1; strncpy(g_block_type, "def", 4); }

    if (is_block) {
        if (strcmp(g_block_type, "def") == 0) {
            parse_def_header(trimmed);
        }
        g_in_block = 1;
        g_block_start = tp_block_store_line(line);
        g_block_line_count = 1;
        g_block_indent = indent;
        return;
    }

    // Single line statement - execute immediately
    tp_clear_error();
    TpValue result = tp_eval_line(trimmed);

    // Print result if it's not None and no error (REPL behavior)
    if (!tp_had_error() && result.type != TP_NONE) {
        switch (result.type) {
            case TP_INT:  Serial.println(result.ival); break;
            case TP_BOOL: Serial.println(result.ival ? "True" : "False"); break;
            case TP_STR:
                Serial.print("'");
                Serial.print(result.sval);
                Serial.println("'");
                break;
            default: break;
        }
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial) { }

    // Initialize hardware
    g_matrix.begin();

    // Initialize TinyPython
    tp_env_init();
    tp_eval_init();
    tp_module_init();

    // Register modules (like MicroPython's built-in modules)
    tp_mod_led_register();
    tp_mod_matrix_register();
    tp_mod_gpio_register();

    show_banner();
    prompt();
}

void loop()
{
    while (Serial.available()) {
        char c = Serial.read();

        if (c == '\r' || c == '\n') {
            // Skip \n immediately after \r (handle \r\n as single newline)
            if (c == '\n' && g_last_char == '\r') {
                g_last_char = c;
                continue;
            }
            g_last_char = c;

            Serial.println();
            g_line_buf[g_line_pos] = '\0';

            if (g_line_pos > 0 || g_in_block) {
                process_line(g_line_buf);
            }

            g_line_pos = 0;
            prompt();
        } else if (c == '\b' || c == 127) {
            g_last_char = c;
            // Backspace
            if (g_line_pos > 0) {
                g_line_pos--;
                Serial.print("\b \b");
            }
        } else if (c == 0x03) {
            // Ctrl+C: cancel current block
            if (g_in_block) {
                g_in_block = 0;
                Serial.println();
                Serial.println("KeyboardInterrupt");
                g_line_pos = 0;
                prompt();
            }
        } else if (g_line_pos < TP_LINE_LEN - 1) {
            g_last_char = c;
            g_line_buf[g_line_pos++] = c;
            Serial.print(c);  // Echo
        }
    }
}
