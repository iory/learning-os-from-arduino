/**
 * tp_mod_matrix.cpp - LED Matrix Module
 *
 * Controls the built-in 12x8 LED matrix on Arduino Uno R4 WiFi.
 *
 * Usage:
 *   >>> import matrix
 *   >>> matrix.clear()
 *   >>> matrix.pixel(3, 5)
 *   >>> matrix.show()
 *   >>> matrix.fill(1)
 */

#include "tp_module.h"
#include <Arduino.h>
#include <string.h>

// Defined in main.cpp
extern uint8_t g_matrix_frame[8][12];
extern "C" void matrix_render(void);

static TpValue fn_clear(TpValue *args, int argc)
{
    (void)args; (void)argc;
    memset(g_matrix_frame, 0, sizeof(g_matrix_frame));
    matrix_render();
    return tp_none();
}

static TpValue fn_pixel(TpValue *args, int argc)
{
    if (argc < 2) return tp_none();
    int row = args[0].ival;
    int col = args[1].ival;
    int val = (argc >= 3) ? args[2].ival : 1;
    if (row >= 0 && row < 8 && col >= 0 && col < 12) {
        g_matrix_frame[row][col] = val ? 1 : 0;
    }
    return tp_none();
}

static TpValue fn_show(TpValue *args, int argc)
{
    (void)args; (void)argc;
    matrix_render();
    return tp_none();
}

static TpValue fn_fill(TpValue *args, int argc)
{
    uint8_t val = (argc >= 1 && args[0].ival) ? 1 : 0;
    memset(g_matrix_frame, val, sizeof(g_matrix_frame));
    matrix_render();
    return tp_none();
}

static TpValue fn_row(TpValue *args, int argc)
{
    if (argc < 2) return tp_none();
    int row = args[0].ival;
    int pattern = args[1].ival;
    if (row >= 0 && row < 8) {
        for (int c = 0; c < 12; c++) {
            g_matrix_frame[row][11 - c] = (pattern >> c) & 1;
        }
        matrix_render();
    }
    return tp_none();
}

static const TpCFuncEntry matrix_funcs[] = {
    {"clear", fn_clear},
    {"pixel", fn_pixel},
    {"show",  fn_show},
    {"fill",  fn_fill},
    {"row",   fn_row},
};

static const TpModule matrix_module = {
    "matrix",
    matrix_funcs,
    sizeof(matrix_funcs) / sizeof(matrix_funcs[0]),
};

void tp_mod_matrix_register(void)
{
    tp_module_register(&matrix_module);
}
