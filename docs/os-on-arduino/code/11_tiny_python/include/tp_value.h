/**
 * tp_value.h - TinyPython Value System
 *
 * All Python values are represented as tagged unions.
 * This is how MicroPython (and CPython) work internally:
 * every value carries its type information at runtime.
 */

#ifndef TP_VALUE_H
#define TP_VALUE_H

#include <stdint.h>
#include "tp_config.h"

// Value types
typedef enum {
    TP_NONE,
    TP_INT,
    TP_BOOL,
    TP_STR,
    TP_FUNC,
} TpType;

// Forward declaration
struct TpFunc;

// A TinyPython value (tagged union)
typedef struct {
    TpType type;
    union {
        int32_t ival;        // TP_INT, TP_BOOL
        const char *sval;    // TP_STR (points into string pool)
        struct TpFunc *fval; // TP_FUNC
    };
} TpValue;

// Function definition
typedef struct TpFunc {
    char name[16];
    char params[TP_MAX_PARAMS][16];
    int param_count;
    // Body stored as line indices into block buffer
    int body_start;    // index into global block storage
    int body_count;    // number of lines
} TpFunc;

// Convenience constructors
static inline TpValue tp_none(void) {
    TpValue v; v.type = TP_NONE; v.ival = 0; return v;
}
static inline TpValue tp_int(int32_t n) {
    TpValue v; v.type = TP_INT; v.ival = n; return v;
}
static inline TpValue tp_bool(int b) {
    TpValue v; v.type = TP_BOOL; v.ival = b ? 1 : 0; return v;
}
static inline TpValue tp_str(const char *s) {
    TpValue v; v.type = TP_STR; v.sval = s; return v;
}

// Check truthiness (like Python's bool())
static inline int tp_is_truthy(TpValue v) {
    switch (v.type) {
        case TP_NONE: return 0;
        case TP_INT:  return v.ival != 0;
        case TP_BOOL: return v.ival != 0;
        case TP_STR:  return v.sval && v.sval[0] != '\0';
        default:      return 1;
    }
}

#endif // TP_VALUE_H
