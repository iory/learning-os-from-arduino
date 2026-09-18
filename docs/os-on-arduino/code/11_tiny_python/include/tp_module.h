/**
 * tp_module.h - TinyPython Module System
 *
 * Modules are how MicroPython organizes built-in functions.
 * Each module is a separate file with a registration function.
 *
 * Usage from TinyPython REPL:
 *   >>> import led
 *   >>> led.blink(3)
 *   >>> import matrix
 *   >>> matrix.clear()
 */

#ifndef TP_MODULE_H
#define TP_MODULE_H

#include "tp_value.h"

#ifdef __cplusplus
extern "C" {
#endif

// C function signature for built-in functions
typedef TpValue (*TpCFunc)(TpValue *args, int argc);

// A single function entry in a module
typedef struct {
    const char *name;
    TpCFunc func;
} TpCFuncEntry;

// A module: name + array of functions
typedef struct {
    const char *name;
    const TpCFuncEntry *funcs;
    int func_count;
} TpModule;

// Maximum modules
#define TP_MAX_MODULES 8

// Register a module (called at startup)
void tp_module_register(const TpModule *mod);

// Find a module by name (returns NULL if not found)
const TpModule *tp_module_find(const char *name);

// Find a function within a module (returns NULL if not found)
TpCFunc tp_module_find_func(const TpModule *mod, const char *func_name);

// Find a global builtin function (not in any module)
TpCFunc tp_builtin_find(const char *name);

// Register a global builtin function
void tp_builtin_register(const char *name, TpCFunc func);

// Initialize module system
void tp_module_init(void);

// Module registration functions (called from main.cpp)
void tp_mod_led_register(void);
void tp_mod_matrix_register(void);
void tp_mod_gpio_register(void);

#ifdef __cplusplus
}
#endif

#endif // TP_MODULE_H
