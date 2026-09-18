/**
 * tp_env.h - TinyPython Environment (Variable Storage)
 *
 * Variables are stored in a hash-map-like structure.
 * Each scope (global, function) has its own variable table.
 * This is similar to Python's namespace/dict system.
 */

#ifndef TP_ENV_H
#define TP_ENV_H

#include "tp_value.h"
#include "tp_config.h"

#ifdef __cplusplus
extern "C" {
#endif

// A variable entry: name -> value
typedef struct {
    char name[16];
    TpValue value;
    uint8_t used;
} TpVar;

// A scope (variable namespace)
typedef struct {
    TpVar vars[TP_MAX_VARS];
} TpScope;

// Initialize the environment system
void tp_env_init(void);

// Push/pop scope (for function calls)
int tp_env_push_scope(void);
void tp_env_pop_scope(void);

// Get/set variable in current scope (falls back to global)
TpValue tp_env_get(const char *name);
void tp_env_set(const char *name, TpValue value);

// Check if variable exists
int tp_env_exists(const char *name);

// String pool: intern a string and return a pointer to the pool
const char *tp_str_intern(const char *s);

// Function storage
TpFunc *tp_func_new(const char *name);
TpFunc *tp_func_find(const char *name);

// Block storage (shared buffer for function/loop bodies)
int tp_block_store_line(const char *line);
const char *tp_block_get_line(int index);

#ifdef __cplusplus
}
#endif

#endif // TP_ENV_H
