/**
 * tp_module.cpp - Module Registry
 *
 * Manages the registry of built-in modules and global builtins.
 */

#include "tp_module.h"
#include <string.h>

// Module registry
static const TpModule *g_modules[TP_MAX_MODULES];
static int g_module_count = 0;

// Global builtins (print, type, len, etc. - no import needed)
#define TP_MAX_BUILTINS 24
static struct {
    const char *name;
    TpCFunc func;
} g_builtins[TP_MAX_BUILTINS];
static int g_builtin_count = 0;

void tp_module_init(void)
{
    g_module_count = 0;
    g_builtin_count = 0;
}

void tp_module_register(const TpModule *mod)
{
    if (g_module_count < TP_MAX_MODULES) {
        g_modules[g_module_count++] = mod;
    }
}

const TpModule *tp_module_find(const char *name)
{
    for (int i = 0; i < g_module_count; i++) {
        if (strcmp(g_modules[i]->name, name) == 0) {
            return g_modules[i];
        }
    }
    return NULL;
}

TpCFunc tp_module_find_func(const TpModule *mod, const char *func_name)
{
    for (int i = 0; i < mod->func_count; i++) {
        if (strcmp(mod->funcs[i].name, func_name) == 0) {
            return mod->funcs[i].func;
        }
    }
    return NULL;
}

void tp_builtin_register(const char *name, TpCFunc func)
{
    if (g_builtin_count < TP_MAX_BUILTINS) {
        g_builtins[g_builtin_count].name = name;
        g_builtins[g_builtin_count].func = func;
        g_builtin_count++;
    }
}

TpCFunc tp_builtin_find(const char *name)
{
    for (int i = 0; i < g_builtin_count; i++) {
        if (strcmp(g_builtins[i].name, name) == 0) {
            return g_builtins[i].func;
        }
    }
    return NULL;
}
