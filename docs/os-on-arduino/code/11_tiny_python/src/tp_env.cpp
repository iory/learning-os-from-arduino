/**
 * tp_env.cpp - Environment Implementation
 *
 * Variable storage, string interning, function registry, block storage.
 */

#include "tp_env.h"
#include <string.h>

// ---- Scope stack ----
static TpScope g_scopes[TP_MAX_SCOPES];
static int g_scope_depth = 0;

void tp_env_init(void)
{
    memset(g_scopes, 0, sizeof(g_scopes));
    g_scope_depth = 0;
}

int tp_env_push_scope(void)
{
    if (g_scope_depth >= TP_MAX_SCOPES - 1) return -1;
    g_scope_depth++;
    memset(&g_scopes[g_scope_depth], 0, sizeof(TpScope));
    return 0;
}

void tp_env_pop_scope(void)
{
    if (g_scope_depth > 0) g_scope_depth--;
}

// Look up variable: check current scope first, then global (scope 0)
TpValue tp_env_get(const char *name)
{
    // Search current scope
    for (int i = 0; i < TP_MAX_VARS; i++) {
        if (g_scopes[g_scope_depth].vars[i].used &&
            strcmp(g_scopes[g_scope_depth].vars[i].name, name) == 0) {
            return g_scopes[g_scope_depth].vars[i].value;
        }
    }
    // Search global scope if not at global
    if (g_scope_depth > 0) {
        for (int i = 0; i < TP_MAX_VARS; i++) {
            if (g_scopes[0].vars[i].used &&
                strcmp(g_scopes[0].vars[i].name, name) == 0) {
                return g_scopes[0].vars[i].value;
            }
        }
    }
    return tp_none();
}

void tp_env_set(const char *name, TpValue value)
{
    TpScope *scope = &g_scopes[g_scope_depth];
    // Update existing
    for (int i = 0; i < TP_MAX_VARS; i++) {
        if (scope->vars[i].used && strcmp(scope->vars[i].name, name) == 0) {
            scope->vars[i].value = value;
            return;
        }
    }
    // Insert new
    for (int i = 0; i < TP_MAX_VARS; i++) {
        if (!scope->vars[i].used) {
            strncpy(scope->vars[i].name, name, 15);
            scope->vars[i].name[15] = '\0';
            scope->vars[i].value = value;
            scope->vars[i].used = 1;
            return;
        }
    }
}

int tp_env_exists(const char *name)
{
    for (int i = 0; i < TP_MAX_VARS; i++) {
        if (g_scopes[g_scope_depth].vars[i].used &&
            strcmp(g_scopes[g_scope_depth].vars[i].name, name) == 0) {
            return 1;
        }
    }
    if (g_scope_depth > 0) {
        for (int i = 0; i < TP_MAX_VARS; i++) {
            if (g_scopes[0].vars[i].used &&
                strcmp(g_scopes[0].vars[i].name, name) == 0) {
                return 1;
            }
        }
    }
    return 0;
}

// ---- String pool ----
static char g_str_pool[TP_STRING_POOL];
static int g_str_pool_pos = 0;

const char *tp_str_intern(const char *s)
{
    // Check if already interned
    int i = 0;
    while (i < g_str_pool_pos) {
        if (strcmp(&g_str_pool[i], s) == 0) {
            return &g_str_pool[i];
        }
        i += strlen(&g_str_pool[i]) + 1;
    }
    // Add to pool
    int len = strlen(s);
    if (g_str_pool_pos + len + 1 > TP_STRING_POOL) {
        return "ERR:pool_full";
    }
    char *dst = &g_str_pool[g_str_pool_pos];
    strcpy(dst, s);
    g_str_pool_pos += len + 1;
    return dst;
}

// ---- Function storage ----
#define TP_MAX_FUNCS 16
static TpFunc g_funcs[TP_MAX_FUNCS];
static int g_func_count = 0;

TpFunc *tp_func_new(const char *name)
{
    if (g_func_count >= TP_MAX_FUNCS) return NULL;
    TpFunc *f = &g_funcs[g_func_count++];
    memset(f, 0, sizeof(TpFunc));
    strncpy(f->name, name, 15);
    return f;
}

TpFunc *tp_func_find(const char *name)
{
    for (int i = 0; i < g_func_count; i++) {
        if (strcmp(g_funcs[i].name, name) == 0) {
            return &g_funcs[i];
        }
    }
    return NULL;
}

// ---- Block storage (shared for function bodies, loops, etc.) ----
static char g_block_buf[TP_BLOCK_LINES][TP_BLOCK_LINE_LEN];
static int g_block_count = 0;

int tp_block_store_line(const char *line)
{
    if (g_block_count >= TP_BLOCK_LINES) return -1;
    strncpy(g_block_buf[g_block_count], line, TP_BLOCK_LINE_LEN - 1);
    g_block_buf[g_block_count][TP_BLOCK_LINE_LEN - 1] = '\0';
    return g_block_count++;
}

const char *tp_block_get_line(int index)
{
    if (index < 0 || index >= g_block_count) return "";
    return g_block_buf[index];
}
