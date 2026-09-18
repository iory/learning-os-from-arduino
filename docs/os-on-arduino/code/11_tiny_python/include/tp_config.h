/**
 * tp_config.h - TinyPython Configuration
 *
 * Memory budget for 32KB SRAM (Cortex-M / 32bit):
 *   Scope stack:  8 * 48 * 28 = 10752 bytes   <- 最大
 *   Block buffer:     48 * 80 =  3840 bytes
 *   String pool:              =  2048 bytes
 *   Function table:   16 * 124 = 1984 bytes
 *   Line buffer:              =   128 bytes
 *   Total:                     ~18.2KB
 *
 * 本書 11.14 節の表と対応している。値を変えたら向こうも直すこと。
 */

#ifndef TP_CONFIG_H
#define TP_CONFIG_H

// Maximum values in the pool (int, string, bool, none, function)
#define TP_MAX_VALUES    128

// Maximum variables per scope
#define TP_MAX_VARS       48

// Maximum scope depth (function calls)
#define TP_MAX_SCOPES      8

// String pool size
#define TP_STRING_POOL  2048

// Maximum length of a single input line
#define TP_LINE_LEN      128

// Maximum lines in a block (if/while/def body)
#define TP_BLOCK_LINES    48
#define TP_BLOCK_LINE_LEN 80

// Maximum function parameters
#define TP_MAX_PARAMS      6

// Maximum nesting depth (for/while/if)
#define TP_MAX_NEST        8

#endif // TP_CONFIG_H
