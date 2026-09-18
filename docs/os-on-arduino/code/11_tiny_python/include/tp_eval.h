/**
 * tp_eval.h - TinyPython Evaluator
 *
 * Recursive descent parser that evaluates as it parses.
 * This is the simplest interpreter architecture:
 *   1. Tokenize the input string
 *   2. Parse according to grammar rules
 *   3. Evaluate immediately during parsing
 *
 * Grammar (simplified):
 *   statement  = assignment | if_stmt | while_stmt | for_stmt
 *              | def_stmt | return_stmt | expr_stmt
 *   assignment = NAME '=' expr
 *   expr       = or_expr
 *   or_expr    = and_expr ('or' and_expr)*
 *   and_expr   = not_expr ('and' not_expr)*
 *   not_expr   = 'not' not_expr | comparison
 *   comparison = add_expr (('==' | '!=' | '<' | '>' | '<=' | '>=') add_expr)*
 *   add_expr   = mul_expr (('+' | '-') mul_expr)*
 *   mul_expr   = unary (('+' | '-' | '%') unary)*
 *   unary      = '-' unary | atom
 *   atom       = INT | STRING | 'True' | 'False' | 'None'
 *              | NAME '(' args ')' | NAME | '(' expr ')'
 */

#ifndef TP_EVAL_H
#define TP_EVAL_H

#include "tp_value.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initialize the evaluator
void tp_eval_init(void);

// Execute a single line of Python code
// Returns the result (TP_NONE for statements)
TpValue tp_eval_line(const char *line);

// Execute multiple lines (for blocks)
// Returns result of last statement, or return value
TpValue tp_eval_lines(int block_start, int block_count, int indent);

// Flag set by 'return' statement
extern int tp_return_flag;
extern TpValue tp_return_value;

// Error handling
int tp_had_error(void);
void tp_clear_error(void);

#ifdef __cplusplus
}
#endif

#endif // TP_EVAL_H
