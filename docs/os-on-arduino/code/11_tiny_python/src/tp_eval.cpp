/**
 * tp_eval.cpp - TinyPython Evaluator
 *
 * A recursive descent parser/evaluator.
 * Parses Python-like syntax and evaluates immediately.
 *
 * Key concept: each parse function corresponds to a grammar rule
 * and returns the evaluated result.
 */

#include "tp_eval.h"
#include "tp_env.h"
#include "tp_module.h"
#include "tp_config.h"
#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

// ---- Tokenizer ----
// The tokenizer breaks input into tokens (words, numbers, operators)

typedef enum {
    TOK_EOF, TOK_INT, TOK_STR, TOK_NAME,
    TOK_PLUS, TOK_MINUS, TOK_STAR, TOK_SLASH, TOK_PERCENT,
    TOK_EQ, TOK_EQEQ, TOK_NEQ, TOK_LT, TOK_GT, TOK_LE, TOK_GE,
    TOK_LPAREN, TOK_RPAREN, TOK_COMMA, TOK_COLON,
    TOK_AND, TOK_OR, TOK_NOT,
    TOK_IF, TOK_ELIF, TOK_ELSE, TOK_WHILE, TOK_FOR, TOK_IN,
    TOK_DEF, TOK_RETURN, TOK_IMPORT, TOK_TRUE, TOK_FALSE, TOK_NONE,
    TOK_LBRACKET, TOK_RBRACKET, TOK_DOT,
    TOK_PLUSEQ, TOK_MINUSEQ,
} TokenType;

static const char *g_src;   // current source pointer
static TokenType g_tok;     // current token type
static int32_t g_tok_ival;  // integer value of current token
static char g_tok_sval[64]; // string/name value of current token

int tp_return_flag = 0;
TpValue tp_return_value;

// ---- Error handling ----
static int g_error_flag = 0;

static void tp_error(const char *type, const char *msg)
{
    g_error_flag = 1;
    Serial.print(type);
    Serial.print(": ");
    Serial.println(msg);
}

// 再帰が深すぎてスコープを積めないとき。掲載コードの行数を増やさないよう関数にしてある。
static TpValue tp_recursion_error(void)
{
    tp_error("RecursionError", "maximum recursion depth exceeded");
    return tp_none();
}

static void tp_error_name(const char *name)
{
    g_error_flag = 1;
    Serial.print("NameError: name '");
    Serial.print(name);
    Serial.println("' is not defined");
}

int tp_had_error(void) { return g_error_flag; }
void tp_clear_error(void) { g_error_flag = 0; }

// Skip whitespace (but not newlines - we handle one line at a time)
static void skip_space(void)
{
    while (*g_src == ' ' || *g_src == '\t') g_src++;
}

// Read next token from g_src
static void next_token(void)
{
    skip_space();

    if (*g_src == '\0' || *g_src == '#') {
        g_tok = TOK_EOF;
        return;
    }

    // Numbers
    if (isdigit(*g_src) || (*g_src == '-' && isdigit(g_src[1]) &&
        (g_tok == TOK_EOF || g_tok == TOK_LPAREN || g_tok == TOK_COMMA ||
         g_tok == TOK_EQ || g_tok == TOK_PLUS || g_tok == TOK_MINUS ||
         g_tok == TOK_STAR || g_tok == TOK_SLASH || g_tok == TOK_COLON ||
         g_tok == TOK_EQEQ || g_tok == TOK_NEQ || g_tok == TOK_LT ||
         g_tok == TOK_GT || g_tok == TOK_LE || g_tok == TOK_GE ||
         g_tok == TOK_RETURN))) {
        char *end;
        g_tok_ival = strtol(g_src, &end, 0);
        g_src = end;
        g_tok = TOK_INT;
        return;
    }

    // Strings
    if (*g_src == '"' || *g_src == '\'') {
        char quote = *g_src++;
        int i = 0;
        while (*g_src && *g_src != quote && i < 63) {
            if (*g_src == '\\' && g_src[1]) {
                g_src++;
                switch (*g_src) {
                    case 'n': g_tok_sval[i++] = '\n'; break;
                    case 't': g_tok_sval[i++] = '\t'; break;
                    case '\\': g_tok_sval[i++] = '\\'; break;
                    default: g_tok_sval[i++] = *g_src; break;
                }
            } else {
                g_tok_sval[i++] = *g_src;
            }
            g_src++;
        }
        if (*g_src == quote) g_src++;
        g_tok_sval[i] = '\0';
        g_tok = TOK_STR;
        return;
    }

    // Names and keywords
    if (isalpha(*g_src) || *g_src == '_') {
        int i = 0;
        while ((isalnum(*g_src) || *g_src == '_') && i < 63) {
            g_tok_sval[i++] = *g_src++;
        }
        g_tok_sval[i] = '\0';

        // Check keywords
        if (strcmp(g_tok_sval, "if") == 0)      { g_tok = TOK_IF; return; }
        if (strcmp(g_tok_sval, "elif") == 0)    { g_tok = TOK_ELIF; return; }
        if (strcmp(g_tok_sval, "else") == 0)    { g_tok = TOK_ELSE; return; }
        if (strcmp(g_tok_sval, "while") == 0)   { g_tok = TOK_WHILE; return; }
        if (strcmp(g_tok_sval, "for") == 0)     { g_tok = TOK_FOR; return; }
        if (strcmp(g_tok_sval, "in") == 0)      { g_tok = TOK_IN; return; }
        if (strcmp(g_tok_sval, "def") == 0)     { g_tok = TOK_DEF; return; }
        if (strcmp(g_tok_sval, "return") == 0)  { g_tok = TOK_RETURN; return; }
        if (strcmp(g_tok_sval, "import") == 0)  { g_tok = TOK_IMPORT; return; }
        if (strcmp(g_tok_sval, "and") == 0)     { g_tok = TOK_AND; return; }
        if (strcmp(g_tok_sval, "or") == 0)      { g_tok = TOK_OR; return; }
        if (strcmp(g_tok_sval, "not") == 0)     { g_tok = TOK_NOT; return; }
        if (strcmp(g_tok_sval, "True") == 0)    { g_tok = TOK_TRUE; return; }
        if (strcmp(g_tok_sval, "False") == 0)   { g_tok = TOK_FALSE; return; }
        if (strcmp(g_tok_sval, "None") == 0)    { g_tok = TOK_NONE; return; }
        g_tok = TOK_NAME;
        return;
    }

    // Two-character operators
    if (g_src[0] == '=' && g_src[1] == '=') { g_src += 2; g_tok = TOK_EQEQ; return; }
    if (g_src[0] == '!' && g_src[1] == '=') { g_src += 2; g_tok = TOK_NEQ; return; }
    if (g_src[0] == '<' && g_src[1] == '=') { g_src += 2; g_tok = TOK_LE; return; }
    if (g_src[0] == '>' && g_src[1] == '=') { g_src += 2; g_tok = TOK_GE; return; }
    if (g_src[0] == '+' && g_src[1] == '=') { g_src += 2; g_tok = TOK_PLUSEQ; return; }
    if (g_src[0] == '-' && g_src[1] == '=') { g_src += 2; g_tok = TOK_MINUSEQ; return; }

    // Single-character operators
    char c = *g_src++;
    switch (c) {
        case '+': g_tok = TOK_PLUS; return;
        case '-': g_tok = TOK_MINUS; return;
        case '*': g_tok = TOK_STAR; return;
        case '/': g_tok = TOK_SLASH; return;
        case '%': g_tok = TOK_PERCENT; return;
        case '=': g_tok = TOK_EQ; return;
        case '<': g_tok = TOK_LT; return;
        case '>': g_tok = TOK_GT; return;
        case '(': g_tok = TOK_LPAREN; return;
        case ')': g_tok = TOK_RPAREN; return;
        case '[': g_tok = TOK_LBRACKET; return;
        case ']': g_tok = TOK_RBRACKET; return;
        case ',': g_tok = TOK_COMMA; return;
        case ':': g_tok = TOK_COLON; return;
        case '.': g_tok = TOK_DOT; return;
        default:  g_tok = TOK_EOF; return;
    }
}

// ---- Parser/Evaluator ----
// Each function parses one grammar rule and returns evaluated result

static TpValue parse_expr(void);

// atom = INT | STR | True | False | None | NAME '(' args ')' | NAME | '(' expr ')'
static TpValue parse_atom(void)
{
    if (g_tok == TOK_INT) {
        TpValue v = tp_int(g_tok_ival);
        next_token();
        return v;
    }
    if (g_tok == TOK_STR) {
        TpValue v = tp_str(tp_str_intern(g_tok_sval));
        next_token();
        return v;
    }
    if (g_tok == TOK_TRUE) { next_token(); return tp_bool(1); }
    if (g_tok == TOK_FALSE) { next_token(); return tp_bool(0); }
    if (g_tok == TOK_NONE) { next_token(); return tp_none(); }

    if (g_tok == TOK_NAME) {
        char name[64];
        strncpy(name, g_tok_sval, 63);
        name[63] = '\0';
        next_token();

        // Dot notation: module.func(args)
        if (g_tok == TOK_DOT) {
            next_token();
            if (g_tok != TOK_NAME) { return tp_none(); }
            char func_name[64];
            strncpy(func_name, g_tok_sval, 63);
            func_name[63] = '\0';
            next_token();

            // Parse arguments
            TpValue args[TP_MAX_PARAMS];
            int argc = 0;
            if (g_tok == TOK_LPAREN) {
                next_token();
                while (g_tok != TOK_RPAREN && g_tok != TOK_EOF) {
                    if (argc >= TP_MAX_PARAMS) break;
                    args[argc++] = parse_expr();
                    if (g_tok == TOK_COMMA) next_token();
                }
                if (g_tok == TOK_RPAREN) next_token();
            }

            // Look up module and function
            const TpModule *mod = tp_module_find(name);
            if (mod) {
                TpCFunc fn = tp_module_find_func(mod, func_name);
                if (fn) return fn(args, argc);
                tp_error("AttributeError", "module has no such function");
                return tp_none();
            }
            tp_error_name(name);
            return tp_none();
        }

        // Function call: name(args)
        if (g_tok == TOK_LPAREN) {
            next_token();
            TpValue args[TP_MAX_PARAMS];
            int argc = 0;
            while (g_tok != TOK_RPAREN && g_tok != TOK_EOF) {
                if (argc >= TP_MAX_PARAMS) break;
                args[argc++] = parse_expr();
                if (g_tok == TOK_COMMA) next_token();
            }
            if (g_tok == TOK_RPAREN) next_token();

            // ---- Built-in functions (always available) ----
            if (strcmp(name, "print") == 0) {
                for (int i = 0; i < argc; i++) {
                    if (i > 0) Serial.print(' ');
                    switch (args[i].type) {
                        case TP_INT:  Serial.print(args[i].ival); break;
                        case TP_BOOL: Serial.print(args[i].ival ? "True" : "False"); break;
                        case TP_STR:  Serial.print(args[i].sval); break;
                        case TP_NONE: Serial.print("None"); break;
                        default: break;
                    }
                }
                Serial.println();
                return tp_none();
            }
            if (strcmp(name, "type") == 0 && argc == 1) {
                switch (args[0].type) {
                    case TP_INT:  return tp_str(tp_str_intern("<int>"));
                    case TP_BOOL: return tp_str(tp_str_intern("<bool>"));
                    case TP_STR:  return tp_str(tp_str_intern("<str>"));
                    case TP_NONE: return tp_str(tp_str_intern("<NoneType>"));
                    case TP_FUNC: return tp_str(tp_str_intern("<function>"));
                }
            }
            if (strcmp(name, "int") == 0 && argc == 1) {
                if (args[0].type == TP_STR) return tp_int(atoi(args[0].sval));
                if (args[0].type == TP_BOOL) return tp_int(args[0].ival);
                return args[0];
            }
            if (strcmp(name, "str") == 0 && argc == 1) {
                char buf[32];
                if (args[0].type == TP_INT) {
                    snprintf(buf, sizeof(buf), "%ld", (long)args[0].ival);
                    return tp_str(tp_str_intern(buf));
                }
                return args[0];
            }
            if (strcmp(name, "len") == 0 && argc == 1 && args[0].type == TP_STR) {
                return tp_int(strlen(args[0].sval));
            }
            if (strcmp(name, "abs") == 0 && argc == 1) {
                return tp_int(args[0].ival < 0 ? -args[0].ival : args[0].ival);
            }
            if (strcmp(name, "range") == 0) {
                return tp_int(argc > 0 ? args[0].ival : 0);
            }
            if (strcmp(name, "min") == 0 && argc == 2) {
                return tp_int(args[0].ival < args[1].ival ? args[0].ival : args[1].ival);
            }
            if (strcmp(name, "max") == 0 && argc == 2) {
                return tp_int(args[0].ival > args[1].ival ? args[0].ival : args[1].ival);
            }

            // Registered global builtins (from modules)
            TpCFunc builtin = tp_builtin_find(name);
            if (builtin) return builtin(args, argc);

            // User-defined function
            TpFunc *fn = tp_func_find(name);
            if (fn) {
                if (tp_env_push_scope() < 0) return tp_recursion_error();
                for (int i = 0; i < fn->param_count && i < argc; i++) {
                    tp_env_set(fn->params[i], args[i]);
                }
                tp_return_flag = 0;
                TpValue result = tp_eval_lines(fn->body_start, fn->body_count, 0);
                if (tp_return_flag) {
                    result = tp_return_value;
                    tp_return_flag = 0;
                }
                tp_env_pop_scope();
                return result;
            }

            tp_error_name(name);
            return tp_none();
        }

        // Variable access
        if (!tp_env_exists(name)) {
            tp_error_name(name);
            return tp_none();
        }
        return tp_env_get(name);
    }

    if (g_tok == TOK_LPAREN) {
        next_token();
        TpValue v = parse_expr();
        if (g_tok == TOK_RPAREN) next_token();
        return v;
    }

    next_token(); // skip unknown
    return tp_none();
}

// unary = '-' unary | 'not' not_expr | atom
static TpValue parse_unary(void)
{
    if (g_tok == TOK_MINUS) {
        next_token();
        TpValue v = parse_unary();
        return tp_int(-v.ival);
    }
    if (g_tok == TOK_NOT) {
        next_token();
        TpValue v = parse_unary();
        return tp_bool(!tp_is_truthy(v));
    }
    return parse_atom();
}

// mul_expr = unary (('*' | '/' | '%') unary)*
static TpValue parse_mul(void)
{
    TpValue left = parse_unary();
    while (g_tok == TOK_STAR || g_tok == TOK_SLASH || g_tok == TOK_PERCENT) {
        TokenType op = g_tok;
        next_token();
        TpValue right = parse_unary();
        if (op == TOK_STAR)   left = tp_int(left.ival * right.ival);
        if (op == TOK_SLASH) {
            if (right.ival == 0) { tp_error("ZeroDivisionError", "division by zero"); return tp_none(); }
            left = tp_int(left.ival / right.ival);
        }
        if (op == TOK_PERCENT) {
            if (right.ival == 0) { tp_error("ZeroDivisionError", "integer modulo by zero"); return tp_none(); }
            left = tp_int(left.ival % right.ival);
        }
    }
    return left;
}

// add_expr = mul_expr (('+' | '-') mul_expr)*
static TpValue parse_add(void)
{
    TpValue left = parse_mul();
    while (g_tok == TOK_PLUS || g_tok == TOK_MINUS) {
        TokenType op = g_tok;
        next_token();
        TpValue right = parse_mul();
        if (op == TOK_PLUS) {
            if (left.type == TP_STR && right.type == TP_STR) {
                char buf[64];
                snprintf(buf, sizeof(buf), "%s%s", left.sval, right.sval);
                left = tp_str(tp_str_intern(buf));
            } else if (left.type == TP_STR || right.type == TP_STR) {
                tp_error("TypeError", "can only concatenate str to str");
                return tp_none();
            } else {
                left = tp_int(left.ival + right.ival);
            }
        }
        if (op == TOK_MINUS) {
            if (left.type == TP_STR || right.type == TP_STR) {
                tp_error("TypeError", "unsupported operand type(s) for -");
                return tp_none();
            }
            left = tp_int(left.ival - right.ival);
        }
    }
    return left;
}

// comparison = add_expr (('==' | '!=' | '<' | '>' | '<=' | '>=') add_expr)*
static TpValue parse_comparison(void)
{
    TpValue left = parse_add();
    while (g_tok == TOK_EQEQ || g_tok == TOK_NEQ ||
           g_tok == TOK_LT || g_tok == TOK_GT ||
           g_tok == TOK_LE || g_tok == TOK_GE) {
        TokenType op = g_tok;
        next_token();
        TpValue right = parse_add();
        int result = 0;
        if (left.type == TP_STR && right.type == TP_STR) {
            int cmp = strcmp(left.sval, right.sval);
            switch (op) {
                case TOK_EQEQ: result = (cmp == 0); break;
                case TOK_NEQ:  result = (cmp != 0); break;
                case TOK_LT:   result = (cmp < 0); break;
                case TOK_GT:   result = (cmp > 0); break;
                case TOK_LE:   result = (cmp <= 0); break;
                case TOK_GE:   result = (cmp >= 0); break;
                default: break;
            }
        } else {
            switch (op) {
                case TOK_EQEQ: result = (left.ival == right.ival); break;
                case TOK_NEQ:  result = (left.ival != right.ival); break;
                case TOK_LT:   result = (left.ival < right.ival); break;
                case TOK_GT:   result = (left.ival > right.ival); break;
                case TOK_LE:   result = (left.ival <= right.ival); break;
                case TOK_GE:   result = (left.ival >= right.ival); break;
                default: break;
            }
        }
        left = tp_bool(result);
    }
    return left;
}

// and_expr = comparison ('and' comparison)*
static TpValue parse_and(void)
{
    TpValue left = parse_comparison();
    while (g_tok == TOK_AND) {
        next_token();
        TpValue right = parse_comparison();
        left = tp_bool(tp_is_truthy(left) && tp_is_truthy(right));
    }
    return left;
}

// or_expr = and_expr ('or' and_expr)*
static TpValue parse_expr(void)
{
    TpValue left = parse_and();
    while (g_tok == TOK_OR) {
        next_token();
        TpValue right = parse_and();
        left = tp_bool(tp_is_truthy(left) || tp_is_truthy(right));
    }
    return left;
}

// ---- Statement evaluation ----

void tp_eval_init(void)
{
    tp_return_flag = 0;
}

// Count leading spaces (for indentation)
static int count_indent(const char *line)
{
    int n = 0;
    while (line[n] == ' ') n++;
    return n;
}

// Execute a block of stored lines
TpValue tp_eval_lines(int block_start, int block_count, int base_indent)
{
    TpValue result = tp_none();
    int i = 0;

    while (i < block_count) {
        if (tp_return_flag || g_error_flag) return result;

        const char *line = tp_block_get_line(block_start + i);
        int indent = count_indent(line);
        const char *trimmed = line + indent;

        if (*trimmed == '\0' || *trimmed == '#') { i++; continue; }

        // Check for if/while/for/def that have sub-blocks
        g_src = trimmed;
        g_tok = TOK_EOF;
        next_token();

        if (g_tok == TOK_IF) {
            next_token();
            TpValue cond = parse_expr();
            // skip colon
            // Find body lines (indented more than current)
            int body_start = block_start + i + 1;
            int body_count = 0;
            while (i + 1 + body_count < block_count) {
                const char *bl = tp_block_get_line(body_start + body_count);
                if (count_indent(bl) <= indent && bl[count_indent(bl)] != '\0') break;
                body_count++;
            }

            if (tp_is_truthy(cond)) {
                result = tp_eval_lines(body_start, body_count, indent + 2);
            } else {
                // Check for elif/else
                i += 1 + body_count;
                while (i < block_count) {
                    const char *nextl = tp_block_get_line(block_start + i);
                    int nindent = count_indent(nextl);
                    if (nindent != indent) break;
                    const char *nt = nextl + nindent;

                    // Check elif
                    if (strncmp(nt, "elif ", 5) == 0) {
                        g_src = nt + 5;
                        g_tok = TOK_EOF;
                        next_token();
                        cond = parse_expr();

                        int elif_body_start = block_start + i + 1;
                        int elif_body_count = 0;
                        while (i + 1 + elif_body_count < block_count) {
                            const char *bl = tp_block_get_line(elif_body_start + elif_body_count);
                            if (count_indent(bl) <= indent && bl[count_indent(bl)] != '\0') break;
                            elif_body_count++;
                        }
                        if (tp_is_truthy(cond)) {
                            result = tp_eval_lines(elif_body_start, elif_body_count, indent + 2);
                            i += 1 + elif_body_count;
                            break;
                        }
                        i += 1 + elif_body_count;
                    }
                    // Check else
                    else if (strncmp(nt, "else:", 5) == 0 || strncmp(nt, "else :", 6) == 0) {
                        int else_body_start = block_start + i + 1;
                        int else_body_count = 0;
                        while (i + 1 + else_body_count < block_count) {
                            const char *bl = tp_block_get_line(else_body_start + else_body_count);
                            if (count_indent(bl) <= indent && bl[count_indent(bl)] != '\0') break;
                            else_body_count++;
                        }
                        result = tp_eval_lines(else_body_start, else_body_count, indent + 2);
                        i += 1 + else_body_count;
                        break;
                    } else {
                        break;
                    }
                }
                continue;
            }
            i += 1 + body_count;
            continue;
        }

        if (g_tok == TOK_WHILE) {
            next_token();
            const char *cond_src = g_src - strlen(g_tok_sval); // reparse each iteration
            // We need to save the condition text
            char cond_text[TP_LINE_LEN];
            strncpy(cond_text, trimmed + 6, sizeof(cond_text) - 1); // skip "while "
            // Remove trailing ':'
            char *colon = strrchr(cond_text, ':');
            if (colon) *colon = '\0';

            int body_start_idx = block_start + i + 1;
            int body_count = 0;
            while (i + 1 + body_count < block_count) {
                const char *bl = tp_block_get_line(body_start_idx + body_count);
                if (count_indent(bl) <= indent && bl[count_indent(bl)] != '\0') break;
                body_count++;
            }

            int iterations = 0;
            while (iterations < 10000) {
                g_src = cond_text;
                g_tok = TOK_EOF;
                next_token();
                TpValue cond = parse_expr();
                if (!tp_is_truthy(cond)) break;
                tp_eval_lines(body_start_idx, body_count, indent + 2);
                if (tp_return_flag || g_error_flag) return result;
                iterations++;
            }
            i += 1 + body_count;
            continue;
        }

        if (g_tok == TOK_FOR) {
            next_token();
            if (g_tok != TOK_NAME) { i++; continue; }
            char var_name[16];
            strncpy(var_name, g_tok_sval, 15);
            var_name[15] = '\0';
            next_token(); // expect 'in'
            if (g_tok != TOK_IN) {
                tp_error("SyntaxError", "expected 'in' after variable in for");
                i++;
                continue;
            }
            next_token();

            // Parse range(...)
            int range_start = 0, range_end = 0, range_step = 1;
            if (g_tok == TOK_NAME && strcmp(g_tok_sval, "range") == 0) {
                next_token(); // skip 'range'
                if (g_tok == TOK_LPAREN) next_token();
                TpValue a = parse_expr();
                if (g_tok == TOK_COMMA) {
                    next_token();
                    TpValue b = parse_expr();
                    range_start = a.ival;
                    range_end = b.ival;
                    if (g_tok == TOK_COMMA) {
                        next_token();
                        TpValue c = parse_expr();
                        range_step = c.ival;
                    }
                } else {
                    range_end = a.ival;
                }
                if (g_tok == TOK_RPAREN) next_token();
            } else {
                tp_error("TypeError", "for loop only supports range() iterable");
            }

            int body_start_idx = block_start + i + 1;
            int body_count = 0;
            while (i + 1 + body_count < block_count) {
                const char *bl = tp_block_get_line(body_start_idx + body_count);
                if (count_indent(bl) <= indent && bl[count_indent(bl)] != '\0') break;
                body_count++;
            }

            if (range_step > 0) {
                for (int32_t j = range_start; j < range_end; j += range_step) {
                    tp_env_set(var_name, tp_int(j));
                    tp_eval_lines(body_start_idx, body_count, indent + 2);
                    if (tp_return_flag || g_error_flag) return result;
                }
            } else if (range_step < 0) {
                for (int32_t j = range_start; j > range_end; j += range_step) {
                    tp_env_set(var_name, tp_int(j));
                    tp_eval_lines(body_start_idx, body_count, indent + 2);
                    if (tp_return_flag || g_error_flag) return result;
                }
            }
            i += 1 + body_count;
            continue;
        }

        // Regular single-line statement
        result = tp_eval_line(trimmed);
        i++;
    }
    return result;
}

// Execute a single line
TpValue tp_eval_line(const char *line)
{
    // Skip empty / comment
    while (*line == ' ' || *line == '\t') line++;
    if (*line == '\0' || *line == '#') return tp_none();

    g_src = line;
    g_tok = TOK_EOF;
    next_token();

    // import statement: import module_name
    if (g_tok == TOK_IMPORT) {
        next_token();
        if (g_tok == TOK_NAME) {
            const TpModule *mod = tp_module_find(g_tok_sval);
            if (mod) {
                Serial.print("import ");
                Serial.println(mod->name);
            } else {
                tp_error("ModuleNotFoundError", g_tok_sval);
            }
        }
        return tp_none();
    }

    // return statement
    if (g_tok == TOK_RETURN) {
        next_token();
        if (g_tok != TOK_EOF) {
            tp_return_value = parse_expr();
        } else {
            tp_return_value = tp_none();
        }
        tp_return_flag = 1;
        return tp_return_value;
    }

    // Assignment: NAME = expr / NAME += expr / NAME -= expr
    if (g_tok == TOK_NAME) {
        char name[64];
        strncpy(name, g_tok_sval, 63);
        name[63] = '\0';
        const char *save = g_src;
        TokenType save_tok = g_tok;
        next_token();

        if (g_tok == TOK_EQ) {
            next_token();
            TpValue val = parse_expr();
            tp_env_set(name, val);
            return val;
        }
        if (g_tok == TOK_PLUSEQ) {
            next_token();
            TpValue right = parse_expr();
            TpValue left = tp_env_get(name);
            TpValue val = tp_int(left.ival + right.ival);
            tp_env_set(name, val);
            return val;
        }
        if (g_tok == TOK_MINUSEQ) {
            next_token();
            TpValue right = parse_expr();
            TpValue left = tp_env_get(name);
            TpValue val = tp_int(left.ival - right.ival);
            tp_env_set(name, val);
            return val;
        }

        // Not an assignment - rewind and parse as expression
        g_src = save;
        g_tok = save_tok;
        // Re-parse from beginning
        g_src = line;
        while (*g_src == ' ' || *g_src == '\t') g_src++;
        g_tok = TOK_EOF;
        next_token();
    }

    // Expression statement
    TpValue result = parse_expr();
    return result;
}
