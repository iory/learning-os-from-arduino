// interpreter.h

#ifndef INTERPRETER_H
#define INTERPRETER_H

#include <stdint.h>

#define MAX_SCRIPT_LINES 32
#define MAX_LINE_LENGTH 32

// 実行結果
typedef enum {
    EXEC_OK,
    EXEC_ERROR,
    EXEC_END
} ExecResult;

// インタプリタ初期化
void interp_init(void);

// スクリプトをロード（複数行を改行で区切って渡す）
int interp_load(const char *script);

// 1行実行
ExecResult interp_step(void);

// スクリプト全体を実行
int interp_run(void);

// 実行中かどうか
bool interp_is_running(void);

// インタプリタタスク
void interpreter_task(void);

#endif // INTERPRETER_H
