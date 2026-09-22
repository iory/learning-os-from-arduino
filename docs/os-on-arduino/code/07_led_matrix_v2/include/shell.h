/**
 * shell.h - Interactive Shell Header
 * Chapter 5: Interactive Shell
 */

#ifndef SHELL_H
#define SHELL_H

#ifdef __cplusplus
extern "C" {
#endif

// Shell task entry point
void shell_task(void);

// Process a single command
void shell_process_command(char *cmd);

#ifdef __cplusplus
}
#endif

#endif
