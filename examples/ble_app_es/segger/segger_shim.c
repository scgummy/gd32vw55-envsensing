#include <stdio.h>

struct __SEGGER_RTL_FILE_impl {
    int sub;
};

static FILE stdin_file = {0};
static FILE stdout_file = {0};
static FILE stderr_file = {0};

FILE *stdin = &stdin_file;
FILE *stdout = &stdout_file;
FILE *stderr = &stderr_file;