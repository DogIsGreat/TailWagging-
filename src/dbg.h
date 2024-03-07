/**
 Standard Debugging file for projects.
 Debugging macros can follow this format:

 #define debug(M, ...) do { \
        fprintf(stderr, "DEBUG %s:%d: " M "\n",  __FILE__, __LINE__, ##__VA_ARGS__); \
        log_to_file("DEBUG %sa:%d: " M, __FILE__, __LINE__, ##__VA_ARGS__); \
} while (0)

*/

#ifndef __dbg_h__
#define __dbg_h__

#include <stdio.h>
#include <errno.h>
#include <string.h>

#ifdef NDEBUG
#define debug(M, ...)
#else
#endif

int init_logging(const char *log_filename);

void log_messages(const char *format, ...);

void close_logging(void);

#define clean_errno() (errno == 0 ? "None" : strerror(errno))

#define log_err(M, ...) do { \
        log_messages("[ERROR] (%s:%d errno: %s) " M "\n",__FILE__, __LINE__, clean_errno(), ##__VA_ARGS__); \
} while (0)

#define log_warn(M, ...) do { \
        log_messages("[WARN] (%s:%d errno: %s) " M "\n",__FILE__, __LINE__, clean_errno(), ##__VA_ARGS__); \
} while (0)

#define log_info(M, ...) do { \
        log_messages("[INFO] (%s:%d) " M "\n",__FILE__, __LINE__, ##__VA_ARGS__); \
} while (0)

#define log_value(M, ...) do { \
        log_messages("[VALUE] (%s:%d) " M "\n",__FILE__, __LINE__, ##__VA_ARGS__); \
} while (0)

#define check_mem(A, M, ...) do { \
        if(!(A)){\
        log_messages("[MEMORY ERROR] (%s:%d errno: %s) " M "\n", __FILE__, __LINE__, clean_errno(), ##__VA_ARGS__);\
        }\
} while (0)

// Rebuild these macros to the new format.
#define check(A, M, ...) if(!(A)) {\
    log_err(M, ##__VA_ARGS__); errno=0; goto error; }

#define sentinel(M, ...)  { log_err(M, ##__VA_ARGS__);\
    errno=0; goto error; }


#define check_debug(A, M, ...) if(!(A)) { debug(M, ##__VA_ARGS__);\
    errno=0; goto error; }
#endif