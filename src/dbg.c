#include "dbg.h"
#include <stdarg.h>

FILE *log_file = NULL;

int init_logging(const char *log_filename){
    log_file = fopen(log_filename, "a");
    if(log_file == NULL){
        return -1;
    }
    return 0;
}

void log_messages(const char *format, ...){
    if(log_file){
        va_list args;
        va_start(args, format);
        vfprintf(log_file, format, args);
        va_end(args);
        fflush(log_file);
    }
 }

void close_logging(void){
    if(log_file){
        fclose(log_file);
        log_file = NULL;
    }
 }