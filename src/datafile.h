#ifndef __datafile_h__
#define __datafile_h__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <gsl/gsl_matrix.h>

typedef struct{
    int key;
    int value;
} Hashitem;

void count_rows_cols(const char *filename, int *rows, int *cols);

void load_data(const char *filename, gsl_matrix *m);

#endif