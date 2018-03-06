/* Utilities for printing fix16_t datatypes. */

#ifndef _FIXSTRING_H_
#define _FIXSTRING_H_

#include <stdio.h>

#include "../fixmath/fix16.h"
#include "../fixmatrix/fixmatrix.h"
#include "../fixmatrix/fixquat.h"
#include "../fixmatrix/fixvector2d.h"
#include "../fixmatrix/fixvector3d.h"

/* All print_*() functions have interface similar to fprintf().
 */
void print_fix16_t(FILE *stream, fix16_t value, uint_fast8_t width, uint_fast8_t decimals);
void print_mf16(FILE *stream, const mf16 *matrix);
void print_qf16(FILE *stream, const qf16 *quat);
void print_v3d(FILE *stream, const v3d *vector);
void print_v2d(FILE *stream, const v2d *vector);

#endif