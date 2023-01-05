#ifndef __ROTATION_H__
#define __ROTATION_H__

#include "stdbool.h"

#define MAG6D_USING_FIXED_POINT false

#ifndef NULL
#define NULL (void*)0
#endif

#if MAG6D_USING_FIXED_POINT
#include "fixed_point.h"
typedef int elem;
#define elem_val(f) FX_FLTTOFIX_Q16(f)
#define elem_flt(e) FX_FIXTOFLT_Q16(e)
#define elem_abs(e) FX_ABS(e)
#define elem_one FX_ONE_Q16
#define elem_zero 0
#define elem_max FX_MAX_Q16
#define elem_min FX_MIN_Q16
#define elem_mul(a,b) FX_MUL_Q16(a,b)
#define elem_div(a,b) FX_DIV_Q16(a,b)
#define elem_sqrt(n) sqrtFxQ16(n)
#define elem_atan2(num, den) arcTan2FxQ16(num, den)
#define elem_acos(n) arcCosFxQ16(n)
#define elem_asin(n) arcSinFxQ16(n)
#define elem_sin(theta, out_sin) {sinCosTanFxQ16(theta, out_sin ,NULL ,NULL);}
#define elem_cos(theta, out_cos) {sinCosTanFxQ16(theta, NULL, out_cos, NULL);}
#define elem_tan(theta, out_tan) {sinCosTanFxQ16(theta, NULL, NULL, out_tan);}
#define elem_sin_cos(theta, out_sin, out_cos) {sinCosTanFxQ16(theta, out_sin, out_cos, NULL);}
#define elem_sin_tan(theta, out_sin, out_tan) {sinCosTanFxQ16(theta, out_sin, NULL, out_tan);}
#define elem_cos_tan(theta, out_cos, out_tan) {sinCosTanFxQ16(theta, NULL, out_cos, out_tan);}
#define elem_sin_cos_tan(theta, out_sin, out_cos, out_tan) {sinCosTanFxQ16(theta, out_sin, out_cos, out_tan);}
#else
typedef unsigned char uint8_t;
typedef float elem;
#include <math.h>
#include <float.h>
#define elem_val(f) ((float)(f))
#define elem_flt(e) (e)
#define elem_abs(e) fabsf(e)
#define elem_one 1.0f
#define elem_zero 0.0f
#define elem_max FLT_MAX
#define elem_min FLT_MIN
#define elem_mul(a,b) ((a)*(b))
#define elem_div(a,b) ((a)/(b))
#define elem_sqrt(n) sqrtf(n)
#define elem_atan2(num, den) atan2f(num, den)
#define elem_acos(n) acosf(n)
#define elem_asin(n) asinf(n)
#define elem_sin(theta,out_sin) {*out_sin=sinf(theta);}
#define elem_cos(theta,out_cos) {*out_cos=cosf(theta);}
#define elem_tan(theta,out_tan) {*out_tan=tanf(theta);}
#define elem_sin_cos(theta,out_sin,out_cos) {*out_sin=sinf(theta); *out_cos=cosf(theta);}
#define elem_sin_tan(theta,out_sin,out_tan) {*out_sin=sinf(theta); *out_tan=tanf(theta);}
#define elem_cos_tan(theta,out_cos,out_tan) {*out_cos=cosf(theta); *out_tan=tanf(theta);}
#define elem_sin_cos_tan(theta,out_sin,out_cos,out_tan) {*out_sin=sinf(theta); *out_cos=cosf(theta); *out_tan=tanf(theta);}
#endif

#define elem_mul3(a,b,c) elem_mul(elem_mul((a),(b)),(c))
#define elem_sqr(e) elem_mul(e, e)


#define PI              elem_val(3.14159265358979323846f)
#define TWO_PI       elem_val(6.28318530717958647692f)
#define RAD_TO_DEG(rad) elem_mul(elem_div(rad,PI),elem_val(180))

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// 3-dimensions vector [x,y,z]
typedef elem vector[3];

#define vector_init_zero {elem_zero, elem_zero, elem_zero}

#define VECTOR_COPY(dst,src) do { (dst)[0] = (src)[0]; (dst)[1] = (src)[1]; (dst)[2] = (src)[2]; } while(0)

// euler angles: right handed axises [x,y,z], positive on anticlockwise
typedef vector euler;

// rotation matrix: right handed axises, column-vector formated
// [ 0 1 2
//   3 4 5
//   6 7 8 ]
typedef elem rotmat[9];

#define rotmat_init_zero { \
    elem_zero, elem_zero, elem_zero, \
    elem_zero, elem_zero, elem_zero, \
    elem_zero, elem_zero, elem_zero}
#define rotmat_init_unit { \
    elem_one, elem_zero, elem_zero, \
    elem_zero, elem_one, elem_zero, \
    elem_zero, elem_zero, elem_one}

// vector add, support in-place
vector* vector_add(vector *dst, vector *a, vector *b);

// vector subtraction, support in-place
vector* vector_sub(vector *dst, vector *a, vector *b);

// vector scaling, support in-place
vector* vector_scale(vector *dst, vector *v, elem scale);

// vector dot product
elem vector_dot_prod(vector *a, vector *b);

// vector length square
#define vector_len_sqr(v_ptr) vector_dot_prod(v_ptr, v_ptr)
#define vector_len(v_ptr) elem_sqrt(vector_len_sqr(v_ptr))

// square of the distance between two vectors
elem vector_distance_sqr(vector *a, vector *b);

// distance between two vectors
#define vector_distance(a, b) elem_sqrt(vector_distance_sqr(a, b))

// calc yaw angle in 0~360, defining [0,y,z] as 0 degree, [x,0,z] as 90 degrees
elem vector_yaw(vector *v);

// calc pitch angle in -90~90, defining [x,y,0] as 0 degree, [0,0,z] as +-90 degrees
elem vector_pitch(vector *v);

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// convert euler angles to rotation matrix
rotmat* euler_to_rotmat(euler *e, rotmat *m);

// copy data from src to dst
rotmat* rotmat_copy(rotmat *dst, rotmat *src);

// initialize m as I
rotmat* rotmat_assign_eye(rotmat *m);

// assign transpose of src to dst, support in-place
rotmat* rotmat_transpose(rotmat *dst, rotmat *m);

// matrix add, support in-place
rotmat* rotmat_add(rotmat *dst, rotmat *a, rotmat *b);

// matrix subtract, supports in-place
rotmat* rotmat_sub(rotmat *dst, rotmat *a, rotmat *b);

// scaling matrix, supports in-place
rotmat* rotmat_scale(rotmat *dst, rotmat *m, elem scale);

// rotate a vector, supports in-place
vector* rotmat_rotate(vector *dst, rotmat *m, vector *v);

// matrix multiply, supports in-place
rotmat* rotmat_mul(rotmat *dst, rotmat *a, rotmat *b);

// matrix algebraic cofactor, supports in-place
rotmat* rotmat_algebraic_cofactor(rotmat *dst, rotmat *m);

// matrix determinant
elem rotmat_determinant(rotmat *m, rotmat *ac);

// matrix inverse, supports in-place
rotmat* rotmat_inverse(rotmat *dst, rotmat *m, rotmat *ac, elem det);

// calc rotation angle
elem rotmat_angle(rotmat *m);


#endif
