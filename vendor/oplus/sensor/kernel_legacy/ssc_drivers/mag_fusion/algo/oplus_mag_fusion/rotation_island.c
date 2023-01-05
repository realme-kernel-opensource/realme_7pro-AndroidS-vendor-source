#include "rotation.h"


// convert euler angles to rotation matrix
rotmat* euler_to_rotmat(euler *e, rotmat *m)
{
    elem sinX, cosX, sinY, cosY, sinZ, cosZ;
    elem_sin_cos((*e)[0], &sinX, &cosX);
    elem_sin_cos((*e)[1], &sinY, &cosY);
    elem_sin_cos((*e)[2], &sinZ, &cosZ);
    (*m)[0] = elem_mul(cosY, cosZ);
    (*m)[1] = elem_mul(cosY, -sinZ);
    (*m)[2] = sinY;
    (*m)[3] = elem_mul3( sinX, sinY, cosZ) + elem_mul(cosX, sinZ);
    (*m)[4] = elem_mul3( sinX, sinY, -sinZ) + elem_mul(cosX, cosZ);
    (*m)[5] = elem_mul (-sinX, cosY);
    (*m)[6] = elem_mul3( cosX, -sinY, cosZ) + elem_mul(sinX, sinZ);
    (*m)[7] = elem_mul3( cosX, sinY, sinZ) + elem_mul(sinX, cosZ);
    (*m)[8] = elem_mul(cosX, cosY);
    return m;
}

// vector add, support in-place
vector* vector_add(vector *dst, vector *a, vector *b)
{
    (*dst)[0] = (*a)[0] + (*b)[0];
    (*dst)[1] = (*a)[1] + (*b)[1];
    (*dst)[2] = (*a)[2] + (*b)[2];
    return dst;
}

// vector subtraction, support in-place
vector* vector_sub(vector *dst, vector *a, vector *b)
{
    (*dst)[0] = (*a)[0] - (*b)[0];
    (*dst)[1] = (*a)[1] - (*b)[1];
    (*dst)[2] = (*a)[2] - (*b)[2];
    return dst;
}

// vector scaling, support in-place
vector* vector_scale(vector *dst, vector *v, elem scale)
{
    (*dst)[0] = elem_mul((*v)[0], scale);
    (*dst)[1] = elem_mul((*v)[1], scale);
    (*dst)[2] = elem_mul((*v)[2], scale);
    return dst;
}

// vector dot product
elem vector_dot_prod(vector *a, vector *b)
{
    return elem_mul((*a)[0], (*b)[0])
        + elem_mul((*a)[1], (*b)[1])
        + elem_mul((*a)[2], (*b)[2]);
}

// square of the distance between two vectors
elem vector_distance_sqr(vector *a, vector *b)
{
    elem ret = elem_zero;
    elem temp;
    temp = (*a)[0] - (*b)[0];
    ret += elem_mul(temp, temp);
    temp = (*a)[1] - (*b)[1];
    ret += elem_mul(temp, temp);
    temp = (*a)[2] - (*b)[2];
    ret += elem_mul(temp, temp);
    return ret;
}

// calc yaw angle in 0~360, defining [0,y,z] as 0 degree, [x,0,z] as 90 degrees
elem vector_yaw(vector *v)
{
    elem yaw = -elem_atan2((*v)[0], (*v)[1]);

    if(yaw < 0) {
        yaw += TWO_PI;
    }

    return yaw;
}

// calc pitch angle in -90~90, defining [x,y,0] as 0 degree, [0,0,z] as +-90 degrees
elem vector_pitch(vector *v)
{
    elem den = elem_sqrt((*v)[0] * (*v)[0] + (*v)[1] * (*v)[1]);

    if (den == 0) {
        return 0;
    }

    return elem_atan2((*v)[2], den);
}

// copy data from src to dst
rotmat* rotmat_copy(rotmat *dst, rotmat *src)
{
    (*dst)[0] = (*src)[0];
    (*dst)[1] = (*src)[1];
    (*dst)[2] = (*src)[2];
    (*dst)[3] = (*src)[3];
    (*dst)[4] = (*src)[4];
    (*dst)[5] = (*src)[5];
    (*dst)[6] = (*src)[6];
    (*dst)[7] = (*src)[7];
    (*dst)[8] = (*src)[8];
    return dst;
}

// assign 'I' to m
rotmat* rotmat_assign_eye(rotmat *m)
{
    (*m)[0] = elem_one;
    (*m)[1] = 0;
    (*m)[2] = 0;
    (*m)[3] = 0;
    (*m)[4] = elem_one;
    (*m)[5] = 0;
    (*m)[6] = 0;
    (*m)[7] = 0;
    (*m)[8] = elem_one;
    return m;
}

// assign transpose of src to dst, support in-place
rotmat* rotmat_transpose(rotmat *dst, rotmat *m)
{
    // 0 1 2    0 3 6
    // 3 4 5 -> 1 4 7
    // 6 7 8    2 5 8
    if(m == dst) {
        elem temp;
        temp = (*m)[1];
        (*dst)[1] = (*m)[3];
        (*dst)[3] = temp;
        temp = (*m)[2];
        (*dst)[2] = (*m)[6];
        (*dst)[6] = temp;
        temp = (*m)[5];
        (*dst)[5] = (*m)[7];
        (*dst)[7] = temp;
        return dst;
    } else {
        (*dst)[0] = (*m)[0];
        (*dst)[1] = (*m)[3];
        (*dst)[2] = (*m)[6];
        (*dst)[3] = (*m)[1];
        (*dst)[4] = (*m)[4];
        (*dst)[5] = (*m)[7];
        (*dst)[6] = (*m)[2];
        (*dst)[7] = (*m)[5];
        (*dst)[8] = (*m)[8];
    }

    return dst;
}

// matrix add, support in-place
rotmat* rotmat_add(rotmat *dst, rotmat *a, rotmat *b)
{
    (*dst)[0] = (*a)[0] + (*b)[0];
    (*dst)[1] = (*a)[1] + (*b)[1];
    (*dst)[2] = (*a)[2] + (*b)[2];
    (*dst)[3] = (*a)[3] + (*b)[3];
    (*dst)[4] = (*a)[4] + (*b)[4];
    (*dst)[5] = (*a)[5] + (*b)[5];
    (*dst)[6] = (*a)[6] + (*b)[6];
    (*dst)[7] = (*a)[7] + (*b)[7];
    (*dst)[8] = (*a)[8] + (*b)[8];
    return dst;
}

// matrix subtract, support in-place
rotmat* rotmat_sub(rotmat *dst, rotmat *a, rotmat *b)
{
    (*dst)[0] = (*a)[0] - (*b)[0];
    (*dst)[1] = (*a)[1] - (*b)[1];
    (*dst)[2] = (*a)[2] - (*b)[2];
    (*dst)[3] = (*a)[3] - (*b)[3];
    (*dst)[4] = (*a)[4] - (*b)[4];
    (*dst)[5] = (*a)[5] - (*b)[5];
    (*dst)[6] = (*a)[6] - (*b)[6];
    (*dst)[7] = (*a)[7] - (*b)[7];
    (*dst)[8] = (*a)[8] - (*b)[8];
    return dst;
}

// scaling matrix, support in-place
rotmat* rotmat_scale(rotmat *dst, rotmat *m, elem scale)
{
    (*dst)[0] = elem_mul( (*m)[0], scale );
    (*dst)[1] = elem_mul( (*m)[1], scale );
    (*dst)[2] = elem_mul( (*m)[2], scale );
    (*dst)[3] = elem_mul( (*m)[3], scale );
    (*dst)[4] = elem_mul( (*m)[4], scale );
    (*dst)[5] = elem_mul( (*m)[5], scale );
    (*dst)[6] = elem_mul( (*m)[6], scale );
    (*dst)[7] = elem_mul( (*m)[7], scale );
    (*dst)[8] = elem_mul( (*m)[8], scale );
    return dst;
}

// rotate a vector, support in-place
vector* rotmat_rotate(vector *dst, rotmat *m, vector *v)
{
    // | 0 1 2 |   | 0 |
    // | 3 4 5 | x | 1 |
    // | 6 7 8 |   | 2 |
    (*dst)[0] = elem_mul((*m)[0], (*v)[0]) + elem_mul((*m)[1], (*v)[1]) + elem_mul((*m)[2], (*v)[2]);
    (*dst)[1] = elem_mul((*m)[3], (*v)[0]) + elem_mul((*m)[4], (*v)[1]) + elem_mul((*m)[5], (*v)[2]);
    (*dst)[2] = elem_mul((*m)[6], (*v)[0]) + elem_mul((*m)[7], (*v)[1]) + elem_mul((*m)[8], (*v)[2]);
    return dst;
}

// matrix multiply, support in-place
rotmat* rotmat_mul(rotmat *dst, rotmat *a, rotmat *b)
{
    if(a == dst || b == dst) {
        rotmat temp;
        rotmat_mul(&temp, a, b);
        rotmat_copy(dst, &temp);
        return dst;
    }

    // 0 1 2     0 1 2     0*0+1*3+2*6 0*1+1*4+2*7 0*2+1*5+2*8
    // 3 4 5  x  3 4 5  =  3*0+4*3+5*6 3*1+4*4+5*7 3*2+4*5+5*8
    // 6 7 8     6 7 8     6*0+7*3+8*6 6*1+7*4+8*7 6*2+7*5+8*8

    // row 1
    (*dst)[0] = elem_mul((*a)[0], (*b)[0]) + elem_mul((*a)[1], (*b)[3]) + elem_mul((*a)[2], (*b)[6]); //col 1
    (*dst)[1] = elem_mul((*a)[0], (*b)[1]) + elem_mul((*a)[1], (*b)[4]) + elem_mul((*a)[2], (*b)[7]); //col 2
    (*dst)[2] = elem_mul((*a)[0], (*b)[2]) + elem_mul((*a)[1], (*b)[5]) + elem_mul((*a)[2], (*b)[8]); //col 3
    // row 2
    (*dst)[3] = elem_mul((*a)[3], (*b)[0]) + elem_mul((*a)[4], (*b)[3]) + elem_mul((*a)[5], (*b)[6]); //col 1
    (*dst)[4] = elem_mul((*a)[3], (*b)[1]) + elem_mul((*a)[4], (*b)[4]) + elem_mul((*a)[5], (*b)[7]); //col 2
    (*dst)[5] = elem_mul((*a)[3], (*b)[2]) + elem_mul((*a)[4], (*b)[5]) + elem_mul((*a)[5], (*b)[8]); //col 3
    // row 3
    (*dst)[6] = elem_mul((*a)[6], (*b)[0]) + elem_mul((*a)[7], (*b)[3]) + elem_mul((*a)[8], (*b)[6]); //col 1
    (*dst)[7] = elem_mul((*a)[6], (*b)[1]) + elem_mul((*a)[7], (*b)[4]) + elem_mul((*a)[8], (*b)[7]); //col 2
    (*dst)[8] = elem_mul((*a)[6], (*b)[2]) + elem_mul((*a)[7], (*b)[5]) + elem_mul((*a)[8], (*b)[8]); //col 3

    return dst;
}

// matrix algebraic cofactor, support in-place
rotmat* rotmat_algebraic_cofactor(rotmat *dst, rotmat *m)
{
    if(m == dst) {
        rotmat temp;
        rotmat_algebraic_cofactor(&temp, m);
        rotmat_copy(dst, &temp);
        return dst;
    }

    // 0 1 2
    // 3 4 5
    // 6 7 8
    (*dst)[0] = +(elem_mul((*m)[4], (*m)[8]) - elem_mul((*m)[7], (*m)[5])); // 1,1
    (*dst)[1] = -(elem_mul((*m)[1], (*m)[8]) - elem_mul((*m)[7], (*m)[2])); // 2,1
    (*dst)[2] = +(elem_mul((*m)[1], (*m)[5]) - elem_mul((*m)[4], (*m)[2])); // 3,1
    (*dst)[3] = -(elem_mul((*m)[3], (*m)[8]) - elem_mul((*m)[6], (*m)[5])); // 1,2
    (*dst)[4] = +(elem_mul((*m)[0], (*m)[8]) - elem_mul((*m)[6], (*m)[2])); // 2,2
    (*dst)[5] = -(elem_mul((*m)[0], (*m)[5]) - elem_mul((*m)[3], (*m)[2])); // 3,2
    (*dst)[6] = +(elem_mul((*m)[3], (*m)[7]) - elem_mul((*m)[6], (*m)[4])); // 1,3
    (*dst)[7] = -(elem_mul((*m)[0], (*m)[7]) - elem_mul((*m)[6], (*m)[1])); // 2,3
    (*dst)[8] = +(elem_mul((*m)[0], (*m)[4]) - elem_mul((*m)[3], (*m)[1])); // 3,3
    return dst;
}

// matrix determinant
elem rotmat_determinant(rotmat *m, rotmat *ac)
{
    // 0 1 2
    // 3 4 5
    // 6 7 8
    if(ac == NULL) {
        rotmat temp;
        temp[0] = +(elem_mul((*m)[4], (*m)[8]) - elem_mul((*m)[7], (*m)[5])); // 1,1
        temp[3] = -(elem_mul((*m)[3], (*m)[8]) - elem_mul((*m)[6], (*m)[5])); // 1,2
        temp[6] = +(elem_mul((*m)[3], (*m)[7]) - elem_mul((*m)[6], (*m)[4])); // 1,3
        return rotmat_determinant(m, &temp);
    }

    return elem_mul((*m)[0], (*ac)[0]) + elem_mul((*m)[1], (*ac)[3]) + elem_mul((*m)[2], (*ac)[6]);
}

// matrix inverse
rotmat* rotmat_inverse(rotmat *dst, rotmat *m, rotmat *ac, elem det)
{
    if(ac == NULL) {
        rotmat temp_ac;
        rotmat_algebraic_cofactor(&temp_ac, m);
        rotmat_inverse(dst, m, &temp_ac, det);
        return dst;
    }

    if(det == 0) {
        det = rotmat_determinant(m, ac);

        if (det == 0) {
            return NULL;
        }
    }

    rotmat_scale(dst, ac, elem_div(elem_one, det));
    // (*dst)[0] = elem_div((*ac)[0],det);
    // (*dst)[1] = elem_div((*ac)[1],det);
    // (*dst)[2] = elem_div((*ac)[2],det);
    // (*dst)[3] = elem_div((*ac)[3],det);
    // (*dst)[4] = elem_div((*ac)[4],det);
    // (*dst)[5] = elem_div((*ac)[5],det);
    // (*dst)[6] = elem_div((*ac)[6],det);
    // (*dst)[7] = elem_div((*ac)[7],det);
    // (*dst)[8] = elem_div((*ac)[8],det);
    return dst;
}

// calc rotation angle
elem rotmat_angle(rotmat *m)
{
    elem cos_val = elem_div( (*m)[0] + (*m)[4] + (*m)[8] - elem_val(1.0f), elem_val(2.0f) );
    cos_val = cos_val < elem_val(-1) ? elem_val(-1) :
        cos_val > elem_val(1) ? elem_val(1) :
        cos_val;
    return elem_acos(cos_val);
}

