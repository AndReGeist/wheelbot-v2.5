#ifndef matrix_calculations__
#define matrix_calculations__

#include "m_general.h"
#include "config.h"

void multiply_matrix_vector(int rows_A, int columns_A, float Mat_A[rows_A][columns_A],
							float Vec_B[columns_A], float Mat_C[rows_A]);
void multiply_matrix_vector_long(int rows_A, int columns_A, int32_t Mat_A[rows_A][columns_A],
							int Vec_B[columns_A], int32_t Mat_C[rows_A]);

void rot_b_IMU(int32_t R_b_IMU[3][3], float rotx, float rotz);

#endif
