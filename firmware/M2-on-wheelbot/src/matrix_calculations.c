#include "matrix_calculations.h"

void multiply_matrix_vector(int rows_A, int columns_A, float Mat_A[rows_A][columns_A],
							float Vec_B[columns_A], float Mat_C[rows_A]) {

		 /*
  for (int i = 0; i < rows_A; i++) { // looping through row i of Matrix A
    Mat_C[i] = 0.0;
    for (int j = 0; j < columns_A; j++) { // looping through column j of Matrix A
      Mat_C[i] += Mat_A[i][j] * Vec_B[j];
    }
  }
		  */

	Mat_C[0] = Mat_A[0][0] * Vec_B[0] + Mat_A[0][1] * Vec_B[1] + Mat_A[0][2] * Vec_B[2];
	Mat_C[1] = Mat_A[1][0] * Vec_B[0] + Mat_A[1][1] * Vec_B[1] + Mat_A[1][2] * Vec_B[2];
	Mat_C[2] = Mat_A[2][0] * Vec_B[0] + Mat_A[2][1] * Vec_B[1] + Mat_A[2][2] * Vec_B[2];
}

void multiply_matrix_vector_long(int rows_A, int columns_A, int32_t Mat_A[rows_A][columns_A],
							int Vec_B[columns_A], int32_t Mat_C[rows_A]) {
	  	/*
  for (int i = 0; i < rows_A; i++) { // looping through row i of Matrix A
    Mat_C[i] = 0.0;
    for (int j = 0; j < columns_A; j++) { // looping through column j of Matrix A
      Mat_C[i] += (Mat_A[i][j] * Vec_B[j] );
    }
  }
  	 */
	Mat_C[0] = Mat_A[0][0] * Vec_B[0] + Mat_A[0][1] * Vec_B[1] + Mat_A[0][2] * Vec_B[2];
	Mat_C[1] = Mat_A[1][0] * Vec_B[0] + Mat_A[1][1] * Vec_B[1] + Mat_A[1][2] * Vec_B[2];
	Mat_C[2] = Mat_A[2][0] * Vec_B[0] + Mat_A[2][1] * Vec_B[1] + Mat_A[2][2] * Vec_B[2];
}

void rot_b_IMU(int32_t R_b_IMU[3][3], float rotx, float rotz) {
	R_b_IMU[0][0] = (long)( 10000 * cos(rotz) );
	R_b_IMU[0][1] = (long)( -10000 * cos(rotx)*sin(rotz) );
	R_b_IMU[0][2] = (long)( 10000 * sin(rotx)*sin(rotz) );
	R_b_IMU[1][0] = (long)( 10000 * sin(rotz) );
	R_b_IMU[1][1] = (long)( 10000 * cos(rotx)*cos(rotz) );
	R_b_IMU[1][2] = (long)( -10000 * sin(rotx)*cos(rotz) );
	R_b_IMU[2][0] = (long)( 0 );
	R_b_IMU[2][1] = (long)( 10000 * sin(rotx) );
	R_b_IMU[2][2] = (long)( 10000 * cos(rotx) );
}