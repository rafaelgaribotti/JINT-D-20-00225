/* Matrix math. */
#include "matrix.h"

/* This could be reduced to a single malloc if it mattered. */
Matrix alloc_matrix(int rows, int cols, int max_rows, int max_cols) {
  int i, j;
  Matrix m;
  m.rows = rows;
  m.cols = cols;
  m.max_rows = max_rows;
  m.max_cols = max_cols;
//  m.data = (double**) calloc( max_rows, sizeof(double*));
  m.data = (double**) malloc(sizeof(double*) * m.max_rows);
  for (i = 0; i < m.max_rows; ++i) {
    m.data[i] = (double*) malloc(sizeof(double) * m.max_cols);
    assert(m.data[i]);
    for (j = 0; j < m.max_cols; ++j) {
      m.data[i][j] = 0.0;
    }
  }
  return m;
}

void free_matrix(Matrix m) {
  assert(m.data != NULL);
  int i;
  for (i = 0; i < m.max_rows; ++i) {
    free(m.data[i]);
  }
  free(m.data);
}

void set_matrix(Matrix m, ...) {
  int i, j;
  va_list ap;
  va_start(ap, m);

  for (i = 0; i < m.rows; ++i) {
    for (j = 0; j < m.cols; ++j) {
      m.data[i][j] = va_arg(ap, double);
    }
  }

  va_end(ap);
}

void set_identity_matrix(Matrix m) {
  assert(m.rows == m.cols);
  int i, j;
  for (i = 0; i < m.rows; ++i) {
    for (j = 0; j < m.cols; ++j) {
      if (i == j) {
	m.data[i][j] = 1.0;
      } else {
	m.data[i][j] = 0.0;
      }
    }
  }
}

void copy_matrix(Matrix source, Matrix destination) {
  assert(source.max_rows == destination.max_rows);
  assert(source.max_cols == destination.max_cols);
  int i, j;
  for (i = 0; i < source.max_rows; ++i) {
    for (j = 0; j < source.max_cols; ++j) {
      destination.data[i][j] = source.data[i][j];
    }
  }
}

void print_matrix(Matrix m) {
  int i, j;
  for (i = 0; i < m.rows; ++i) {
    for (j = 0; j < m.cols; ++j) {
      if (j > 0) {
        printf(" ");
        // printf("\t");
      }
      printf("%.10f", m.data[i][j]);
    }
    // printf("\t");
    printf("\n");
  }
  // printf("\n");
}

void add_matrix(Matrix a, Matrix b, Matrix c) {
  assert(a.rows == b.rows);
  assert(a.rows == c.rows);
  assert(a.cols == b.cols);
  assert(a.cols == c.cols);
  int i, j;
  for (i = 0; i < a.rows; ++i) {
    for (j = 0; j < a.cols; ++j) {
      c.data[i][j] = a.data[i][j] + b.data[i][j];
    }
  }
}

void subtract_matrix(Matrix a, Matrix b, Matrix c) {
  assert(a.rows == b.rows);
  assert(a.rows == c.rows);
  assert(a.cols == b.cols);
  assert(a.cols == c.cols);
  int i, j;
  for (i = 0; i < a.rows; ++i) {
    for (j = 0; j < a.cols; ++j) {
      c.data[i][j] = a.data[i][j] - b.data[i][j];
    }
  }
}

void subtract_from_identity_matrix(Matrix a) {
  assert(a.rows == a.cols);
  int i, j;
  for (i = 0; i < a.rows; ++i) {
    for (j = 0; j < a.cols; ++j) {
      if (i == j) {
	a.data[i][j] = 1.0 - a.data[i][j];
      } else {
	a.data[i][j] = 0.0 - a.data[i][j];
      }
    }
  }
}

void multiply_matrix(Matrix a, Matrix b, Matrix c) {
  assert(a.cols == b.rows);
  assert(a.rows == c.rows);
  assert(b.cols == c.cols);
  int i, j, k;
  for (i = 0; i < c.rows; ++i) {
    for (j = 0; j < c.cols; ++j) {
      /* Calculate element c.data[i][j] via a dot product of one row of a
	 with one column of b */
      c.data[i][j] = 0.0;
      for (k = 0; k < a.cols; ++k) {
      	c.data[i][j] += a.data[i][k] * b.data[k][j];
      }
    }
  }
}

/* This is multiplying a by b-tranpose so it is like multiply_matrix
   but references to b reverse rows and cols. */
void multiply_by_transpose_matrix(Matrix a, Matrix b, Matrix c) {
  assert(a.cols == b.cols);
  assert(a.rows == c.rows);
  assert(b.rows == c.cols);
  int i, j, k;
  for (i = 0; i < c.rows; ++i) {
    for (j = 0; j < c.cols; ++j) {
      /* Calculate element c.data[i][j] via a dot product of one row of a
         with one row of b */
      c.data[i][j] = 0.0;
      for (k = 0; k < a.cols; ++k) {
        c.data[i][j] += a.data[i][k] * b.data[j][k];
      }
    }
  }
}
void multiply2_by_transpose_matrix(Matrix a, Matrix b, Matrix c, uint8_t row, uint8_t col) {
  int i, j, k;
  for (i = 0; i < row; ++i) {
    for (j = 0; j < col; ++j) {
      /* Calculate element c.data[i][j] via a dot product of one row of a
         with one row of b */
      c.data[i][j] = 0.0;
      for (k = 0; k < row; ++k) {
        c.data[i][j] += a.data[i][k] * b.data[j][k];
      }
    }
  }
}

void transpose_matrix(Matrix input, Matrix output) {
  assert(input.rows == output.cols);
  assert(input.cols == output.rows);
  int i, j;
  for (i = 0; i < input.rows; ++i) {
    for (j = 0; j < input.cols; ++j) {
      output.data[j][i] = input.data[i][j];
    }
  }
}

int equal_matrix(Matrix a, Matrix b, double tolerance) {
  assert(a.rows == b.rows);
  assert(a.cols == b.cols);
  int i, j;
  for (i = 0; i < a.rows; ++i) {
    for (j = 0; j < a.cols; ++j) {
      if (fabs(a.data[i][j] - b.data[i][j]) > tolerance) {
	      return 0;
      }
    }
  }
  return 1;
}

void scale_matrix(Matrix m, double scalar) {
  //assert(scalar != 0.0);
  int i, j;
  for (i = 0; i < m.rows; ++i) {
    for (j = 0; j < m.cols; ++j) {
      m.data[i][j] *= scalar;
    }
  }
}

void swap_rows(Matrix m, int r1, int r2) {
  assert(r1 != r2);
  double* tmp = m.data[r1];
  m.data[r1] = m.data[r2];
  m.data[r2] = tmp;
}

void scale_row(Matrix m, int r, double scalar) {
  assert(scalar != 0.0);
  int i;
  for (i = 0; i < m.cols; ++i) {
    m.data[r][i] *= scalar;
  }
}

/* Add scalar * row r2 to row r1. */
void shear_row(Matrix m, int r1, int r2, double scalar) {
  assert(r1 != r2);
  int i;
  for (i = 0; i < m.cols; ++i) {
    m.data[r1][i] += scalar * m.data[r2][i];
  }
}

/* Uses Gauss-Jordan elimination.

   The elimination procedure works by applying elementary row
   operations to our input matrix until the input matrix is reduced to
   the identity matrix.
   Simultaneously, we apply the same elementary row operations to a
   separate identity matrix to produce the inverse matrix.
   If this makes no sense, read wikipedia on Gauss-Jordan elimination.

   This is not the fastest way to invert matrices, so this is quite
   possibly the bottleneck. */
int destructive_invert_matrix(Matrix input, Matrix output) {
  assert(input.rows == input.cols);
  assert(input.rows == output.rows);
  assert(input.rows == output.cols);
  int i, j;

  set_identity_matrix(output);

  /* Convert input to the identity matrix via elementary row operations.
     The ith pass through this loop turns the element at i,i to a 1
     and turns all other elements in column i to a 0. */
  for (i = 0; i < input.rows; ++i) {
    if (input.data[i][i] == 0.0) {
      /* We must swap rows to get a nonzero diagonal element. */
      int r;
      for (r = i + 1; r < input.rows; ++r) {
	if (input.data[r][i] != 0.0) {
	  break;
	}
      }
      if (r == input.rows) {
	/* Every remaining element in this column is zero, so this
	   matrix cannot be inverted. */
	return 0;
      }
      swap_rows(input, i, r);
      swap_rows(output, i, r);
    }

    /* Scale this row to ensure a 1 along the diagonal.
       We might need to worry about overflow from a huge scalar here. */
    double scalar = 1.0 / input.data[i][i];
    scale_row(input, i, scalar);
    scale_row(output, i, scalar);

    /* Zero out the other elements in this column. */
    for (j = 0; j < input.rows; ++j) {
      if (i == j) {
      	continue;
      }
      double shear_needed = -input.data[j][i];
      shear_row(input, j, i, shear_needed);
      shear_row(output, j, i, shear_needed);
    }
  }

  return 1;
}

void gg_ss(Matrix m,  Matrix const v){
    assert(m.rows == m.cols);
    assert(v.rows == 3);

    m.data[0][1] = -v.data[2][0];
    m.data[0][2] =  v.data[1][0];
    m.data[1][0] =  v.data[2][0];
    m.data[1][2] = -v.data[0][0];
    m.data[2][0] = -v.data[1][0];
    m.data[2][1] =  v.data[0][0];
}

void gg_diag (Matrix m, Matrix const v){
    assert(m.rows == v.rows);
    assert(m.cols == v.rows);
    int i;

    for(i = 0; i < v.rows; i++)
        m.data[i][i] = v.data[i][0];
}

void gg_mean(Matrix mean, Matrix const m, int resolution){
    assert(mean.rows == m.rows);
    assert(resolution <= m.cols);
    int i, j;

    for(i = 0; i < mean.rows; i++)
        for(j = 0; j < resolution; j++)
            mean.data[i][0] += m.data[0][j];

    for(i = 0; i < mean.rows; i++)
        mean.data[i][0] /= resolution;
}

void gg_variance(Matrix v, Matrix const mat, Matrix const mean, int biasSensorN){
    assert(v.rows == mat.rows);
    assert(v.rows == mean.rows);
    assert(v.cols == mean.cols);
    int i, j;

    for(i = 0; i < v.rows; i++)
        for(j = 0; j < biasSensorN; j++)
            v.data[i][0] += pow(mat.data[i][j] - mean.data[i][0],2);

    for(i = 0; i < v.rows; i++)
        v.data[i][0] /= biasSensorN;
}

void gg_xdot(Matrix xdot, Matrix const x, double T){
    assert(xdot.rows == x.rows);
    assert(xdot.cols == x.cols);
    int i, j;

    for(i = 0; i < x.rows; i++){
        for(j = 0; j < x.cols; j++){
            if(j == (x.cols-1))
                xdot.data[i][j] = (x.data[i][0]-x.data[i][j])/T;
            else
                xdot.data[i][j] = (x.data[i][(j+1)]-x.data[i][j])/T;
        }
    }
}

void gg_jacobian_rtu(Matrix J, Matrix const q, Matrix const u){
    assert(J.rows == u.rows);
    assert(J.cols == q.rows);
    Matrix e, et, Se, Su, idt, eut, uet, Weu, etu, temp, seu, sunweu;
    double n;
    int i, j;

    e  = alloc_matrix(3, 1, 3, 1);
    et = alloc_matrix(1, 3, 1, 3);
    Se = alloc_matrix(3, 3, 3, 3);
    Su = alloc_matrix(3, 3, 3, 3);
    seu = alloc_matrix(3, 1, 3, 1);
    Weu = alloc_matrix(3, 3, 3, 3);
    idt = alloc_matrix(3, 3, 3, 3);
    eut = alloc_matrix(3, 3, 3, 3);
    etu = alloc_matrix(1, 1, 1, 1);
    uet = alloc_matrix(3, 3, 3, 3);
    temp= alloc_matrix(3, 3, 3, 3);
    sunweu= alloc_matrix(3, 3, 3, 3);

    set_identity_matrix(idt);
    n  = q.data[0][0];
    set_matrix(e, q.data[1][0],
                  q.data[2][0],
                  q.data[3][0]);

    gg_ss(Se, e);
    gg_ss(Su, u);

    // Weu = e*u' - 2*u*e' + e'*u*eye(3)
    multiply_by_transpose_matrix(e, u, eut);
    multiply_by_transpose_matrix(u, e, uet);
    scale_matrix(uet, -2);
    transpose_matrix(e, et);
    multiply_matrix(et, u, etu);
    scale_matrix(idt, etu.data[0][0]);
    add_matrix(eut, uet, temp);
    add_matrix(temp, idt, Weu);

    // J_rtu = 2*[-Se*u   Su*n+Weu]
    multiply_matrix(Se, u, seu);
    scale_matrix(seu, -2);
    scale_matrix(Su, n);
    add_matrix(Su, Weu, sunweu);
    scale_matrix(sunweu, 2);

    for(i = 0; i < seu.rows; i++)
        J.data[i][0] = seu.data[i][0];

    for(i = 0; i < sunweu.rows; i++)
        for(j = 0; j < sunweu.cols; j++)
            J.data[i][(j+1)] = sunweu.data[i][j];

    free_matrix(e);
    free_matrix(et);
    free_matrix(Se);
    free_matrix(Su);
    free_matrix(Weu);
    free_matrix(idt);
    free_matrix(eut);
    free_matrix(uet);
    free_matrix(etu);
    free_matrix(seu);
    free_matrix(sunweu);
}
