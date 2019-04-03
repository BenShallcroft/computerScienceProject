/*
Original version by Alastair Channon: a.d.channon@keele.ac.uk

Amended version by Ben Shallcroft: b@shallcroft.com
	Amended version adds a matrix function create a skew symmetric matrix. Found in the matrix class.

*/

#ifndef   DONOTINLINE_math3d
#ifndef   math3d_h
#warning All of the functions in this file are "extern inline".  Unless compiling with -DDONOTINLINE_math3d, this file is #included by its .h file and should not be compiled separately.
#endif // math3d_h
#endif // DONOTINLINE_math3d

#include <iostream>
#include <cmath>
#include "math3d.h"

// Vector  -------------------------------------------------------------------------------------------------------------------------
//Provide a matrix and print the matrix
extern_inline std::ostream& operator<< (std::ostream& out, const Vector& v) 
{ 
	out<< v.x <<","<< v.y <<","<< v.z;
	return out; 
}

//Provide two matrices and multiply them together
extern_inline Vector multiplyElements(const Vector& v1, const Vector& v2) 
{ 
	Vector rv;
	rv.x=v1.x*v2.x; rv.y=v1.y*v2.y; rv.z=v1.z*v2.z; 
	return rv; 
}

//Provide two matrices, perform the dot product on them
extern_inline real dotProduct(const Vector& v1, const Vector& v2) 
{ return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z; }

//Provide a matrix, perform the dot product 
extern_inline Vector Vector::operator%(const Vector& v) const // cross product
{ Vector rv;
  rv.x = y*v.z - z*v.y;
  rv.y = z*v.x - x*v.z;
  rv.z = x*v.y - y*v.x;
  return rv;
}

extern_inline Vector Vector::operator+(const Vector& v) const 
{ Vector rv; rv.x = x+v.x; rv.y = y+v.y; rv.z = z+v.z; return rv; }

extern_inline void   Vector::operator+=(const Vector& v) 
{ x+=v.x; y+=v.y; z+=v.z; }

extern_inline Vector Vector::operator-(const Vector& v) const 
{ Vector rv; rv.x = x-v.x; rv.y = y-v.y; rv.z = z-v.z; return rv; }

extern_inline void   Vector::operator-=(const Vector& v) 
{ x-=v.x; y-=v.y; z-=v.z; }

extern_inline Vector Vector::operator-() const 
{ Vector rv; rv.x = -x; rv.y = -y; rv.z = -z; return rv; }

extern_inline Vector Vector::operator*(const real& d) const 
{ Vector rv; rv.x=x*d; rv.y=y*d; rv.z=z*d; return rv; }

extern_inline Vector operator*(const real& d, const Vector& v) 
{ Vector rv; rv.x=d*v.x; rv.y=d*v.y; rv.z=d*v.z; return rv; }

extern_inline void   Vector::operator*=(const real& d) 
{ x*=d; y*=d; z*=d; }

extern_inline Vector Vector::operator/(const real& d) const 
{ return *this * (1/d); }

extern_inline void   Vector::operator/=(const real& d) { *this *= (1/d); }

extern_inline void Vector::zero() {x=y=z=0;}

extern_inline void Vector::normalize() // make unit length
{ real d = 1.0/sqrt(x*x+y*y+z*z);
  x*=d; y*=d; z*=d;
}

// No need for this? It uses 12*,7+ cf. simply doing two cross products: 12*,6+ ... Although pipelining may help here?
extern_inline Vector vectorTripleProduct(const Vector& a, const Vector& b, const Vector& c) // returns a%(b%c) = (a.c)b - (a.b)c = -(b%c)%a
{ real aDOTc = a.x*c.x + a.y*c.y + a.z*c.z;
  real aDOTb = a.x*b.x + a.y*b.y + a.z*b.z;
  return aDOTc*b + (-aDOTb)*c;
}

// Matrix  -------------------------------------------------------------------------------------------------------------------------

extern_inline std::ostream& operator<< (std::ostream& out, const Matrix& M)
{ out<< M.m00 <<","<< M.m01 <<","<< M.m02 <<std::endl;
  out<< M.m10 <<","<< M.m11 <<","<< M.m12 <<std::endl;
  out<< M.m20 <<","<< M.m21 <<","<< M.m22 <<std::endl;
  return out;
}

extern_inline Vector Matrix::operator*(const Vector& v) const
{ Vector rv;
  rv.x = m00*v.x+m01*v.y+m02*v.z;
  rv.y = m10*v.x+m11*v.y+m12*v.z;
  rv.z = m20*v.x+m21*v.y+m22*v.z;
  return rv;
}

extern_inline Matrix Matrix::operator*(const Matrix& M) const
{ Matrix rM;
  rM.m00 = m00*M.m00+m01*M.m10+m02*M.m20; rM.m01 = m00*M.m01+m01*M.m11+m02*M.m21; rM.m02 = m00*M.m02+m01*M.m12+m02*M.m22;
  rM.m10 = m10*M.m00+m11*M.m10+m12*M.m20; rM.m11 = m10*M.m01+m11*M.m11+m12*M.m21; rM.m12 = m10*M.m02+m11*M.m12+m12*M.m22;
  rM.m20 = m20*M.m00+m21*M.m10+m22*M.m20; rM.m21 = m20*M.m01+m21*M.m11+m22*M.m21; rM.m22 = m20*M.m02+m21*M.m12+m22*M.m22;
  return rM;
}

extern_inline Matrix Matrix::operator*(const real& d) const
{ Matrix rM;
  rM.m00 = m00*d; rM.m01 = m01*d; rM.m02 = m02*d;
  rM.m10 = m10*d; rM.m11 = m11*d; rM.m12 = m12*d;
  rM.m20 = m20*d; rM.m21 = m21*d; rM.m22 = m22*d;
  return rM;
}

extern_inline Matrix operator*(const real& d, const Matrix& M)
{ // Matrix rM = M * d;
  Matrix rM(M); rM *= d;
  return rM;
}

extern_inline void   Matrix::operator*=(const real& d)
{ m00*=d; m01*=d; m02*=d;
  m10*=d; m11*=d; m12*=d;
  m20*=d; m21*=d; m22*=d;
}

extern_inline Matrix Matrix::operator/(const real& d) const { return *this * (1/d); }
extern_inline void   Matrix::operator/=(const real& d) { *this *= (1/d); }

extern_inline Matrix Matrix::operator+(const Matrix& M) const
{ /*
  Matrix rM;
  rM.m00=m00+M.m00; rM.m01=m01+M.m01; rM.m02=m02+M.m02;
  rM.m10=m10+M.m10; rM.m11=m11+M.m11; rM.m12=m12+M.m12;
  rM.m20=m20+M.m20; rM.m21=m21+M.m21; rM.m22=m22+M.m22;
  */
  Matrix rM(M);
  rM.m00+=m00; rM.m01+=m01; rM.m02+=m02;
  rM.m10+=m10; rM.m11+=m11; rM.m12+=m12;
  rM.m20+=m20; rM.m21+=m21; rM.m22+=m22;
  return rM;
}

extern_inline void   Matrix::operator+=(const Matrix& M)
{ m00+=M.m00; m01+=M.m01; m02+=M.m02;
  m10+=M.m10; m11+=M.m11; m12+=M.m12;
  m20+=M.m20; m21+=M.m21; m22+=M.m22;
}

extern_inline Matrix Matrix::operator-(const Matrix& M) const
{ /*
  Matrix rM;
  rM.m00=m00-M.m00; rM.m01=m01-M.m01; rM.m02=m02-M.m02;
  rM.m10=m10-M.m10; rM.m11=m11-M.m11; rM.m12=m12-M.m12;
  rM.m20=m20-M.m20; rM.m21=m21-M.m21; rM.m22=m22-M.m22;
  */
  Matrix rM(*this);
  rM.m00-=M.m00; rM.m01-=M.m01; rM.m02-=M.m02;
  rM.m10-=M.m10; rM.m11-=M.m11; rM.m12-=M.m12;
  rM.m20-=M.m20; rM.m21-=M.m21; rM.m22-=M.m22;
  return rM;
}

extern_inline void   Matrix::operator-=(const Matrix& M)
{ m00-=M.m00; m01-=M.m01; m02-=M.m02;
  m10-=M.m10; m11-=M.m11; m12-=M.m12;
  m20-=M.m20; m21-=M.m21; m22-=M.m22;
}

extern_inline Matrix Matrix::operator-() const { Matrix rM; rM.m00=-m00; rM.m01=-m01; rM.m02=-m02; rM.m10=-m10; rM.m11=-m11; rM.m12=-m12; rM.m20=-m20; rM.m21=-m21; rM.m22=-m22; return rM; }

extern_inline void Matrix::zero() { m00=m01=m02=m10=m11=m12=m20=m21=m22=0; }

extern_inline void Matrix::reorthogonalize(int& reorthogonalizeRotor)
{ Vector x,y,z;
  if (reorthogonalizeRotor<3) { x.x=m00; x.y=m10; x.z=m20; y.x=m01; y.y=m11; y.z=m21; z.x=m02; z.y=m12; z.z=m22; } // by column
  else                        { x.x=m00; x.y=m01; x.z=m02; y.x=m10; y.y=m11; y.z=m12; z.x=m20; z.y=m21; z.z=m22; } // by row

  switch (reorthogonalizeRotor) {
    case 0: case 3: x.normalize(); z=x%y;z.normalize(); y=z%x;y.normalize(); break;
    case 1: case 4: y.normalize(); x=y%z;x.normalize(); z=x%y;z.normalize(); break;
    case 2: case 5: z.normalize(); y=z%x;y.normalize(); x=y%z;x.normalize(); break;
    default: std::cerr << "Bad reorthogonalizeRotor in Matrix::reorthogonalize" << std::endl;
  }
    
  if (reorthogonalizeRotor<3) { m00=x.x; m10=x.y; m20=x.z;   m01=y.x; m11=y.y; m21=y.z;   m02=z.x; m12=z.y; m22=z.z; } // by column
  else                        { m00=x.x; m01=x.y; m02=x.z;   m10=y.x; m11=y.y; m12=y.z;   m20=z.x; m21=z.y; m22=z.z; } // by row

  //reorthogonalizeRotor++; if (reorthogonalizeRotor==6) reorthogonalizeRotor=0;
}

//Added by Ben Shallcroft
extern_inline void Matrix::reorthogonalize()
{
	Vector X, Y, Z;
	X.x = m00; X.y = m10; X.z = m20;
	Y.x = m01; Y.y = m11; Y.z = m21;
	
	X.normalize();
	Z = (X%Y); Z.normalize();
	Y = (Z%X); Y.normalize();
	
	m00 = X.x; m01 = Y.x; m02 = Z.x;
	m10 = X.y; m11 = Y.y; m12 = Z.y;
	m20 = X.z; m21 = Y.z; m22 = Z.z;
}

extern_inline Matrix transpose(const Matrix& M)
{ Matrix rM;
  rM.m00=M.m00; rM.m01=M.m10; rM.m02=M.m20;
  rM.m10=M.m01; rM.m11=M.m11; rM.m12=M.m21;
  rM.m20=M.m02; rM.m21=M.m12; rM.m22=M.m22;
  return rM;
}

extern_inline Matrix inverse(const Matrix& M)
{ real invdet = 1/(M.m00*M.m11*M.m22 + M.m01*M.m12*M.m20 + M.m02*M.m10*M.m21 - M.m02*M.m11*M.m20 - M.m00*M.m12*M.m21 - M.m01*M.m10*M.m22);
  Matrix rM;
  rM.m00=M.m11*M.m22-M.m12*M.m21; rM.m01=M.m02*M.m21-M.m01*M.m22; rM.m02=M.m01*M.m12-M.m02*M.m11;
  rM.m10=M.m12*M.m20-M.m10*M.m22; rM.m11=M.m00*M.m22-M.m02*M.m20; rM.m12=M.m02*M.m10-M.m00*M.m12;
  rM.m20=M.m10*M.m21-M.m11*M.m20; rM.m21=M.m01*M.m20-M.m00*M.m21; rM.m22=M.m00*M.m11-M.m01*M.m10;
  rM *= invdet;
  return rM;
}

extern_inline Matrix multiply(const Vector& v0, const Vector& v1) // returns v0 * transpose(v1), which is a Matrix (3x1 * 1x3 => 3x3)
{ Matrix rM;
  rM.m00=v0.x*v1.x; rM.m01=v0.x*v1.y; rM.m02=v0.x*v1.z;
  rM.m10=v0.y*v1.x; rM.m11=v0.y*v1.y; rM.m12=v0.y*v1.z;
  rM.m20=v0.z*v1.x; rM.m21=v0.z*v1.y; rM.m22=v0.z*v1.z;
  return rM;
}

extern_inline Vector multiply(const Vector& v, const Matrix& M) // returns transpose(transpose(v) * M), which is a Vector (transpose[1x3 * 3x3 => 1x3])
{ Vector rv;
  rv.x = v.x*M.m00 + v.y*M.m10 + v.z*M.m20;
  rv.y = v.x*M.m01 + v.y*M.m11 + v.z*M.m21;
  rv.z = v.x*M.m02 + v.y*M.m12 + v.z*M.m22;
  return rv;
}

/* tilda(v) = (   0 -v.z  v.y) => tilda(v)*r = (v.y*r.z - v.z*r.y) = v%r
              ( v.z    0 -v.x)                 (v.z*r.x - v.x*r.z)
              (-v.y  v.x    0)                 (v.x*r.y - v.y*r.x)
*/

extern_inline Matrix tildaMultiply(const Vector& v, const Matrix& M)              // returns   tilda(v) * M = v % each column of M
{ Matrix rM;
  rM.m00 = v.y*M.m20 - v.z*M.m10;   rM.m01 = v.y*M.m21 - v.z*M.m11;   rM.m02 = v.y*M.m22 - v.z*M.m12;
  rM.m10 = v.z*M.m00 - v.x*M.m20;   rM.m11 = v.z*M.m01 - v.x*M.m21;   rM.m12 = v.z*M.m02 - v.x*M.m22;
  rM.m20 = v.x*M.m10 - v.y*M.m00;   rM.m21 = v.x*M.m11 - v.y*M.m01;   rM.m22 = v.x*M.m12 - v.y*M.m02;
  return rM;
}

extern_inline void   tildaMultiply(const Vector& v, const Matrix& M, Matrix& rM)  // sets rM = tilda(v) * M = v % each column of M
{ rM.m00 = v.y*M.m20 - v.z*M.m10;   rM.m01 = v.y*M.m21 - v.z*M.m11;   rM.m02 = v.y*M.m22 - v.z*M.m12;
  rM.m10 = v.z*M.m00 - v.x*M.m20;   rM.m11 = v.z*M.m01 - v.x*M.m21;   rM.m12 = v.z*M.m02 - v.x*M.m22;
  rM.m20 = v.x*M.m10 - v.y*M.m00;   rM.m21 = v.x*M.m11 - v.y*M.m01;   rM.m22 = v.x*M.m12 - v.y*M.m02;
}

extern_inline Matrix tildaMultiply(const Matrix& M, const Vector& v)              // returns   M * tilda(v) // = -transpose(tilda(v)*transpose(M))
{ Matrix rM;
  rM.m00 = M.m01*v.z - M.m02*v.y;   rM.m01 = M.m02*v.x - M.m00*v.z;   rM.m02 = M.m00*v.y - M.m01*v.x;
  rM.m10 = M.m11*v.z - M.m12*v.y;   rM.m11 = M.m12*v.x - M.m10*v.z;   rM.m12 = M.m10*v.y - M.m11*v.x;
  rM.m20 = M.m21*v.z - M.m22*v.y;   rM.m21 = M.m22*v.x - M.m20*v.z;   rM.m22 = M.m20*v.y - M.m21*v.x;
  return rM;
}

extern_inline void   tildaMultiply(const Matrix& M, const Vector& v, Matrix& rM)  // sets rM = M * tilda(v) // = -transpose(tilda(v)*transpose(M))
{ rM.m00 = M.m01*v.z - M.m02*v.y;   rM.m01 = M.m02*v.x - M.m00*v.z;   rM.m02 = M.m00*v.y - M.m01*v.x;
  rM.m10 = M.m11*v.z - M.m12*v.y;   rM.m11 = M.m12*v.x - M.m10*v.z;   rM.m12 = M.m10*v.y - M.m11*v.x;
  rM.m20 = M.m21*v.z - M.m22*v.y;   rM.m21 = M.m22*v.x - M.m20*v.z;   rM.m22 = M.m20*v.y - M.m21*v.x;
}

extern_inline Vector transposeMultiply(const Matrix& M, const Vector& v) // returns transpose(M) * v
{ Vector rv;
  rv.x = M.m00*v.x+M.m10*v.y+M.m20*v.z;
  rv.y = M.m01*v.x+M.m11*v.y+M.m21*v.z;
  rv.z = M.m02*v.x+M.m12*v.y+M.m22*v.z;
  return rv;
}

extern_inline Matrix rotationMatrix(const real& angle, const Vector& axis, bool axisIsUnitLength)
{ Matrix rM;
  real x,y,z; if (axisIsUnitLength) {x=axis.x; y=axis.y; z=axis.z;} else {Vector v=axis; v.normalize(); x=v.x; y=v.y; z=v.z;}
  real c=cos(angle),s=sin(angle),d=1-c, xd=x*d,yd=y*d,zd=z*d, xxd=x*xd,xyd=x*yd,xzd=x*zd,yyd=y*yd,yzd=y*zd,zzd=z*zd, xs=x*s,ys=y*s,zs=z*s;
  rM.m00 = c+xxd;  rM.m01 = xyd-zs;  rM.m02 = xzd+ys;
  rM.m10 = xyd+zs; rM.m11 = c+yyd;   rM.m12 = yzd-xs;
  rM.m20 = xzd-ys; rM.m21 = yzd+xs;  rM.m22 = c+zzd;
  return rM;
}
/* Derivation:
Matrices that rotate the x-axis to be axis A=(x) are special-orthogonal and have the form M=(A B C) some vectors B,C; matrix that rotates about x-axis: T=(1 0  0) where { c=cos(angle)
                                             (y)                                                                                                          (0 c -s)       { s=sin(angle)
                                             (z)                                                                                                          (0 s  c)
R = M*T*inverse(M) = (A c*B+s*C -s*B+c*C) (A.x A.y A.z) = (A.x*A+B.x*(c*B+s*C)+C.x*(-s*B+c*C) ...) = (A.x*A+c*(B.x*B+C.x*C)+s*(B.x*C-C.x*B) ...)
                                          (B.x B.y B.z)
                                          (C.x C.y C.z)
B.x*B+C.x*C = (B.x*B.x + C.x*C.x) = (1-x*x) because first          row  of M is unit-length: x*x + B.x*B.x + C.x*C.x = 1 ; B.x*C-C.x*B = (B.x*C.x - C.x*B.x) = ( 0)
              (B.x*B.y + C.x*C.y)   ( -x*y) because first & second rows of M are orthoginal: x*y + B.x*B.y + C.x*C.y = 0                 (B.x*C.y - C.x*B.y)   ( z) } because A=B%C
              (B.x*B.z + C.x*C.z)   ( -x*z) because first & third  rows of M are orthoginal: x*z + B.x*B.z + C.x*C.z = 0                 (B.x*C.z - C.x*B.z)   (-y) }
Similarly for second and third columns of R (where ".x"s are replaced by ".y"s and ".z"s => ...)
=> R = (x*x+(1-x*x)*c y*x-y*x*c-z*s z*x-z*x*c+y*s) = (c+xxd  yxd-zs zxd+ys) = (c+xxd  xyd-zs xzd+ys) ... where d=1-c, xxd=x*x*d, etc.
       (x*y-x*y*c+z*s y*y+(1-y*y)*c z*y-z*y*c-x*s)   (xyd+zs c+yyd  zyd-xs)   (xyd+zs c+yyd  yzd-xs)
       (x*z-x*z*c-y*s y*z-y*z*c+x*s z*z+(1-z*z)*c)   (xzd-ys yzd+xs c+zzd )   (xzd-ys yzd+xs c+zzd )
*/

//Added by Ben Shallcroft
extern_inline Matrix skewSymmetric(const Vector& v)
{
	Matrix rM;
	rM.m00 = 0.0;  rM.m01 = -v.z; rM.m02 = v.y;
	rM.m10 = v.z;  rM.m11 = 0.0;  rM.m12 = -v.x;
	rM.m20 = -v.y; rM.m21 = v.x;  rM.m22 = 0.0;
	
	return rM;
}

//Added by Ben Shallcroft
extern_inline bool Matrix::isZero() 
{ 
	//m00=m01=m02=m10=m11=m12=m20=m21=m22=0; 
	if((m00==0.0) && (m01==0.0) && (m02==0.0) && (m10==0.0) && (m11==0.0) && (m12==0.0) && (m20==0.0) && (m21==0.0) && (m22==0.0))
	{
		return true;
	}
	else
	{
		return false;
	}
}

// convenience functions -----------------------------------------------------------------------------------------------------------

extern_inline void o(const Vector& v) {std::cout << v <<std::endl;}
extern_inline void o(const Matrix& M) {std::cout << M;}

// int bad(const real& d) {int f=std::fpclassify(d); if (f==FP_ZERO || f==FP_NORMAL) return 0; if (f==FP_NAN) return 1; if (f==FP_INFINITE) return 2; if (f==FP_SUBNORMAL) return 4; return 8;}
extern_inline int bad(const real& d) {switch(std::fpclassify(d)) {case FP_ZERO : case FP_NORMAL : return 0; case FP_NAN: return 1; case FP_INFINITE: return 2; case FP_SUBNORMAL: return 4; default: return 8;}}
extern_inline int bad(const Vector& v) {return bad(v.x) | bad(v.y) | bad(v.z);}
extern_inline int bad(const Matrix& M) {return bad(M.m00) | bad(M.m01) | bad(M.m02) | bad(M.m10) | bad(M.m11) | bad(M.m12) | bad(M.m20) | bad(M.m21) | bad(M.m22);}
