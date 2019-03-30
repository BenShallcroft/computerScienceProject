/*
Original version by Alastair Channon: a.d.channon@keele.ac.uk

Amended version by Ben Shallcroft: b@shallcroft.com
	Amended version adds a matrix function create a skew symmetric matrix. Found in the matrix class.

*/

#ifndef   math3d_h
#define   math3d_h

#include <iostream>
#include "types.h"

#undef extern_inline
#undef inline
#ifndef   DONOTINLINE_math3d
#define extern_inline extern inline
#else  // DONOTINLINE_math3d
#define extern_inline // no inlining
#define inline        // no inlining
#endif // DONOTINLINE_math3d

class Vector {
  public: real x,y,z;
   inline Vector operator%(const Vector& v) const; // cross product (nb. % preferred to ^ so that operation takes precedence over + and -)
   inline Vector operator+(const Vector& v) const;
   inline void   operator+=(const Vector& v);
   inline Vector operator-(const Vector& v) const;
   inline void   operator-=(const Vector& v);
   inline Vector operator-() const;
   inline Vector operator*(const real& d) const; // using "const real&" arguments helps by improving timings with -pg (so making code profiling more accurate / simpler), although may not help once "-O3"/etc. used
   inline void   operator*=(const real& d);
   inline Vector operator/(const real& d) const;
   inline void   operator/=(const real& d);
   inline void zero();
   inline void normalize(); // make unit length
          // not implemented: == != [] () ...
};

extern_inline std::ostream& operator<< (std::ostream& out, const Vector& v);
extern_inline Vector operator*(const real& d, const Vector& v);
extern_inline Vector multiplyElements(const Vector& v1, const Vector& v2);
extern_inline real dotProduct(const Vector& v1, const Vector& v2);
extern_inline Vector vectorTripleProduct(const Vector& a, const Vector& b, const Vector& c);

class Matrix {
  public: real m00,m01,m02, m10,m11,m12, m20,m21,m22;
   inline Vector operator*(const Vector& v) const;
   inline Matrix operator*(const Matrix& M) const;
   inline Matrix operator*(const real& d) const;
   inline void   operator*=(const real& d);
   inline Matrix operator/(const real& d) const;
   inline void   operator/=(const real& d);
   inline Matrix operator+(const Matrix& M) const;
   inline void   operator+=(const Matrix& M);
   inline Matrix operator-(const Matrix& M) const;
   inline void   operator-=(const Matrix& M);
   inline Matrix operator-() const;
   inline void zero();
   inline void reorthogonalize(int& reorthogonalizeRotor);
   inline void reorthogonalize();
   inline bool isZero();	//Added by Ben Shallcroft
};

extern_inline std::ostream& operator<< (std::ostream& out, const Matrix& M);
extern_inline Matrix operator*(const real& d, const Matrix& M);
extern_inline Matrix transpose(const Matrix& M);
extern_inline Matrix inverse(const Matrix& M);
extern_inline Matrix multiply(const Vector& v0, const Vector& v1); // returns v0 * transpose(v1), which is a Matrix (3x1 * 1x3 => 3x3)
extern_inline Vector multiply(const Vector& v, const Matrix& M); // returns transpose(transpose(v) * M), which is a Vector (transpose[1x3 * 3x3 => 1x3])

extern_inline Matrix tildaMultiply(const Vector& v, const Matrix& M);             // returns   tilda(v) * M = v % each column of M
extern_inline void   tildaMultiply(const Vector& v, const Matrix& M, Matrix& rM); // sets rM = tilda(v) * M = v % each column of M
extern_inline Matrix tildaMultiply(const Matrix& M, const Vector& v);             // returns   M * tilda(v) // = -transpose(tilda(v)*transpose(M))
extern_inline void   tildaMultiply(const Matrix& M, const Vector& v, Matrix& rM); // sets rM = M * tilda(v) // = -transpose(tilda(v)*transpose(M))

extern_inline Vector transposeMultiply(const Matrix& M, const Vector& v); // returns transpose(M) * v

extern_inline Matrix rotationMatrix(const real& angle, const Vector& axis, bool axisIsUnitLength=false); // return the matrix that rotates angle radians about axis

//Added by Ben Shallcroft
extern_inline Matrix skewSymmetric(const Vector& v);

// convenience functions to output to std::cout
extern_inline void o(const Vector& v);
extern_inline void o(const Matrix& M);

// convenience function(s) that return(s) 0 if and only if none of the values within their arguments are bad (eg. nan, infinite or "subnormal" floating point values). this is easy to extend into classes using these types/classes
extern_inline int bad(const real& d);
extern_inline int bad(const Vector& v);
extern_inline int bad(const Matrix& M);

#ifndef   DONOTINLINE_math3d
#include "math3d.cc"
#endif // DONOTINLINE_math3d

#endif // math3d_h
