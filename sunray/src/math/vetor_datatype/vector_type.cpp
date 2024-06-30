#include "vector_type.h"

//------------------- Constructors -------------------

vec3_t::vec3_t() {}

// Specific components
vec3_t::vec3_t( float _x, float _y, float _z ) {
    x = _x;
    y = _y;
    z = _z;
}

// 2D vector:
vec3_t::vec3_t( float _x, float _y ) {
    x = _x;
    y = _y;
    z = 0;  
}

// Convert from array
vec3_t::vec3_t( float arr[] ) {
    x = arr[0];
    y = arr[1];
    z = arr[2];
}
  
//---------------- Basic operations ------------------

//-- Addition and subtration:

// Addition
vec3_t vec3_t::operator + ( const vec3_t &r ) {
    vec3_t v = { x + r.x ,
                 y + r.y ,
                 z + r.z };
    return v;
}

// Subtraction
vec3_t vec3_t::operator - ( const vec3_t &r ) {
    vec3_t v = { x - r.x ,
                 y - r.y ,
                 z - r.z };
    return v;
}

// Negation
vec3_t vec3_t::operator - ( void ) {
    vec3_t v = { -x ,
                 -y ,
                 -z };
    return v;
}

// Increment
void vec3_t::operator += ( const vec3_t &r ) {
    x += r.x;
    y += r.y;
    z += r.z;
}

// Decrement
void vec3_t::operator -= ( const vec3_t &r ) {
    x -= r.x;
    y -= r.y;
    z -= r.z;
} 


//----------- Element-wise multiplication ------------
vec3_t vec3_t::operator ^ ( const vec3_t &r ) {
	vec3_t v = { x * r.x ,
	             y * r.y ,
	             z * r.z };
	return v;
}

void vec3_t::operator ^= ( const vec3_t &r ) {
	x *= r.x;
	y *= r.y;
	z *= r.z;
}  

//-------- Scalar multiplication and division --------
  
// Scalar product
vec3_t vec3_t::operator * ( const float s ) {
    vec3_t v = { x * s ,
                 y * s ,
                 z * s };
    return v;
}

// Scalar division
vec3_t vec3_t::operator / ( const float s ) {
    vec3_t v = { x / s ,
                 y / s ,
                 z / s };
    return v;
}

// Self multiply
void vec3_t::operator *= ( const float s ) {
    x *= s;
    y *= s;
    z *= s;
}

// Self divide
void vec3_t::operator /= ( const float s ) {
    x /= s;
    y /= s;
    z /= s;
}

// Reverse order - Scalar product --- [Global operator] 
vec3_t operator * ( const float s, const vec3_t &r ) {
    vec3_t v = { r.x * s ,
               r.y * s ,
               r.z * s };
    return v;
}

//--------------- Setters and getters ----------------

void vec3_t::copyArray( float s[] ) {
	s[0] = x;
	s[1] = y;
	s[2] = z;
}

void vec3_t::set(const int i, float input) {
	switch(i) {
		case 0:
			x = input;
			break;
		case 1:
			y = input;
			break;
		case 2:
			z = input;
			break;
	}
}

float vec3_t::get(const int i) {
	switch(i) {
		case 0:
			return x;
			break;
		case 1:
			return y;
			break;
		case 2:
        default:
			return z;
			break;
	}
}

//--------------- Important operations ---------------

// Dot product
float vec3_t::dot( const vec3_t r ) {
    return x*r.x + y*r.y + z*r.z;  
}

// Cross product
vec3_t vec3_t::cross( const vec3_t r ) {
    vec3_t v = { y*r.z - z*r.y  ,
                -x*r.z + z*r.x  ,
                 x*r.y - y*r.x  }; 
    return v;
}

// Magnitude
float vec3_t::mag() {
    return sqrt( x*x + y*y + z*z );
}

// Normalize
vec3_t vec3_t::norm() {
    vec3_t v = { x, y, z };
    return v/mag();
}
