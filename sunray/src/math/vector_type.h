#include <Arduino.h>

#ifndef vector_type_h
#define vector_type_h

//--------- 3D vector ----------

struct vec3_t { 
    // Components
    float x;
    float y;
    float z;

    // 0. Constructors:
    vec3_t();
    vec3_t( float, float, float );
    vec3_t( float, float );
    vec3_t( float [] );

    // 1. Basic operations:
    	// Addition and subtration:
    vec3_t operator + ( const vec3_t & );
    vec3_t operator - ( const vec3_t & );
    vec3_t operator - ( void );
    void operator += ( const vec3_t & );
    void operator -= ( const vec3_t & );

		// Element-wise multiplication
	vec3_t operator ^ ( const vec3_t & );
	void operator ^= ( const vec3_t & );
    
    	// Scalar multiplication and division:
    vec3_t operator * ( const float );
    vec3_t operator / ( const float );
    void operator *= ( const float );
    void operator /= ( const float );

    	// Important operations: 
    float dot( const vec3_t );
    vec3_t cross( const vec3_t );
    float mag();
    vec3_t norm();
    
    // 2. setters and getters
    void copyArray( float [] );
    void set(const int, float);
	float get(const int);
};

// 1B. Reverse order - Scalar product
vec3_t operator * ( const float, const vec3_t & );

#endif
