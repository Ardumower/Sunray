#include <Arduino.h>
#include "vector_type.h"

#ifndef quaternion_type_h
#define quaternion_type_h

// input labels
#define LOCAL_FRAME     true
#define GLOBAL_FRAME    false

#define SMALL_ANGLE     true
#define LARGE_ANGLE     false
 
//------- 4D Quaternion --------

struct quat_t {
    // Components
    float  w;       // scalar
    vec3_t v;       // vector

    // 0. Constructors:
    quat_t();
    quat_t( float, float, float, float );
    quat_t( float, vec3_t );
    quat_t( vec3_t );
    quat_t( float [] );
    
    // 0A. Setters and getters
    void copyArray( float [] );
    void set(const int, float);
	float get(const int);
    
    // 1. Basic Operations:
        // Addition and subtraction:
    quat_t operator + ( const quat_t & );
    quat_t operator - ( const quat_t & );
    quat_t operator - ( void );
    void operator += ( const quat_t & );
    void operator -= ( const quat_t & );    

		// Element-wise multiplication
	quat_t operator ^ ( const quat_t & );
	void operator ^= ( const quat_t & );

        // Scalar product and division:
    quat_t operator * ( const float );
    quat_t operator / ( const float );
    void operator *= ( const float );
    void operator /= ( const float );

        // Quaternion multiplication and division:
    quat_t operator * ( const quat_t & );
    quat_t operator / ( quat_t & );
    void operator *= ( const quat_t & );
    void operator /= ( quat_t & );

    // 3. Important operations:
    quat_t conj();
    quat_t norm();
    float inner();
    float mag();

    // 4. Rotation transform:
        // axis and angle 
    void setRotation( vec3_t, float, const bool );
    void setRotation( vec3_t, const bool );

        // vector rotation
    vec3_t rotate( vec3_t, const bool );

        // axis projections
    vec3_t axisX( const bool );
    vec3_t axisY( const bool );
    vec3_t axisZ( const bool );
};

// 1B. Global operators:
    // vector multiplication
quat_t operator * ( vec3_t &, vec3_t & );

    // reverse order scalar product
quat_t operator * ( const float, quat_t & );

#endif
