#include "quaternion_type.h"

//------------------- Constructors -------------------

quat_t::quat_t() {}

// Individual components
quat_t::quat_t( float _w, float _x, float _y, float _z ) {
    w = _w;
    v = { _x, _y, _z };
}

// Scalar and vector components
quat_t::quat_t( float _w, vec3_t _v ) {
    w = _w;
    v = _v;
}

// Convert to vector
quat_t::quat_t( vec3_t _v ) {
    w = 0;
    v = _v;  
}

// Convert from array
quat_t::quat_t( float arr[] ) {
    w = arr[0];
    v.x = arr[1];
    v.y = arr[2];
    v.z = arr[3];
}

//--------------- Setters and getters -----------------

void quat_t::copyArray( float s[] ) {
	s[0] = w;
	v.copyArray(s + 1);
}

void quat_t::set(const int i, float input) {
	if( i == 0 ) {
		w = input;
	} else {
		v.set(i - 1, input);	
	}
}

float quat_t::get(const int i) {
	if( i == 0 ) {
		return w;
	} else {
		return v.get(i - 1);
	}
}

//----------------- Basic Operations ------------------

//-- Addition and subtraction:

// Addition
quat_t quat_t::operator + ( const quat_t &r ) {
    quat_t q = { w + r.w ,
                 v + r.v };
    return q;
}

// Subtraction
quat_t quat_t::operator - ( const quat_t &r ) {
    quat_t q = { w - r.w ,
                 v - r.v };
    return q;
}
// Negation
quat_t quat_t::operator - ( void ) {
    quat_t q = { -w ,
                 -v };
    return q;
}

// Increment
void quat_t::operator += ( const quat_t &r ) {
    w += r.w;
    v += r.v;
}
// Decrement 
void quat_t::operator -= ( const quat_t &r ) {
    w -= r.w;
    v -= r.v;  
}

//-- Element-wise multiplication:
quat_t quat_t::operator ^ ( const quat_t &r ) {
	quat_t q = { w * r.w , 
	             v ^ r.v }; 
	return q;
} 

void quat_t::operator ^= ( const quat_t &r ) {
	w *= r.w;
	v ^= r.v;
} 

//-- Scalar product and division:

// Scalar product
quat_t quat_t::operator * ( const float s ) {
    quat_t q = { w * s ,
                 v * s };
    return q;
}

// Scalar division
quat_t quat_t::operator / ( const float s ) {
    quat_t q = { w / s , 
                 v / s };
    return q;
}

// Self scalar multiply
void quat_t::operator *= ( const float s ) {
    w *= s;
    v *= s;   
}

// Self scalar divide
void quat_t::operator /= ( const float s ) {
    w /= s;
    v /= s;
}

//-- Quaternion multiplication and division:
      
// Multiplication
quat_t quat_t::operator * ( const quat_t &r ) {
    quat_t q = { w*r.w - v.dot(r.v)           ,
                 w*r.v + v*r.w + v.cross(r.v) };  
    return q;
}
 
// Division
quat_t quat_t::operator / ( quat_t &r ) {
    quat_t q = { w, v };
    return ( q*r.conj() )/r.inner();  
}

// Self multiply 
void quat_t::operator *= ( const quat_t &r ) {
    quat_t q = { w, v };
    q = q*r; 
    w = q.w;
    v = q.v;
}

// Self divide
void quat_t::operator /= ( quat_t &r ) {
    quat_t q = { w, v };
    q = q/r;
    w = q.w;
    v = q.v;
}

//-- Global operators:

// Vector multiplication - Result is quaternion
quat_t operator * ( vec3_t &v, vec3_t &r ) {
    quat_t q = { -v.dot(r)  ,
               v.cross(r) };
    return q;
}

// Reverse order - Scalar product
quat_t operator * ( const float s, quat_t &r ) {
    quat_t q = { r.w * s ,
               r.v * s };
    return q;
}

//--------------- Important operations ---------------

// Conjugate
quat_t quat_t::conj() {
    quat_t q = { w , 
                -v };
    return q;
}

// Inner product
float quat_t::inner() {
    return w*w + v.dot(v);  
}

// Magnitude
float quat_t::mag() {
    return sqrt( inner() );
}

// Normalize
quat_t quat_t::norm() { 
    quat_t q = { w, v };
    return q/mag();  
}

//--------------- Rotation transform -----------------

// Transform as axis and angle 
void quat_t::setRotation( vec3_t axis, float ang, const bool SMALL_ANG ) {
    ang *= 0.5;
    if( SMALL_ANG ) {
        w = 1 - ang*ang;
        v = ang * axis.norm(); 
    } else {
        w = cos(ang);
        v = sin(ang) * axis.norm();
    }
}

// Transform as vector with magnitude of sin(angle)
void quat_t::setRotation( vec3_t u, const bool SMALL_ANG ) {
    if( SMALL_ANG ) {
        v = 0.5 * u;
        w = 1 - 0.5*v.dot(v);      
    } else {
        float mag = u.dot(u);
        float sine = ( 1 - sqrt(1 - mag) )*0.5;  
        w = sqrt(1 - sine);
        v = sqrt(sine/mag) * u;
    }
}

// Rotate vector
vec3_t quat_t::rotate( vec3_t r, const bool TO_GLOBAL ) {
    float cross = 2*w;
    if( TO_GLOBAL ) {
        cross = -cross;
    }
    return ( w*w - v.dot(v) )*r + cross*v.cross(r) + ( 2*v.dot(r) )*v;
}

//-- Axis vector projections: 

vec3_t quat_t::axisX( const bool TO_GLOBAL ) {
    float w_vz = w*v.z;
    float w_vy = w*v.y;
    
    if( TO_GLOBAL ) {
      w_vz = -w_vz;
      w_vy = -w_vy;    
    }
    vec3_t u = { 2*( v.x*v.x + w*w  ) - 1 , 
                 2*( v.x*v.y + w_vz )     ,
                 2*( v.x*v.z - w_vy )     };
    return u;
}

vec3_t quat_t::axisY( const bool TO_GLOBAL ) {
    float w_vz = w*v.z;
    float w_vx = w*v.x;
      
    if( TO_GLOBAL ) {
      w_vz = -w_vz;
      w_vx = -w_vx;    
    }
    vec3_t u = { 2*( v.y*v.x - w_vz )     ,
                 2*( v.y*v.y + w*w  ) - 1 , 
                 2*( v.y*v.z + w_vx )     };              
    return u;
}

vec3_t quat_t::axisZ( const bool TO_GLOBAL ) {
    float w_vy = w*v.y;
    float w_vx = w*v.x;
      
    if( TO_GLOBAL ) {
      w_vy = -w_vy;
      w_vx = -w_vx;    
    }
    vec3_t u = { 2*( v.z*v.x + w_vy )     ,
                 2*( v.z*v.y - w_vx )     ,
                 2*( v.z*v.z + w*w  ) - 1 };              
    return u;
}
