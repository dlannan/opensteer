// ----------------------------------------------------------------------------
//
//
// OpenSteer -- Steering Behaviors for Autonomous Characters
//
// Copyright (c) 2002-2005, Sony Computer Entertainment America
// Original author: Craig Reynolds <craig_reynolds@playstation.sony.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.
//
//
// ----------------------------------------------------------------------------
//
// Vec3: OpenSteer's generic type for 3d vectors
//
// This file defines the class Vec3, which is used throughout OpenSteer to
// manipulate 3d geometric data.  It includes standard vector operations (like
// vector addition, subtraction, scale, dot, cross...) and more idiosyncratic
// utility functions.
//
// When integrating OpenSteer into a preexisting 3d application, it may be
// important to use the 3d vector type of that application.  In that case Vec3
// can be changed to inherit from the preexisting application' vector type and
// to match the interface used by OpenSteer to the interface provided by the
// preexisting 3d vector type.
//
// 10-04-04 bk:  put everything into the OpenSteer namespace
// 03-26-03 cwr: created to replace for Hiranabe-san's execellent but larger
//               vecmath package (http://objectclub.esm.co.jp/vecmath/)
//
// ----------------------------------------------------------------------------


// ----------------------------------------------------------------------------
// Generic interpolation

function square( a ) {
    return a * a;
}

function interpolate (alpha, x0, x1) {
    return x0 + ((x1 - x0) * alpha);
}


function interpolateV (alpha, x0, x1) {
    return x0.add((x1.sub(x0)).mult(alpha));
}

function maxXXX ( x, y) {if (x > y) return x; else return y;}
function minXXX ( x, y) {if (x < y) return x; else return y;}


// ----------------------------------------------------------------------------
// Random number utilities


// Returns a float randomly distributed between 0 and 1

function frandom01() {
    return (Math.random());
}


// Returns a float randomly distributed between lowerBound and upperBound

function frandom2( lowerBound, upperBound ) {
    var diff = upperBound - lowerBound;
    return Math.floor((Math.random() * diff) + lowerBound); 
}


// ----------------------------------------------------------------------------
// Constrain a given value (x) to be between two (ordered) bounds: min
// and max.  Returns x if it is between the bounds, otherwise returns
// the nearer bound.

function clip( x, minv, maxv) {
    if (x < minv) return minv;
    if (x > maxv) return maxv;
    return x;
}

// ----------------------------------------------------------------------------
// remap a value specified relative to a pair of bounding values
// to the corresponding value relative to another pair of bounds.
// Inspired by (dyna:remap-interval y y0 y1 z0 z1)


function remapInterval (x, in0, in1, out0, out1) {
    // uninterpolate: what is x relative to the interval in0:in1?
    var relative = (x - in0) / (in1 - in0);

    // now interpolate between output interval based on relative x
    return interpolate(relative, out0, out1);
}


// Like remapInterval but the result is clipped to remain between
// out0 and out1
function remapIntervalClip(x, in0, in1, out0, out1) {
    // uninterpolate: what is x relative to the interval in0:in1?
    var relative = (x - in0) / (in1 - in0);

    // now interpolate between output interval based on relative x
    return interpolate(clip(relative, 0, 1), out0, out1);
}


// Like remapInterval but the result is clipped to remain between
// out0 and out1
function remapIntervalClipV (x, in0, in1, out0, out1) {
    // uninterpolate: what is x relative to the interval in0:in1?
    var relative = (x - in0) / (in1 - in0);

    // now interpolate between output interval based on relative x
    return interpolateV(clip(relative, 0.0, 1.0), out0, out1);
}


// ----------------------------------------------------------------------------
// classify a value relative to the interval between two bounds:
//     returns -1 when below the lower bound
//     returns  0 when between the bounds (inside the interval)
//     returns +1 when above the upper bound


function intervalComparison ( x, lowerBound, upperBound) {
    if (x < lowerBound) return -1;
    if (x > upperBound) return +1;
    return 0;
}

// ----------------------------------------------------------------------------


function scalarRandomWalk(initial, walkspeed, minv, maxv) {
    var next = initial + (((frandom01() * 2) - 1) * walkspeed);
    if (next < minv) return minv;
    if (next > maxv) return maxv;
    return next;
}

// ----------------------------------------------------------------------------

function blendIntoAccumulator( smoothRate, newValue, smoothedAccumulator ) {
    return interpolate( clip(smoothRate, 0.0, 1.0), smoothedAccumulator, newValue);
}

// ----------------------------------------------------------------------------

function blendIntoAccumulatorV( smoothRate, newValue, smoothedAccumulator ) {
    return interpolateV( clip(smoothRate, 0.0, 1.01), smoothedAccumulator, newValue);
}

// ----------------------------------------------------------------------------

function clamp( valueToClamp, minValue, maxValue) {

    if ( valueToClamp < minValue ) {
        return minValue;
    } else if ( valueToClamp > maxValue ) {
        return maxValue;
    }    
    return valueToClamp;
}

// ----------------------------------------------------------------------------

var Vec3 = function() {

    // ----------------------------------------- generic 3d vector operations
    // three-dimensional Cartesian coordinates
    this.x = 0.0;
    this.y = 0.0;
    this.z = 0.0;

    // vector addition
    this.add = function(v) {
        var res = Vec3Set( this.x+v.x, this.y+v.y, this.z+v.z );
        return res;
    };

    // vector subtraction
    this.sub = function(v) {
        var res = Vec3Set( this.x-v.x, this.y-v.y, this.z-v.z );
        return res;
    };

    // unary minus
    this.neg = function() {
        var res = Vec3Set( -this.x, -this.y, -this.z );
        return res;
    };
    
    // vector times scalar product (scale length of vector times argument)
    this.mult = function(s) {
        var res = Vec3Set( this.x*s, this.y*s, this.z*s );
        return res;
    };

    // vector divided by a scalar (divide length of vector by argument)
    this.div = function(s) {
        var res = Vec3Set( this.x/s, this.y/s, this.z/s );
        return res;
    };

    // dot product
    this.dot = function( v ) {
        return (this.x * v.x) + (this.y * v.y) + (this.z * v.z);
    };

    // length
    this.length = function() {
        return Math.sqrt( this.lengthSquared() );
    };

    // length squared
    this.lengthSquared = function() {
        return this.dot(this);
    };

    // normalize: returns normalized version (parallel to this, length = 1)
    this.normalize = function() {
        // skip divide if length is zero
        var len = this.length();
        return (len>0)? this.div(len) : (this);
    };

    // cross product (modify "*this" to be A x B)
    // [XXX  side effecting -- deprecate this function?  XXX]
    this.cross = function ( a, b ) {
        this.set((a.y * b.z) - (a.z * b.y),
                        (a.z * b.x) - (a.x * b.z),
                        (a.x * b.y) - (a.y * b.x));
    };

    // assignment
    this.setV = function(v) {
        this.x=v.x; this.y=v.y; this.z=v.z; 
        return this;
    };

    // set XYZ coordinates to given three floats
    this.set = function( _x, _y, _z) {
        this.x = _x; this.y = _y; this.z = _z; 
    };

    // +=
    this.addV = function(v) {
        this.add(v)
        return this;
    };

    // -=
    this.subV = function(v) {
        this.sub(v);
        return this; 
    };

    // *=
    this.multV = function(s) {
        this.mult(s);
        return this;
    };
    
    this.divV = function(d) { 
        this.div(d);
        return this;  
    };
    
    // equality/inequality
    this.eq = function(v) { return (this.x===v.x) && (this.y===v.y) && (this.z===v.z); }
    this.neq = function(v) { return !(this.eq(v)); }

    // --------------------------- utility member functions used in OpenSteer

    // return component of vector parallel to a unit basis vector
    // (IMPORTANT NOTE: assumes "basis" has unit magnitude (length==1))
    this.parallelComponent = function( unitBasis ) {
        projection = this.dot ( unitBasis );
        return unitBasis.mult( projection );
    };

    // return component of vector perpendicular to a unit basis vector
    // (IMPORTANT NOTE: assumes "basis" has unit magnitude (length==1))

    this.perpendicularComponent = function( unitBasis ) {
        return this.sub( this.parallelComponent(unitBasis) );
    };

    // clamps the length of a given vector to maxLength.  If the vector is
    // shorter its value is returned unaltered, if the vector is longer
    // the value returned has length of maxLength and is paralle to the
    // original input.
    this.truncateLength = function( maxLength)  {

        var maxLengthSquared = maxLength * maxLength;
        var vecLengthSquared = this.lengthSquared();
        if (vecLengthSquared <= maxLengthSquared)
            return this;
        else
            return this.mult(maxLength / Math.sqrt(vecLengthSquared));
    };

    // forces a 3d position onto the XZ (aka y=0) plane
    this.setYtoZero = function() { this.y = 0.0; }

    // rotate this vector about the global Y (up) axis by the given angle
    this.rotateAboutGlobalY = function(angle) {
        var s = Math.sin(angle);
        var c = Math.cos(angle);
        return Vec3Set((this.x * c) + (this.z * s), (this.y), (this.z * c) - (this.x * s));
    };

    // version for caching sin/cos computation
    this.rotateAboutGlobalY = function( angle, sin, cos) {
        // is both are zero, they have not be initialized yet
        if (sin==0 && cos==0) {
            sin = Math.sin(angle);
            cos = Math.cos(angle);
        }
        return Vec3Set ((this.x * cos) + (this.z * sin), (this.y), (this.z * cos) - (this.x * sin));
    };

    // if this position is outside sphere, push it back in by one diameter
    this.sphericalWrapAround = function( center, radius ) {
        var offset = this.sub( center );
        var r = offset.length();
        if (r > radius)
            return this.add((offset.div(r)).mult(radius * -2));
        else
            return this;
    };
};


function Vec3Set( X, Y, Z ) {

    var newVec = new Vec3();
    newVec.set(X, Y, Z);
    return newVec;
};

// ----------------------------------------------------------------------------
// names for frequently used vector constants
Vec3.zero = Vec3Set(0, 0, 0);
Vec3.side = Vec3Set(-1, 0, 0);
Vec3.up = Vec3Set(0, 1, 0);
Vec3.forward = Vec3Set(0, 0, 1);

// @todo Remove - use @c distance from the Vec3Utilitites header instead.
// XXX experimental (4-1-03 cwr): is this the right approach?  defining
// XXX "Vec3 distance (vec3, Vec3)" collided with STL's distance template.
Vec3.distance = function( a, b ) { return(a.sub(b)).length(); }


// return cross product a x b
function crossProduct(a, b) {
    
    var result = Vec3Set((a.y * b.z) - (a.z * b.y),
                (a.z * b.x) - (a.x * b.z),
                (a.x * b.y) - (a.y * b.x));
    return result;
}

// ----------------------------------------------------------------------------
// Returns a position randomly distributed inside a sphere of unit radius
// centered at the origin.  Orientation will be random and length will range
// between 0 and 1

function RandomVectorInUnitRadiusSphere () {
    var v = new Vec3();
    do {
        v.set((frandom01()*2) - 1, (frandom01()*2) - 1, (frandom01()*2) - 1);
    } while (v.length() >= 1);
    return v;
}


// ----------------------------------------------------------------------------
// Returns a position randomly distributed on a disk of unit radius
// on the XZ (Y=0) plane, centered at the origin.  Orientation will be
// random and length will range between 0 and 1

function randomVectorOnUnitRadiusXZDisk() {
    var v = new Vec3();
    do {
        v.set((frandom01()*2) - 1,  0,  (frandom01()*2) - 1);
    }
    while (v.length() >= 1);
    return v;
}


// ----------------------------------------------------------------------------
// Returns a position randomly distributed on the surface of a sphere
// of unit radius centered at the origin.  Orientation will be random
// and length will be 1
function RandomUnitVector () {
    var v = RandomVectorInUnitRadiusSphere();
    return v.normalize();
}


// ----------------------------------------------------------------------------
// Returns a position randomly distributed on a circle of unit radius
// on the XZ (Y=0) plane, centered at the origin.  Orientation will be
// random and length will be 1
function RandomUnitVectorOnXZPlane () {
    var v = RandomVectorInUnitRadiusSphere();
    v.setYtoZero();
    return v.normalize();
}


// ----------------------------------------------------------------------------
// used by limitMaxDeviationAngle / limitMinDeviationAngle below
function vecLimitDeviationAngleUtility (insideOrOutside, source, cosineOfConeAngle, basis) {
    
    // immediately return zero length input vectors
    var sourceLength = source.length();
    if (sourceLength == 0) return source;

    // measure the angular diviation of "source" from "basis"
    var direction = source.div(sourceLength);
    var cosineOfSourceAngle = direction.dot(basis);

    // Simply return "source" if it already meets the angle criteria.
    // (note: we hope this top "if" gets compiled out since the flag
    // is a constant when the function is inlined into its caller)
    if (insideOrOutside) {
        // source vector is already inside the cone, just return it
        if (cosineOfSourceAngle >= cosineOfConeAngle) return source;
    }
    else {
        // source vector is already outside the cone, just return it
        if (cosineOfSourceAngle <= cosineOfConeAngle) return source;
    }

    // find the portion of "source" that is perpendicular to "basis"
    var perp = source.perpendicularComponent(basis);

    // normalize that perpendicular
    var unitPerp = perp.normalize();

    // construct a new vector whose length equals the source vector,
    // and lies on the intersection of a plane (formed the source and
    // basis vectors) and a cone (whose axis is "basis" and whose
    // angle corresponds to cosineOfConeAngle)
    var perpDist = Math.sqrt(1 - (cosineOfConeAngle * cosineOfConeAngle));
    var c0 = basis.mult( cosineOfConeAngle );
    var c1 = unitPerp.mult( perpDist );
    return (c0.add(c1)).mult( sourceLength );
}


// ----------------------------------------------------------------------------
// Enforce an upper bound on the angle by which a given arbitrary vector
// diviates from a given reference direction (specified by a unit basis
// vector).  The effect is to clip the "source" vector to be inside a cone
// defined by the basis and an angle.
function limitMaxDeviationAngle ( source, cosineOfConeAngle, basis) {
    return vecLimitDeviationAngleUtility (true, source, cosineOfConeAngle, basis);
}


// ----------------------------------------------------------------------------
// Enforce a lower bound on the angle by which a given arbitrary vector
// diviates from a given reference direction (specified by a unit basis
// vector).  The effect is to clip the "source" vector to be outside a cone
// defined by the basis and an angle.


function limitMinDeviationAngle ( source, cosineOfConeAngle, basis) {    
    return vecLimitDeviationAngleUtility (false, source, cosineOfConeAngle, basis);
}


// ----------------------------------------------------------------------------
// Returns the distance between a point and a line.  The line is defined in
// terms of a point on the line ("lineOrigin") and a UNIT vector parallel to
// the line ("lineUnitTangent")


function distanceFromLine (point, lineOrigin, lineUnitTangent) {
    var offset = point.sub( lineOrigin );
    var perp = offset.perpendicularComponent (lineUnitTangent);
    return perp.length();
}


// ----------------------------------------------------------------------------
// given a vector, return a vector perpendicular to it (note that this
// arbitrarily selects one of the infinitude of perpendicular vectors)
this.findPerpendicularIn3d = function(direction) {
    // to be filled in:
    var quasiPerp = Vec3();  // a direction which is "almost perpendicular"
    var result = Vec3();     // the computed perpendicular to be returned

    // three mutually perpendicular basis vectors
    var i = Vec3Set(1, 0, 0);
    var j = Vec3Set(0, 1, 0);
    var k = Vec3Set(0, 0, 1);

    // measure the projection of "direction" onto each of the axes
    var id = i.dot (direction);
    var jd = j.dot (direction);
    var kd = k.dot (direction);

    // set quasiPerp to the basis which is least parallel to "direction"
    if ((id <= jd) && (id <= kd)) {
        quasiPerp = i;               // projection onto i was the smallest
    }
    else {
        if ((jd <= id) && (jd <= kd))
            quasiPerp = j;           // projection onto j was the smallest
        else
            quasiPerp = k;           // projection onto k was the smallest
    }

    // return the cross product (direction x quasiPerp)
    // which is guaranteed to be perpendicular to both of them
    result.cross(direction, quasiPerp);
    return result;
}


function nearestPointOnSegment( point, segmentPoint0, segmentPoint1 ) {
    // convert the test point to be "local" to ep0
    var local = new Vec3();
    local.setV( point.sub( segmentPoint0 ));
    
    // find the projection of "local" onto "segmentNormal"
    var segment = new Vec3();
    segment.setV( segmentPoint1.sub( segmentPoint0 ));
    var segmentLength = segment.length();
    
    //assert( 0 != segmentLength && "Segment mustn't be of length zero." );
    
    var segmentNormalized = new Vec3();
    segmentNormalized.setV( segment.div(segmentLength) ); 
    var segmentProjection = segmentNormalized.dot(local);
    
    segmentProjection = clamp( segmentProjection, 0.0, segmentLength );
    
    var result = new Vec3();
    result.setV( segmentNormalized.mult( segmentProjection ));
    result =  result.add(segmentPoint0);
    return result;    
}

function pointToSegmentDistance ( point, segmentPoint0, segmentPoint1) {
    return Vec3.distance( point, nearestPointOnSegment( point, segmentPoint0, segmentPoint1 ) );
}

// ----------------------------------------------------------------------------
// candidates for global utility functions
//
// dot
// cross
// length
// distance
// normalized

    
// ----------------------------------------------------------------------------

