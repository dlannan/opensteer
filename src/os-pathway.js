

// construct a PolylinePathway given the number of points (vertices),
// an array of points, and a path radius.
function PolylinePathway1 (_pointCount, _points, _radius, _cyclic) {
    var pw = new PolylinePathway();
    pw.initialize (_pointCount, _points, _radius, _cyclic);
    return pw;
}


// ----------------------------------------------------------------------------
// PolylinePathway: a simple implementation of the Pathway protocol.  The path
// is a "polyline" a series of line segments between specified points.  A
// radius defines a volume for the path which is the union of a sphere at each
// point and a cylinder along each segment.

var PolylinePathway = function() {

    // xxx shouldn't these 5 just be local variables?
    // xxx or are they used to pass secret messages between calls?
    // xxx seems like a bad design
    this.segmentLength = 0.0;
    this.segmentProjection = 0.0;
    this.local = new Vec3();
    this.chosen = new Vec3();
    this.segmentNormal = new Vec3();

    this.lengths = [];
    this.normals = [];
    this.totalPathLength = 0.0;

    // is the given point inside the path tube?
    this.isInsidePath = function(point) {
        var outside; 
        var tangent = new Vec3();
        var res = mapPointToPath(point, tangent, outside);
        return res.outside < 0.0;
    }

    // how far outside path tube is the given point?  (negative is inside)
    this.howFarOutsidePath = function(point) {
        var outside; 
        var tangent = new Vec3();
        var res = mapPointToPath(point, tangent, outside);
        return res.outside;
    }

    this.pointCount = 0;
    this.points = [];
    this.radius = 0.0;
    this.cyclic = false;


    // utility for constructors in derived classes
    this.initialize = function( _pointCount, _points, _radius, _cyclic) {
        // set data members, allocate arrays
        this.radius = _radius;
        this.cyclic = _cyclic;
        this.pointCount = _pointCount;
        this.totalPathLength = 0.0;
        if (this.cyclic) this.pointCount++;
        this.lengths = [];
        this.points  = [];
        this.normals = [];

        // loop over all points
        for (var i = 0; i < this.pointCount; i++)
        {
            // copy in point locations, closing cycle when appropriate
            var closeCycle = this.cyclic && (i == this.pointCount-1);
            var j = closeCycle ? 0 : i;
            this.points[i] = _points[j];

            // for the end of each segment
            if (i > 0) {
                // compute the segment length
                this.normals[i] = this.points[i].sub( this.points[i-1] );
                this.lengths[i] = this.normals[i].length();

                // find the normalized vector parallel to the segment
                this.normals[i] = this.normals[i].mult(1.0 / this.lengths[i]);

                // keep running total of segment lengths
                this.totalPathLength = this.totalPathLength + this.lengths[i];
            }
        }
    }

    // Given an arbitrary point ("A"), returns the nearest point ("P") on
    // this path.  Also returns, via output arguments, the path tangent at
    // P and a measure of how far A is outside the Pathway's "tube".  Note
    // that a negative distance indicates A is inside the Pathway.
    this.mapPointToPath = function( point, tangent, outside) {

        var d;
        var minDistance = Number.MAX_VALUE;
        var onPath = new Vec3();
    
        // loop over all segments, find the one nearest to the given point
        for (var i = 1; i < this.pointCount; i++) {
            this.segmentLength = this.lengths[i];
            this.segmentNormal = this.normals[i];
            d = this.pointToSegmentDistance(point, this.points[i-1], this.points[i]);
            if (d < minDistance) {
                minDistance = d;
                onPath = this.chosen;
                tangent = this.segmentNormal;
            }
        }
    
        // measure how far original point is outside the Pathway's "tube"
        outside = Vec3.distance(onPath, point) - this.radius;
        var res = { onPath:onPath, tangent:tangent, outside:outside };
        // return point on path
        return res;
    }


    // given an arbitrary point, convert it to a distance along the path
    this.mapPointToPathDistance = function(point) {
        var d;
        var minDistance = Number.MAX_VALUE;
        var segmentLengthTotal = 0.0;
        var pathDistance = 0.0;
    
        for (var i = 1; i < this.pointCount; i++) {
            this.segmentLength = this.lengths[i];
            this.segmentNormal = this.normals[i];
            d = this.pointToSegmentDistance(point, this.points[i-1], this.points[i]);
            if (d < minDistance) {
                minDistance = d;
                pathDistance = segmentLengthTotal + this.segmentProjection;
            }
            segmentLengthTotal = segmentLengthTotal + this.segmentLength;
        }
    
        // return distance along path of onPath point
        return pathDistance;
    }

    // given a distance along the path, convert it to a point on the path
    this.mapPathDistanceToPoint = function(pathDistance) {
        
        // clip or wrap given path distance according to cyclic flag
        var remaining = pathDistance;
        if (this.cyclic) {
            remaining = (pathDistance % totalPathLength);
        } else {
            if (pathDistance < 0.0) return this.points[0];
            if (pathDistance >= this.totalPathLength) return this.points[this.pointCount-1];
        }

        // step through segments, subtracting off segment lengths until
        // locating the segment that contains the original pathDistance.
        // Interpolate along that segment to find 3d point value to return.
        var result = new Vec3();
        for (var i = 1; i < this.pointCount; i++) {
            this.segmentLength = this.lengths[i];
            if (this.segmentLength < remaining) {
                remaining = remaining - this.segmentLength;
            } else {
                var ratio = remaining / this.segmentLength;
                result = interpolateV(ratio, this.points[i-1], this.points[i]);
                break;
            }
        }
        return result;
    }

    // utility methods

    // compute minimum distance from a point to a line segment
    this.pointToSegmentDistance = function( point, ep0, ep1) {

        // convert the test point to be "local" to ep0
        this.local = point.sub( ep0 );

        // find the projection of "local" onto "segmentNormal"
        this.segmentProjection = this.segmentNormal.dot(this.local);

        // handle boundary cases: when projection is not on segment, the
        // nearest point is one of the endpoints of the segment
        if (this.segmentProjection < 0.0)  {
            this.chosen = ep0;
            this.segmentProjection = 0;
            return Vec3.distance(point, ep0);
        }
        if (this.segmentProjection > this.segmentLength)
        {
            this.chosen = ep1;
            this.segmentProjection = this.segmentLength;
            return Vec3.distance(point, ep1);
        }

        // otherwise nearest point is projection point on segment
        this.chosen = this.segmentNormal.mult( this.segmentProjection);
        this.chosen = this.chosen.add( ep0 );
        return Vec3.distance (point, this.chosen);
    }

    // assessor for total path length;
    this.getTotalPathLength = function() { return this.totalPathLength; };
};

