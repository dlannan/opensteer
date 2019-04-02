// ----------------------------------------------------------------------------
// Obstacle: a pure virtual base class for an abstract shape in space, to be
// used with obstacle avoidance.
//
// XXX this should define generic methods for querying the obstacle shape

// ----------------------------------------------------------------------------
// SphericalObstacle a simple concrete type of obstacle

function SphericalObstacle1 ( r,  c ) {
    var so = new SphericalObstacle();
    so.center = c;
    so.radius = r;
    return so;
}

var SphericalObstacle = function() {

    this.radius = 1.0;
    this.center = new Vec3();
    this._seenFrom = undefined;

    // constructors
    this.seenFrom = function() {return _seenFrom;}
    this.setSeenFrom = function( s) { this._seenFrom = s;}

    // XXX 4-23-03: Temporary work around (see comment above)
    //
    // Checks for intersection of the given spherical obstacle with a
    // volume of "likely future vehicle positions": a cylinder along the
    // current path, extending minTimeToCollision seconds along the
    // forward axis from current position.
    //
    // If they intersect, a collision is imminent and this function returns
    // a steering force pointing laterally away from the obstacle's center.
    //
    // Returns a zero vector if the obstacle is outside the cylinder
    //
    // xxx couldn't this be made more compact using localizePosition?

    this.steerToAvoid = function( v, minTimeToCollision) {

        // minimum distance to obstacle before avoidance is required
        var minDistanceToCollision = minTimeToCollision * v.speed();
        var minDistanceToCenter = minDistanceToCollision + this.radius;

        // contact distance: sum of radii of obstacle and vehicle
        var totalRadius = this.radius + v.radius();

        // obstacle center relative to vehicle position
        var localOffset = this.center.sub( v.position() );

        // distance along vehicle's forward axis to obstacle's center
        var forwardComponent = localOffset.dot (v.forward());
        var forwardOffset = forwardComponent.mult( v.forward());

        // offset from forward axis to obstacle's center
        var offForwardOffset = localOffset.sub(forwardOffset);

        // test to see if sphere overlaps with obstacle-free corridor
        var inCylinder = offForwardOffset.length() < totalRadius;
        var nearby = forwardComponent < minDistanceToCenter;
        var inFront = forwardComponent > 0;

        // if all three conditions are met, steer away from sphere center
        if (inCylinder && nearby && inFront) {
            return offForwardOffset.mult( -1 );
        } else {
            return Vec3.zero;
        }
    }
};
