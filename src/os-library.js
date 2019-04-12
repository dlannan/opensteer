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
//
// SteerLibraryMixin
//
// This mixin (class with templated superclass) adds the "steering library"
// functionality to a given base class.  SteerLibraryMixin assumes its base
// class supports the AbstractVehicle interface.
//
// 10-04-04 bk:  put everything into the OpenSteer namespace
// 02-06-03 cwr: create mixin (from "SteerMass")
// 06-03-02 cwr: removed TS dependencies
// 11-21-01 cwr: created
//
//
// ----------------------------------------------------------------------------


var PathIntersection = function() {
    this.intersect = 0;
    this.distance = 0.0;
    this.surfacePoint = new Vec3();
    this.surfaceNormal = new Vec3();
    this.obstacle = SphericalObstacle();
} ;

// ----------------------------------------------------------------------------
// Assign SteerLibrary functions to a mover object

function SteerLibrary( mover ) {

    // initial state of wander behavior
    // Wander behavior
    mover.WanderSide = 0.0;
    mover.WanderUp = 0.0;

    // -------------------------------------------------- steering behaviors

    mover.steerForWander = function(dt) {

        // random walk WanderSide and WanderUp between -1 and +1
        var speed = 12.0 * dt; // maybe this (12) should be an argument?
        mover.WanderSide = scalarRandomWalk(mover.WanderSide, speed, -1, +1);
        mover.WanderUp   = scalarRandomWalk(mover.WanderUp,   speed, -1, +1);

        // return a pure lateral steering vector: (+/-Side) + (+/-Up)
        return (mover.side().mult(mover.WanderSide)).add(mover.up().mult(mover.WanderUp));
    }

    // Seek behavior
    mover.steerForSeek = function(target) {

        var desiredVelocity = target.sub( mover.position() );
        return desiredVelocity.sub( mover.velocity() );
    }

    // Flee behavior
    mover.steerForFlee = function(target) {

        var desiredVelocity = mover.position.sub(target);
        return desiredVelocity.sub( mover.velocity() );
    }

    // xxx proposed, experimental new seek/flee [cwr 9-16-02]
    mover.xxxsteerForFlee = function(target) {
        //  const Vec3 offset = position - target;
        var offset = mover.position().sub( target );
        var desiredVelocity = offset.truncateLength (mover.maxSpeed ()); //xxxnew
        return desiredVelocity.sub( mover.velocity() );
    }

    mover.xxxsteerForSeek = function(target) {
        //  const Vec3 offset = target - position;
        var offset = target.sub( mover.position() );
        var desiredVelocity = offset.truncateLength (mover.maxSpeed ()); //xxxnew
        return desiredVelocity.sub( mover.velocity() );
    }

    // Path Following behaviors
    mover.steerToFollowPath = function( direction, predictionTime, path) {

        // our goal will be offset from our path distance by this amount
        var pathDistanceOffset = direction * predictionTime * mover.speed();

        // predict our future position
        var futurePosition = mover.predictFuturePosition(predictionTime);

        // measure distance along path of our current and predicted positions
        var nowPathDistance = path.mapPointToPathDistance(mover.position());
        var futurePathDistance = path.mapPointToPathDistance(futurePosition);

        // are we facing in the correction direction?
        var rightway = ((pathDistanceOffset > 0.0) ? (nowPathDistance < futurePathDistance) : (nowPathDistance > futurePathDistance));

        // find the point on the path nearest the predicted future position
        // XXX need to improve calling sequence, maybe change to return a
        // XXX special path-defined object which includes two Vec3s and a 
        // XXX bool (onPath,tangent (ignored), withinPath)
        tangent = new Vec3();
        var outside = 0.0;
        var res = path.mapPointToPath(futurePosition, tangent, outside);

        // no steering is required if (a) our future position is inside
        // the path tube and (b) we are facing in the correct direction
        if ((res.outside < 0.0) && (rightway == true)) {
            // all is well, return zero steering
            return Vec3.zero;
        }
        else {
            // otherwise we need to steer towards a target point obtained
            // by adding pathDistanceOffset to our current path position

            var targetPathDistance = nowPathDistance + pathDistanceOffset;
            var target = path.mapPathDistanceToPoint(targetPathDistance);

            // return steering to seek target on path
            return mover.steerForSeek(target);
        }
    }

    mover.steerToStayOnPath = function(predictionTime, path) {
        // predict our future position
        var futurePosition = mover.predictFuturePosition(predictionTime);

        // find the point on the path nearest the predicted future position
        var tangent = new Vec3();
        var outside = 0.0;
        var res = path.mapPointToPath(futurePosition, tangent, outside); 

        if (res.outside < 0.0) {
            // our predicted future position was in the path,
            // return zero steering.
            return Vec3.zero;
        }
        else {
            // our predicted future position was outside the path, need to
            // steer towards it.  Use onPath projection of futurePosition
            // as seek target
            return mover.steerForSeek(res.onPath);
        }
    }

    // ------------------------------------------------------------------------
    // Obstacle Avoidance behavior
    //
    // Returns a steering force to avoid a given obstacle.  The purely
    // lateral steering force will turn our vehicle towards a silhouette edge
    // of the obstacle.  Avoidance is required when (1) the obstacle
    // intersects the vehicle's current path, (2) it is in front of the
    // vehicle, and (3) is within minTimeToCollision seconds of travel at the
    // vehicle's current velocity.  Returns a zero vector value (Vec3::zero)
    // when no avoidance is required.


    mover.steerToAvoidObstacle = function( minTimeToCollision, obstacle) {
        var avoidance = obstacle.steerToAvoid(mover, minTimeToCollision);    
        return avoidance;
    }

    // avoids all obstacles in an ObstacleGroup
    mover.steerToAvoidObstacles = function( minTimeToCollision, obstacles) {
        
        var avoidance = new Vec3();
        var nearest = new PathIntersection();
        var next = new PathIntersection();
        var minDistanceToCollision = minTimeToCollision * mover.speed();
    
        next.intersect = false;
        nearest.intersect = false;
    
        // test all obstacles for intersection with my forward axis,
        // select the one whose point of intersection is nearest
        for (var o = 0; o < obstacles.length; o++) {
            var obst = obstacles[o];
            // xxx this should be a generic call on Obstacle, rather than
            // xxx this code which presumes the obstacle is spherical
            next = mover.findNextIntersectionWithSphere(obst, next);
    
            if ((nearest.intersect == false) || ((next.intersect != false) && (next.distance < nearest.distance))) {
                nearest = next;
            }
        }
    
        // when a nearest intersection was found
        if ((nearest.intersect != false) && (nearest.distance < minDistanceToCollision)) {
            // compute avoidance steering force: take offset from obstacle to me,
            // take the component of that which is lateral (perpendicular to my
            // forward direction), set length to maxForce, add a bit of forward
            // component (in capture the flag, we never want to slow down)
            var offset = mover.position().sub( nearest.obstacle.center );
            avoidance = offset.perpendicularComponent(mover.forward());
            avoidance = avoidance.normalize();
            avoidance = avoidance.mult(mover.maxForce());
            avoidance = avoidance.add(mover.forward().mult( mover.maxForce () * 0.75) );
        }
    
        return avoidance;
    }


    // ------------------------------------------------------------------------
    // Unaligned collision avoidance behavior: avoid colliding with other
    // nearby vehicles moving in unconstrained directions.  Determine which
    // (if any) other other vehicle we would collide with first, then steers
    // to avoid the site of that potential collision.  Returns a steering
    // force vector, which is zero length if there is no impending collision.


    mover.steerToAvoidNeighbors = function( minTimeToCollision, others) {
        
        // first priority is to prevent immediate interpenetration
        var separation = this.steerToAvoidCloseNeighbors(0.0, others);
        if (separation.neq(Vec3.zero)) return separation;

        // otherwise, go on to consider potential future collisions
        var steer = 0;
        var threat = undefined;

        // Time (in seconds) until the most immediate collision threat found
        // so far.  Initial value is a threshold: don't look more than this
        // many frames into the future.
        var minTime = minTimeToCollision;

        // xxx solely for annotation
        var xxxThreatPositionAtNearestApproach = new Vec3();
        var xxxOurPositionAtNearestApproach = new Vec3();

        // for each of the other vehicles, determine which (if any)
        // pose the most immediate threat of collision.
        for (var i = 0; i < others.length; i++)
        {
            var other = others[i];
            if(other != mover) {	
                // avoid when future positions are this close (or less)
                var collisionDangerThreshold = mover.radius() * 2;

                // predicted time until nearest approach of "this" and "other"
                var time = mover.predictNearestApproachTime (other);

                // If the time is in the future, sooner than any other
                // threatened collision...
                if ((time >= 0) && (time < minTime)) {
                    // if the two will be close enough to collide,
                    // make a note of it
                    if (mover.computeNearestApproachPositions (other, time) < collisionDangerThreshold) {
                        minTime = time;
                        threat = other;
                        xxxThreatPositionAtNearestApproach = mover.hisPositionAtNearestApproach;
                        xxxOurPositionAtNearestApproach = mover.ourPositionAtNearestApproach;
                    }
                }
            }
        }

        // if a potential collision was found, compute steering to avoid
        if (threat) {
            // parallel: +1, perpendicular: 0, anti-parallel: -1
            var parallelness = mover.forward().dot(threat.mover.forward());
            var angle = 0.707;

            if (parallelness < -angle) {
                // anti-parallel "head on" paths:
                // steer away from future threat position
                var offset = xxxThreatPositionAtNearestApproach.sub( mover.position() );
                var sideDot = offset.dot(mover.side());
                steer = (sideDot > 0) ? -1.0 : 1.0;
            }
            else {
                if (parallelness > angle) {
                    // parallel paths: steer away from threat
                    var offset = threat.mover.position().sub( mover.position() );
                    var sideDot = offset.dot(mover.side());
                    steer = (sideDot > 0) ? -1.0 : 1.0;
                }
                else {
                    // perpendicular paths: steer behind threat
                    // (only the slower of the two does this)
                    if (threat.mover.speed() <= mover.speed())  {
                        var sideDot = mover.side().dot(threat.mover.velocity());
                        steer = (sideDot > 0) ? -1.0 : 1.0;
                    }
                }
            }
        }

        return mover.side().mult(steer);
    }


    // Given two vehicles, based on their current positions and velocities,
    // determine the time until nearest approach
    mover.predictNearestApproachTime = function(otherVehicle) {

        // imagine we are at the origin with no velocity,
        // compute the relative velocity of the other vehicle
        var myVelocity = mover.velocity();
        var otherVelocity = otherVehicle.mover.velocity();
        var relVelocity = otherVelocity.sub( myVelocity );
        var relSpeed = relVelocity.length();

        // for parallel paths, the vehicles will always be at the same distance,
        // so return 0 (aka "now") since "there is no time like the present"
        if (relSpeed == 0) return 0;

        // Now consider the path of the other vehicle in this relative
        // space, a line defined by the relative position and velocity.
        // The distance from the origin (our vehicle) to that line is
        // the nearest approach.

        // Take the unit tangent along the other vehicle's path
        var relTangent = relVelocity.div(relSpeed);

        // find distance from its path to origin (compute offset from
        // other to us, find length of projection onto path)
        var relPosition = mover.position().sub( otherVehicle.mover.position() );
        var projection = relTangent.dot(relPosition);

        return projection / relSpeed;
    }

    // Given the time until nearest approach (predictNearestApproachTime)
    // determine position of each vehicle at that time, and the distance
    // between them
    mover.computeNearestApproachPositions = function( otherVehicle, time) {

        var    myTravel = mover.forward().mult( mover.speed () * time );
        var otherTravel = otherVehicle.mover.forward().mult( otherVehicle.mover.speed() * time );
    
        var    myFinal = mover.position().add( myTravel );
        var otherFinal = otherVehicle.mover.position().add( otherTravel );
    
        // xxx for annotation
        mover.ourPositionAtNearestApproach = myFinal;
        mover.hisPositionAtNearestApproach = otherFinal;
    
        return Vec3.distance(myFinal, otherFinal);
    }

    /// XXX globals only for the sake of graphical annotation
    mover.hisPositionAtNearestApproach = new Vec3();
    mover.ourPositionAtNearestApproach = new Vec3();

    // ------------------------------------------------------------------------
    // avoidance of "close neighbors" -- used only by steerToAvoidNeighbors
    //
    // XXX  Does a hard steer away from any other agent who comes withing a
    // XXX  critical distance.  Ideally this should be replaced with a call
    // XXX  to steerForSeparation.

    mover.steerToAvoidCloseNeighbors = function( minSeparationDistance, others) {
        // for each of the other vehicles...
        for (var i = 0; i < others.length; i++) {
            var other = others[i];
            if (other !== mover)  {
                var sumOfRadii = mover.radius() + other.mover.radius();
                var minCenterToCenter = minSeparationDistance + sumOfRadii;
                var offset = other.mover.position().sub( mover.position() );
                var currentDistance = offset.length();

                if (currentDistance < minCenterToCenter) {
                    return (offset.neg()).perpendicularComponent(mover.forward());
                }
            }
        }

        // otherwise return zero
        return Vec3.zero;
    }


    // ------------------------------------------------------------------------
    // used by boid behaviors

    mover.inBoidNeighborhood = function( otherVehicle, minDistance, maxDistance, cosMaxAngle) {
        
        if (otherVehicle.mover === mover) {
            return false;
        }
        else {
            var offset = otherVehicle.mover.position().sub( mover.position() );
            var distanceSquared = offset.lengthSquared();
    
            // definitely in neighborhood if inside minDistance sphere
            if (distanceSquared < (minDistance * minDistance)) {
                return true;
            }
            else {
                // definitely not in neighborhood if outside maxDistance sphere
                if (distanceSquared > (maxDistance * maxDistance))  {
                    return false;
                }
                else {
                    // otherwise, test angular offset from forward axis
                    var unitOffset = offset.divV( Math.sqrt(distanceSquared) );
                    var forwardness = mover.forward().dot(unitOffset);
                    return forwardness > cosMaxAngle;
                }
            }
        }
    }

    // ------------------------------------------------------------------------
    // Separation behavior -- determines the direction away from nearby boids

    mover.steerForSeparation = function( maxDistance, cosMaxAngle, flock) {

        // steering accumulator and count of neighbors, both initially zero
        var steering = new Vec3();
        var neighbors = 0;

        // for each of the other vehicles...        
        for (var i=0; i<flock.length; i++) {
            var otherVehicle = flock[i];
            if (mover.inBoidNeighborhood (otherVehicle, mover.radius()*3, maxDistance, cosMaxAngle)) {
                // add in steering contribution
                // (opposite of the offset direction, divided once by distance
                // to normalize, divided another time to get 1/d falloff)
                var offset = otherVehicle.mover.position().sub( mover.position() );
                var distanceSquared = offset.dot(offset);
                steering = steering.add(offset.div(-distanceSquared));

                // count neighbors
                neighbors++;
            }
        }

        // divide by neighbors, then normalize to pure direction
        // bk: Why dividing if you normalize afterwards?
        //     As long as normilization tests for @c 0 we can just call normalize
        //     and safe the branching if.
        /*
        if (neighbors > 0) {
            steering /= neighbors;
            steering = steering.normalize();
        }
        */
        steering = steering.normalize();
        return steering;
    }


    // ------------------------------------------------------------------------
    // Alignment behavior

    mover.steerForAlignment = function( maxDistance, cosMaxAngle, flock) {

        // steering accumulator and count of neighbors, both initially zero
        var steering = new Vec3();
        var neighbors = 0;

        // for each of the other vehicles...
        for (var i=0; i<flock.length; i++) {
            var otherVehicle = flock[i];
            if (mover.inBoidNeighborhood (otherVehicle, mover.radius()*3, maxDistance, cosMaxAngle))  {
                // accumulate sum of neighbor's heading
                steering = steering.add(otherVehicle.mover.forward());
                // count neighbors
                neighbors++;
            }
        }

        // divide by neighbors, subtract off current heading to get error-
        // correcting direction, then normalize to pure direction
        if (neighbors > 0) steering = ((steering.div(neighbors)).sub(mover.forward())).normalize();
        return steering;
    }

    // ------------------------------------------------------------------------
    // Cohesion behavior

    mover.steerForCohesion = function( maxDistance, cosMaxAngle, flock) {

        // steering accumulator and count of neighbors, both initially zero
        var steering = new Vec3();
        var neighbors = 0;

        // for each of the other vehicles...
        for (var i=0; i<flock.length; i++) {
            var otherVehicle = flock[i];
            if (mover.inBoidNeighborhood (otherVehicle, mover.radius()*3, maxDistance, cosMaxAngle)) {
                // accumulate sum of neighbor's positions
                steering = steering.add(otherVehicle.mover.position());

                // count neighbors
                neighbors++;
            }
        }

        // divide by neighbors, subtract off current position to get error-
        // correcting direction, then normalize to pure direction
        if (neighbors > 0) steering = ((steering.div(neighbors)).sub(mover.forward())).normalize();

        return steering;
    }

    // ------------------------------------------------------------------------
    // pursuit of another vehicle (& version with ceiling on prediction time)

    mover.steerForPursuit = function( quarry) {
        return steerForPursuit (quarry, Number.MAX_VALUE);
    }

    mover.steerForPursuit = function( quarry, maxPredictionTime ) {
        
        // offset from this to quarry, that distance, unit vector toward quarry
        var offset = quarry.position().sub( mover.position() );
        var distance = offset.length();
        var unitOffset = offset.dinv( distance );

        // how parallel are the paths of "this" and the quarry
        // (1 means parallel, 0 is pependicular, -1 is anti-parallel)
        var parallelness = mover.forward().dot(quarry.forward());

        // how "forward" is the direction to the quarry
        // (1 means dead ahead, 0 is directly to the side, -1 is straight back)
        var forwardness = mover.forward().dot(unitOffset);

        var directTravelTime = distance / mover.speed();
        var f = intervalComparison(forwardness,  -0.707, 0.707);
        var p = intervalComparison(parallelness, -0.707, 0.707);

        var timeFactor = 0; // to be filled in below

        // Break the pursuit into nine cases, the cross product of the
        // quarry being [ahead, aside, or behind] us and heading
        // [parallel, perpendicular, or anti-parallel] to us.
        switch (f)
        {
        case +1:
            switch (p)
            {
            case +1:          // ahead, parallel
                timeFactor = 4;
                break;
            case 0:           // ahead, perpendicular
                timeFactor = 1.8;
                break;
            case -1:          // ahead, anti-parallel
                timeFactor = 0.85;
                break;
            }
            break;
        case 0:
            switch (p)
            {
            case +1:          // aside, parallel
                timeFactor = 1;
                break;
            case 0:           // aside, perpendicular
                timeFactor = 0.8;
                break;
            case -1:          // aside, anti-parallel
                timeFactor = 4;
                break;
            }
            break;
        case -1:
            switch (p)
            {
            case +1:          // behind, parallel
                timeFactor = 0.5;
                break;
            case 0:           // behind, perpendicular
                timeFactor = 2;
                break;
            case -1:          // behind, anti-parallel
                timeFactor = 2;
                break;
            }
            break;
        }

        // estimated time until intercept of quarry
        var et = directTravelTime * timeFactor;

        // xxx experiment, if kept, this limit should be an argument
        var etl = (et > maxPredictionTime) ? maxPredictionTime : et;

        // estimated position of quarry at intercept
        var target = quarry.predictFuturePosition(etl);

        // annotation
        //this->annotationLine (position(), target, gaudyPursuitAnnotation ? color : gGray40);

        return mover.steerForSeek(target);
    }

    // ------------------------------------------------------------------------
    // evasion of another vehicle

    mover.steerForEvasion = function( menace, maxPredictionTime) {

        // offset from this to menace, that distance, unit vector toward menace
        var offset = mover.menace.position.sub( mover.position );
        var distance = offset.length();

        var roughTime = distance / menace.speed();
        var predictionTime = ((roughTime > maxPredictionTime) ? maxPredictionTime : roughTime);

        var target = menace.predictFuturePosition(predictionTime);
        return steerForFlee (target);
    };

    // ------------------------------------------------------------------------
    // tries to maintain a given speed, returns a maxForce-clipped steering
    // force along the forward/backward axis

    mover.steerForTargetSpeed = function(targetSpeed) {
        var mf = mover.maxForce ();
        var speedError = targetSpeed - mover.speed();
        return mover.forward() * clip(speedError, -mf, +mf);    
    }

    mover.findNextIntersectionWithSphere = function( obs, intersection) {
        // xxx"SphericalObstacle& obs" should be "const SphericalObstacle&
        // obs" but then it won't let me store a pointer to in inside the
        // PathIntersection

        // This routine is based on the Paul Bourke's derivation in:
        //   Intersection of a Line and a Sphere (or circle)
        //   http://www.swin.edu.au/astronomy/pbourke/geometry/sphereline/

        var b, c, d, p, q, s;
        var lc = new Vec3();

        // initialize pathIntersection object
        intersection.intersect = false;
        intersection.obstacle = obs;

        // find "local center" (lc) of sphere in boid's coordinate space
        lc = mover.localizePosition (obs.center);

        // computer line-sphere intersection parameters
        b = -2 * lc.z;
        c = square (lc.x) + square (lc.y) + square (lc.z) - 
        square (obs.radius + mover.radius());
        d = (b * b) - (4 * c);

        // when the path does not intersect the sphere
        if (d < 0) return intersection;

        // otherwise, the path intersects the sphere in two points with
        // parametric coordinates of "p" and "q".
        // (If "d" is zero the two points are coincident, the path is tangent)
        s = Math.sqrt(d);
        p = (-b + s) / 2;
        q = (-b - s) / 2;

        // both intersections are behind us, so no potential collisions
        if ((p < 0) && (q < 0)) return intersection; 

        // at least one intersection is in front of us
        intersection.intersect = true;
        if((p > 0) && (q > 0)) {
            // both intersections are in front of us, find nearest one
            intersection.distance = ((p < q) ? p : q);
        } else {
            // otherwise only one intersections is in front, select it
            intersection.distance = ((p > 0) ? p : q);
        }
        return intersection;
    }

    // ----------------------------------------------------------- utilities
    // XXX these belong somewhere besides the steering library
    // XXX above AbstractVehicle, below SimpleVehicle
    // XXX ("utility vehicle"?)

    // xxx cwr experimental 9-9-02 -- names OK?
    mover.isAhead = function(target)  {return isAhead(target, 0.707); };
    mover.isAside = function(target)  {return isAside (target, 0.707); };
    mover.isBehind = function(target) {return isBehind (target, -0.707); };

    mover.isAhead = function(target, cosThreshold) {
        var targetDirection = target.sub( mover.position()).normalize();
        return mover.forward().dot(targetDirection) > cosThreshold;
    };
    mover.isAside = function(target, cosThreshold) {
        var targetDirection = target.sub( mover.position () ).normalize ();
        var dp = mover.forward().dot(targetDirection);
        return (dp < cosThreshold) && (dp > -cosThreshold);
    };
    mover.isBehind = function( target, cosThreshold) {
        var targetDirection = target.sub( mover.position()).normalize ();
        return mover.forward().dot(targetDirection) < cosThreshold;
    };
};


