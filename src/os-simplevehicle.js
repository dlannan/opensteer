
var serialNumberCounter = 0;

// SimpleVehicle adds concrete vehicle methods to SimpleVehicle_3
var SimpleVehicle = function() {

    // maintain unique serial numbers
    this.serialNumber = serialNumberCounter++;        

    this._mass = 0.0;       // mass (defaults to unity so acceleration=force)
    this._radius = 0.0;     // size of bounding sphere, for obstacle avoidance, etc.

    this._speed = 0.0;      // speed along Forward direction.  Because local space
                        // is velocity-aligned, velocity = Forward * Speed

    this._maxForce = 0.0;   // the maximum steering force this vehicle can apply
                        // (steering force is clipped to this magnitude)

    this._maxSpeed = 0.0;   // the maximum speed this vehicle is allowed to move
                        // (velocity is clipped to this magnitude)

    this._curvature = 0.0;
    this._lastForward = new Vec3();
    this._lastPosition = new Vec3();
    this._smoothedPosition = new Vec3();
    this._smoothedCurvature = 0.0;
    this._smoothedAcceleration = new Vec3();

    SteerLibrary(this);
    LocalSpace(this);

    // reset vehicle state
    this.reset = function() {
        // reset LocalSpace state
        this.resetLocalSpace(this.local);

        this.setMass(1.0);          // mass (defaults to 1 so acceleration=force)
        this.setSpeed(0.0);         // speed along Forward direction.

        this.setRadius(0.5);     // size of bounding sphere

        this.setMaxForce(0.1);   // steering force is clipped to this magnitude
        this.setMaxSpeed(1.0);   // velocity is clipped to this magnitude

        // reset bookkeeping to do running averages of these quanities
        this.resetSmoothedPosition();
        this.resetSmoothedCurvature();
        this.resetSmoothedAcceleration();
    }

    // get/set mass
    this.mass = function() {return this._mass;}
    this.setMass = function(m) { return this._mass = m; }

    // get velocity of vehicle
    this.velocity = function() { return this.forward().mult(this._speed); }

    // get/set speed of vehicle  (may be faster than taking mag of velocity)
    this.speed = function() { return this._speed; }
    this.setSpeed = function(s) { return this._speed = s; }

    // size of bounding sphere, for obstacle avoidance, etc.
    this.radius = function() { return this._radius; }
    this.setRadius = function(m) { return this._radius = m; }

    // get/set maxForce
    this.maxForce = function() {return this._maxForce;}
    this.setMaxForce = function(mf) {return this._maxForce = mf;}

    // get/set maxSpeed
    this.maxSpeed = function()  {return this._maxSpeed;}
    this.setMaxSpeed = function(ms) {return this._maxSpeed = ms;}

    // ratio of speed to max possible speed (0 slowest, 1 fastest)
    this.relativeSpeed = function() { return this.speed() / this.maxSpeed(); }

    // apply a given steering force to our momentum,
    // adjusting our orientation to maintain velocity-alignment.
    this.applySteeringForce = function( force, elapsedTime ){

        var adjustedForce = this.adjustRawSteeringForce(force, elapsedTime);

        // enforce limit on magnitude of steering force
        var clippedForce = adjustedForce.truncateLength(this.maxForce());
    
        // compute acceleration and velocity
        var newAcceleration = clippedForce.div( this.mass());
        var newVelocity = this.velocity();
    
        // damp out abrupt changes and oscillations in steering acceleration
        // (rate is proportional to time step, then clipped into useful range)
        if (elapsedTime > 0.0)  {
            var smoothRate = clip(9 * elapsedTime, 0.15, 0.4);
            this._smoothedAcceleration = blendIntoAccumulatorV(smoothRate, newAcceleration, this._smoothedAcceleration);
        }

        // Euler integrate (per frame) acceleration into velocity
        var accel = this._smoothedAcceleration.mult( elapsedTime );
        newVelocity = newVelocity.add( accel );
    
        // enforce speed limit
        newVelocity = newVelocity.truncateLength( this.maxSpeed ());

        // update Speed
        this.setSpeed(newVelocity.length());
    
        // Euler integrate (per frame) velocity into position
        this.setPosition(this.position().add(newVelocity.mult(elapsedTime)));
    
        // regenerate local space (by default: align vehicle's forward axis with
        // new velocity, but this behavior may be overridden by derived classes.)
        this.regenerateLocalSpace(newVelocity, elapsedTime);

        // maintain path curvature information
        this.measurePathCurvature(elapsedTime);
    
        // running average of recent positions
        this._smoothedPosition = blendIntoAccumulatorV(elapsedTime * 0.06, this.position(), this._smoothedPosition);
    }

    // the default version: keep FORWARD parallel to velocity, change
    // UP as little as possible.
    this.regenerateLocalSpace = function( newVelocity, elapsedTime) {
        // adjust orthonormal basis vectors to be aligned with new velocity
        if (this.speed() > 0.0) this.regenerateOrthonormalBasisUF(newVelocity.div(this.speed()));
    };

    // alternate version: keep FORWARD parallel to velocity, adjust UP
    // according to a no-basis-in-reality "banking" behavior, something
    // like what birds and airplanes do.  (XXX experimental cwr 6-5-03)
    this.regenerateLocalSpaceForBanking = function( newVelocity, elapsedTime ) {
        // the length of this global-upward-pointing vector controls the vehicle's
        // tendency to right itself as it is rolled over from turning acceleration
        var globalUp = Vec3Set(0, 0.2, 0);

        // acceleration points toward the center of local path curvature, the
        // length determines how much the vehicle will roll while turning
        var accelUp = this._smoothedAcceleration.mult( 0.05 );

        // combined banking, sum of UP due to turning and global UP
        var bankUp = accelUp.add( globalUp );

        // blend bankUp into vehicle's UP basis vector
        var smoothRate = elapsedTime * 3;
        var tempUp = this.up();
        tempUp = blendIntoAccumulatorV(smoothRate, bankUp, tempUp);
        this.local.setUp(tempUp.normalize());

    //  annotationLine (position(), position() + (globalUp * 4), gWhite);  // XXX
    //  annotationLine (position(), position() + (bankUp   * 4), gOrange); // XXX
    //  annotationLine (position(), position() + (accelUp  * 4), gRed);    // XXX
    //  annotationLine (position(), position() + (up ()    * 1), gYellow); // XXX

        // adjust orthonormal basis vectors to be aligned with new velocity
        if (this.speed() > 0) regenerateOrthonormalBasisUF (newVelocity.div(this.speed()));
    };

    // adjust the steering force passed to applySteeringForce.
    // allows a specific vehicle class to redefine this adjustment.
    // default is to disallow backward-facing steering at low speed.
    // xxx experimental 8-20-02
    this.adjustRawSteeringForce = function( force, deltaTime ) {

        var maxAdjustedSpeed = 0.2 * this.maxSpeed();

        if ((this.speed() > maxAdjustedSpeed) || force.eq(Vec3.zero) ) {
            return force;
        }
        else {
            var range = this.speed() / maxAdjustedSpeed;
            // const float cosine = interpolate (pow (range, 6), 1.0f, -1.0f);
            // const float cosine = interpolate (pow (range, 10), 1.0f, -1.0f);
            // const float cosine = interpolate (pow (range, 20), 1.0f, -1.0f);
            // const float cosine = interpolate (pow (range, 100), 1.0f, -1.0f);
            // const float cosine = interpolate (pow (range, 50), 1.0f, -1.0f);
            var cosine = interpolate(Math.pow (range, 20), 1.0, -1.0);
            return limitMaxDeviationAngle (force, cosine, this.forward());
        }        
    }

    // apply a given braking force (for a given dt) to our momentum.
    // xxx experimental 9-6-02
    this.applyBrakingForce = function( rate, deltaTime ) {
        var rawBraking = this.speed () * rate;
        var clipBraking = ((rawBraking < this.maxForce ()) ?rawBraking : this.maxForce());
        this.setSpeed (this.speed () - (clipBraking * deltaTime));
    }

    // predict position of this vehicle at some time in the future
    // (assumes velocity remains constant)
    this.predictFuturePosition = function( predictionTime ) {
        return this.position().add( this.velocity().mult(predictionTime) );
    }

    // get instantaneous curvature (since last update)
    this.curvature = function() { return this._curvature; }

    // get/reset smoothedCurvature, smoothedAcceleration and smoothedPosition
    this.smoothedCurvature = function() { return this._smoothedCurvature; };
    this.resetSmoothedCurvature = function( value = 0 ) {
        this._lastForward.setV( Vec3.zero );
        this._lastPosition.setV( Vec3.zero );
        return this._smoothedCurvature = this._curvature = value;
    }
    this.smoothedAcceleration = function() {return this._smoothedAcceleration;}
    this.resetSmoothedAcceleration = function(value) {
        if(!value) value = Vec3.zero;
        this._smoothedAcceleration.setV(value);
        return this._smoothedAcceleration;
    }
    this.smoothedPosition = function() { return this._smoothedPosition; }
    this.resetSmoothedPosition = function( value ) {
        if(!value) value = Vec3.zero;
        this._smoothedPosition.setV(value);
        return this._smoothedPosition;
    }

    // give each vehicle a unique number
    this.serialNumber = 0.0;

    // set a random "2D" heading: set local Up to global Y, then effectively
    // rotate about it by a random angle (pick random forward, derive side).
    this.randomizeHeadingOnXZPlane = function() {
        this.setUp(Vec3.up);
        this.setForward(RandomUnitVectorOnXZPlane());
        this.setSide(this.localRotateForwardToSide(this.forward()));
    }

    // measure path curvature (1/turning-radius), maintain smoothed version
    this.measurePathCurvature = function( elapsedTime ) {

        if (elapsedTime > 0) {
            var dP = this._lastPosition.sub( this.position() );
            var dF = (this._lastForward.sub( this.forward() )).div( dP.length() );
            var lateral = dF.perpendicularComponent( this.forward() );
            var sign = (lateral.dot(this.side()) < 0) ? 1.0 : -1.0;
            this._curvature = lateral.length() * sign;
            this._smoothedCurvature = blendIntoAccumulator(elapsedTime * 4.0, this._curvature, this._smoothedCurvature);
            this._lastForward = this.forward();
            this._lastPosition = this.position();
        }
    };  
};
