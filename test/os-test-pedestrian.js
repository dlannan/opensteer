

// Test the OpenSteer pedestrian demo.
//    - start testing as soon as document loaded. Include this to test.

var gTestPath = undefined;

var gObstacle1 = new SphericalObstacle();
var gObstacle2 = new SphericalObstacle();
var gObstacles = [];

var gEndpoint0 = new Vec3();
var gEndpoint1 = new Vec3();
var gUseDirectedPathFollowing = true;

// this was added for debugging tool, but I might as well leave it in
var gWanderSwitch = false;


var Pedestrian = function( pd ) {

    // allocate one and share amoung instances just to save memory usage
    // (change to per-instance allocation to be more MP-safe)
    this.neighbors = [];

    // path to be followed by this pedestrian
    // XXX Ideally this should be a generic Pathway, but we use the
    // XXX getTotalPathLength and radius methods (currently defined only
    // XXX on PolylinePathway) to set random initial positions.  Could
    // XXX there be a "random position inside path" method on Pathway?
    this.path = undefined;

    // direction for path following (upstream or downstream)
    this.pathDirection = 1;

    // type for a group of Pedestrians
    //typedef std::vector<Pedestrian*> groupType;
    this.mover = new SimpleVehicle();

    // a pointer to this boid's interface object for the proximity database
    this.proximityToken = undefined;

    // switch to new proximity database -- just for demo purposes
    this.newToken = function( newpd ) {

        // delete this boid's token in the old proximity database
        if(this.proximityToken) delete this.proximityToken;

        // allocate a token for this boid in the proximity database
        this.proximityToken = newpd.allocateToken(this);
    }

    // destructor
    this.delete = function(){
        // delete this boid's token in the proximity database
        delete this.proximityToken;
    }

    // reset all instance state
    this.reset = function() {

        this.newToken( pd );

        // reset the vehicle 
        this.mover.reset ();

        // max speed and max steering force (maneuverability) 
        this.mover.setMaxSpeed(2.0);
        this.mover.setMaxForce(8.0);

        // initially stopped
        this.mover.setSpeed(0.0);

        // size of bounding sphere, for obstacle avoidance, etc.
        this.mover.setRadius(0.5); // width = 0.7, add 0.3 margin, take half

        // set the path for this Pedestrian to follow
        this.path = getTestPath();

        // set initial position
        // (random point on path + random horizontal offset)
        var d = this.path.getTotalPathLength() * frandom01();
        var r = this.path.radius;
        var randomOffset = randomVectorOnUnitRadiusXZDisk().mult( r );
        this.mover.setPosition(this.path.mapPathDistanceToPoint(d).add( randomOffset));

        // randomize 2D heading
        this.mover.randomizeHeadingOnXZPlane ();

        // pick a random direction for path following (upstream or downstream)
        this.pathDirection = (frandom01() > 0.5) ? -1 : +1;

        // notify proximity database that our position has changed
        this.proximityToken.updateForNewPosition( this.mover.position() );
    }

    // per frame simulation update
    this.update = function( currentTime, elapsedTime ) {
               
        var steer = this.determineCombinedSteering(elapsedTime);
        
        // apply steering force to our momentum
        this.mover.applySteeringForce(steer, elapsedTime);

        // reverse direction when we reach an endpoint
        if (gUseDirectedPathFollowing == true)  {

            if (Vec3.distance(this.mover.position(), gEndpoint0) < this.path.radius) {
                this.pathDirection = +1;
            }
            if (Vec3.distance(this.mover.position(), gEndpoint1) < this.path.radius) {
                this.pathDirection = -1;
            }
        }

        // notify proximity database that our position has changed
        this.proximityToken.updateForNewPosition( this.mover.position() );
    }

    // compute combined steering force: move forward, avoid obstacles
    // or neighbors if needed, otherwise follow the path and wander
    this.determineCombinedSteering = function(elapsedTime) {
        
        // move forward
        var steeringForce = this.mover.forward();

        // probability that a lower priority behavior will be given a
        // chance to "drive" even if a higher priority behavior might
        // otherwise be triggered.
        var leakThrough = 0.1;

        // determine if obstacle avoidance is required
        var obstacleAvoidance = Vec3.zero;
        if (leakThrough < frandom01()) {
            var oTime = 6.0; // minTimeToCollision = 6 seconds
            obstacleAvoidance = this.mover.steerToAvoidObstacles(oTime, gObstacles);
        }
        
        // if obstacle avoidance is needed, do it
        if (obstacleAvoidance.neq(Vec3.zero)) {
            steeringForce = steeringForce.add(obstacleAvoidance);
        }
        else {
            // otherwise consider avoiding collisions with others
            var collisionAvoidance = Vec3Set(0.0, 0.0, 0.0);
            var caLeadTime = 3.0;

            // find all neighbors within maxRadius using proximity database
            // (radius is largest distance between vehicles traveling head-on
            // where a collision is possible within caLeadTime seconds.)
            var maxRadius = caLeadTime * this.mover.maxSpeed() * 2.0;

            this.neighbors = this.proximityToken.findNeighbors(this.mover.position(), maxRadius);
            
            if ((leakThrough < frandom01()) && (this.neighbors.length > 0)) {
                collisionAvoidance = (this.mover.steerToAvoidNeighbors(caLeadTime, this.neighbors)).mult(10.0);
            }

            // if collision avoidance is needed, do it
            if (collisionAvoidance.neq(Vec3.zero)) {
                steeringForce = steeringForce.add(collisionAvoidance);
                drawVector(this.mover.position(), steeringForce, 5.0);
            }
            else {
                // add in wander component (according to user switch)
                if (gWanderSwitch == true) {
                    steeringForce = steeringForce.add( this.mover.steerForWander(elapsedTime));
                }

                // do (interactively) selected type of path following
                var pfLeadTime = 3.0;
                var pathFollow;
                if(gUseDirectedPathFollowing == true) {
                    pathFollow = this.mover.steerToFollowPath(this.pathDirection, pfLeadTime, this.path);
                } else {
                    pathFollow = this.mover.steerToStayOnPath(pfLeadTime, this.path);
                }

                // add in to steeringForce
                steeringForce = steeringForce.add(pathFollow.mult(0.5));
            }
        }

        // return steering constrained to global XZ "ground" plane
        steeringForce.setYtoZero();
        return steeringForce;
    }


    // draw this pedestrian into scene
    this.draw = function()
    {
    }

    this.reset();
};


function getTestPath () {
    if (gTestPath == undefined) {

        var pathRadius = 2.0;

        var pathPointCount = 7;
        var size = 30.0;
        var top = 2.0 * size;
        var gap = 1.2 * size;
        var out = 2.0 * size;
        var h = 0.5;
        var pathPoints = [
             new Vec3Set (h+gap-out,     0,0,  h+top-out),  // 0 a
             new Vec3Set (h+gap,         0.0,  h+top),      // 1 b
             new Vec3Set (h+gap+(top/2), 0.0,  h+top/2),    // 2 c
             new Vec3Set (h+gap,         0.0,  h),          // 3 d
             new Vec3Set (h,             0.0,  h),          // 4 e
             new Vec3Set (h,             0.0,  h+top),      // 5 f
             new Vec3Set (h+gap,         0.0,  h+top/2)     // 6 g
        ];

        gObstacle1.center = interpolateV (0.2, pathPoints[0], pathPoints[1]);
        gObstacle2.center = interpolateV (0.5, pathPoints[2], pathPoints[3]);
        gObstacle1.radius = 3.0;
        gObstacle2.radius = 5.0;
        gObstacles.push(gObstacle1);
        gObstacles.push(gObstacle2);

        gEndpoint0 = pathPoints[0];
        gEndpoint1 = pathPoints[pathPointCount-1];

        gTestPath = PolylinePathway1(pathPointCount, pathPoints, pathRadius, false);
        console.log("Path:", gTestPath);
    }
    return gTestPath;
};

function drawObstacle( ctx, x, z, scale, r) {

    ctx.beginPath();
    ctx.arc(x, z, scale * r, 0, Math.PI*2);
    ctx.stroke();
}

function drawPath(ctx, scale, xoff, yoff) {

    var ct = 0;
    var lastpt = undefined;

    var x = gObstacle1.center.x * scale + scale * xoff;
    var z = gObstacle1.center.z * scale + scale * yoff;
    drawObstacle(ctx, x,z, scale, gObstacle1.radius);
    x = gObstacle2.center.x * scale + scale * xoff;
    z = gObstacle2.center.z * scale + scale * yoff;
    drawObstacle(ctx, x,z, scale, gObstacle2.radius);

    ctx.beginPath();
    for (var idx in gTestPath.points) {
        var pt = gTestPath.points[idx];
        var x = pt.x * scale + scale * xoff;
        var z = pt.z * scale + scale * yoff;
        if(ct == 0) {
            ctx.moveTo(x, z);
        }
        if(ct > 0) {
            ctx.lineTo(x, z);
        }
        ct = ct+1
    }
    ctx.stroke();
}

var center = new Vec3();
var div = 20.0;
var divisions = Vec3Set(div, 1.0, div);
var diameter = 80.0; //XXX need better way to get this
var dimensions = Vec3Set(diameter, diameter, diameter);
var GPD = new LQProximityDatabase( center, dimensions, divisions);

var population = 0;
var crowd = [];
var selectedVehicle;

var character;
var oldTime;
var currentTime; 
var elapsedTime;

var ctx;
var scale = 10.0;
var xoff = 30.0;
var yoff = 10.0;

function drawTarget( x, z, sz ) {
    
    var x = x * scale + scale * xoff;
    var z = z * scale + scale * yoff;
    
    ctx.strokeStyle = "#FF0000";
    ctx.beginPath();
    ctx.moveTo(x-scale*sz, z);
    ctx.lineTo(x+scale*sz, z);
    ctx.moveTo(x, z-scale*sz);
    ctx.lineTo(x, z+scale*sz);
    ctx.stroke();
}

function drawVector( p, v, sz ) {
    
    var x = p.x * scale + scale * xoff;
    var z = p.z * scale + scale * yoff;
    var vx = (p.x + v.x * sz) * scale + scale * xoff;
    var vz = (p.z + v.z * sz) * scale + scale * yoff;
    
    ctx.strokeStyle = "#0000FF";
    ctx.beginPath();
    ctx.moveTo(x, z);
    ctx.lineTo(vx, vz);
    ctx.stroke();
}


function addPedestrianToCrowd() {

    population++;
    var pedestrian = new Pedestrian( GPD );
    crowd.push(pedestrian);
}

function pedestrianUpdater() {

    oldTime = currentTime;
    currentTime = Date.now() / 1000.0;
    elapsedTime = currentTime - oldTime;
    
    ctx.clearRect(0,0,1000,800);
    ctx.strokeStyle = "#000000";
    drawPath(ctx, scale, xoff, yoff);

    // update each Pedestrian
    for (var i = 0; i < crowd.length; i++) {
        crowd[i].update(currentTime, elapsedTime);
        var pos = crowd[i].mover.position();
        var fwd = crowd[i].mover.forward();

        ctx.beginPath();
        ctx.arc(pos.x * scale + scale * xoff, pos.z * scale + scale * yoff, scale, 0, Math.PI*2);
        ctx.fill();
    }
    setTimeout( pedestrianUpdater, 20 );
}

$(document).ready(function() {

    console.log("Testing OpenSteer Pedestrians.");
    console.log("LQDB:", GPD);

    var canvas = document.getElementById("pedcanvas");
    ctx = canvas.getContext("2d");

    // create the specified number of Pedestrians
    population = 0;
    for (var i = 0; i < 100; i++) {
        addPedestrianToCrowd();    
    }

    currentTime = Date.now() / 1000.0;
    pedestrianUpdater();
});