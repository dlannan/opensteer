// ----------------------------------------------------------------------------
//
//
// OpenSteer -- Steering Behaviors for Autonomous Characters
//
// Copyright (c) 2002-2003, Sony Computer Entertainment America
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
// OpenSteer Boids
// 
// 09-26-02 cwr: created 
//
//
// ----------------------------------------------------------------------------


var Boid = function(pd) {

    // allocate one and share amoung instances just to save memory usage
    // (change to per-instance allocation to be more MP-safe)
    this.neighbors = [];

    // allocate a token for this boid in the proximity database
    this.proximityToken = undefined;

    // type for a group of Pedestrians
    //typedef std::vector<Pedestrian*> groupType;
    this.mover = new SimpleVehicle();

    // destructor
    this.deleteBoid = function() {
        // delete this boid's token in the proximity database
        if(this.proximityToken) delete this.proximityToken;
    }

    // reset state
    this.reset = function() {
        // reset the vehicle
        this.mover.reset();

        // steering force is clipped to this magnitude
        this.mover.setMaxForce(27.0);

        // velocity is clipped to this magnitude
        this.mover.setMaxSpeed(9.0);

        // initial slow speed
        this.mover.setSpeed(this.mover.maxSpeed() * 0.6);

        // randomize initial orientation
        this.mover.regenerateOrthonormalBasisUF(RandomUnitVector());

        // randomize initial position
        this.mover.setPosition(RandomVectorInUnitRadiusSphere().mult(20.0) );

        // notify proximity database that our position has changed
        this.proximityToken.updateForNewPosition (this.mover.position());
    }

    // draw this boid into the scene
    this.draw = function() {
        // drawTrail ();
    }

    // per frame simulation update
    this.update = function(currentTime, elapsedTime) {
        // steer to flock and perhaps to stay within the spherical boundary
        this.mover.applySteeringForce (this.steerToFlock().add( this.handleBoundary()), elapsedTime);

        // notify proximity database that our position has changed
        this.proximityToken.updateForNewPosition (this.mover.position());
    }

    // basic flocking
    this.steerToFlock = function() {
        var separationRadius =  5.0;
        var separationAngle  = -0.707;
        var separationWeight =  12.0;

        var alignmentRadius = 7.5;
        var alignmentAngle  = 0.7;
        var alignmentWeight = 8.0;

        var cohesionRadius = 9.0;
        var cohesionAngle  = -0.15;
        var cohesionWeight = 8.0;

        var maxRadius = maxXXX(separationRadius, maxXXX (alignmentRadius, cohesionRadius));

        // find all flockmates within maxRadius using proximity database
        this.neighbors = this.proximityToken.findNeighbors(this.mover.position(), maxRadius);

        // determine each of the three component behaviors of flocking
        var separation = this.mover.steerForSeparation(separationRadius, separationAngle, this.neighbors);
        var alignment  = this.mover.steerForAlignment(alignmentRadius, alignmentAngle, this.neighbors);
        var cohesion   = this.mover.steerForCohesion(cohesionRadius, cohesionAngle, this.neighbors);

        // apply weights to components (save in variables for annotation)
        var separationW = separation.mult(separationWeight);
        var alignmentW = alignment.mult(alignmentWeight);
        var cohesionW = cohesion.mult(cohesionWeight);

        // annotation
        // const float s = 0.1;
        // annotationLine (position, position + (separationW * s), gRed);
        // annotationLine (position, position + (alignmentW  * s), gOrange);
        // annotationLine (position, position + (cohesionW   * s), gYellow);

        return separationW.add(alignmentW.add(cohesionW));
    }


    // Take action to stay within sphereical boundary.  Returns steering
    // value (which is normally zero) and may take other side-effecting
    // actions such as kinematically changing the Boid's position.
    this.handleBoundary = function() {
        // while inside the sphere do noting
        if (this.mover.position().length() < Boid.worldRadius) return Vec3.zero;

        // once outside, select strategy
        if(Boid.boundaryCondition === 0) {
            // steer back when outside
            var seek = this.mover.xxxsteerForSeek(Vec3.zero);
            var lateral = seek.perpendicularComponent(this.mover.forward ());
            return lateral;
        }
        if(Boid.boundaryCondition === 1) {
            // wrap around (teleport)
            this.mover.setPosition(this.mover.position().sphericalWrapAround (Vec3.zero, Boid.worldRadius));
            return Vec3.zero;
        }
        return Vec3.zero; // should not reach here
    }


    // make boids "bank" as they fly
    this.regenerateLocalSpace = function(newVelocity, elapsedTime) {
        this.mover.regenerateLocalSpaceForBanking (newVelocity, elapsedTime);
    }

    // switch to new proximity database -- just for demo purposes
    this.newPD = function(pd) {
        // delete this boid's token in the old proximity database
        delete this.proximityToken;

        // allocate a token for this boid in the proximity database
        this.proximityToken = pd.allocateToken(this);
    }

    // cycle through various boundary conditions
    this.nextBoundaryCondition = function() {
        var max = 2;
        Boid.boundaryCondition = (Boid.boundaryCondition + 1) % max;
    }

    this.newPD(pd);
    // reset all boid state
    this.reset();
};

Boid.worldRadius  =  50.0;
Boid.boundaryCondition = 0;

// ----------------------------------------------------------------------------

var center = new Vec3();
var div = 10.0;
var divisions = Vec3Set(div, div, div);
var diameter = Boid.worldRadius * 1.1 * 2;
var dimensions = Vec3Set(diameter, diameter, diameter);
console.log(divisions, dimensions);
var GPD = new LQProximityDatabase( center, dimensions, divisions);

var population = 0;
var flock = [];

var oldTime;
var currentTime; 
var elapsedTime;

var ctx;
var scale = 5.0;
var xoff = 60.0;
var yoff = 60.0;

function addBoidToFlock (pd) {
    population++;
    var boid = new Boid(pd);
    flock.push(boid);
}

function removeBoidFromFlock () {
    if (population > 0) {
        // save a pointer to the last boid, then remove it from the flock
        var boid = flock[-1];
        flock.pop();
        population--;

        // delete the Boid
        boid.deleteBoid();
    }
}

function handleFunctionKeys (keyNumber) {
    if(keyNumber === 1)  addBoidToFlock();
    if(keyNumber === 2)  removeBoidFromFlock (); 
}

function boidUpdater() {

    oldTime = currentTime;
    currentTime = Date.now() / 1000.0;
    elapsedTime = currentTime - oldTime;
    
    ctx.clearRect(0,0,1000,800);

    // update each Boid
    for (var i = 0; i < flock.length; i++) {
        flock[i].update(currentTime, elapsedTime);

        var pos = flock[i].mover.position();
        var fwd = flock[i].mover.forward();
        var side = flock[i].mover.side();

        var x = pos.x * scale;
        var z = pos.z * scale;
        // the triangle
        ctx.beginPath();
        ctx.moveTo(x + scale * xoff, z + scale * yoff);
        ctx.lineTo(x - (fwd.x-side.x) * scale + xoff * scale, z-(fwd.y-side.y) * scale + yoff * scale);
        ctx.lineTo(x - (fwd.x+side.x) * scale + xoff * scale, z-(fwd.y+side.y) * scale + yoff * scale);
        ctx.closePath();
        ctx.stroke();

        // ctx.beginPath();
        // ctx.arc(pos.x * scale + scale * xoff, pos.z * scale + scale * yoff, scale, 0, Math.PI*2);
        // ctx.fill();        
    }
    setTimeout( boidUpdater, 20 );
}

$(document).ready(function() {

    console.log("Testing OpenSteer Boids.");
    console.log("LQDB:", GPD);

    var canvas = document.getElementById("boidcanvas");
    ctx = canvas.getContext("2d");

    // create the specified number of Pedestrians
    population = 0;
    for (var i = 0; i < 100; i++) {
        addBoidToFlock(GPD);    
    }

    currentTime = Date.now() / 1000.0;
    boidUpdater();
});