
// Test the OpenSteer soccer demo.
//    - start testing as soon as document loaded. Include this to test.

// ----------------------------------------------------------------------------
//
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
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.
//
//
// ----------------------------------------------------------------------------
//
// Simple soccer game by Michael Holm, IO Interactive A/S
//
// I made this to learn opensteer, and it took me four hours. The players will
// hunt the ball with no team spirit, each player acts on his own accord.
//
// I challenge the reader to change the behavour of one of the teams, to beat my
// team. It should not be too hard. If you make a better team, please share the
// source code so others get the chance to create a team that'll beat yours :)
//
// You are free to use this code for whatever you please.
//
// (contributed on July 9, 2003)
//
// ----------------------------------------------------------------------------

var selectedVehicle;
var soccer;
var context;
var scale = 15.0;
var xoff = scale * 25.0;
var yoff = scale * 12.0;

var playerPosition = [
    Vec3Set(4,0,0),
    Vec3Set(7,0,-5),
    Vec3Set(7,0,5),
    Vec3Set(10,0,-3),
    Vec3Set(10,0,3),
    Vec3Set(15,0, -8),
    Vec3Set(15,0,0),
    Vec3Set(15,0,8),
    Vec3Set(4,0,0)
];

// ----------------------------------------------------------------------------
// a box object for the field and the goals.
var AABBox = function(_min, _max) {

    this.m_min = _min;
    this.m_max = _max;

    this.InsideX = function(p){ if((p.x < this.m_min.x) || (p.x > this.m_max.x)) { return false;} else {return true;} }
    this.InsideZ = function(p){ if((p.z < this.m_min.z) || (p.z > this.m_max.z)) {return false;} else {return true;} }
    this.draw = function(ctx) {
        var b = Vec3Set(this.m_min.x, 0, this.m_max.z);
        var c = Vec3Set(this.m_max.x, 0, this.m_min.z);

        if(ctx) {
            ctx.fillStyle = "#008800";
            ctx.strokeStyle = "#000000";
            ctx.beginPath();
            ctx.moveTo(this.m_min.x * scale+xoff, this.m_min.z * scale+yoff);
            ctx.lineTo(b.x * scale+xoff, b.z * scale+yoff);
            ctx.lineTo(this.m_max.x * scale+xoff, this.m_max.z * scale+yoff);
            ctx.lineTo(c.x * scale+xoff, c.z * scale+yoff);
            ctx.closePath();
            ctx.fill();
            ctx.stroke();
        }
    }
};

// The ball object
var Ball = function(bbox) {

    this.m_bbox = bbox;
    this.mover = new SimpleVehicle();

    // reset state
    this.reset = function() {

        this.mover.reset (); // reset the vehicle 
        this.mover.setSpeed(0.0);         // speed along Forward direction.
        this.mover.setMaxForce(9.0);      // steering force is clipped to this magnitude
        this.mover.setMaxSpeed(9.0);         // velocity is clipped to this magnitude

        this.mover.setPosition(Vec3Set(0,0,0));
        //this.mover.clearTrailHistory ();    // prevent long streaks due to teleportation 
        //this.mover.setTrailParameters (100, 6000);
    }

    // per frame simulation update
    this.update = function( currentTime, elapsedTime) {

        this.mover.applyBrakingForce(1.5, elapsedTime);
        this.mover.applySteeringForce(this.mover.velocity(), elapsedTime);
        // are we now outside the field?
        if(!m_bbox.InsideX(this.mover.position())) {
            var d = this.mover.velocity();
            this.mover.regenerateOrthonormalBasis(Vec3Set(-d.x, d.y, d.z));
            this.mover.applySteeringForce(this.mover.velocity(), elapsedTime);
        }
        if(!m_bbox.InsideZ(this.mover.position())) {
            var d = this.mover.velocity();
            this.mover.regenerateOrthonormalBasis(Vec3Set(d.x, d.y, -d.z));
            this.mover.applySteeringForce(this.mover.velocity(), elapsedTime);
        }
    // recordTrailVertex (currentTime, position());
    }

    this.kick = function( dir, elapsedTime ){
        this.mover.setSpeed(dir.length());
        this.mover.regenerateOrthonormalBasis(dir);
    }

    // draw this character/vehicle into the scene
    this.draw = function(ctx) {

        ctx.strokeStyle = "#000000";
        ctx.fillStyle = "#FFFFFF";
        ctx.beginPath();
        var pos = this.mover.position();
        ctx.arc(pos.x * scale+xoff, pos.z * scale+yoff, this.mover.radius() * scale, 0, Math.PI * 2);
        ctx.fill();
    }

    this.reset();
};

var Player = function( others, allplayers, ball, isTeamA, id) {

    // constructor
    this.mover = new SimpleVehicle();

    this.m_others = others;
    this.m_AllPlayers = allplayers;
    this.m_Ball = ball;
    this.b_ImTeamA = isTeamA;
    this.m_MyID = id;

    // reset state
    this.reset = function() {
        this.mover.reset(); // reset the vehicle 
        this.mover.setSpeed (0.0);         // speed along Forward direction.
        this.mover.setMaxForce (3000.7);      // steering force is clipped to this magnitude
        this.mover.setMaxSpeed (10);         // velocity is clipped to this magnitude

        // Place me on my part of the field, looking at oponnents goal
        this.mover.setPosition( (this.b_ImTeamA === true) ? frandom01()*20 : -frandom01()*20, 0, (frandom01()-0.5)*20);
        if(this.m_MyID < 9) {
            if(this.b_ImTeamA)
                this.mover.setPosition(playerPosition[this.m_MyID]);
            else
                this.mover.setPosition(Vec3Set(-playerPosition[this.m_MyID].x, playerPosition[this.m_MyID].y, playerPosition[this.m_MyID].z));
            }
        this.m_home = this.mover.position().clone();
        //clearTrailHistory ();    // prevent long streaks due to teleportation 
        //setTrailParameters (10, 60);
    }

    // per frame simulation update
    // (parameter names commented out to prevent compiler warning from "-W")
    this.update = function( currentTime, elapsedTime) {
        // if I hit the ball, kick it.

        var distToBall = Vec3.distance (this.mover.position(), this.m_Ball.mover.position());
        var sumOfRadii = this.mover.radius() + this.m_Ball.mover.radius();
        if(distToBall < sumOfRadii)
            this.m_Ball.kick((this.m_Ball.mover.position().sub(this.mover.position())).mult(50.0), elapsedTime);


        // otherwise consider avoiding collisions with others
        var collisionAvoidance = this.mover.steerToAvoidNeighbors(1, this.m_AllPlayers);
        if(collisionAvoidance.neq(Vec3.zero)) {
            this.mover.applySteeringForce (collisionAvoidance, elapsedTime);
        }
        else {
            var distHomeToBall = Vec3.distance (this.m_home, this.m_Ball.mover.position());
            if( distHomeToBall < 12.0) {
                // go for ball if I'm on the 'right' side of the ball
                if( this.b_ImTeamA ? this.mover.position().x > this.m_Ball.mover.position().x : this.mover.position().x < this.m_Ball.mover.position().x) {
                    var seekTarget = this.mover.xxxsteerForSeek(this.m_Ball.mover.position());
                    this.mover.applySteeringForce(seekTarget, elapsedTime);
                }
                else {
                    if( distHomeToBall < 12.0) {
                        var Z = this.m_Ball.mover.position().z - this.mover.position().z > 0 ? -1.0 : 1.0;
                        var behindBall = this.m_Ball.mover.position().add( (this.b_ImTeamA ? Vec3Set(2,0,Z) : Vec3Set(-2,0,Z)) );
                        var behindBallForce = this.mover.xxxsteerForSeek(behindBall);
                        //annotationLine (position(), behindBall , Vec3(0,1,0));
                        var evadeTarget = this.mover.xxxsteerForFlee(this.m_Ball.mover.position());
                        this.mover.applySteeringForce(behindBallForce.mult(10.0).add(evadeTarget), elapsedTime);
                    }
                }
            }
            else { // Go home
                var seekTarget = this.mover.xxxsteerForSeek(this.m_home);
                var seekHome = this.mover.xxxsteerForSeek(this.m_home);
                this.mover.applySteeringForce (seekTarget.add(seekHome), elapsedTime);
            }

        }
    }

    // draw this character/vehicle into the scene
    this.draw = function (ctx)    {

        ctx.strokeStyle = "#000000";
        if(this.b_ImTeamA === true) 
            ctx.fillStyle = "#0000FF";
        else
            ctx.fillStyle = "#FF0000";

        ctx.beginPath();
        var pos = this.mover.position();
        ctx.arc(pos.x * scale+xoff, pos.z * scale+yoff, this.mover.radius() * scale, 0, Math.PI * 2);
        ctx.fill();
    }

    // per-instance reference to its group
    //const std::vector<Player*>	m_others;
    //const std::vector<Player*>	m_AllPlayers;
    //Ball*	m_Ball;
    //bool	b_ImTeamA;
    //int		m_MyID;
    //Vec3		m_home; 

    this.reset();
};



// ----------------------------------------------------------------------------


var	m_PlayerCountA = 0;
var m_PlayerCountB = 0;
var TeamA = [];
var TeamB = [];
var m_AllPlayers = [];

var     m_Ball;
var     m_bbox;
var     m_TeamAGoal;
var	    m_TeamBGoal;
var     junk;
var		m_redScore;
var		m_blueScore;

function SoccerSetup() {
    // Make a field
    m_bbox = new AABBox(Vec3Set(-20,0,-10), Vec3Set(20,0,10));
    // Red goal
    m_TeamAGoal = new AABBox(Vec3Set(-21,0,-7), Vec3Set(-19,0,7));
    // Blue Goal
    m_TeamBGoal = new AABBox(Vec3Set(19,0,-7), Vec3Set(21,0,7));
    // Make a ball
    m_Ball = new Ball(m_bbox);
    // Build team A
    m_PlayerCountA = 8;

    for(var i=0; i < m_PlayerCountA ; i++) {
        var pMicTest = new Player(TeamA, m_AllPlayers, m_Ball, true, i);
        selectedVehicle = pMicTest;
        TeamA.push(pMicTest);
        m_AllPlayers.push(pMicTest);
    }
    // Build Team B
    m_PlayerCountB = 8;
    for(var i=0; i < m_PlayerCountB ; i++)  {
        var pMicTest = new Player(TeamB, m_AllPlayers, m_Ball, false, i);
        selectedVehicle = pMicTest;
        TeamB.push(pMicTest);
        m_AllPlayers.push(pMicTest);
    }
    // initialize camera
    m_redScore = 0;
    m_blueScore = 0;
}

function update ( currentTime, elapsedTime) {
    // update simulation of test vehicle
    for( var i=0; i < m_PlayerCountA ; i++)
        TeamA[i].update (currentTime, elapsedTime);
    for( var i=0; i < m_PlayerCountB ; i++)
        TeamB[i].update (currentTime, elapsedTime);
    m_Ball.update(currentTime, elapsedTime);

    if(m_TeamAGoal.InsideX(m_Ball.mover.position()) && m_TeamAGoal.InsideZ(m_Ball.mover.position())) {
        m_Ball.reset();	// Ball in blue teams goal, red scores
        m_blueScore++;
    }
    if(m_TeamBGoal.InsideX(m_Ball.mover.position()) && m_TeamBGoal.InsideZ(m_Ball.mover.position())) {
        m_Ball.reset();	// Ball in red teams goal, blue scores
        m_redScore++;
    }
}

function redraw( ctx, currentTime, elapsedTime) {
    m_bbox.draw(ctx);
    m_TeamAGoal.draw(ctx);
    m_TeamBGoal.draw(ctx);

    // draw test vehicle
    for(var i=0; i < m_PlayerCountA ; i++)
        TeamA[i].draw(ctx);
    for(var i=0; i < m_PlayerCountB ; i++)
        TeamB[i].draw(ctx);
    m_Ball.draw(ctx);
}

function close() {
    for(var i=0; i < m_PlayerCountA ; i++)
        delete TeamA[i];
    TeamA = [];
    for(var i=0; i < m_PlayerCountB ; i++)
        delete TeamB[i];
    TeamB = [];
    m_AllPlayers = [];
}

function reset() {
    // reset vehicle
    for(var i=0; i < m_PlayerCountA ; i++)
        TeamA[i].reset ();
    for(var i=0; i < m_PlayerCountB ; i++)
        TeamB[i].reset ();
    m_Ball.reset();
}

// ----------------------------------------------------------------------------


var center = new Vec3();
var div = 20.0;
var divisions = new Vec3Set(div, 1.0, div);
var diameter = 80.0; //XXX need better way to get this
var dimensions = new Vec3Set(diameter, diameter, diameter);
var GPD = new LQProximityDatabase( center, dimensions, divisions);

var character;
var oldTime;
var currentTime; 
var elapsedTime;


function soccerUpdater() {

    oldTime = currentTime;
    currentTime = Date.now() / 1000.0;;
    elapsedTime = currentTime - oldTime;

    context.clearRect(0,0,1000,800);
    context.strokeStyle = "#000000";    

    context.font = "20px Arial";
    context.strokeText("Red: " + m_redScore, 10, 20); 
    context.strokeText("Blue: " + m_blueScore, 10 + scale * 48.0 - scale, 20); 

    update( currentTime, elapsedTime );
    redraw( context, currentTime, elapsedTime );

    setTimeout( soccerUpdater, 20 );
}

$(document).ready(function() {

    console.log("OpenSteer - Michael's Simple Soccer.");
    console.log("LQDB:", GPD);

    var canvas = document.getElementById("soccercanvas");
    context = canvas.getContext("2d");

    SoccerSetup();
    currentTime = Date.now() / 1000.0;;
    soccerUpdater();
});