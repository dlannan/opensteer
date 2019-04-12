

// called by LQ for each clientObject in the specified neighborhood:
// push that clientObject onto the ContentType vector in void*
// clientQueryState
// (parameter names commented out to prevent compiler warning from "-W")
var perNeighborCallBackFunction = function(clientObject, distanceSquared, clientQueryState) {
    
    //console.log("adding results:", clientObject);
    clientQueryState.results.push(clientObject);
};


// (parameter names commented out to prevent compiler warning from "-W")
var counterCallBackFunction = function( clientObject, distanceSquared, clientQueryState ) {

    clientQueryState.count++;
};


// ----------------------------------------------------------------------------
// A AbstractProximityDatabase-style wrapper for the LQ bin lattice system

// constructor
var LQProximityDatabase = function(center, dimensions, divisions) {

    this.halfsize = dimensions.mult(0.5);
    this.origin = center.sub(this.halfsize);

    this.lq = lqCreateDatabase(this.origin.x, this.origin.y, this.origin.z, dimensions.x, dimensions.y, dimensions.z,  (divisions.x | 0), (divisions.y | 0), (divisions.z | 0));

    // destructor
    this.delLQProximityDatabase = function() {
        lqDeleteDatabase (this.lq);
        this.lq = undefined;
    };

    // constructor
    this.tokenType = function(parentObject, lqsd) {
        
        this.proxy = lqInitClientProxy(this.proxy, parentObject);
        this.lq = lqsd.lq;

        // destructor
        this.deltokenType = function() {
            lqRemoveFromBin (this.proxy);
        };

        // the client object calls this each time its position changes
        this.updateForNewPosition = function( p ) {
            lqUpdateForNewLocation(this.lq, this.proxy, p.x, p.y, p.z);
        };

        // find all neighbors within the given sphere (as center and radius)
        this.findNeighbors = function( center, radius ) {
            
            var state = { "results": [] };
            lqMapOverAllObjectsInLocality(this.lq, center.x, center.y, center.z, radius, perNeighborCallBackFunction, state);
            return state.results;
        };

        // Get statistics about bin populations: min, max and
        // average of non-empty bins.
        this.getBinPopulationStats = function( min, max, average ) {
            lqGetBinPopulationStats (this.lq, min, max, average);
        };
    };

    // allocate a token to represent a given client object in this database
    this.allocateToken = function(parentObject) {
        return new this.tokenType(parentObject, this);
    };

    // count the number of tokens currently in the database
    this.getPopulation = function() {
        var state = { count: 0 };
        lqMapOverAllObjects(this.lq, counterCallBackFunction, count);
        return state.count;
    };
}


