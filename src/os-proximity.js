

// called by LQ for each clientObject in the specified neighborhood:
// push that clientObject onto the ContentType vector in void*
// clientQueryState
// (parameter names commented out to prevent compiler warning from "-W")
var perNeighborCallBackFunction = function(clientObject, distanceSquared, clientQueryState) {
    
    var results = clientQueryState;
    results.push(clientObject);
};


// ----------------------------------------------------------------------------
// A AbstractProximityDatabase-style wrapper for the LQ bin lattice system

// constructor
var LQProximityDatabase = function(center, dimensions, divisions) {

    this.halfsize = dimensions.mult(0.5);
    this.origin = center.sub(this.halfsize);

    this.lq = lqCreateDatabase (origin.x, origin.y, origin.z, 
                            dimensions.x, dimensions.y, dimensions.z,  
                            parseInt(divisions.x, 10),
                            parseInt(divisions.y, 10),
                            parseInt(divisions.z, 10));

    // destructor
    this.delLQProximityDatabase = function() {
        lqDeleteDatabase (this.lq);
        this.lq = undefined;
    };

    // constructor
    this.tokenType = function(parentObject, lqsd) {
        
        this.proxy = lqInitClientProxy (this.proxy, parentObject);
        this.lq = lqsd.lq;

        // destructor
        this.deltokenType = function() {
            lqRemoveFromBin (this.proxy);
        };

        // the client object calls this each time its position changes
        this.updateForNewPosition = function( p ) {
            lqUpdateForNewLocation (this.lq, this.proxy, p.x, p.y, p.z);
        };

        // find all neighbors within the given sphere (as center and radius)
        this.findNeighbors = function( center, radius ) {
            
            var results = [];
            lqMapOverAllObjectsInLocality (this.lq, center.x, center.y, center.z, radius, perNeighborCallBackFunction, results);
            return results;
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
        var count = 0;
        lqMapOverAllObjects (this.lq, counterCallBackFunction, count);
        return count;
    };
    
    // (parameter names commented out to prevent compiler warning from "-W")
    this.counterCallBackFunction = function( clientObject, distanceSquared, clientQueryState ) {

        clientQueryState++;
    };
}


