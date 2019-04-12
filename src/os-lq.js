/*
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
// ----------------------------------------------------------------------------
*/
/* ------------------------------------------------------------------ */
/*                                                                    */
/* Locality Query facility                                            */
/*                                                                    */
/* (by Craig Reynolds, see lq.h file for documentation)               */
/*                                                                    */
/*  5-17-99: created                                                  */
/*  5-20-99: found elusive "allocate 0 bins" bug                      */
/*  5-28-99: lqMapOverAllObjectsInLocality: clipped, incremental      */
/*  6- 7-99: clean up, split off annotation stuff into debuglq.c      */
/*  6- 8-99: tried screening by sum of coords ("first mean"?) but     */
/*           it was slightly slower, moved unused code to debuglq     */
/* 10-19-99: Change lqClientObject, lqObject from: "struct x {};" to  */
/*           "typedef struct x {} x;" for EE compiler.                */
/* 12- 2-00: Make lqObject "private" using lqInternalDB.              */
/* 12- 5-00: Rename lqObject to lqDB, lqClientObject to lqClientProxy */
/* 12- 6-00: Change lqCallBackFunction from arglist of (void*) to:    */
/*           (void* clientObject, float distanceSquared, void*        */
/*           clientQueryState).  Add void* clientQueryState arg to    */
/*           lqMapOverAllObjectsInLocality and its helper functions   */
/*           lqMapOverAllObjectsInLocalityClipped and                 */
/*           lqMapOverAllOutsideObjects. Change macro                 */
/*           lqTraverseBinClientObjectList to invoke callback         */
/*           function with three arguments, add "state" to its        */
/*           arglist.  Remove extern lqDistanceSquared.               */
/* 12- 7-00: Rename lqInitClientObject to lqInitClientProxy, make     */
/*           "func" be an argument to lqTraverseBinClientObjectList,  */
/*           add comments.                                            */
/* 12- 8-00: Add lqFindNearestNeighborWithinRadius and related        */
/*           definitions: lqFindNearestHelper lqFindNearestState      */
/*           Add lqMapOverAllObjects and lqRemoveAllObjects (plus:    */
/*           lqMapOverAllObjectsInBin and lqRemoveAllObjectsInBin)    */
/*                                                                    */
/* ------------------------------------------------------------------ */


// Converted to JS by David Lannan
// lq.h included here.

/* ------------------------------------------------------------------ */
/*                                                                    */
/*                   Locality Query (LQ) Facility                     */
/*                                                                    */
/* ------------------------------------------------------------------ */
/*

    This utility is a spatial database which stores objects each of
    which is associated with a 3d point (a location in a 3d space).
    The points serve as the "search key" for the associated object.
    It is intended to efficiently answer "sphere inclusion" queries,
    also known as range queries: basically questions like:

        Which objects are within a radius R of the location L?

    In this context, "efficiently" means significantly faster than the
    naive, brute force O(n) testing of all known points.  Additionally
    it is assumed that the objects move along unpredictable paths, so
    that extensive preprocessing (for example, constructing a Delaunay
    triangulation of the point set) may not be practical.

    The implementation is a "bin lattice": a 3d rectangular array of
    brick-shaped (rectangular parallelepipeds) regions of space.  Each
    region is represented by a pointer to a (possibly empty) doubly-
    linked list of objects.  All of these sub-bricks are the same
    size.  All bricks are aligned with the global coordinate axes.

    Terminology used here: the region of space associated with a bin
    is called a sub-brick.  The collection of all sub-bricks is called
    the super-brick.  The super-brick should be specified to surround
    the region of space in which (almost) all the key-points will
    exist.  If key-points move outside the super-brick everything will
    continue to work, but without the speed advantage provided by the
    spatial subdivision.  For more details about how to specify the
    super-brick's position, size and subdivisions see lqCreateDatabase
    below.

    Overview of usage: an application using this facility would first
    create a database with lqCreateDatabase.  For each client object
    the application wants to put in the database it creates a
    lqClientProxy and initializes it with lqInitClientProxy.  When a
    client object moves, the application calls lqUpdateForNewLocation.
    To perform a query lqMapOverAllObjectsInLocality is passed an
    application-supplied call-back function to be applied to all
    client objects in the locality.  See lqCallBackFunction below for
    more detail.  The lqFindNearestNeighborWithinRadius function can
    be used to find a single nearest neighbor using the database.

    Note that "locality query" is also known as neighborhood query,
    neighborhood search, near neighbor search, and range query.  For
    additional information on this and related topics see:
    http://www.red3d.com/cwr/boids/ips.html

    For some description and illustrations of this database in use,
    see this paper: http://www.red3d.com/cwr/papers/2000/pip.html

*/


/* ------------------------------------------------------------------ */
/* This structure is a proxy for (and contains a pointer to) a client
   (application) object in the spatial database.  One of these exists
   for each client object.  This might be included within the
   structure of a client object, or could be allocated separately.  */


function lqClientProxy() {
    /* previous object in this bin, or NULL */
    this.prev = undefined;

    /* next object in this bin, or NULL */
    this.next = undefined;

    /* bin ID (pointer to pointer to bin contents list) */
    this.bin = undefined;

    /* pointer to client object */
    this.object = undefined;

    /* the object's location ("key point") used for spatial sorting */
    this.x = 0.0;
    this.y = 0.0;
    this.z = 0.0;
};

/* ------------------------------------------------------------------ */
/* Apply an application-specific function to all objects in a certain
   locality.  The locality is specified as a sphere with a given
   center and radius.  All objects whose location (key-point) is
   within this sphere are identified and the function is applied to
   them.  The application-supplied function takes three arguments:

     (1) a void* pointer to an lqClientProxy's "object".
     (2) the square of the distance from the center of the search
         locality sphere (x,y,z) to object's key-point.
     (3) a void* pointer to the caller-supplied "client query state"
         object -- typically NULL, but can be used to store state
         between calls to the lqCallBackFunction.

   This routine uses the LQ database to quickly reject any objects in
   bins which do not overlap with the sphere of interest.  Incremental
   calculation of index values is used to efficiently traverse the
   bins of interest. */


/* type for a pointer to a function used to map over client objects */
function lqCallBackFunction (clientObject, distanceSquared, clientQueryState) {}


/* ------------------------------------------------------------------ */
/* This structure represents the spatial database.  Typically one of
   these would be created, by a call to lqCreateDatabase, for a given
   application.  */


function lqInternalDB() {

    /* the origin is the super-brick corner minimum coordinates */
    this.originx = 0.0;
    this.originy = 0.0;
    this.originz = 0.0;

    /* length of the edges of the super-brick */
    this.sizex = 0.0;
    this.sizey = 0.0;
    this.sizez = 0.0;

    /* number of sub-brick divisions in each direction */
    this.divx = 0;
    this.divy = 0;
    this.divz = 0;

    /* pointer to an array of pointers, one for each bin */
    this.bins = [];

    /* extra bin for "everything else" (points outside super-brick) */
    this.other = undefined;

};

// Global lq proximity database handle.
var glq = new lqInternalDB();

/* ------------------------------------------------------------------ */
/* Allocate and initialize an LQ database, return a pointer to it.
   The application needs to call this before using the LQ facility.
   The nine parameters define the properties of the "super-brick":
      (1) origin: coordinates of one corner of the super-brick, its
          minimum x, y and z extent.
      (2) size: the width, height and depth of the super-brick.
      (3) the number of subdivisions (sub-bricks) along each axis.
   This routine also allocates the bin array, and initialize its
   contents. */


function lqCreateDatabase (originx, originy, originz, sizex, sizey, sizez, divx, divy, divz)
{
    var lq = new lqInternalDB();

    lqInitDatabase(lq, originx, originy, originz, sizex, sizey, sizez, divx, divy, divz);
    return lq;
}


/* ------------------------------------------------------------------ */
/* Deallocate the memory used by the LQ database */


function lqDeleteDatabase(lq) {
    delete lq.bins;
    delete lq;
}


/* ------------------------------------------------------------------ */
/* Given an LQ database object and the nine basic parameters: fill in
   the object's slots, allocate the bin array, and initialize its
   contents. */


function lqInitDatabase (lq, originx, originy, originz, sizex, sizey, sizez, divx, divy, divz) {
    lq.originx = originx;
    lq.originy = originy;
    lq.originz = originz;
    lq.sizex = sizex;
    lq.sizey = sizey;
    lq.sizez = sizez;
    lq.divx = divx;
    lq.divy = divy;
    lq.divz = divz;
    
	var bincount = divx * divy * divz;
    lq.bins = [];
    lq.bins.length = bincount;
	for (var i=0; i<bincount; i++) {
        lq.bins[i] = undefined;
    } 

    lq.other = undefined;
}


/* ------------------------------------------------------------------ */
/* Determine index into linear bin array given 3D bin indices */


function lqBinCoordsToBinIndex(lq, ix, iy, iz) {
    return ((ix * lq.divy * lq.divz) + (iy * lq.divz) + iz);
}


/* ------------------------------------------------------------------ */
/* Find the bin ID for a location in space.  The location is given in
   terms of its XYZ coordinates.  The bin ID is a pointer to a pointer
   to the bin contents list.  */


function lqBinForLocation (lq, x, y, z) {
    var i, ix, iy, iz;

    /* if point outside super-brick, return the "other" bin */
    if (x < lq.originx)              return (lq.other);
    if (y < lq.originy)              return (lq.other);
    if (z < lq.originz)              return (lq.other);
    if (x >= lq.originx + lq.sizex) return (lq.other);
    if (y >= lq.originy + lq.sizey) return (lq.other);
    if (z >= lq.originz + lq.sizez) return (lq.other);

    /* if point inside super-brick, compute the bin coordinates */
    ix = (((x - lq.originx) / lq.sizex) * lq.divx) | 0;
    iy = (((y - lq.originy) / lq.sizey) * lq.divy) | 0;
    iz = (((z - lq.originz) / lq.sizez) * lq.divz) | 0;

    /* convert to linear bin number */
    i = lqBinCoordsToBinIndex (lq, ix, iy, iz);

    /* return pointer to that bin */
    return i;
}


/* ------------------------------------------------------------------ */
/* The application needs to call this once on each lqClientProxy at
   setup time to initialize its list pointers and associate the proxy
   with its client object. */ 


function lqInitClientProxy (proxy, clientObject) {
    
    var proxy = new lqClientProxy();
    proxy.object = clientObject;
    return proxy;
}


/* ------------------------------------------------------------------ */
/* Adds a given client object to a given bin, linking it into the bin
   contents list. */


function lqAddToBin(lq, object, idx) {
    
    var bin = lq.bins[idx];
    /* if bin is currently empty */    
    if (bin === undefined) {
        object.prev = undefined;
        object.next = undefined;
        bin = object;
    }
    else {
        object.prev = undefined;
        object.next = idx;
        bin.prev = object.bin;
        bin = object;
    }

    /* record bin ID in proxy object */
    object.bin = idx;
    return bin;
}


/* ------------------------------------------------------------------ */
/* Removes a given client object from its current bin, unlinking it
   from the bin contents list. */


function lqRemoveFromBin(lq, object) {

    /* adjust pointers if object is currently in a bin */
    if (lq.bins[object.bin] !== undefined) {
        /* If this object is at the head of the list, move the bin
        pointer to the next item in the list (might be NULL). */
        if (lq.bins[object.bin] === object) { lq.bins[object.bin] = lq.bins[object.next]; }

        /* If there is a prev object, link its "next" pointer to the
        object after this one. */
        if (object.prev !== undefined) { lq.bins[object.prev].next = object.next; }

        /* If there is a next object, link its "prev" pointer to the
        object before this one. */
        if (object.next !== undefined) { lq.bins[object.next].prev = object.prev; }
    }

    /* Null out prev, next and bin pointers of this object. */
    object.prev = undefined;
    object.next = undefined;
    object.bin = undefined;
}


/* ------------------------------------------------------------------ */
/* Call for each client object every time its location changes.  For
   example, in an animation application, this would be called each
   frame for every moving object.  */


function lqUpdateForNewLocation(lq, object, x, y, z) {
    /* find bin for new location */
    var idx = lqBinForLocation(lq, x, y, z);
    var newBin = lq.bins[idx];

    /* store location in client object, for future reference */
    object.x = x;
    object.y = y;
    object.z = z;

    /* has object moved into a new bin? */
    if((newBin === undefined) || (idx !== object.bin)) {
        lqRemoveFromBin (lq, object);
        lqAddToBin(lq, object, idx);
    }
}


/* ------------------------------------------------------------------ */
/* Given a bin's list of client proxies, traverse the list and invoke
   the given lqCallBackFunction on each object that falls within the
   search radius.  */


function lqTraverseBinClientObjectList(lq, x, y, z, co, radiusSquared, func, state) {
    
    while (co !== undefined) {                                                                 

        /* compute distance (squared) from this client   */           
        /* object to given locality sphere's centerpoint */           
        var dx = x - co.x;                                         
        var dy = y - co.y;                                         
        var dz = z - co.z;                                         
        var distanceSquared = (dx * dx) + (dy * dy) + (dz * dz);    
                                                                        
        /* apply function if client object within sphere */           
        if (distanceSquared < radiusSquared) {                         
            func(co.object, distanceSquared, state);             
        }
                                                                        
        /* consider next client object in bin list */                 
        co = lq.bins[co.next];  
    }
}


/* ------------------------------------------------------------------ */
/* This subroutine of lqMapOverAllObjectsInLocality efficiently
   traverses of subset of bins specified by max and min bin
   coordinates. */


function lqMapOverAllObjectsInLocalityClipped ( lq, x, y, z, radius, func, clientQueryState, minBinX, minBinY, minBinZ, maxBinX, maxBinY, maxBinZ) {
    var i, j, k;
    var iindex, jindex, kindex;
    var slab = lq.divy * lq.divz;
    var row = lq.divz;
    var istart = minBinX * slab;
    var jstart = minBinY * row;
    var kstart = minBinZ;
    var co;
    var bin;
    var radiusSquared = radius * radius;

    /* loop for x bins across diameter of sphere */
    iindex = istart;
    for (var i = minBinX; i <= maxBinX; i++) {
        /* loop for y bins across diameter of sphere */
        jindex = jstart;
        for(var j = minBinY; j <= maxBinY; j++) {
            /* loop for z bins across diameter of sphere */
            kindex = kstart;
            for (var k = minBinZ; k <= maxBinZ; k++)  {
                /* get current bin's client object list */
                bin = lq.bins[iindex + jindex + kindex];
                co = bin;

                /* traverse current bin's client object list */
                lqTraverseBinClientObjectList (lq, x, y, z, co, radiusSquared, func, clientQueryState);
                kindex += 1;
            }
            jindex += row;
        }
        iindex += slab;
    }
}


/* ------------------------------------------------------------------ */
/* If the query region (sphere) extends outside of the "super-brick"
   we need to check for objects in the catch-all "other" bin which
   holds any object which are not inside the regular sub-bricks  */


function lqMapOverAllOutsideObjects ( lq, x, y, z, radius, func, clientQueryState)
{
    var co = lq.other;
    var radiusSquared = radius * radius;

    /* traverse the "other" bin's client object list */
    lqTraverseBinClientObjectList ( lq, co, radiusSquared, func, clientQueryState);
}


/* ------------------------------------------------------------------ */
/* Apply an application-specific function to all objects in a certain
   locality.  The locality is specified as a sphere with a given
   center and radius.  All objects whose location (key-point) is
   within this sphere are identified and the function is applied to
   them.  The application-supplied function takes three arguments:

     (1) a void* pointer to an lqClientProxy's "object".
     (2) the square of the distance from the center of the search
         locality sphere (x,y,z) to object's key-point.
     (3) a void* pointer to the caller-supplied "client query state"
         object -- typically NULL, but can be used to store state
         between calls to the lqCallBackFunction.

   This routine uses the LQ database to quickly reject any objects in
   bins which do not overlap with the sphere of interest.  Incremental
   calculation of index values is used to efficiently traverse the
   bins of interest. */


function lqMapOverAllObjectsInLocality( lq, x, y, z, radius, func, clientQueryState) {
    var partlyOut = 0;
    var completelyOutside = (((x + radius) < lq.originx) || ((y + radius) < lq.originy) || ((z + radius) < lq.originz));
    completelyOutside = completelyOutside || ((x - radius) >= lq.originx + lq.sizex) || ((y - radius) >= lq.originy + lq.sizey) || ((z - radius) >= lq.originz + lq.sizez);
    var minBinX, minBinY, minBinZ, maxBinX, maxBinY, maxBinZ;

    /* is the sphere completely outside the "super brick"? */
    if(completelyOutside === true) {
	    lqMapOverAllOutsideObjects (lq, x, y, z, radius, func, clientQueryState);
	    return;
    }

    /* compute min and max bin coordinates for each dimension */
    minBinX = ((((x - radius) - lq.originx) / lq.sizex) * lq.divx) | 0;
    minBinY = ((((y - radius) - lq.originy) / lq.sizey) * lq.divy) | 0;
    minBinZ = ((((z - radius) - lq.originz) / lq.sizez) * lq.divz) | 0;
    maxBinX = ((((x + radius) - lq.originx) / lq.sizex) * lq.divx) | 0;
    maxBinY = ((((y + radius) - lq.originy) / lq.sizey) * lq.divy) | 0;
    maxBinZ = ((((z + radius) - lq.originz) / lq.sizez) * lq.divz) | 0;

    /* clip bin coordinates */
    if (minBinX < 0)         {partlyOut = 1; minBinX = 0;}
    if (minBinY < 0)         {partlyOut = 1; minBinY = 0;}
    if (minBinZ < 0)         {partlyOut = 1; minBinZ = 0;}
    if (maxBinX >= lq.divx) {partlyOut = 1; maxBinX = lq.divx - 1;}
    if (maxBinY >= lq.divy) {partlyOut = 1; maxBinY = lq.divy - 1;}
    if (maxBinZ >= lq.divz) {partlyOut = 1; maxBinZ = lq.divz - 1;}

    /* map function over outside objects if necessary (if clipped) */
    if (partlyOut) { 
        lqMapOverAllOutsideObjects (lq, x, y, z, radius, func, clientQueryState);
    }

    /* map function over objects in bins */
    lqMapOverAllObjectsInLocalityClipped (lq, x, y, z, radius, func, clientQueryState, minBinX, minBinY, minBinZ, maxBinX, maxBinY, maxBinZ);
}


/* ------------------------------------------------------------------ */
/* internal helper function */


var lqFindNearestState = function() {
    this.ignoreObject = undefined;
    this.nearestObject = undefined;
    this.minDistanceSquared = 0.0;

};

function lqFindNearestHelper (clientObject, distanceSquared, clientQueryState) {
    var fns = clientQueryState;

    /* do nothing if this is the "ignoreObject" */
    if (fns.ignoreObject !== clientObject) {
        /* record this object if it is the nearest one so far */
        if (fns.minDistanceSquared > distanceSquared) {
            fns.nearestObject = clientObject;
            fns.minDistanceSquared = distanceSquared;
        }
    }
}


/* ------------------------------------------------------------------ */
/* Search the database to find the object whose key-point is nearest
   to a given location yet within a given radius.  That is, it finds
   the object (if any) within a given search sphere which is nearest
   to the sphere's center.  The ignoreObject argument can be used to
   exclude an object from consideration (or it can be NULL).  This is
   useful when looking for the nearest neighbor of an object in the
   database, since otherwise it would be its own nearest neighbor.
   The function returns a void* pointer to the nearest object, or
   NULL if none is found.  */


function lqFindNearestNeighborWithinRadius (lq, x, y, z, radius, ignoreObject) {
    
    /* initialize search state */
    var lqFNS = new lqFindNearestState();
    lqFNS.nearestObject = undefined;
    lqFNS.ignoreObject = ignoreObject;
    lqFNS.minDistanceSquared = Number.MAX_VALUE;

    /* map search helper function over all objects within radius */
    lqMapOverAllObjectsInLocality (lq, x, y, z, radius, lqFindNearestHelper, lqFNS);

    /* return nearest object found, if any */
    return lqFNS.nearestObject;
}


/* ------------------------------------------------------------------ */
/* internal helper function */


function lqMapOverAllObjectsInBin (binProxyList, func, clientQueryState)
{
    /* walk down proxy list, applying call-back function to each one */
    while (binProxyList != undefined) {
	    func(binProxyList.object, 0, clientQueryState);
	    binProxyList = binProxyList.next;
    }
}


/* ------------------------------------------------------------------ */
/* Apply a user-supplied function to all objects in the database,
   regardless of locality (cf lqMapOverAllObjectsInLocality) */

function lqMapOverAllObjects (lq, func, clientQueryState)
{
    var bincount = lq.divx * lq.divy * lq.divz;
    for (var i=0; i<bincount; i++) {
    	lqMapOverAllObjectsInBin (lq.bins[i], func, clientQueryState);
    }
    lqMapOverAllObjectsInBin (lq.other, func, clientQueryState);
}


/* ------------------------------------------------------------------ */
/* internal helper function */


function lqRemoveAllObjectsInBin(lq, bin) {
    while (bin !== undefined) {
        lqRemoveFromBin(lq, bin);
    }
}


/* ------------------------------------------------------------------ */
/* Removes (all proxies for) all objects from all bins */


function lqRemoveAllObjects (lq) {

    var bincount = lq.divx * lq.divy * lq.divz;
    for (i=0; i<bincount; i++) {
	    lqRemoveAllObjectsInBin (lq, lq.bins[i]);
    }
    lqRemoveAllObjectsInBin (lq.other);
}


/* ------------------------------------------------------------------ */
