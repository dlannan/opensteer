
var LocalSpace = function( obj ) {


    obj._side = new Vec3();     //    side-pointing unit basis vector
    obj._up = new Vec3();       //  upward-pointing unit basis vector
    obj._forward = new Vec3();  // forward-pointing unit basis vector
    obj._position = new Vec3(); // origin of local space

    // accessors (get and set) for side, up, forward and position
    obj.side = function() {
        return obj._side;
    }

    obj.setSide = function(s) {
        obj._side.setV(s);
    }

    obj.up = function() {
        return obj._up;
    }

    obj.setUp = function(u) {
        obj._up.setV(u);
    }

    obj.forward = function() {
        return obj._forward;
    }

    obj.setForward = function(f) {
        obj._forward.setV(f);
    }

    obj.position = function() {
        return obj._position;
    }

    obj.setPosition = function(p) {
        obj._position.setV(p);
    }

    // use right-(or left-)handed coordinate space
    obj.rightHanded = function() {
        return true;
    }

    // reset transform to identity
    obj.resetLocalSpace = function() {
        obj._forward.set(0, 0, 1);
        obj._side = obj.localRotateForwardToSide(obj._forward);
        obj._up.set(0, 1, 0);
        obj._position.set(0, 0, 0);
    };

    // transform a direction in global space to its equivalent in local space
    obj.localizeDirection = function(globalDirection) {
        // dot offset with local basis vectors to obtain local coordiantes
        return Vec3Set (globalDirection.dot(obj._side),
                        globalDirection.dot(obj._up),
                        globalDirection.dot(obj._forward));
    }

    // transform a point in global space to its equivalent in local space
    obj.localizePosition = function(globalPosition) {
        // global offset from local origin
        var globalOffset = globalPosition.sub( obj._position );

        // dot offset with local basis vectors to obtain local coordiantes
        return obj.localizeDirection(globalOffset);        
    }

    // transform a point in local space to its equivalent in global space
    obj.globalizePosition = function(localPosition) {
        return obj._position.add( obj.globalizeDirection (localPosition) );
    }

    // transform a direction in local space to its equivalent in global space
    obj.globalizeDirection = function(localDirection){
        return ((obj._side.mult(localDirection.x)).add(obj._up.mult(localDirection.y)).add(obj._forward.mult(localDirection.z)));
    }

    // set "side" basis vector to normalized cross product of forward and up
    obj.setUnitSideFromForwardAndUp = function() {
        // derive new unit side basis vector from forward and up
        if (obj.rightHanded())
            obj._side.cross(obj._forward, obj._up);
        else
            obj._side.cross(obj._up, obj._forward);
        obj._side = obj._side.normalize ();        
    }

    // regenerate the orthonormal basis vectors given a new forward
    // (which is expected to have unit length)
    obj.regenerateOrthonormalBasisUF = function( newUnitForward) {
        obj._forward = newUnitForward;

        // derive new side basis vector from NEW forward and OLD up
        obj.setUnitSideFromForwardAndUp ();

        // derive new Up basis vector from new Side and new Forward
        // (should have unit length since Side and Forward are
        // perpendicular and unit length)
        if (obj.rightHanded())
            obj._up.cross(obj._side, obj._forward);
        else
            obj._up.cross(obj._forward, obj._side);
    }

    // for when the new forward is NOT of unit length
    obj.regenerateOrthonormalBasis = function(newForward) {
        obj.regenerateOrthonormalBasisUF (newForward.normalize());
    }

    // for supplying both a new forward and and new up
    obj.regenerateOrthonormalBasis = function(newForward, newUp) {
        obj._up = newUp;
        obj.regenerateOrthonormalBasis (newForward.normalize());
    }

    // rotate 90 degrees in the direction implied by rightHanded()
    obj.localRotateForwardToSide = function(v) {
        if(obj.rightHanded()) {
            return Vec3Set( -v.z, v.y, v.x );
        } else {
            return Vec3Set( v.z, v.y, v.x );
        }
    }

    obj.globalRotateForwardToSide = function(globalForward) {
        var localForward = obj.localizeDirection(globalForward);
        var localSide = obj.localRotateForwardToSide(localForward);
        return obj.globalizeDirection(localSide);        
    };
}

var gGlobalSpace = {} 
LocalSpace( gGlobalSpace );
