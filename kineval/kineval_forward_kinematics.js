
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | forward kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotForwardKinematics = function robotForwardKinematics () { 

    if (typeof robot.links_geom_imported === "undefined"){
        robot.links_geom_imported = false;
    }  
    
    mstack = [];
    /*
    if (robot.links_geom_imported) {
        mstack.push([
                    [0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [1, 0, 0, 0],
                    [0, 0, 0, 1]
                    ]);
    }
    else {*/
    mstack.push(generate_identity(4));
    // } 
    
    kineval.buildFKTransforms();
    
    if (typeof kineval.buildFKTransforms === 'undefined') {
        textbar.innerHTML = "forward kinematics not implemented";
        return;
    }
    // STENCIL: implement kineval.buildFKTransforms();

}

kineval.buildFKTransforms = function buildFKTransforms() {
    traverseFKBase();
    mstack.pop();
}

    // STENCIL: reference code alternates recursive traversal over 
    //   links and joints starting from base, using following functions: 
    //     traverseFKBase
    //     traverseFKLink
    //     traverseFKJoint
    //
    // user interface needs the heading (z-axis) and lateral (x-axis) directions
    //   of robot base in world coordinates stored as 4x1 matrices in
    //   global variables "robot_heading" and "robot_lateral"
    //
    // if geometries are imported and using ROS coordinates (e.g., fetch),
    //   coordinate conversion is needed for kineval/threejs coordinates:
    //

    
function traverseFKBase() { //     traverseFKBase
    
    /*
    var temp_origin_xyz = [];
    var temp_origin_rpy = [];
    if (robot.links_geom_imported){
        temp_origin_xyz[1] = robot.origin.xyz[0];
        temp_origin_xyz[2] = robot.origin.xyz[1];
        temp_origin_xyz[0] = robot.origin.xyz[2];
        temp_origin_rpy[1] = robot.origin.rpy[0];
        temp_origin_rpy[2] = robot.origin.rpy[1];
        temp_origin_rpy[0] = robot.origin.rpy[2];
    }
    else {
        temp_origin_xyz = robot.origin.xyz;
        temp_origin_rpy = robot.origin.rpy;
    //}*/
    
    var mat_tran = generate_translation_matrix(robot.origin.xyz[0],robot.origin.xyz[1],robot.origin.xyz[2]);
    var mat_X = generate_rotation_matrix_X(robot.origin.rpy[0]);
    var mat_Y = generate_rotation_matrix_Y(robot.origin.rpy[1]);
    var mat_Z = generate_rotation_matrix_Z(robot.origin.rpy[2]);
    var mat =  matrix_multiply(mat_tran,matrix_multiply(mat_Z,matrix_multiply(mat_Y, mat_X)));
    robot_heading = matrix_multiply(mat,[[0],[0],[1],[1]]);
    robot_lateral = matrix_multiply(mat,[[1],[0],[0],[1]]);
    if (robot.links_geom_imported) {
        var mat_transfer = [
                           [0, 1, 0, 0],
                           [0, 0, 1, 0],
                           [1, 0, 0, 0],
                           [0, 0, 0, 1]
                           ];
        mat = matrix_multiply(mat,mat_transfer);
    }
    
    var mat_xform = matrix_multiply(mstack[mstack.length-1],mat);
    robot.origin.xform = mat_xform;
    robot.links[robot.base].xform = mat_xform;
    mstack.push(mat_xform);

    var i;
    var num = robot.links[robot.base].children.length
    if( num !== 0) {
        for (i=0;i<num;i++) {
            traverseFKJoint(robot.links[robot.base].children[i]);
        }
    }
}
    
function traverseFKJoint(jointName) {//     traverseFKJoint
    var mat_tran = generate_translation_matrix(robot.joints[jointName].origin.xyz[0],robot.joints[jointName].origin.xyz[1],robot.joints[jointName].origin.xyz[2]);
    var mat_X = generate_rotation_matrix_X(robot.joints[jointName].origin.rpy[0]);
    var mat_Y = generate_rotation_matrix_Y(robot.joints[jointName].origin.rpy[1]);
    var mat_Z = generate_rotation_matrix_Z(robot.joints[jointName].origin.rpy[2]);
    var type;
    if (typeof robot.joints[jointName].type === "undefined") {
        type = "continuous";     
    }
    else {
        type = robot.joints[jointName].type;
    }
    var q = quaternion_from_axisangle(robot.joints[jointName].axis, robot.joints[jointName].angle, type);
    var mat_rot = quaternion_to_rotation_matrix(q, robot.joints[jointName].axis, robot.joints[jointName].angle, type);
    var mat =  matrix_multiply(matrix_multiply(mat_tran,matrix_multiply(mat_Z,matrix_multiply(mat_Y, mat_X))),mat_rot);
    var mat_xform = matrix_multiply(mstack[mstack.length-1],mat); 
    robot.joints[jointName].xform = mat_xform;
    mstack.push(mat_xform);
    reachEnd = traverseFKLink(robot.joints[jointName].child);
    if (reachEnd) {
        mstack.pop();
    }

}

function traverseFKLink(linkName) {//    traverseFKLink
    robot.links[linkName].xform = mstack[mstack.length-1];
    var num = robot.links[linkName].children.length;
    var i,reachEnd;
    if( num === 0 ){
        reachEnd = true;
        return reachEnd;
    }
    else {
        for (i=0;i<num;i++) {
            traverseFKJoint(robot.links[linkName].children[i]);
        }
    }
    reachEnd = true;
    return reachEnd;
}




