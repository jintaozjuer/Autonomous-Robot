
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | inverse kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotInverseKinematics = function robot_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // compute joint angle controls to move location on specified link to Cartesian location
    if ((kineval.params.update_ik)||(kineval.params.persist_ik)) { 
        // if update requested, call ik iterator and show endeffector and target
        kineval.iterateIK(endeffector_target_world, endeffector_joint, endeffector_position_local);
        if (kineval.params.trial_ik_random.execute)
            kineval.randomizeIKtrial();
        else // KE: this use of start time assumes IK is invoked before trial
            kineval.params.trial_ik_random.start = new Date();
    }

    kineval.params.update_ik = false; // clear IK request for next iteration
}



kineval.randomizeIKtrial = function randomIKtrial () {

   // update time from start of trial
   cur_time = new Date();
   kineval.params.trial_ik_random.time = cur_time.getTime()-kineval.params.trial_ik_random.start.getTime();

   // get endeffector Cartesian position in the world
   endeffector_world = matrix_multiply(robot.joints[robot.endeffector.frame].xform,robot.endeffector.position);

   // compute distance of endeffector to target
   kineval.params.trial_ik_random.distance_current = Math.sqrt(
           Math.pow(kineval.params.ik_target.position[0][0]-endeffector_world[0][0],2.0)
           + Math.pow(kineval.params.ik_target.position[1][0]-endeffector_world[1][0],2.0)
           + Math.pow(kineval.params.ik_target.position[2][0]-endeffector_world[2][0],2.0) );

   // if target reached, increment scoring and generate new target location
   // KE 2 : convert hardcoded constants into proper parameters
   if (kineval.params.trial_ik_random.distance_current < 0.01) {
       kineval.params.ik_target.position[0][0] = 1.2*(Math.random()-0.5);
       kineval.params.ik_target.position[1][0] = 1.2*(Math.random()-0.5)+1.5;
       kineval.params.ik_target.position[2][0] = 0.7*(Math.random()-0.5)+0.5;
       kineval.params.trial_ik_random.targets += 1;
       textbar.innerHTML = "IK trial Random: target " + kineval.params.trial_ik_random.targets + " reached at time " + kineval.params.trial_ik_random.time;
   }

}


kineval.iterateIK = function iterate_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // STENCIL: implement inverse kinematics iteration

    kineval.params.ik_target.position = endeffector_target_world.position;
    kineval.params.ik_target.orientation = endeffector_target_world.orientation;

    var endeffector_position_world = matrix_multiply(robot.joints[endeffector_joint].xform,endeffector_position_local);
    endeffector_position_world = matrix_transpose(endeffector_position_world);
    var J_column = [];  
    var current_joint = endeffector_joint;
    var J_i,J_iv;
    var current_axis,current_xyz;
    var keepLoop = true;
    var i = 0;

    while(keepLoop) {
        current_axis = vector_copy(robot.joints[current_joint].axis);
        current_axis.push(0);
        current_axis = matrix_transpose([current_axis]);
        current_axis = matrix_multiply(robot.joints[current_joint].xform,current_axis);
        current_axis = matrix_transpose(current_axis);
       
        current_xyz = [[0],[0],[0],[1]];
        current_xyz = matrix_multiply(robot.joints[current_joint].xform,current_xyz);
        current_xyz = matrix_transpose(current_xyz);

        if (kineval.params.ik_orientation_included) {
        	if (typeof robot.joints[current_joint].type !== 'undefined' && robot.joints[current_joint].type === "prismatic") {
        		J_i = [current_axis[0][0],current_axis[0][1],current_axis[0][2],0,0,0];
        	}
        	else {
            	J_iv = vector_cross(current_axis[0], vector_subtract(endeffector_position_world[0],current_xyz[0]));
            	J_i = [J_iv[0],J_iv[1],J_iv[2],current_axis[0][0],current_axis[0][1],current_axis[0][2]];
        	}
        }

        else {
        	if (typeof robot.joints[current_joint].type !== 'undefined' && robot.joints[current_joint].type === "prismatic") {
        		J_i = [current_axis[0][0],current_axis[0][1],current_axis[0][2]];
        	}
        	else {
            	J_iv = vector_cross(current_axis[0], vector_subtract(endeffector_position_world[0],current_xyz[0]));
            	J_i = [J_iv[0],J_iv[1],J_iv[2]];
        	}
        }
        

        J_column[i] = J_i;
        if (robot.joints[current_joint].parent !== robot.base) {
            current_joint = robot.links[robot.joints[current_joint].parent].parent;
            i = i+1;
        }
        else {
            keepLoop = false;
        }
        
    }

    var J = matrix_transpose(J_column);
    
    var endeffector_rpy_world = [];
    var R = robot.joints[endeffector_joint].xform;
    var p = -Math.asin(R[2][0])
    endeffector_rpy_world[1]= p;

    if (R[2][0] === 1)
    {
        endeffector_rpy_world[2]=0;
        endeffector_rpy_world[0] = Math.atan2(-R[0][1],-R[0][2]);
    }
    else if(R[2][0] === -1)
    {
        endeffector_rpy_world[2]=0;
        endeffector_rpy_world[0] = Math.atan2(R[0][1],R[0][2]);
    }
    else
    {
        endeffector_rpy_world[0] = Math.atan2(R[2][1]/Math.cos(p),R[2][2]/Math.cos(p));
        endeffector_rpy_world[2] = Math.atan2(R[1][0]/Math.cos(p),R[0][0]/Math.cos(p));
    }

    var R_target = matrix_multiply(generate_rotation_matrix_X(endeffector_target_world.orientation[0]),
                                   matrix_multiply(generate_rotation_matrix_Y(endeffector_target_world.orientation[1]),
                                                   generate_rotation_matrix_Z(endeffector_target_world.orientation[2])
                                                  )
                                  );

    var p = -Math.asin(R_target[2][0]);
    var target_rpy_world = [];
    target_rpy_world[1]= p;
    if (R_target[2][0] === 1)
    {
        target_rpy_world[2]=0;
        target_rpy_world[0] = Math.atan2(-R_target[0][1],-R_target[0][2]);
    }
    else if(R_target[2][0] === -1)
    {
        target_rpy_world[2]=0;
        target_rpy_world[0] = Math.atan2(R_target[0][1],R_target[0][2]);
    }
    else
    {
        target_rpy_world[0] = Math.atan2(R_target[2][1]/Math.cos(p),R_target[2][2]/Math.cos(p));
        target_rpy_world[2] = Math.atan2(R_target[1][0]/Math.cos(p),R_target[0][0]/Math.cos(p));
    }
    
    if (kineval.params.ik_orientation_included) {
        var delta_x = [[endeffector_target_world.position[0][0]-endeffector_position_world[0][0]],
                       [endeffector_target_world.position[1][0]-endeffector_position_world[0][1]],
                       [endeffector_target_world.position[2][0]-endeffector_position_world[0][2]],
                       [target_rpy_world[0]-endeffector_rpy_world[0]],
                       [target_rpy_world[1]-endeffector_rpy_world[1]],
                       [target_rpy_world[2]-endeffector_rpy_world[2]]
                      ];
    }
    else {
        var delta_x = [[endeffector_target_world.position[0][0]-endeffector_position_world[0][0]],
                       [endeffector_target_world.position[1][0]-endeffector_position_world[0][1]],
                       [endeffector_target_world.position[2][0]-endeffector_position_world[0][2]]
                      ];
    }


    var q,delta_theta;

    if (kineval.params.ik_pseudoinverse) {
        var J_pse = matrix_pseudoinverse(J);
        delta_theta = matrix_multiply(J_pse,delta_x);
    }
    else {
        delta_theta = matrix_multiply(matrix_transpose(J),delta_x);
    }
    delta_theta = matrix_transpose(delta_theta);
    q =  vector_scalar(delta_theta[0],kineval.params.ik_steplength);
    

    current_joint = endeffector_joint;
    keepLoop = true;
    i = 0;
    while(keepLoop) {
        robot.joints[current_joint].control = q[i];
        if (robot.joints[current_joint].parent !== robot.base) {
            current_joint = robot.links[robot.joints[current_joint].parent].parent;
            i = i+1;
        }
        else {
            keepLoop = false;
        }
    }
}



