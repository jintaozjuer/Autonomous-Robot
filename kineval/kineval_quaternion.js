//////////////////////////////////////////////////
/////     QUATERNION TRANSFORM ROUTINES 
//////////////////////////////////////////////////

    // STENCIL: reference quaternion code has the following functions:
    //   quaternion_from_axisangle
    //   quaternion_normalize
    //   quaternion_to_rotation_matrix
    //   quaternion_multiply

function quaternion_from_axisangle(axis, angle, type) { 
	var q = [];
	var axis_norm = vector_normalize(axis);
	if (type === "prismatic" || type === "fixed") {
		q = [1,0,0,0];
	}
	else {
		q[0] = Math.cos(angle/2);
	    q[1] = axis_norm[0] * Math.sin(angle/2);
		q[2] = axis_norm[1] * Math.sin(angle/2);
		q[3] = axis_norm[2] * Math.sin(angle/2);
	}
	return q;	
}

function quaternion_normalize(q) {
	var q_norm = [];
	var norm = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
	q_norm[0] = q[0]/norm;
	q_norm[1] = q[1]/norm;
	q_norm[2] = q[2]/norm;
	q_norm[3] = q[3]/norm;

	return q_norm;
}

function quaternion_to_rotation_matrix(q, axis, angle, type) {
    var axis_norm = vector_normalize(axis);
	if (type === "fixed") {
		var m = [
            	[1, 0, 0, 0],
            	[0, 1, 0, 0],
            	[0, 0, 1, 0],
            	[0, 0, 0, 1]
            	];
	}
	else if (type === "prismatic") {
		var dx = axis_norm[0] * angle;
		var dy = axis_norm[1] * angle;
		var dz = axis_norm[2] * angle;
		var m = [
            	[1, 0, 0, dx],
            	[0, 1, 0, dy],
            	[0, 0, 1, dz],
            	[0, 0, 0, 1]
            	];
	}
	else {
        var q0 = q[0];
	    var q1 = q[1];
		var q2 = q[2];
		var q3 = q[3];

		var m11 = q0*q0 + q1*q1 - q2*q2 - q3*q3;
		var m12 = 2*(q1*q2 - q0*q3);
		var m13 = 2*(q0*q2 + q1*q3);
		var m21 = 2*(q1*q2 + q0*q3);
		var m22 = q0*q0 - q1*q1 + q2*q2 - q3*q3;
		var m23 = 2*(q2*q3 - q0*q1);
		var m31 = 2*(q1*q3 - q0*q2);
		var m32 = 2*(q0*q1 + q2*q3);
		var m33 = q0*q0 - q1*q1 - q2*q2 + q3*q3;

		var m = [
            	[m11, m12, m13, 0],
            	[m21, m22, m23, 0],
           	 	[m31, m32, m33, 0],
            	[0,   0,   0,   1]
            	];
	}
    return m;
}

function quaternion_multiply(q1, q2) {
	var q = [];
	q[0] = (q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3]);
	q[1] = (q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2]);
	q[2] = (q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1]);
	q[3] = (q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0]);
    return q;
}