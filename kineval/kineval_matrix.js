//////////////////////////////////////////////////
/////     MATRIX ALGEBRA AND GEOMETRIC TRANSFORMS 
//////////////////////////////////////////////////

function matrix_copy(m1) {
    // returns 2D array that is a copy of m1

    var mat = [];
    var i,j;

    for (i=0;i<m1.length;i++) { // for each row of m1
        mat[i] = [];
        for (j=0;j<m1[0].length;j++) { // for each column of m1
            mat[i][j] = m1[i][j];
        }
    }
    return mat;
}


    // STENCIL: reference matrix code has the following functions:

function matrix_multiply(m1, m2) {
    // returns 2D array that is product of m1 and m2

    var mat = [];
    var i,j;

    var len = m2[0].length;
    var wid = m1.length;

    for (i=0;i<wid;i++) { //for each row of mat
        mat[i] = [];
        for (j=0;j<len;j++) { // for each column of mat
            var val = 0;
            //iterating to calculate mat[i][j]
            for(k=0;k<m1[0].length;k++) {
                val = val + m1[i][k]*m2[k][j];
            }
            mat[i][j] = val;
        }
    }

    return mat;
}

function matrix_transpose(m1) {
    // returns 2D array that is transpose of m1 
    var mat = [];
    var i,j;

    for (i=0;i<m1[0].length;i++) { //for each row of mat
        mat[i] = [];
        for (j=0;j<m1.length;j++) { // for each column of mat
            mat[i][j] = m1[j][i];
        }
    }
    return mat;
}

function getdim(x) {
    var y,z;
    if(typeof x === "object") {
        y = x[0];
        if(typeof y === "object") {
            z = y[0];
            if(typeof z === "object") {
                return numeric._dim(x);
            }
            return [x.length,y.length];
        }
        return [x.length];
    }
    return [];
}


function matrix_pseudoinverse(m1) {
    var mat = [];
    var m1_T = matrix_transpose(m1);
    var val = matrix_invert_affine(matrix_multiply(m1,m1_T));
    if (val === false) {
       return m1_T;
    }
    mat = matrix_multiply(m1_T,val);
    return mat; 
}
/*
function matrix_invert_affine_old(x) {//   matrix_invert_affine
    var s = getdim(x), abs = Math.abs, m = s[0], n = s[1];
    var A = matrix_copy(x), Ai, Aj;
    var I = generate_identity(m), Ii, Ij;
    var i,j,k,x;
    for(j=0;j<n;++j) {
        var i0 = -1;
        var v0 = -1;
        for(i=j;i!==m;++i) { k = abs(A[i][j]); if(k>v0) { i0 = i; v0 = k; } }
        Aj = A[i0]; A[i0] = A[j]; A[j] = Aj;
        Ij = I[i0]; I[i0] = I[j]; I[j] = Ij;
        x = Aj[j];
        for(k=j;k!==n;++k)    Aj[k] /= x; 
        for(k=n-1;k!==-1;--k) Ij[k] /= x;
        for(i=m-1;i!==-1;--i) {
            if(i!==j) {
                Ai = A[i];
                Ii = I[i];
                x = Ai[j];
                for(k=j+1;k!==n;++k)  Ai[k] -= Aj[k]*x;
                for(k=n-1;k>0;--k) { Ii[k] -= Ij[k]*x; --k; Ii[k] -= Ij[k]*x; }
                if(k===0) Ii[0] -= Ij[0]*x;
            }
        }
    }
    return I;
}
*/




function matrix_invert_affine(M){
    // I use Guassian Elimination to calculate the inverse:
    // (1) 'augment' the matrix (left) by the identity (on the right)
    // (2) Turn the matrix on the left into the identity by elemetry row ops
    // (3) The matrix on the right is the inverse (was the identity matrix)
    // There are 3 elemtary row ops: (I combine b and c in my code)
    // (a) Swap 2 rows
    // (b) Multiply a row by a scalar
    // (c) Add 2 rows
    
    //if the matrix isn't square: exit (error)
    if(M.length !== M[0].length){
        return false;
    }
    
    //create the identity matrix (I), and a copy (C) of the original
    var i=0, ii=0, j=0, dim=M.length, e=0, t=0;
    var I = [], C = [];
    for(i=0; i<dim; i+=1){
        // Create the row
        I[I.length]=[];
        C[C.length]=[];
        for(j=0; j<dim; j+=1){
            
            //if we're on the diagonal, put a 1 (for identity)
            if(i==j){ I[i][j] = 1; }
            else{ I[i][j] = 0; }
            
            // Also, make the copy of the original
            C[i][j] = M[i][j];
        }
    }
    
    // Perform elementary row operations
    for(i=0; i<dim; i+=1){
        // get the element e on the diagonal
        e = C[i][i];
        
        // if we have a 0 on the diagonal (we'll need to swap with a lower row)
        if(e==0){
            //look through every row below the i'th row
            for(ii=i+1; ii<dim; ii+=1){
                //if the ii'th row has a non-0 in the i'th col
                if(C[ii][i] != 0){
                    //it would make the diagonal have a non-0 so swap it
                    for(j=0; j<dim; j++){
                        e = C[i][j];       //temp store i'th row
                        C[i][j] = C[ii][j];//replace i'th row by ii'th
                        C[ii][j] = e;      //repace ii'th by temp
                        e = I[i][j];       //temp store i'th row
                        I[i][j] = I[ii][j];//replace i'th row by ii'th
                        I[ii][j] = e;      //repace ii'th by temp
                    }
                    //don't bother checking other rows since we've swapped
                    break;
                }
            }
            //get the new diagonal
            e = C[i][i];
            //if it's still 0, not invertable (error)
            if(e==0){return false;}
        }
        
        // Scale this row down by e (so we have a 1 on the diagonal)
        for(j=0; j<dim; j++){
            C[i][j] = C[i][j]/e; //apply to original matrix
            I[i][j] = I[i][j]/e; //apply to identity
        }
        
        // Subtract this row (scaled appropriately for each row) from ALL of
        // the other rows so that there will be 0's in this column in the
        // rows above and below this one
        for(ii=0; ii<dim; ii++){
            // Only apply to other rows (we want a 1 on the diagonal)
            if(ii==i){continue;}
            
            // We want to change this element to 0
            e = C[ii][i];
            
            // Subtract (the row above(or below) scaled by e) from (the
            // current row) but start at the i'th column and assume all the
            // stuff left of diagonal is 0 (which it should be if we made this
            // algorithm correctly)
            for(j=0; j<dim; j++){
                C[ii][j] -= e*C[i][j]; //apply to original matrix
                I[ii][j] -= e*I[i][j]; //apply to identity
            }
        }
    }
    
    //we've done all operations, C should be the identity
    //matrix I should be the inverse:
    return I;
}



        
    
function vector_normalize(vector) { //   vector_normalize
    var vec = [];
    var norm = 0;
    var i;
    for (i=0;i<vector.length;i++) {
        norm = norm + Math.pow(vector[i],2);
    }
    norm = Math.sqrt(norm);
    for (i=0;i<vector.length;i++) {
        vec[i] = vector[i]/norm;
    }
    return vec;
}


    
function vector_cross(vec1, vec2) {//   vector_cross
    var vec = [];
    vec[0] = vec1[1]*vec2[2] - vec1[2]*vec2[1];
    vec[1] = vec1[2]*vec2[0] - vec1[0]*vec2[2];
    vec[2] = vec1[0]*vec2[1] - vec1[1]*vec2[0];
    return vec;
}

function vector_subtract(vec1, vec2) {
    var vec = [];
    vec[0] = vec1[0] - vec2[0];
    vec[1] = vec1[1] - vec2[1];
    vec[2] = vec1[2] - vec2[2];
    return vec;
}

function vector_copy(vec1) {
    var vec = [];
    vec[0] = vec1[0];
    vec[1] = vec1[1];
    vec[2] = vec1[2];
    return vec;
}
function vector_scalar(vec1,n) {
    var vec = [];
    var i;
    for(i=0; i<vec1.length; i++){
        vec[i] = n*vec1[i];
    }
    return vec;
}



function generate_identity(n) {  //   generate_identity
    var mat = [];
    var i,j;

    for (i=0;i<n;i++) {
        mat[i] = [];
        for (j=0;j<n;j++) {
            if (j === i) {
                mat[i][j] = 1;
            }
            else {
                mat[i][j] = 0;
            }
        }
    }
    return mat;
}
   
function generate_translation_matrix(x,y,z) { //   generate_translation_matrix
    var mat = [
              [1, 0, 0, x],
              [0, 1, 0, y],
              [0, 0, 1, z],
              [0, 0, 0, 1]
              ];
    return mat;
}

function generate_rotation_matrix_X(r) { //   generate_rotation_matrix_X
    var theta;
    theta = r;
    var mat = [
              [1, 0, 0, 0],
              [0, Math.cos(theta), -Math.sin(theta), 0],
              [0, Math.sin(theta), Math.cos(theta), 0],
              [0, 0, 0, 1]
              ];
    return mat;
}

function generate_rotation_matrix_Y(p) { //   generate_rotation_matrix_Y
    var theta;
    theta = p;
    var mat = [
              [Math.cos(theta), 0,  Math.sin(theta), 0],
              [0, 1, 0, 0],
              [-Math.sin(theta), 0, Math.cos(theta), 0],
              [0, 0, 0, 1]
              ];
    return mat;
}

function generate_rotation_matrix_Z(y) { //   generate_rotation_matrix_Y
    var theta;
    theta = y;
    var mat = [
              [Math.cos(theta), -Math.sin(theta), 0, 0],
              [Math.sin(theta), Math.cos(theta), 0, 0],
              [0, 0, 1, 0],
              [0, 0, 0, 1]
              ];
    return mat;
}

