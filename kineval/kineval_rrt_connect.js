
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | RRT motion planning

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

//////////////////////////////////////////////////
/////     RRT MOTION PLANNER
//////////////////////////////////////////////////

// STUDENT: 
// compute motion plan and output into robot_path array 
// elements of robot_path are vertices based on tree structure in tree_init() 
// motion planner assumes collision checking by kineval.poseIsCollision()

/* KE 2 : Notes:
   - Distance computation needs to consider modulo for joint angles
   - robot_path[] should be used as desireds for controls
   - Add visualization of configuration for current sample
   - Add cubic spline interpolation
   - Add hook for random configuration
   - Display planning iteration number in UI
*/

/*
STUDENT: reference code has functions for:

*/

kineval.planMotionRRTConnect = function motionPlanningRRTConnect() {

    // exit function if RRT is not implemented
    //   start by uncommenting kineval.robotRRTPlannerInit 
    if (typeof kineval.robotRRTPlannerInit === 'undefined') return;

    if ((kineval.params.update_motion_plan) && (!kineval.params.generating_motion_plan)) {
        kineval.robotRRTPlannerInit();
        kineval.params.generating_motion_plan = true;
        kineval.params.update_motion_plan = false;
        kineval.params.planner_state = "initializing";
    }
    if (kineval.params.generating_motion_plan) {
        rrt_result = robot_rrt_planner_iterate();
        if (rrt_result === "reached") {
            kineval.params.update_motion_plan = false; // KE T needed due to slight timing issue
            kineval.params.generating_motion_plan = false;
            textbar.innerHTML = "planner execution complete";
            kineval.params.planner_state = "complete";
        }
        else kineval.params.planner_state = "searching";
    }
    else if (kineval.params.update_motion_plan_traversal||kineval.params.persist_motion_plan_traversal) {

        if (kineval.params.persist_motion_plan_traversal&&kineval.motion_plan.length!==0) {
            kineval.motion_plan_traversal_index = (kineval.motion_plan_traversal_index+1)%kineval.motion_plan.length;
            textbar.innerHTML = "traversing planned motion trajectory";
        }
        else
            kineval.params.update_motion_plan_traversal = false;

        // set robot pose from entry in planned robot path
        if (kineval.motion_plan.length!==0) {
        robot.origin.xyz = [
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[0],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[1],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[2]
        ];

        robot.origin.rpy = [
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[3],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[4],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[5]
        ];

        // KE 2 : need to move q_names into a global parameter
        for (x in robot.joints) {
            robot.joints[x].angle = kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[q_names[x]];
        }

        }


    }
}


    // STENCIL: uncomment and complete initialization function
kineval.robotRRTPlannerInit = function robot_rrt_planner_init() {

    // form configuration from base location and joint angles
    q_start_config = [
        robot.origin.xyz[0],
        robot.origin.xyz[1],
        robot.origin.xyz[2],
        robot.origin.rpy[0],
        robot.origin.rpy[1],
        robot.origin.rpy[2]
    ];

    q_names = {};  // store mapping between joint names and q DOFs
    q_index = [];  // store mapping between joint names and q DOFs

    for (x in robot.joints) {
        q_names[x] = q_start_config.length;
        q_index[q_start_config.length] = x;
        q_start_config = q_start_config.concat(robot.joints[x].angle);
    }

    // set goal configuration as the zero configuration
    var i; 
    q_goal_config = new Array(q_start_config.length);
    for (i=0;i<q_goal_config.length;i++) q_goal_config[i] = 0;

    // flag to continue rrt iterations
    rrt_iterate = true;
    rrt_iter_count = 0;

    //initialize treeA and treeB
    T_a = tree_init(q_start_config);
    T_b = tree_init(q_goal_config);
    T_star = tree_init(q_start_config);

    kineval.motion_plan = [];

    // make sure the rrt iterations are not running faster than animation update
    cur_time = Date.now();
}



function robot_rrt_planner_iterate() {

    var i;
    rrt_alg = 2;  // 0: basic rrt (OPTIONAL), 1: rrt_connect (REQUIRED), 2:rrt_star

    if (rrt_iterate && (Date.now()-cur_time > 10)) {
        cur_time = Date.now();
        if (rrt_alg === 1){
            var qrand = random_config();
            if (rrt_extend(T_a,qrand) !== "Trapped"){
                var qnew = T_a.vertices[T_a.newest].vertex;
                if (rrt_connect(T_b,qnew) === "Reached"){
                
                    T_a.vertices[T_a.newest].edges.push(T_b.vertices[T_b.newest]);
                    T_b.vertices[T_b.newest].edges.push(T_a.vertices[T_a.newest]);

                    kineval.motion_plan = find_path();
                    drawHighlightedPath(kineval.motion_plan);
                    return "reached";
                }
            var temp = T_a;
            T_a = T_b;
            T_b = temp;
            return "unreached";
            }
        }
        if (rrt_alg === 2){
            
            //if(T_star.flag = false){
                var zrand =  random_config();
            //}
            //else{
                //var zrand =  new_config(q_goal_config,T_star.vertices[T_star.vertices.length-1].vertex);
            //}
        
            var zrand =  random_config();
            var index = nearest_neighbor(zrand,T_star);
            var znearest = T_star.vertices[index];
            var znew = steer(znearest.vertex,zrand);
            if(znew!==false){
                R = 1.01*T_star.eps;
                var Znear = Near(T_star,znew,R);
                var zmin;
                var cost;
                [zmin,cost] = ChooseParent(Znear,znearest,znew);
                insertNode(zmin,znew,T_star,cost);

                var znew_node = T_star.vertices[T_star.vertices.length-1];
                ReWire(T_star,Znear,zmin,znew_node);
                var bias = 0;
                for(var i=0;i<6;i++){
                    bias = bias + Math.pow(znew_node.vertex[i],2);
                }
                bias  =  Math.sqrt(bias);
                if(bias<2*T_star.eps)
                {
                    //if(cal_distance(znew_node.vertex,q_goal_config)<T_star.eps){
                        var num = Math.floor(cal_distance(znew_node.vertex,q_goal_config)/T_star.eps);
                        for (var i=0;i<num;i++){
                            zmin = T_star.vertices[T_star.vertices.length-1];
                            znew = new_config(q_goal_config,zmin.vertex);
                            cost = zmin.cost + cal_distance(zmin.vertex,znew);
                            insertNode(zmin,znew,T_star,cost);
                        }
                        kineval.motion_plan = find_Path_RRTStar(T_star);
                        drawHighlightedPath(kineval.motion_plan);
                        return "reached";
                    //}
                }
            }
            return "unreached";
        }
    }
    
    

    // STENCIL: implement single rrt iteration here. an asynch timing mechanism 
    //   is used instead of a for loop to avoid blocking and non-responsiveness 
    //   in the browser.
    //
    //   once plan is found, highlight vertices of found path by:
    //     tree.vertices[i].vertex[j].geom.material.color = {r:1,g:0,b:0};
    //
    //   provided support functions:
    //
    //   kineval.poseIsCollision - returns if a configuration is in collision
    //   tree_init - creates a tree of configurations
    //   tree_add_vertex - adds and displays new configuration vertex for a tree
    //   tree_add_edge - adds and displays new tree edge between configurations

}

//////////////////////////////////////////////////
/////     STENCIL SUPPORT FUNCTIONS
//////////////////////////////////////////////////
function steer(z1,z2)
{
    var ztest=z1;
    while(true)
    {
        ztest = new_config(z2,ztest);
        if(kineval.poseIsCollision(ztest)!==false)
        {
            return false;
        }
        else if(cal_distance(ztest,z2)<T_star.eps)
        {
            return new_config(z2,z1);
        }
    }
}
function Near(tree,z,R) {
    var Znear = [];
    var i;
    for (i=0;i<tree.vertices.length;i++)
    {
        if (cal_distance(z,tree.vertices[i].vertex)<=R)
        {
            Znear.push(tree.vertices[i]);
        }
    }
    return Znear;
}

function ChooseParent(Znear,znearest,znew) 
{
    var zmin=znearest;
    var cmin = znearest.cost+cal_distance(zmin.vertex,znew);
    var i;

    for(i=0;i<Znear.length;i++)
    {
        if(steer(Znear[i].vertex,znew)!==false)
        {
            if(Znear[i].cost+cal_distance(Znear[i].vertex,znew)<cmin)
            {
                cmin = cal_distance(Znear[i].vertex,znew)+Znear[i].cost;
                zmin = Znear[i];
            }
        }
    }
    return [zmin,cmin];
}

function insertNode(zmin,znew,tree,cost)
{

    new_vertex = {};
    new_vertex.parent = zmin;
    new_vertex.vertex = znew;
    new_vertex.index = tree.vertices.length;
    new_vertex.cost = cost
    tree.vertices.push(new_vertex);   
    add_config_origin_indicator_geom(new_vertex);
    tree.newest = tree.vertices.length - 1;
}

function ReWire(tree,Znear,zmin,znew)
{
    for(var i = 0;i<Znear.length;i++)
    {
        if (Znear[i]!==zmin)
        {
            if(steer(znew.vertex,Znear[i].vertex)!==false)
            {
                if(cal_distance(znew.vertex,Znear[i].vertex)+znew.cost<Znear[i].cost)
                {
                    Znear[i].cost = cal_distance(znew.vertex,Znear[i].vertex)+znew.cost;
                    Znear[i].parent = znew;
                }
            }
        }
    }

}

function find_Path_RRTStar(tree)
{
    var path=[];
    path.push(T_b.vertices[0]);
    var cur_node = tree.vertices[tree.vertices.length-1];
    path.push(cur_node);

    while(typeof cur_node.parent!=="undefined")
    {
        cur_node = cur_node.parent;
        path.push(cur_node);
    }
    return path;
}

function tree_init(q) {

    // create tree object
    var tree = {};

    // initialize with vertex for given configuration
    tree.vertices = [];
    tree.vertices[0] = {};
    tree.vertices[0].vertex = q;
    tree.vertices[0].edges = [];
    tree.vertices[0].index = 0;
    tree.vertices[0].cost = 0;

    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(tree.vertices[0]);

    // maintain index of newest vertex added to tree
    tree.newest = 0;
    tree.eps = 0.98;
    tree.flag = false;

    return tree;
}

function tree_add_vertex(tree,q) {


    // create new vertex object for tree with given configuration and no edges
    var new_vertex = {};
    new_vertex.edges = [];
    new_vertex.vertex = q;

    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(new_vertex);

    // maintain index of newest vertex added to tree
    tree.vertices.push(new_vertex);
    tree.newest = tree.vertices.length - 1;
}

function add_config_origin_indicator_geom(vertex) {

    // create a threejs rendering geometry for the base location of a configuration
    // assumes base origin location for configuration is first 3 elements 
    // assumes vertex is from tree and includes vertex field with configuration

    temp_geom = new THREE.CubeGeometry(0.1,0.1,0.1);
    temp_material = new THREE.MeshLambertMaterial( { color: 0xffff00, transparent: true, opacity: 0.7 } );
    temp_mesh = new THREE.Mesh(temp_geom, temp_material);
    temp_mesh.position.x = vertex.vertex[0];
    temp_mesh.position.y = vertex.vertex[1];
    temp_mesh.position.z = vertex.vertex[2];
    scene.add(temp_mesh);
    vertex.geom = temp_mesh;
}


function tree_add_edge(tree,q1_idx,q2_idx) {

    // add edge to first vertex as pointer to second vertex
    tree.vertices[q1_idx].edges.push(tree.vertices[q2_idx]);

    // add edge to second vertex as pointer to first vertex
    tree.vertices[q2_idx].edges.push(tree.vertices[q1_idx]);

    // can draw edge here, but not doing so to save rendering computation
}

//////////////////////////////////////////////////
/////     RRT IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////


    // STENCIL: implement RRT-Connect functions here, such as:
    //   rrt_extend
function rrt_extend(tree,q) {
    var qnear_index = nearest_neighbor(q,tree);
    var qnear = tree.vertices[qnear_index].vertex;
    var qnew = new_config(q,qnear);
    if (kineval.poseIsCollision(qnew)===false) {
        tree_add_vertex(tree,qnew);
        tree_add_edge(tree,qnear_index,tree.newest);
        var distance = cal_distance(q,qnew);
        if (distance<tree.eps) {
            return "Reached";
        }
        else {
            return "Advanced";
        }
    }
    else {
        return "Trapped";
    }
}

    //   rrt_connect
function rrt_connect(tree,q) {
    var S;
    while (true) {
        S = rrt_extend(tree,q)
        if (S!=="Advanced"){
            return S;
        }
    }
}
    //   random_config
 function random_config() {
    while (true) {
        var q_random_config = [(robot_boundary[1][0]-robot_boundary[0][0])*Math.random()+robot_boundary[0][0],
                               0,
                               (robot_boundary[1][2]-robot_boundary[0][2])*Math.random()+robot_boundary[0][2],
                               0,
                               Math.random()*2*Math.PI,
                               0
                              ];

        for (x in robot.joints) {
            if ( typeof robot.joints[x].type === 'undefined') {
                q_random_config = q_random_config.concat(Math.random()*2*Math.PI);
            }
            else{
                if (robot.joints[x].type === "prismatic" || robot.joints[x].type === "revolute") {
                    var angle = (robot.joints[x].limit.upper-robot.joints[x].limit.lower)*Math.random()+robot.joints[x].limit.lower;
                    q_random_config = q_random_config.concat(angle);
                }
                else if(robot.joints[x].type === "fixed") {
                    q_random_config = q_random_config.concat(0);
                }
                else {
                    q_random_config = q_random_config.concat(Math.random()*2*Math.PI);
                }
            }
        }

        if (kineval.poseIsCollision(q_random_config)===false){
            return q_random_config;
        }
    }
}
    //   new_config
function new_config(q,qnear) {
    var i;
    var qnew = [];
    var normq = normalize_joint_state(q,qnear);
    for(i=0;i<q.length;i++){
        qnew[i] = qnear[i] + T_a.eps*normq[i];
    }
    return qnew;
}

//   nearest_neighbor
function nearest_neighbor(qin,tree) {
    var i;
    var index = 0;
    var distance=100000000,distance_new;
    var q;
    for (i=0;i<tree.vertices.length;i++) {
        q = tree.vertices[i].vertex;
        distance_new = cal_distance(q,qin);
        if (distance_new<distance) {
            index = i;
            distance = distance_new;
        }    
    }
    return index;
}

function cal_distance(q1,q2){
    var distance = 0;
    for(var i=0;i<q1.length;i++){
        distance = distance + Math.pow(q1[i]-q2[i],2);
    }
    distance = Math.sqrt(distance);
    return distance;
}
    //   normalize_joint_state
function normalize_joint_state(q1,q2) {
    var norm  = 0;
    var qnorm = [];
    for(var i=0;i<q1.length;i++){
        norm = norm + Math.pow(q1[i]-q2[i],2);
    }
    norm = Math.sqrt(norm);
    for(var i=0;i<q1.length;i++){
        qnorm[i] = (q1[i]-q2[i])/norm;
    }
    return qnorm;
}
    //   find_path

function find_path(){
    var q = T_a.vertices[0].vertex;
    var distance = cal_distance(q,q_goal_config);
    if(distance<0.0001){
        var temp;
        temp = T_a;
        T_a = T_b;
        T_b = temp;
    }
    var i;
    for(i=0;i<T_a.vertices.length;i++){
        T_a.vertices[i].distance = 100000000000;
        T_a.vertices[i].parent = "none";
        T_a.vertices[i].visited = false;
    }
    for(i=0;i<T_b.vertices.length;i++){
        T_b.vertices[i].distance = 100000000000;
        T_b.vertices[i].parent = "none";
        T_b.vertices[i].visited = false;
    }
    var visit_stack = [];
    T_a.vertices[0].distance = 0;
    T_a.vertices[0].visited = true;

    visit_stack.push(T_a.vertices[0]);
    var cur_node = T_a.vertices[0];
    while(visit_stack.length!==0 && cal_distance(cur_node.vertex,q_goal_config)>T_a.eps) 
    {
        cur_node = visit_stack.pop();
        cur_node.visited = true;
        var i;
        for(i=0;i<cur_node.edges.length;i++){
            if (cur_node.edges[i].visited === false){
                visit_stack.push(cur_node.edges[i]);
                var distance_nbrcur = cal_distance(cur_node.edges[i].vertex,cur_node.vertex);
                if(cur_node.edges[i].distance>(cur_node.distance+distance_nbrcur)){
                    cur_node.edges[i].parent = cur_node;
                    cur_node.edges[i].distance = cur_node.distance + distance_nbrcur;
                }
            }
        }
    }
    var path=[];
    path.push(T_b.vertices[0]);
    while(cur_node.parent !== "none"){
        path.push(cur_node);
        cur_node = cur_node.parent;
    }
    path.push(cur_node);
    path.reverse();
    return path;
}
    //   path_dfs

function drawHighlightedPath(path) {
	var i;
	for(i=0;i<path.length;i++){
		path[i].geom.material.color = {r:1,g:0,b:0};
	}	
}









