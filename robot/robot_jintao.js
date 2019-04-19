//   CREATE ROBOT STRUCTURE

// KE 

links_geom_imported = false;

//////////////////////////////////////////////////
/////     DEFINE ROBOT AND LINKS
//////////////////////////////////////////////////

// create robot data object
robot = new Object(); // or just {} will create new object

// give the robot a name
robot.name = "jintao";

// initialize start pose of robot in the world
robot.origin = {xyz: [0,2.5,0], rpy:[0,0,0]};  

// specify base link of the robot; robot.origin is transform of world to the robot base
robot.base = "body";  

        
// specify and create data objects for the links of the robot
robot.links = {
    "body": {},  
    "head": {}, 
    "lower_body": {},
    "left_arm": {},
    "right_arm": {},
    "left_leg": {},
    "right_leg": {},
    "left_forearm": {},
    "right_forearm": {},
    "left_shank": {},
    "right_shank": {}
   // "upperarm_right": {}, 
    //"forearm_right": {} 
};
/* for you to do
, "shoulder_left": {}  , "upperarm_left": {} , "forearm_left": {} };
*/

//////////////////////////////////////////////////
/////     DEFINE JOINTS AND KINEMATIC HIERARCHY
//////////////////////////////////////////////////

/*      joint definition template
        // specify parent/inboard link and child/outboard link
        robot.joints.joint1 = {parent:"link1", child:"link2"};
        // joint origin's offset transform from parent link origin
        robot.joints.joint1.origin = {xyz: [5,3,0], rpy:[0,0,0]}; 
        // joint rotation axis
        robot.joints.joint1.axis = [0.0,0.0,1.0]; 
*/


// roll-pitch-yaw defined by ROS as corresponding to x-y-z 
//http://wiki.ros.org/urdf/Tutorials/Create%20your%20own%20urdf%20file

// specify and create data objects for the joints of the robot
robot.joints = {};

robot.joints.neck = {parent:"body", child:"head"};
robot.joints.neck.origin = {xyz: [0.0,2.0,0.0], rpy:[-Math.PI/2,0,0]};
robot.joints.neck.axis = [0.0,0.0,-1.0]; 

robot.joints.waist = {parent:"body", child:"lower_body"};
robot.joints.waist.origin = {xyz: [0.0,0.0,0.0], rpy:[Math.PI/2,0,0]};
robot.joints.waist.axis = [0.0,0.0,1.0]; 

robot.joints.left_shoulder = {parent:"body", child:"left_arm"};
robot.joints.left_shoulder.origin = {xyz: [-0.5,1.8,0.0], rpy:[0,-Math.PI/2,0]};
robot.joints.left_shoulder.axis = [0.0,0.0,1.0]; 

robot.joints.right_shoulder = {parent:"body", child:"right_arm"};
robot.joints.right_shoulder.origin = {xyz: [0.5,1.8,0.0], rpy:[0,Math.PI/2,0]};
robot.joints.right_shoulder.axis = [0.0,0.0,1.0]; 

robot.joints.left_hip = {parent:"lower_body", child:"left_leg"};
robot.joints.left_hip.origin = {xyz: [-0.25,0.0,0.2], rpy:[0,-Math.PI/2,0]};
robot.joints.left_hip.axis = [0.0,0.0,-1.0]; 

robot.joints.right_hip = {parent:"lower_body", child:"right_leg"};
robot.joints.right_hip.origin = {xyz: [0.25,0.0,0.2], rpy:[0,Math.PI/2,0]};
robot.joints.right_hip.axis = [0.0,0.0,-1.0]; 

robot.joints.left_elbow = {parent:"left_arm", child:"left_forearm"};
robot.joints.left_elbow.origin = {xyz: [0,-0.9,0.15], rpy:[0,0,Math.PI/4]};
robot.joints.left_elbow.axis = [0.0,0.0,1.0]; 

robot.joints.right_elbow = {parent:"right_arm", child:"right_forearm"};
robot.joints.right_elbow.origin = {xyz: [0,-0.9,0.15], rpy:[0,0,-Math.PI/4]};
robot.joints.right_elbow.axis = [0.0,0.0,-1.0]; 

robot.joints.left_kneel = {parent:"left_leg", child:"left_shank"};
robot.joints.left_kneel.origin = {xyz: [0.9,0,0.15], rpy:[0,0,0]};
robot.joints.left_kneel.axis = [0.0,0.0,1.0]; 

robot.joints.right_kneel = {parent:"right_leg", child:"right_shank"};
robot.joints.right_kneel.origin = {xyz: [-0.9,0,0.15], rpy:[0,0,0]};
robot.joints.right_kneel.axis = [0.0,0.0,-1.0]; 

// specify name of endeffector frame
robot.endeffector = {};
robot.endeffector.frame = "right_elbow";
robot.endeffector.position = [[0],[-0.8],[0],[1]]

//////////////////////////////////////////////////
/////     DEFINE LINK threejs GEOMETRIES
//////////////////////////////////////////////////

/*  threejs geometry definition template, will be used by THREE.Mesh() to create threejs object
    // create threejs geometry and insert into links_geom data object
    links_geom["link1"] = new THREE.CubeGeometry( 5+2, 2, 2 );

    // example of translating geometry (in object space)
    links_geom["link1"].applyMatrix( new THREE.Matrix4().makeTranslation(5/2, 0, 0) );

    // example of rotating geometry 45 degrees about y-axis (in object space)
    var temp3axis = new THREE.Vector3(0,1,0);
    links_geom["link1"].rotateOnAxis(temp3axis,Math.PI/4);
*/

// define threejs geometries and associate with robot links 
links_geom = {};

links_geom["body"] = new THREE.CubeGeometry( 1, 2, 0.5 );
links_geom["body"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 1, 0) );

links_geom["head"] = new THREE.CubeGeometry( 0.6, 0.6, 0.6 );
links_geom["head"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.4) );

links_geom["lower_body"] = new THREE.CubeGeometry( 0.5, 0.5, 0.4 );
links_geom["lower_body"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.2) );

links_geom["left_arm"] = new THREE.CubeGeometry( 0.3, 1, 0.3 );
links_geom["left_arm"].applyMatrix( new THREE.Matrix4().makeTranslation(0, -0.4, 0.15) );

links_geom["right_arm"] = new THREE.CubeGeometry( 0.3, 1, 0.3 );
links_geom["right_arm"].applyMatrix( new THREE.Matrix4().makeTranslation(0, -0.4, 0.15) );

links_geom["left_leg"] = new THREE.CubeGeometry( 1, 0.4, 0.3 );
links_geom["left_leg"].applyMatrix( new THREE.Matrix4().makeTranslation(0.4, 0, 0.15) );

links_geom["right_leg"] = new THREE.CubeGeometry( 1, 0.4, 0.3 );
links_geom["right_leg"].applyMatrix( new THREE.Matrix4().makeTranslation(-0.4, 0, 0.15) );

links_geom["left_forearm"] = new THREE.CubeGeometry( 0.3, 0.8, 0.3 );
links_geom["left_forearm"].applyMatrix( new THREE.Matrix4().makeTranslation(0, -0.4, 0) );

links_geom["right_forearm"] = new THREE.CubeGeometry( 0.3, 0.8, 0.3 );
links_geom["right_forearm"].applyMatrix( new THREE.Matrix4().makeTranslation(0, -0.4, 0) );

links_geom["left_shank"] = new THREE.CubeGeometry( 1.4, 0.3, 0.3 );
links_geom["left_shank"].applyMatrix( new THREE.Matrix4().makeTranslation(0.7, 0, 0) );

links_geom["right_shank"] = new THREE.CubeGeometry( -1.4, 0.3, 0.3 );
links_geom["right_shank"].applyMatrix( new THREE.Matrix4().makeTranslation(-0.7, 0, 0) );


