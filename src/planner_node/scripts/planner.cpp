/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_msgs/Header.h>

#include <math.h>
#include <random>
#include <vector>
#include <array>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <chrono>

#include <limits>
#include <cmath>

#include <tuple>
#include <string>
#include <stdexcept>
#include <regex> // For regex and split logic
#include <iostream> // cout, endl
#include <fstream> // For reading/writing files
#include <assert.h>

/* Planner Ids */
#define RRT         0
#define RRTCONNECT  1
#define RRTSTAR     2

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define PI 3.141592654

using std::vector;
using std::array;
using std::string;
using std::runtime_error;
using std::tuple;
using std::make_tuple;
using std::tie;
using std::cout;
using std::endl;

using namespace std;

//*******************************************************************************************************************//
//                                                                                                                   //
//                                                INITIAL FUNCTIONS                                                  //
//                                                                                                                   //
//*******************************************************************************************************************//

// Splits string based on deliminator
vector<string> split(const string& str, const string& delim) {   
		// https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c/64886763#64886763
		const std::regex ws_re(delim);
		return { std::sregex_token_iterator(str.begin(), str.end(), ws_re, -1), std::sregex_token_iterator() };
}


double* doubleArrayFromString(string str) {
	vector<string> vals = split(str, ",");
	double* ans = new double[vals.size()];
	for (int i = 0; i < vals.size(); ++i) {
		ans[i] = std::stod(vals[i]) * PI/180;
	}
	return ans;
}

bool equalDoubleArrays(double* v1, double *v2, int size) {
    for (int i = 0; i < size; ++i) {
        if (abs(v1[i]-v2[i]) > 1e-3) {
            cout << endl;
            return false;
        }
    }
    return true;
}

std::vector<double> doubleArrayToVector(const double* arr, int size) {
    std::vector<double> vec;
    vec.reserve(size); // Optional: Reserve memory for the vector to avoid reallocations
    
    for (int i = 0; i < size; ++i) {
        vec.push_back(arr[i]);
    }
    
    return vec;
}

// check if the arm is in collision
int IsValidArmConfiguration(const planning_scene::PlanningScenePtr& planning_scene, const robot_model::RobotModelPtr& kinematic_model, double* joint_values_arr){
    // auto joint_values = doubleArrayToVector(joint_values_arr, 6);

    // // Create a RobotState object for collision checking
    // robot_state::RobotState current_state(kinematic_model);
    // current_state.setJointGroupPositions("xarm6", joint_values);
    
    // // Check for collisions
    // collision_detection::CollisionRequest collision_request;
    // collision_detection::CollisionResult collision_result;
    // planning_scene->checkCollision(collision_request, collision_result, current_state);

    // if (collision_result.collision)
    // {
    //     // ROS_INFO("Collision detected for the given joint configuration!");    // auto joint_values = doubleArrayToVector(joint_values_arr, 6);

    // // Create a RobotState object for collision checking
    // robot_state::RobotState current_state(kinematic_model);
    // current_state.setJointGroupPositions("xarm6", joint_values);
    
    // // Check for collisions
    // collision_detection::CollisionRequest collision_request;
    // collision_detection::CollisionResult collision_result;
    // planning_scene->checkCollision(collision_request, collision_result, current_state);

    // if (collision_result.collision)
    // {
    //     // ROS_INFO("Collision detected for the given joint configuration!");
    //     return 0;
    // }
    // else
    // {
    //     // ROS_INFO("No collision detected for the given joint configuration.");
    //     return 1;


    auto joint_values = doubleArrayToVector(joint_values_arr, 6);

    // Create a RobotState object for collision checking
    robot_state::RobotState current_state(kinematic_model);
    current_state.setJointGroupPositions("xarm6", joint_values);
    current_state.update(); // Update the robot state

    // Set up collision request and result
    collision_detection::CollisionRequest collision_request;
    collision_request.group_name = "xarm6"; // Specify the group name if needed
    collision_detection::CollisionResult collision_result;

    // Check for collisions with both the robot itself and the environment
    collision_request.contacts = true; // Optional: set to true if you want detailed contact information
    collision_request.max_contacts = 1000; // Optional: set maximum number of contacts to report

    // Check collision with the updated planning scene
    planning_scene->checkCollision(collision_request, collision_result, current_state);

    if (collision_result.collision) {
        // ROS_INFO("Collision detected for the given joint configuration!");
        return 0;
    } else {
        // ROS_INFO("No collision detected for the given joint configuration.");
        return 1;
    }




}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                              RRT IMPLEMENTATION                                                   //
//                                                                                                                   //
//*******************************************************************************************************************//

// joint angles sum
double getPlanQuality(double*** plan, int* planlength, int numofDOFs) {
    double cost = 0;
    for (int i = 0; i < *planlength - 1; i++) {
        double* current_config = (*plan)[i];
        double* next_config = (*plan)[i+1];
        double diff = 0;
        for (int j = 0; j < numofDOFs; j++) {
            diff = abs(current_config[j] - next_config[j]);
		    diff = min(diff, 2*M_PI-diff);
        }
        cost += diff;
    }
    return cost;
}

// represents a joint angle configuration
struct Config{
	vector<double> values;
	Config(int numofDOFs) : values(numofDOFs, 0.0) {}
};

// represents the graph structure
struct Node {
	Config config;
	Node* parent;
    double cost;
    Node(const Config& conf) : config(conf), parent(nullptr), cost(0.0) {}  // initialize to 0
};


class RRTPlanner {
	public:
        vector<Node*> start_nodes;
        vector<Node*> goal_nodes;
		double epsilon;
    	vector<double> start;
    	vector<double> goal;
    	int numofDOFs;

        // class attributes for kinematic_model and planning_scene
        robot_model::RobotModelPtr kinematic_model_;
        planning_scene::PlanningScenePtr planning_scene_;

        // Constructor
        RRTPlanner(double epsilon, vector<double> start, vector<double> goal, int numofDOFs)
            : epsilon(epsilon), start(start), goal(goal), numofDOFs(numofDOFs) {

            // Load the robot model
            robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
            kinematic_model_ = robot_model_loader.getModel();
            planning_scene_ = std::make_shared<planning_scene::PlanningScene>(kinematic_model_);

            // Define the pose of the table (transformed to robot's frame)
            geometry_msgs::Pose pose;
            pose.position.x = -0.32;
            pose.position.y = 0;
            pose.position.z = 0;
            pose.orientation.w = 1;
            pose.orientation.x = 0;
            pose.orientation.y = 0;
            pose.orientation.z = 0;

            // Create a Collision Object
            moveit_msgs::CollisionObject collision_object;
            collision_object.id = "table";  // Unique ID for the object
            collision_object.header.frame_id = "link_base";  // Set the frame of your robot

            // Define the shape and size of the table (e.g., a box)
            shape_msgs::SolidPrimitive primitive;
            primitive.type = shape_msgs::SolidPrimitive::BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[0] = 0.85;
            primitive.dimensions[1] = 1.5;
            primitive.dimensions[2] = 0.08;

            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(pose);

            // Add the collision object to the planning scene
            planning_scene_->processCollisionObjectMsg(collision_object);

            // Update the planning scene
            planning_scene_->getCurrentStateNonConst().update();
        }

		Node* nearestNeighbor(const Config& q, bool from_start = true);
        Node* nearestNeighborStar(const Config& q);
        vector<Node*> nearbyNodes(const Config& qNew, double radius);
		bool newConfig(const Node* qNear, const Config& q, Config& qNew);
		void addVertex(const Config& qNew, bool from_start = true);
		void addEdge(Node* parent, Node* child);
        void addEdgeStar(Node* parent_node, Node* child_node);
        pair<bool,Node*> extendRRT(const Config& q, bool from_start = true);
        Node* extendRRTStar(const Config& q);
		Node* buildRRT(int K);
		void extractPath(Node* goalNode, double ***plan, int *pathLength);
        void extractPathConnect(Node* startConnectNode, Node* goalConnectNode, double ***plan, int *planlength);
		bool checkDist(const Config& q1, const Config& q2);
        pair<Node*, Node*> buildRRTConnect(int K);
        Node* connect(const Config& q, bool from_start);
        Node* buildRRTStar(int K);
};


// returns nearest neighboring node
Node* RRTPlanner::nearestNeighbor(const Config& q, bool from_start) {
    vector<Node*>& nodes = from_start ? start_nodes : goal_nodes;

    Node* nearest = nullptr;
    double minDist = numeric_limits<double>::max();

    for (Node* node : nodes) {
        double dist = 0.0;
        for (size_t i = 0; i < numofDOFs; i++) {
            dist += pow(node->config.values[i] - q.values[i], 2);
        }
        dist = sqrt(dist);

        if (dist < minDist) {
            minDist = dist;
            nearest = node;
        }
    }
    return nearest;
}

// move towards q (return true if moved, false if trapped)
bool RRTPlanner::newConfig(const Node* qNear, const Config& q, Config& qNew) {
    double dist = 0.0;
    for (size_t i = 0; i < numofDOFs; i++) {
        dist += pow(qNear->config.values[i] - q.values[i], 2);
    }
    dist = sqrt(dist);

    double stepSize = min(epsilon, dist) / dist;  // scaling factor
    // double stepSize = 0.1;
    bool advancementMade = false;

    // interpolate from qNear towards q and check for collisions
    for (double alpha = stepSize; alpha <= 1.0; alpha += stepSize) {
        Config tempConfig(numofDOFs);  // temp config for collision checking

        for (size_t i = 0; i < numofDOFs; i++) {
            tempConfig.values[i] = qNear->config.values[i] + alpha * (q.values[i] - qNear->config.values[i]);
        }

        // convert to array
        double* config_arr = new double[numofDOFs];
        for (int i = 0; i < numofDOFs; ++i) {
            config_arr[i] = tempConfig.values[i];
        }

        // if the configuration is valid, update qNew and mark advancement
        if (IsValidArmConfiguration(planning_scene_, kinematic_model_, config_arr)) {
            qNew = tempConfig;
            advancementMade = true;
        } else {
            break;  // stop interpolating if we hit a collision
        }
    }
    // if any advancement was made towards q, return true
    return advancementMade;
}

// add vertex to graph
void RRTPlanner::addVertex(const Config& qNew, bool from_start) {
    vector<Node*>& nodes = from_start ? start_nodes : goal_nodes;
    Node* new_node = new Node(qNew);
    nodes.push_back(new_node);
    cout << "Number of nodes: " << nodes.size() << endl;
}

// add edge to graph
void RRTPlanner::addEdge(Node* parent_node, Node* child_node) {
    child_node->parent = parent_node;
}

// extend qNear towards q
pair<bool,Node*> RRTPlanner::extendRRT(const Config& q, bool from_start) {
    Node* qNear = nearestNeighbor(q, from_start);
    vector<Node*>& nodes = from_start ? start_nodes : goal_nodes;
    Config qNew(numofDOFs);

    if (newConfig(qNear, q, qNew)) { // advanced or reached
        addVertex(qNew, from_start);
        Node* qNewNode = nodes.back();
        addEdge(qNear, qNewNode);
        return make_pair(true,qNewNode);
    }
    return make_pair(false,qNear); // trapped
}

bool RRTPlanner::checkDist(const Config& q1, const Config& q2){
    double sum = 0.0;

    // euclidean dist
    for (size_t i = 0; i < numofDOFs; i++) {
        sum += (q1.values[i] - q2.values[i]) * (q1.values[i] - q2.values[i]);
    }
    double distance = sqrt(sum);

    // check if dist within thresh
    return distance <= 1e-3;
}

Node* RRTPlanner::buildRRT(int K) {
	Config qInit(numofDOFs);
	qInit.values = start;
    addVertex(qInit);

	Config qGoal(numofDOFs);
	qGoal.values = goal;

    for (int k = 0; k < K; k++) {
        Config qRand(numofDOFs);
        double biasProbability = static_cast<double>(rand()) / RAND_MAX; // random value between 0 and 1

        // 10% bias towards the goal
        if (biasProbability <= 0.1) {
            qRand = qGoal;
        } else {
            qRand.values[0] = ((double) rand() / RAND_MAX) * 2 * M_PI; // 0 to 2pi
            qRand.values[1] = ((double) rand() / RAND_MAX) * 4 - 2; // -2 to 2
            qRand.values[2] = ((double) rand() / RAND_MAX) * -3.9 + 0.1; // -3.8 to 0.1
            qRand.values[3] = ((double) rand() / RAND_MAX) * 2 * M_PI; // 0 to 2pi
            qRand.values[4] = ((double) rand() / RAND_MAX) * -4.6 + 3; // -1.6 to 3.0
            qRand.values[5] = ((double) rand() / RAND_MAX) * 2 * M_PI; // 0 to 2pi
        }
        auto result = extendRRT(qRand);

        if (checkDist(result.second->config, qGoal)) { // reached the goal
            return result.second;
        }
    }
    return nullptr; // could not find a path after K iterations
}

void RRTPlanner::extractPath(Node* result, double ***plan, int *planlength) {
    // find path length by backtracking from the goal
    int len = 0;
    Node* current = result;
    while (current != nullptr) {
        len++;
        current = current->parent;
    }

    // extract the path
    current = result;
    *plan = (double**) malloc(len*sizeof(double*));
    for (int i = len-1; i >= 0; i--){
        (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
        for(int j = 0; j < numofDOFs; j++){
            (*plan)[i][j] = current->config.values[j];
        }
        current = current->parent;
    }
    *planlength = len;
}

static void plannerRRT(
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength
    )
{
    // start clock
    auto start_time = chrono::high_resolution_clock::now();

    // initialize values
	double epsilon = 0.1; 
	vector<double> start(armstart_anglesV_rad, armstart_anglesV_rad+numofDOFs);
	vector<double> goal(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs);
	RRTPlanner rrt(epsilon,start,goal,numofDOFs);

    // plan
    Node* result = rrt.buildRRT(100000);

    // planning time
    auto plan_end = chrono::high_resolution_clock::now();
    auto planning_time = chrono::duration_cast<chrono::milliseconds>(plan_end - start_time);

    // extract path
	if (result) {
        rrt.extractPath(result, plan, planlength);
	}
    else{
        cout << "No Goal Found" << endl;
    }

    // full solution time
    auto end_time = chrono::high_resolution_clock::now();
    auto total_time = chrono::duration_cast<chrono::milliseconds>(end_time - start_time);

    // planner metrics
    string under_five = (total_time.count() < 5000) ? "Yes" : "No";
    double plan_quality = getPlanQuality(plan, planlength, numofDOFs);
    cout << "Algorithm: RRT" << endl;
    cout << "Degrees of Freedom: " << numofDOFs << endl;
    cout << "Planning Time: " << planning_time.count() << " milliseconds" << endl;
    cout << "Generated Solution in Under Five Seconds: " << under_five << endl;
    cout << "Nodes Generated: " << rrt.start_nodes.size() + rrt.goal_nodes.size() << endl;
    cout << "Path Quality: " << plan_quality << endl;
    cout << "Path Length: " << *planlength << endl;

    return;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                         RRT CONNECT IMPLEMENTATION                                                //
//                                                                                                                   //
//*******************************************************************************************************************//

// connect towards q until it's reached or trapped
Node* RRTPlanner::connect(const Config& q, bool from_start){
    while(true){
        auto result = extendRRT(q, from_start);
        if (result.first){
            if (checkDist(result.second->config, q)){
                return result.second; // reached
            }
        } else {
            return nullptr; // trapped
        }
    }
}

void RRTPlanner::extractPathConnect(Node* startConnectNode, Node* goalConnectNode, double ***plan, int *planlength) {
    vector<Node*> path1, path2;

    Node* current = startConnectNode;
    // extract path from the meeting point to the start
    while (current != nullptr) {
        path1.push_back(current);
        current = current->parent;
    }

    current = goalConnectNode;
    // extract path from the meeting point to the goal
    while (current != nullptr) {
        path2.push_back(current);
        current = current->parent;
    }

    // combine both paths
    *planlength = path1.size() + path2.size();
    *plan = (double**) malloc(*planlength * sizeof(double*));

    int index = 0;
    // add path1 in reverse (from start to meeting point)
    for (int i = path1.size() - 1; i >= 0; i--) {
        (*plan)[index] = (double*) malloc(numofDOFs * sizeof(double));
        for (int j = 0; j < numofDOFs; j++) {
            (*plan)[index][j] = path1[i]->config.values[j];
        }
        index++;
    }

    // add path2 (from meeting point to goal)
    for (Node* node : path2) {
        (*plan)[index] = (double*) malloc(numofDOFs * sizeof(double));
        for (int j = 0; j < numofDOFs; j++) {
            (*plan)[index][j] = node->config.values[j];
        }
        index++;
    }
}

// buildRRTConnect method
pair<Node*, Node*> RRTPlanner::buildRRTConnect(int K) {
    
    bool from_start = true;
    Config qInit(numofDOFs);
    qInit.values = start;
    addVertex(qInit, true);  // add initial vertex to the start tree

    Config qGoal(numofDOFs);
    qGoal.values = goal;
    addVertex(qGoal, false); // add goal vertex to the goal tree

    for (int k = 0; k < K; k++) {

        // get random config
        Config qRand(numofDOFs);
        double biasProbability = static_cast<double>(rand()) / RAND_MAX; // random value between 0 and 1
        if (biasProbability <= 0.1) { // 10% bias towards the goal or start
            qRand = from_start ? qGoal : qInit;
        } else {
            qRand.values[0] = ((double) rand() / RAND_MAX) * 2 * M_PI; // 0 to 2pi
            qRand.values[1] = ((double) rand() / RAND_MAX) * 4 - 2; // -2 to 2
            qRand.values[2] = ((double) rand() / RAND_MAX) * -3.9 + 0.1; // -3.8 to 0.1
            qRand.values[3] = ((double) rand() / RAND_MAX) * 2 * M_PI; // 0 to 2pi
            qRand.values[4] = ((double) rand() / RAND_MAX) * -4.6 + 3; // -1.6 to 3.0
            qRand.values[5] = ((double) rand() / RAND_MAX) * 2 * M_PI; // 0 to 2pi
        }

        // extend from one side
        Node* resultNode = extendRRT(qRand, from_start).second;
        
        // connect from other side
        Node* connectNode = connect(resultNode->config, !from_start);
        if (connectNode){ // both sides reached each other
            auto pathEnds = from_start ? make_pair(resultNode,connectNode) : make_pair(connectNode,resultNode);
            return pathEnds; // returning connection node
        }

        // swap sides
        from_start = !from_start;
    }

    return make_pair(nullptr, nullptr);
}

static void plannerRRTConnect(
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength
    )
{
    // start clock
    auto start_time = chrono::high_resolution_clock::now();

    // initialize values
	double epsilon = 0.1;
	vector<double> start(armstart_anglesV_rad, armstart_anglesV_rad+numofDOFs);
	vector<double> goal(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs);
	RRTPlanner rrt(epsilon,start,goal,numofDOFs);

    // plan
	auto result = rrt.buildRRTConnect(100000);

    // planning time
    auto plan_end = chrono::high_resolution_clock::now();
    auto planning_time = chrono::duration_cast<chrono::milliseconds>(plan_end - start_time);

    // extract path
	if (result.first && result.second) {
        rrt.extractPathConnect(result.first, result.second, plan, planlength);
	}
    else{
        cout << "No Goal Found" << endl;
    }

    // full solution time
    auto end_time = chrono::high_resolution_clock::now();
    auto total_time = chrono::duration_cast<chrono::milliseconds>(end_time - start_time);

    // planner metrics
    string under_five = (total_time.count() < 5000) ? "Yes" : "No";
    double plan_quality = getPlanQuality(plan, planlength, numofDOFs);
    cout << "Algorithm: RRT Connect" << endl;
    cout << "Degrees of Freedom: " << numofDOFs << endl;
    cout << "Planning Time: " << planning_time.count() << " milliseconds" << endl;
    cout << "Generated Solution in Under Five Seconds: " << under_five << endl;
    cout << "Nodes Generated: " << rrt.start_nodes.size() + rrt.goal_nodes.size() << endl;
    cout << "Path Quality: " << plan_quality << endl;
    cout << "Path Length: " << *planlength << endl;

    return;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                           RRT STAR IMPLEMENTATION                                                 //
//                                                                                                                   //
//*******************************************************************************************************************//

// using euclidean dist for cost
double getCost(const Config& q1, const Config& q2) {
    double sum = 0.0;
    for (size_t i = 0; i < q1.values.size(); i++) {
        sum += (q1.values[i] - q2.values[i]) * (q1.values[i] - q2.values[i]);
    }
    return sqrt(sum);
}

// function to find the nearest neighbor based on cost
Node* RRTPlanner::nearestNeighborStar(const Config& q) {
    Node* nearest = nullptr;
    double minCost = numeric_limits<double>::max();

    for (Node* node : start_nodes) {
        double cost = node->cost;
        if (cost < minCost) {
            minCost = cost;
            nearest = node;
        }
    }
    return nearest;
}

// function to find all nearby nodes within radius
vector<Node*> RRTPlanner::nearbyNodes(const Config& qNew, double radius) {
    vector<Node*> nearby;
    for (Node* node : start_nodes) {
        double dist = 0.0;
        for (size_t i = 0; i < numofDOFs; i++) {
            dist += pow(node->config.values[i] - qNew.values[i], 2);
        }
        dist = sqrt(dist);
        if (dist < radius) {
            nearby.push_back(node);
        }
    }
    return nearby;
}

void RRTPlanner::addEdgeStar(Node* parent_node, Node* child_node) {
    child_node->parent = parent_node;
    child_node->cost = parent_node->cost + getCost(child_node->config, parent_node->config); // update cost
}

Node* RRTPlanner::extendRRTStar(const Config& q) {
    Node* qNear = nearestNeighbor(q);
    Config qNew(numofDOFs);
    if (newConfig(qNear, q, qNew)) {

        // new part for RRT*
        double minCost = qNear->cost + getCost(qNear->config, qNew);  
        Node* minCostNode = qNear;

        // check for nodes in neighborhood
        auto nearby = nearbyNodes(qNew, 5);
        for (Node* neighbor : nearby) {
            double potentialCost = neighbor->cost + getCost(neighbor->config, qNew);
            if (potentialCost < minCost) {
                minCost = potentialCost;
                minCostNode = neighbor;
            }
        }
        
        addVertex(qNew);
        Node* qNewNode = start_nodes.back();
        qNewNode->cost = minCost;  
        addEdgeStar(minCostNode, qNewNode);  // set parent to the minCostNode
        
        // rewire the tree
        for (Node* neighbor : nearby) {
            double potentialCost = qNewNode->cost + getCost(qNewNode->config, neighbor->config);
            if (potentialCost < neighbor->cost) {
                neighbor->parent = qNewNode;
                neighbor->cost = potentialCost;
            }
        }
        
        return qNewNode;
    }
    return nullptr;
}

vector<double> random_config(vector<double> center)
{
    double max_angles[6];
    double min_angles[6];
    std::vector<double> rand_angles;
    double range = 1.0;

    max_angles[0] = 2.0*M_PI;
    max_angles[1] = 2.0;
    max_angles[2] = 0.1;
    max_angles[3] = 2.0*M_PI;
    max_angles[4] = 3.0;
    max_angles[5] = 2.0*M_PI;
    min_angles[0] = 0.0;
    min_angles[1] = -2.0;
    min_angles[2] = -3.8;
    min_angles[3] = 0.0;
    min_angles[4] = -1.6;
    min_angles[5] = 0.0;

    std::random_device rd;
    std::mt19937 generator(rd());

    for(int i=0; i<6; i++)
    {
        std::uniform_real_distribution<double> distribution(center[i] - range, center[i] + range);
        double sample = distribution(generator);
        
        if(sample > max_angles[i]){
            sample = max_angles[i];
        }
        else if(sample < min_angles[i]) {
            sample = min_angles[i];
        }
        rand_angles.push_back(sample);
    }

    return rand_angles;
}   

Node* RRTPlanner::buildRRTStar(int K) {
    // nodes.clear();

	Config qInit(numofDOFs);
	qInit.values = start;
    addVertex(qInit);

	Config qGoal(numofDOFs);
	qGoal.values = goal;

    for (int k = 0; k < K; k++) {
        Config qRand(numofDOFs);
        double biasProbability = static_cast<double>(rand()) / RAND_MAX; // random value between 0 and 1

        // 10% bias towards the goal
        if (biasProbability <= 0.1){
            qRand = qGoal;
            // qRand.values = random_config(qGoal.values);
        // } 
        // else if (biasProbability > 0.1  && biasProbability <= 0.2){
        //     qRand.values = random_config(qInit.values);
        }
        else {
            qRand.values[0] = ((double) rand() / RAND_MAX) * 2 * M_PI; // 0 to 2pi
            qRand.values[1] = ((double) rand() / RAND_MAX) * 4 - 2; // -2 to 2
            qRand.values[2] = ((double) rand() / RAND_MAX) * -3.9 + 0.1; // -3.8 to 0.1
            qRand.values[3] = ((double) rand() / RAND_MAX) * 2 * M_PI; // 0 to 2pi
            qRand.values[4] = ((double) rand() / RAND_MAX) * -4.6 + 3; // -1.6 to 3.0
            qRand.values[5] = ((double) rand() / RAND_MAX) * 2 * M_PI; // 0 to 2pi
        }
        auto result = extendRRTStar(qRand);

        if (result && checkDist(result->config, qGoal)) { // reached the goal
            return result;
        }
    }
    return nullptr; // could not find a path after K iterations
}

static void plannerRRTStar(
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength
    )
{
    // start clock
    auto start_time = chrono::high_resolution_clock::now();

    // initialize values
	double epsilon = 0.5;
	vector<double> start(armstart_anglesV_rad, armstart_anglesV_rad+numofDOFs);
	vector<double> goal(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs);
	RRTPlanner rrt(epsilon,start,goal,numofDOFs);

    // plan
	Node* result = rrt.buildRRTStar(10000);

    // planning time
    auto plan_end = chrono::high_resolution_clock::now();
    auto planning_time = chrono::duration_cast<chrono::milliseconds>(plan_end - start_time);

    // extract path
	if (result) {
        rrt.extractPath(result, plan, planlength);
	}
    else{
        cout << "No Goal Found" << endl;
    }

    // full solution time
    auto end_time = chrono::high_resolution_clock::now();
    auto total_time = chrono::duration_cast<chrono::milliseconds>(end_time - start_time);

    // planner metrics
    string under_five = (total_time.count() < 5000) ? "Yes" : "No";
    double plan_quality = getPlanQuality(plan, planlength, numofDOFs);
    cout << "Algorithm: RRT*" << endl;
    cout << "Degrees of Freedom: " << numofDOFs << endl;
    cout << "Planning Time: " << planning_time.count() << " milliseconds" << endl;
    cout << "Generated Solution in Under Five Seconds: " << under_five << endl;
    cout << "Nodes Generated: " << rrt.start_nodes.size() + rrt.goal_nodes.size() << endl;
    cout << "Path Quality: " << plan_quality << endl;
    cout << "Path Length: " << *planlength << endl;

    return;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                                MAIN FUNCTION                                                      //
//                                                                                                                   //
//*******************************************************************************************************************//

int main(int argc, char** argv) {

    // Initialize node
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle nh;

    // Publisher to move the arm
    ros::Publisher traj_publisher = nh.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/xarm/xarm6_traj_controller/follow_joint_trajectory/goal", 10);

    ros::Rate loop_rate(1);

    // Load the robot model
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    planning_scene::PlanningScenePtr planning_scene;
    planning_scene = std::make_shared<planning_scene::PlanningScene>(kinematic_model);

    // Define the pose of the table (transformed to robot's frame)
    geometry_msgs::Pose pose;
    pose.position.x = -0.32;
    pose.position.y = 0;
    pose.position.z = 0;
    pose.orientation.w = 1;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;

    // Create a Collision Object
    moveit_msgs::CollisionObject collision_object;
    collision_object.id = "table";  // Unique ID for the object
    collision_object.header.frame_id = "link_base";  // Set the frame of your robot

    // Define the shape and size of the table (e.g., a box)
    shape_msgs::SolidPrimitive primitive;
    primitive.type = shape_msgs::SolidPrimitive::BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.85;
    primitive.dimensions[1] = 1.5;
    primitive.dimensions[2] = 0.08;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose);

    // Add the collision object to the planning scene
    planning_scene->processCollisionObjectMsg(collision_object);

    // Update the planning scene
    planning_scene->getCurrentStateNonConst().update();

    srand(time(NULL));

    // params
	int numOfDOFs = 6;
    int whichPlanner = 2;
    string outputFile = "output.txt";
    // string start_pos_str = "0.0,0.0,0.0,0.0,0.0,0.0";
    string start_pos_str = "0,-29,-31,0,61,0";
    string goal_pos_str = "-135,32,-48,-135,77,-9";

    double* startPos = doubleArrayFromString(start_pos_str); // convert to arr and convert to radians
    double* goalPos = doubleArrayFromString(goal_pos_str);

    cout << "start position: " << endl;
    for (size_t i = 0; i < numOfDOFs; i++) {
        cout << startPos[i] << endl;
    }
    cout << "goal position: " << endl;
    for (size_t i = 0; i < numOfDOFs; i++) {
        cout << goalPos[i] << endl;
    }

    // check initial config
	if(!IsValidArmConfiguration(planning_scene, kinematic_model, startPos)||
			!IsValidArmConfiguration(planning_scene, kinematic_model, goalPos)) {
		throw runtime_error("Invalid start or goal configuration!\n");
	}

	double** plan = NULL;
	int planlength = 0;

    // Call the corresponding planner function
    if (whichPlanner == RRT)
    {
        plannerRRT(startPos, goalPos, numOfDOFs, &plan, &planlength);
    }
    else if (whichPlanner == RRTCONNECT)
    {
        plannerRRTConnect(startPos, goalPos, numOfDOFs, &plan, &planlength);
    }
    else if (whichPlanner == RRTSTAR)
    {
        plannerRRTStar(startPos, goalPos, numOfDOFs, &plan, &planlength);
    }

    // The solution's path should start with startPos and end with goalPos
    if (!equalDoubleArrays(plan[0], startPos, numOfDOFs) || 
    	!equalDoubleArrays(plan[planlength-1], goalPos, numOfDOFs)) {
		throw std::runtime_error("Start or goal position not matching");
	}

    // Vector of vectors to store the trajectory
    std::vector<std::vector<double>> trajectory;

    cout << "Generated Trajectory:" << endl;
    for (int i = 0; i < planlength; ++i) {
        // Temporary vector to store the current set of joint angles
        std::vector<double> jointAngles;
        
        for (int k = 0; k < numOfDOFs; ++k) {
            cout << plan[i][k] << ",";

            // Store the joint angle in the temporary vector
            jointAngles.push_back(plan[i][k]);
        }

        // Add the set of joint angles to the trajectory
        trajectory.push_back(jointAngles);

        cout << endl;
    }

    // Publish trajectory to arm
    int count = 1;
    control_msgs::FollowJointTrajectoryActionGoal joint_goal;
    ros::Time start_time = ros::Time::now();

    joint_goal.header.seq = 0;
    joint_goal.header.stamp = start_time;
    joint_goal.header.frame_id = "";
    joint_goal.goal_id.stamp = start_time;
    joint_goal.goal_id.id = "get_a_bottle";

    joint_goal.goal.trajectory.header.seq = 0;
    joint_goal.goal.trajectory.header.stamp = ros::Time(0);
    joint_goal.goal.trajectory.header.frame_id = "world";
    joint_goal.goal.trajectory.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

    for (const auto &pos : trajectory) {
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions = pos;
        point.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        point.accelerations = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        point.time_from_start = ros::Duration(count); // Adjust the time_from_start as needed
        joint_goal.goal.trajectory.points.push_back(point);
        count+=5;
    }

    traj_publisher.publish(joint_goal);

    ros::Duration(10.0).sleep();

    while (ros::ok()) {
        loop_rate.sleep();
    }



}