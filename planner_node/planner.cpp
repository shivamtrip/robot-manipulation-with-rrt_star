/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/


// MOVE IT IMPORTS
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
//

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

// #include "RRT.hpp"

/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ARMSTART_IN	prhs[1]
#define	ARMGOAL_IN     prhs[2]
#define	PLANNER_ID_IN     prhs[3]

/* Planner Ids */
#define RRT         0
#define RRTCONNECT  1
#define RRTSTAR     2
#define PRM         3

/* Output Arguments */
#define	PLAN_OUT	plhs[0]
#define	PLANLENGTH_OUT	plhs[1]

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define PI 3.141592654

//the length of each link in the arm
#define LINKLENGTH_CELLS 10

// Some potentially helpful imports
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

// Helper functions - remnants of HW

double* doubleArrayFromString(string str) {
	vector<string> vals = split(str, ",");
	double* ans = new double[vals.size()];
	for (int i = 0; i < vals.size(); ++i) {
		ans[i] = std::stod(vals[i]);
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

//*******************************************************************************************************************//
//                                                                                                                   //
//                                              COLLISION CHECKING                                                   //
//                                                                                                                   //
//*******************************************************************************************************************//


int IsValidLineSegment(double* start, double* end) {
    // TODO: tune based on speed/accuracy constraints
    int divisions = 10;
    double* increment = (start - end)/10;
    for (int i = 0; i < divisions; ++i) {
        double* intermediate = start + i*increment;
        if (!IsValidArmConfiguration(intermediate)) {
            return 0;
        }
    }
    return 1;
}

int IsValidArmConfiguration(double* state) {
    // TODO: move robot_model_loader and planning_scene to global scope, do not neet to instantiate every time

     // Load the robot model
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    planning_scene::PlanningScene planning_scene(kinematic_model);

    // Create a RobotState object for collision checking
    robot_state::RobotState current_state(kinematic_model);

    std::vector<double> joint_values = {state[0], state[1], state[2], state[3], state[4], state[5]}; // Replace with your joint values
    current_state.setJointGroupPositions("xarm6", joint_values); // "arm" is the planning group name for your xArm6

    // Check for collisions
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    planning_scene.checkCollision(collision_request, collision_result, current_state);
	
    if(!collision_result.collision){
		//check the validity of the corresponding line segment
		if(!IsValidLineSegment(double* state, double* state_prime))
			return 0;
	}    
	return 1;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                          DEFAULT PLANNER FUNCTION                                                 //
//                                                                                                                   //
//*******************************************************************************************************************//

static void planner(
			double* map,
			int x_size,
			int y_size,
			double* armstart_anglesV_rad,
			double* armgoal_anglesV_rad,
            int numofDOFs,
            double*** plan,
            int* planlength)
{
	//no plan by default
	*plan = NULL;
	*planlength = 0;
		
    //for now just do straight interpolation between start and goal checking for the validity of samples

    double distance = 0;
    int i,j;
    for (j = 0; j < numofDOFs; j++){
        if(distance < fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]))
            distance = fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]);
    }
    int numofsamples = (int)(distance/(PI/20));
    if(numofsamples < 2){
        printf("the arm is already at the goal\n");
        return;
    }
    *plan = (double**) malloc(numofsamples*sizeof(double*));
    int firstinvalidconf = 1;
    for (i = 0; i < numofsamples; i++){
        (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
        for(j = 0; j < numofDOFs; j++){
            (*plan)[i][j] = armstart_anglesV_rad[j] + ((double)(i)/(numofsamples-1))*(armgoal_anglesV_rad[j] - armstart_anglesV_rad[j]);
        }
        if(!IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size) && firstinvalidconf) {
            firstinvalidconf = 1;
            printf("ERROR: Invalid arm configuration!!!\n");
        }
    }
    *planlength = numofsamples;
    
    return;
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
	    double *map;
    	int x_size;
    	int y_size;
    	vector<double> start;
    	vector<double> goal;
    	int numofDOFs;

		RRTPlanner(double epsilon, double* map, int x_size, int y_size, vector<double> start, vector<double> goal, int numofDOFs):
			epsilon(epsilon), map(map), x_size(x_size), y_size(y_size), start(start), goal(goal), numofDOFs(numofDOFs) {}

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
        if (IsValidArmConfiguration(config_arr, numofDOFs, map, x_size, y_size)) {
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
            // generate random configuration
            // for (size_t i = 0; i < numofDOFs; i++) {
                // qRand.values[i] = ((double) rand() / RAND_MAX) * 2 * M_PI; // random value between 0 and 2pi
            // }
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
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength)
{
    // start clock
    auto start_time = chrono::high_resolution_clock::now();

    // initialize values
	double epsilon = 0.1; 
	vector<double> start(armstart_anglesV_rad, armstart_anglesV_rad+numofDOFs);
	vector<double> goal(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs);
	RRTPlanner rrt(epsilon,map,x_size,y_size,start,goal,numofDOFs);

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
            // for (size_t i = 0; i < numofDOFs; i++) {
            //     qRand.values[i] = ((double) rand() / RAND_MAX) * 2 * M_PI;
            // }
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
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength)
{
    // start clock
    auto start_time = chrono::high_resolution_clock::now();

    // initialize values
	double epsilon = 0.1;
	vector<double> start(armstart_anglesV_rad, armstart_anglesV_rad+numofDOFs);
	vector<double> goal(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs);
	RRTPlanner rrt(epsilon,map,x_size,y_size,start,goal,numofDOFs);

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
        auto nearby = nearbyNodes(qNew, 10);
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
        if (biasProbability <= 0.1) {
            qRand = qGoal;
        } else {
            // renerate random configuration
            // for (size_t i = 0; i < numofDOFs; i++) {
                // qRand.values[i] = ((double) rand() / RAND_MAX) * 2 * M_PI; // random value between 0 and 2pi
            // }
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
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength)
{
    // start clock
    auto start_time = chrono::high_resolution_clock::now();

    // initialize values
	double epsilon = 0.1;
	vector<double> start(armstart_anglesV_rad, armstart_anglesV_rad+numofDOFs);
	vector<double> goal(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs);
	RRTPlanner rrt(epsilon,map,x_size,y_size,start,goal,numofDOFs);

    // plan
	Node* result = rrt.buildRRTStar(100000);

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
//                                              PRM IMPLEMENTATION                                                   //
//                                                                                                                   //
//*******************************************************************************************************************//

// interpolate between angles
double* interpolate(double* start, 
                    double* end, 
                    double alpha,
					int numofDOFs)
{
    double* intermediate = new double[numofDOFs];
    for (int i = 0; i < numofDOFs; ++i) {
        intermediate[i] = (1 - alpha) * start[i] + alpha * end[i];
    }
    return intermediate;
}

// check validity of an edge
bool checkEdge( double* start, 
			    double* end, 
                int numofDOFs, 
                int steps,
				double* map,
				int x_size,
				int y_size)
{
    for (int i = 0; i <= steps; ++i) {
        double alpha = static_cast<double>(i) / steps;
        double* point = interpolate(start, end, alpha, numofDOFs);
		if (!IsValidArmConfiguration(point, numofDOFs, map, x_size, y_size)){
			return false;
		}
    }
    return true;
}

// generate a random configuration
double* randomConfig(int numofDOFs) {
	double* config = new double[numofDOFs];
    for (int i = 0; i < numofDOFs; ++i) {
        config[i] = ((double) rand() / RAND_MAX) * 2 * M_PI;
    }
    return config;
}

// function to get euclidean distance between two joint configs
double getDistance(double* a, double* b, int numofDOFs){
	double sum = 0.0;
	for (size_t i = 0; i < numofDOFs; ++i) {
		double diff = abs(a[i] - b[i]);
		double dist = min(diff, 2*M_PI-diff);
		sum += dist*dist;
	}
	return sqrt(sum);
}

// get vertices in neighborhood
vector<double*> getNeighbors(double neighborhood_size, double* vertex, unordered_map<int, double*> nodes, int numofDOFs){
	vector<double*> neighbors;
    for (const auto& n : nodes) {
        if (getDistance(vertex, n.second, numofDOFs) <= neighborhood_size) {
            neighbors.push_back(n.second);
        }
    }
    return neighbors;
}

// add edge
static void add_edge(unordered_map<int, unordered_set<int>>& edges, int alpha_i, int q_i){
	// if alpha_i in edges, add edge to q_i
    if (alpha_i == q_i){
        return;
    }
    else if (edges.find(alpha_i) != edges.end()) {
        edges[alpha_i].insert(q_i);
    }
	// add alpha_i to edges and add edge to q_i
	else {
        unordered_set<int> q_set = {q_i};
        edges.insert(make_pair(alpha_i, q_set));
    }

    // add reverse order as well
    if (edges.find(q_i) != edges.end()) {
        edges[q_i].insert(alpha_i);
    }
	else {
        unordered_set<int> q_set = {alpha_i};
        edges.insert(make_pair(q_i, q_set));
    }
}

// find closest node, add to nodes list, and add edge
void connectClosest(double* vertex, unordered_map<int, double*>& nodes, int numofDOFs, unordered_map<int, unordered_set<int>>& edges, int index, bool start){
    int i;
    double min_dist = numeric_limits<double>::max();
    for (const auto& n : nodes) {
        double dist = getDistance(vertex, n.second, numofDOFs);
        if (dist <= min_dist) {
            min_dist = dist;
            i = n.first;
        }
    }
    if (start){
        add_edge(edges,index,i);
    }
    else{
        add_edge(edges,i,index);
    }
    
    nodes.insert(make_pair(index, vertex));
}

// get index from node
int getNodeIndex(const unordered_map<int, double*>& nodes, double* node){
	for (const auto& n: nodes){
		if (n.second == node){
			return n.first;
		}
	}
	cout << "node not found" << endl;
	return -10; // node not found
}

struct AStarNode {
    int index;
    double cost; 
    double heuristic;
    int parent;
};

struct CompareAStarNode {
    bool operator()(const AStarNode& n1, const AStarNode& n2) const {
        return (n1.cost + n1.heuristic) > (n2.cost + n2.heuristic);
    }
};

vector<int> searchGraph(int startIdx, 
                        int goalIdx,
                        unordered_map<int, unordered_set<int>>& edges,
                        unordered_map<int, double*>& nodes,
                        int numofDOFs) {
    
	priority_queue<AStarNode, vector<AStarNode>, CompareAStarNode> openList;
    unordered_set<int> closedList;
    unordered_map<int, AStarNode> parentMap;

    openList.push({startIdx, 0, getDistance(nodes[startIdx], nodes[goalIdx], numofDOFs), -1});

    while (!openList.empty()) {
        AStarNode current = openList.top();
        openList.pop();

        if (current.index == goalIdx) {
            cout << "FOUND GOAL" << endl;
            // Backtrack and return the plan
            vector<int> plan;
            while (current.parent != -1) {
                plan.push_back(current.index);
                current = parentMap[current.parent];
            }
            plan.push_back(startIdx);
            reverse(plan.begin(), plan.end());
            return plan;
        }

        if (closedList.find(current.index) != closedList.end()) {
            continue;
        }

        closedList.insert(current.index);
        parentMap[current.index] = current;

        for (int neighbor : edges[current.index]) {
            if (closedList.find(neighbor) == closedList.end()) {
                double newCost = current.cost + getDistance(nodes[current.index], nodes[neighbor], numofDOFs);
                double heuristic = getDistance(nodes[neighbor], nodes[goalIdx], numofDOFs);
                openList.push({neighbor, newCost, heuristic, current.index});
            }
        }
    }

    return {}; // return an empty plan if no path is found
}

static void plannerPRM(
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength)
{

    // start clock
    auto start_time = chrono::high_resolution_clock::now();

    // initialize values
	int steps = 20;
	double neighborhood_size = 20;
	unordered_map<int, unordered_set<int>> edges;
	unordered_map<int, double*> nodes;
	int i = 0;

    // build graph
	while (i < 1000){

		// get random vertex
		double* alpha = randomConfig(numofDOFs);

		// add vertex to graph if in Cfree
		if (IsValidArmConfiguration(alpha, numofDOFs, map, x_size, y_size)){
			nodes.insert(make_pair(i, alpha));

			// check neighborhood for points
			vector<double*> neighbors = getNeighbors(neighborhood_size, alpha, nodes, numofDOFs);

			// add edges
    		for (const auto& q : neighbors) {
				if (checkEdge(alpha, q, numofDOFs, steps, map, x_size, y_size)){
					int q_i = getNodeIndex(nodes, q);
                    if (edges[i].size() < 10){
                        add_edge(edges, q_i, i);
                    }
				}
			}
			i++;
		}
	}

    // connect closest nodes to start and goal
    int startIdx = i+1;
    int goalIdx = i+2;
    connectClosest(armstart_anglesV_rad, nodes, numofDOFs, edges, startIdx, true);
    connectClosest(armgoal_anglesV_rad, nodes, numofDOFs, edges, goalIdx, false);

    // search graph using A*
    vector<int> pathIndices = searchGraph(startIdx, goalIdx, edges, nodes, numofDOFs);

    // planning time
    auto plan_end = chrono::high_resolution_clock::now();
    auto planning_time = chrono::duration_cast<chrono::milliseconds>(plan_end - start_time);

    // populate plan
    if (!pathIndices.empty()) {
        *planlength = pathIndices.size();
        *plan = (double**) malloc(*planlength * sizeof(double*));
        
        for (int i = 0; i < *planlength; ++i) {
            (*plan)[i] = nodes[pathIndices[i]];
        }
    }
	else {
        cout << "Failed to find a path." << endl;
    }

    // full solution time
    auto end_time = chrono::high_resolution_clock::now();
    auto total_time = chrono::duration_cast<chrono::milliseconds>(end_time - start_time);

    // planner metrics
    string under_five = (total_time.count() < 5000) ? "Yes" : "No";
    double plan_quality = getPlanQuality(plan, planlength, numofDOFs);
    cout << "Algorithm: PRM" << endl;
    cout << "Degrees of Freedom: " << numofDOFs << endl;
    cout << "Planning Time: " << planning_time.count() << " milliseconds" << endl;
    cout << "Generated Solution in Under Five Seconds: " << under_five << endl;
    cout << "Nodes Generated: " << nodes.size() << endl;
    cout << "Path Quality: " << plan_quality << endl;
    cout << "Path Length: " << *planlength << endl;

}


//*******************************************************************************************************************//
//                                                                                                                   //
//                                                MAIN FUNCTION                                                      //
//                                                                                                                   //
//*******************************************************************************************************************//

/** Your final solution will be graded by an grading script which will
 * send the default 6 arguments:
 *    map, numOfDOFs, commaSeparatedStartPos, commaSeparatedGoalPos, 
 *    whichPlanner, outputFilePath
 * An example run after compiling and getting the planner.out executable
 * >> ./planner.out map1.txt 5 1.57,0.78,1.57,0.78,1.57 0.392,2.35,3.14,2.82,4.71 0 output.txt
 * See the hw handout for full information.
 * If you modify this for testing (e.g. to try out different hyper-parameters),
 * make sure it can run with the original 6 commands.
 * Programs that do not will automatically get a 0.
 * */
int main(int argc, char** argv) {
	double* map;
	int x_size, y_size;
    srand(time(NULL));

	tie(map, x_size, y_size) = loadMap(argv[1]);
	const int numOfDOFs = std::stoi(argv[2]);
	double* startPos = doubleArrayFromString(argv[3]);
	double* goalPos = doubleArrayFromString(argv[4]);
	int whichPlanner = std::stoi(argv[5]);
	string outputFile = argv[6];

    // find a random valid start and goal position (only need this section for the write up)
    while(true){
        // for (size_t i = 0; i < numOfDOFs; i++) {
        //     startPos[i] = ((double) rand() / RAND_MAX) * 2 * M_PI;
        //     goalPos[i] = ((double) rand() / RAND_MAX) * 2 * M_PI;   
        // }
        startPos[0] = ((double) rand() / RAND_MAX) * 2 * M_PI; // 0 to 2pi
        startPos[1] = ((double) rand() / RAND_MAX) * 4 - 2; // -2 to 2
        startPos[2] = ((double) rand() / RAND_MAX) * -3.9 + 0.1; // -3.8 to 0.1
        startPos[3] = ((double) rand() / RAND_MAX) * 2 * M_PI; // 0 to 2pi
        startPos[4] = ((double) rand() / RAND_MAX) * -4.6 + 3; // -1.6 to 3.0
        startPos[5] = ((double) rand() / RAND_MAX) * 2 * M_PI; // 0 to 2pi

        goalPos[0] = ((double) rand() / RAND_MAX) * 2 * M_PI; // 0 to 2pi
        goalPos[1] = ((double) rand() / RAND_MAX) * 4 - 2; // -2 to 2
        goalPos[2] = ((double) rand() / RAND_MAX) * -3.9 + 0.1; // -3.8 to 0.1
        goalPos[3] = ((double) rand() / RAND_MAX) * 2 * M_PI; // 0 to 2pi
        goalPos[4] = ((double) rand() / RAND_MAX) * -4.6 + 3; // -1.6 to 3.0
        goalPos[5] = ((double) rand() / RAND_MAX) * 2 * M_PI; // 0 to 2pi

        if( IsValidArmConfiguration(startPos, numOfDOFs, map, x_size, y_size) && IsValidArmConfiguration(goalPos, numOfDOFs, map, x_size, y_size) ){
            break;
        }
    }
    cout << "start position: " << endl;
    for (size_t i = 0; i < numOfDOFs; i++) {
        cout << startPos[i] << endl;
    }
    cout << "goal position: " << endl;
    for (size_t i = 0; i < numOfDOFs; i++) {
        cout << goalPos[i] << endl;
    }


	if(!IsValidArmConfiguration(startPos, numOfDOFs, map, x_size, y_size)||
			!IsValidArmConfiguration(goalPos, numOfDOFs, map, x_size, y_size)) {
		throw runtime_error("Invalid start or goal configuration!\n");
	}

	///////////////////////////////////////
	//// Feel free to modify anything below. Be careful modifying anything above.

	double** plan = NULL;
	int planlength = 0;

    // Call the corresponding planner function
    if (whichPlanner == PRM)
    {
        plannerPRM(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }
    else if (whichPlanner == RRT)
    {
        plannerRRT(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }
    else if (whichPlanner == RRTCONNECT)
    {
        plannerRRTConnect(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }
    else if (whichPlanner == RRTSTAR)
    {
        plannerRRTStar(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }
    else
    {
        planner(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }

	//// Feel free to modify anything above.
	//// If you modify something below, please change it back afterwards as my 
	//// grading script will not work and you will recieve a 0.
	///////////////////////////////////////

    // Your solution's path should start with startPos and end with goalPos
    if (!equalDoubleArrays(plan[0], startPos, numOfDOFs) || 
    	!equalDoubleArrays(plan[planlength-1], goalPos, numOfDOFs)) {
		throw std::runtime_error("Start or goal position not matching");
	}

	/** Saves the solution to output file
	 * Do not modify the output log file output format as it is required for visualization
	 * and for grading.
	 */
	std::ofstream m_log_fstream;
	m_log_fstream.open(outputFile, std::ios::trunc); // Creates new or replaces existing file
	if (!m_log_fstream.is_open()) {
		throw std::runtime_error("Cannot open file");
	}
	// m_log_fstream << argv[1] << endl; // Write out map name first
	/// Then write out all the joint angles in the plan sequentially
	for (int i = 0; i < planlength; ++i) {
		for (int k = 0; k < numOfDOFs; ++k) {
			m_log_fstream << plan[i][k] << ",";
		}
		m_log_fstream << endl;
	}
}