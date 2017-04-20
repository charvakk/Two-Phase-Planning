/*
 * 2phase_planning.cpp
 *
 *  Created on: Mar 28, 2017
 *      Author: Charvak Kondapalli
 */

#include "2phase_planning.h"
#include <algorithm>
#include <limits>
#include <math.h>
#include <openrave/utils.h>
#include <openrave/planningutils.h>
#include <ctime>
#include <iostream>
#include <fstream>
#include <set>

#define COMPUTATION_TIME 35000
#define INPUT_SIZE 52
#define CLOSE_ENOUGH 0.2
#define SMOOTHING_ITERATIONS 200

typedef boost::shared_ptr<Node> NodePtr;
typedef boost::shared_ptr<NodeTree> TreePtr;

/* Node method implementations.*/
int Node::sm_nodeCounter = 0;

Node::Node(vector<dReal> configuration, NodePtr parent):
m_vdconfiguration(configuration),
m_ffCost(0),
m_fgCost(0),
m_fhCost(0),
m_pparentNode(parent)
{
m_iid = ++sm_nodeCounter;
}

Node::Node():
m_vdconfiguration(),
m_ffCost(0),
m_fgCost(0),
m_fhCost(0),
m_pparentNode(nullptr)
{
m_iid = ++sm_nodeCounter;
}

const vector<dReal>& Node::getConfiguration() const{
  return m_vdconfiguration;
}

void Node::setConfiguration(const vector<dReal>& configuration){
  m_vdconfiguration = configuration;
}

int Node::getId() const{
  return m_iid;
}

const NodePtr Node::getParentNode() const{
  return m_pparentNode;
}

void Node::setParentNode(const NodePtr parentNode){
  m_pparentNode = parentNode;
}

bool Node::operator ==(Node& other){
  for(size_t i = 0; i < m_vdconfiguration.size(); ++i){
    if(m_vdconfiguration[i] != other.getConfiguration()[i])
      return false;
  }
  return true;
}

bool Node::operator !=(Node& other){
  return !this->operator ==(other);
}

float Node::getFCost() const{
  return m_ffCost;
}

void Node::setFCost(float fCost){
  m_ffCost = fCost;
}

float Node::getGCost() const{
  return m_fgCost;
}

void Node::setGCost(float gCost){
  m_fgCost = gCost;
}

float Node::getHCost() const{
  return m_fhCost;
}

void Node::setHCost(float hCost){
  m_fhCost = hCost;
}

float Node::getEKey() const{
  return m_feKey;
}

void Node::setEKey(float eKey){
    m_feKey = eKey;
}

/*---------------------------------------------------------------------------------------------------------------------------*/


/* NodeTree method implementations.*/
NodePtr NodeTree::getMostRecentNode() const {
  return m_vpnodes.back();
}

bool NodeTree::addNode(NodePtr p_node){
  try{
    m_vpnodes.push_back(p_node);
    return true;
  }catch(exception &e){
    cout << e.what() << endl;
    return false;
  }
}

void NodeTree::deleteNode(NodePtr node){
  m_vpnodes.erase(std::remove(m_vpnodes.begin(), m_vpnodes.end(), node), m_vpnodes.end());
  }

NodePtr NodeTree::getNode(int id){
  for(NodePtr n : m_vpnodes){
    if(n->getId() == id)
      return n;
  }
  throw 0;
}

vector<NodePtr> NodeTree::getPathTo(int id){
  // Find the node
  NodePtr final = getNode(id);

  // Find the path from the parents
  vector<NodePtr> path;
  path.push_back(final);
  while(final->getParentNode() != nullptr){
    final = final->getParentNode();
    path.push_back(final);
  }
  return path;
}

vector<NodePtr>& NodeTree::getAllNodes(){
  return m_vpnodes;
}

size_t NodeTree::getSize(){
  return m_vpnodes.size();
}


/*--------------------------------------------------------------------------------------------------------------------*/


class rrt_module : public ModuleBase {
public:
  rrt_module(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv),
  STEP_SIZE(0.4),
  GOAL_BIAS(31)
  {
    _penv = penv;
    __description = "Implementation of RRT-Connect for RBE 550.";
    RegisterCommand("birrt",boost::bind(&rrt_module::BiRRT,this,_1,_2),
                    "Plans and executes path to given goal configuration using Bidirectional RRTs.");
    RegisterCommand("rrtconnect",boost::bind(&rrt_module::RRTConnect,this,_1,_2),
                    "Plans and executes path to given goal configuration using RRTConnect.");
    RegisterCommand("rrtstar",boost::bind(&rrt_module::RRTStar,this,_1,_2),
                    "Plans and executes path to given goal configuration using RRTStar.");
    RegisterCommand("astar",boost::bind(&rrt_module::AStar,this,_1,_2),
                    "Plans and executes path to given goal configuration using AStar.");
    RegisterCommand("arastar",boost::bind(&rrt_module::ARAStar,this,_1,_2),
                    "Plans and executes path to given goal configuration using ARAStar.");
    RegisterCommand("anastar",boost::bind(&rrt_module::ANAStar,this,_1,_2),
                    "Plans and executes path to given goal configuration using ANAStar.");
    RegisterCommand("setbias",boost::bind(&rrt_module::SetBias,this,_1,_2),
                    "Sets the goal bias for planning with the RRT-Connect. Default: 31");
    RegisterCommand("test",boost::bind(&rrt_module::Test,this,_1,_2),
                        "Test");
    RegisterCommand("set_step_size",boost::bind(&rrt_module::SetStepSize,this,_1,_2),
                        "Sets the Step Size for planning with the AStar variants. Default: 0.4");
  }
  virtual ~rrt_module() {}

  bool SetBias(ostream& sout, istream& sin){
    vector<dReal> in = GetInputAsVector(sout, sin);
    try{
      GOAL_BIAS = in[0];
    }catch(exception &e){
      cout << e.what() << endl;
      return false;
    }
    cout << "Goal bias set to " << GOAL_BIAS << endl;
    return true;
  }

  bool SetStepSize(ostream& sout, istream& sin){
    vector<dReal> in = GetInputAsVector(sout, sin);
    try{
    STEP_SIZE = in[0];
    }catch(exception &e){
      cout << e.what() << endl;
      return false;
    }
    cout << "Step size set to " << STEP_SIZE << endl;
    return true;
  }

  /*-----------------------------------------------------------------------------Bi-Directional RRT----------------------------------------------------------------*/

  bool BiRRT(ostream& sout, istream& sin){

    // Initialize private members from the input
    Init(sout, sin);

    // Initialize two trees with the start and goal nodes at the respective roots.
    TreePtr treeA(new NodeTree());
    treeA->addNode(_startNode);

    TreePtr treeB(new NodeTree());
    treeB->addNode(_goalNode);

    for(int k = 0; k < COMPUTATION_TIME; ++k){
      NodePtr randomNode = CreateRandomNode();

      if(Extend(treeA, randomNode) != "Trapped"){
        if(Connect(treeB, treeA->getMostRecentNode()) == "Reached"){
          treeA->getAllNodes().pop_back();
          vector<NodePtr> pathA = treeA->getPathTo(treeA->getMostRecentNode()->getId());
          vector<NodePtr> pathB = treeB->getPathTo(treeB->getMostRecentNode()->getId());
          vector<NodePtr> fullPath;

          std::reverse(pathA.begin(), pathA.end());
          fullPath.reserve(pathA.size() + pathB.size()); // preallocate memory
          fullPath.insert(fullPath.end(), pathA.begin(), pathA.end()); //TODO check if this can be done with end, begin to avoid reversing
          fullPath.insert(fullPath.end(), pathB.begin(), pathB.end());

          vector< vector<dReal> > configPath;
          configPath.reserve(fullPath.size());
          for(NodePtr pnode : fullPath)
            configPath.push_back(pnode->getConfiguration());

          cout << "Found a path!!!" << endl;
          cout << "Executing the path." << endl;

          cout << "Number of nodes explored: " << treeA->getSize() + treeB->getSize() << endl;
          cout << "Path length :" << configPath.size() << endl;

          endTime = clock();

          DrawPath(configPath, red);

          ShortcutSmoothing(fullPath);

          vector< vector<dReal> > configPath2;
          configPath2.reserve(fullPath.size());
          for(NodePtr pnode : fullPath)
            configPath2.push_back(pnode->getConfiguration());

          cout << "Smoothed path length :" << configPath2.size() << endl;

          clock_t endAfterSmoothing = clock();
          DrawPath(configPath2, blue);

          double timeForAlgorithm = (endTime-startTime)/(double)CLOCKS_PER_SEC;
          double timeForSmoothing = (endAfterSmoothing-endTime)/(double)CLOCKS_PER_SEC;

//          WriteBiStuffToFile(timeForAlgorithm, timeForSmoothing, (treeA->getSize()+treeB->getSize()), configPath.size());
//
          cout << "Time for computing the path: " << timeForAlgorithm << endl;
          cout << "Time for smoothing the path: " << timeForSmoothing << endl;
          ExecuteTrajectory(configPath2);
          return true;
        }
      }
      swap(treeA, treeB);
      if(k % 5000 == 0)
        cout << k << ". Searching..." << endl;
    }
    cout << "Time up :(" << endl;
    return false;
  }

  /*-----------------------------------------------------------------------------Bi-Directional RRT----------------------------------------------------------------*/



  /*--------------------------------------------------------------------------------RRT-Connect--------------------------------------------------------------------*/

  bool RRTConnect(ostream& sout, istream& sin){
    // Initialize private members from the input
    Init(sout, sin);

    for(size_t i=0 ; i < _pathAStar.size() ; i++){
        vector<dReal> tempConfig = _pathAStar[i]->getConfiguration();
        cout << "Config ID "<< i << " : " << tempConfig[0] << " " << tempConfig[1] << " " << tempConfig[2] << endl;
    }

    //NodePtr testNode = CreateNodeWithGaussianBias();

    // Initialize the tree with the start node at the root.
    TreePtr tree(new NodeTree());
    tree->addNode(_startNode);

    for(int k = 0; k < COMPUTATION_TIME; ++k){
      //NodePtr randomNode = CreateRandomNodeWithBias();
      //cout << _pathAStar.size() << endl;
      NodePtr randomNode = CreateNodeWithGaussianBias();

      string status = Connect(tree, randomNode);
      if(status == "GoalReached"){
        vector<NodePtr> path = tree->getPathTo(_goalNode->getId());

        vector< vector<dReal> > configPath;
        configPath.reserve(path.size());
        for(NodePtr pnode : path)
          configPath.push_back(pnode->getConfiguration());


        cout << "Found a path!!!" << endl;
        cout << "Executing the path." << endl;

        cout << "Number of nodes explored :" << endl;
        cout << tree->getSize() << endl;

        cout << "Path length: " << configPath.size() << endl;

        endTime = clock();

//        DrawPath(configPath, red);
//        DrawPath(configPath);

        ShortcutSmoothing(path);

        vector< vector<dReal> > configPath2;
        configPath2.reserve(path.size());
        for(NodePtr pnode : path)
          configPath2.push_back(pnode->getConfiguration());

        cout << "Smoothed path length :" << configPath2.size() << endl;

        clock_t endAfterSmoothing = clock();

//        DrawPath(configPath2, blue);
        DrawPath(configPath2);

        double timeForAlgorithm = (endTime-startTime)/(double)CLOCKS_PER_SEC;
        double timeForSmoothing = (endAfterSmoothing-endTime)/(double)CLOCKS_PER_SEC;


        cout << "Time for computing the path: " << timeForAlgorithm << endl;
        cout << "Time for smoothing the path: " << timeForSmoothing << endl;

        //        WriteStuffToFile(timeForAlgorithm, timeForSmoothing, tree->getSize(), configPath.size(), configPath2.size());

        ExecuteTrajectory(configPath2);
        return true;
      }
      if(k % 5000 == 0)
        cout << k << ". Searching..." << endl;
    }
    cout << "Time up :(" << endl;
    return false;
  }

  /*--------------------------------------------------------------------------------RRT-Connect--------------------------------------------------------------------*/


  /*--------------------------------------------------------------------------------RRTStar--------------------------------------------------------------------*/

  bool RRTStar(ostream& sout, istream& sin){
    // Initialize private members from the input
    Init(sout, sin);

    // Initialize the tree with the start node at the root.
    TreePtr tree(new NodeTree());
    tree->addNode(_startNode);

    for(int k = 0; k < COMPUTATION_TIME; ++k){
      NodePtr randomNode = CreateRandomNodeWithBias();

      string status = RRTStarExtend(tree, randomNode);
      if(status == "GoalReached"){
        vector<NodePtr> path = tree->getPathTo(_goalNode->getId());

        vector< vector<dReal> > configPath;
        configPath.reserve(path.size());
        for(NodePtr pnode : path)
          configPath.push_back(pnode->getConfiguration());


        cout << "Found a path!!!" << endl;
        cout << "Executing the path." << endl;

        cout << "Number of nodes explored :" << endl;
        cout << tree->getSize() << endl;

        cout << "Path length: " << configPath.size() << endl;

        endTime = clock();

//        DrawPath(configPath, red);
//        DrawPath(configPath);

        ShortcutSmoothing(path);

        vector< vector<dReal> > configPath2;
        configPath2.reserve(path.size());
        for(NodePtr pnode : path)
          configPath2.push_back(pnode->getConfiguration());
//
        cout << "Smoothed path length :" << configPath2.size() << endl;
//
        clock_t endAfterSmoothing = clock();

//        DrawPath(configPath2, blue);
        DrawPath(configPath2);

        double timeForAlgorithm = (endTime-startTime)/(double)CLOCKS_PER_SEC;
        double timeForSmoothing = (endAfterSmoothing-endTime)/(double)CLOCKS_PER_SEC;


        cout << "Time for computing the path: " << timeForAlgorithm << endl;
        cout << "Time for smoothing the path: " << timeForSmoothing << endl;

        //        WriteStuffToFile(timeForAlgorithm, timeForSmoothing, tree->getSize(), configPath.size(), configPath2.size());

        ExecuteTrajectory(configPath2);
        return true;
      }
      if(k % 500 == 0)
        cout << k << ". Searching..." << endl;
    }
    cout << "Time up :(" << endl;
    return false;
  }

  /*--------------------------------------------------------------------------------RRTStar--------------------------------------------------------------------*/


  /*--------------------------------------------------------------------------------AStar--------------------------------------------------------------------*/

  bool AStar(ostream& sout, istream& sin){

    InitAStar(sout, sin);
    string status;

    while(_openSet.size() != 0){
      NodePtr currentNode = *_openSet.begin();
      _openSet.erase(_openSet.begin());

      DrawPoint(currentNode);

      if(CheckCollision(currentNode))
        continue;
      else{
        if(UnweightedDistance(currentNode, _goalNode) < STEP_SIZE){
          status = "Found";
          _goalNode->setParentNode(currentNode);
          vector<NodePtr> path = GetAStarPath();

          vector< vector<dReal> > configPath;
          configPath.reserve(path.size());
          for(NodePtr pnode : path)
            configPath.push_back(pnode->getConfiguration());

          cout << "Found a path!!!" << endl;
          cout << "Executing the path." << endl;

          //        cout << "Number of nodes explored :" << endl;
          //        cout << tree->getSize() << endl;

          cout << "Path length: " << configPath.size() << endl;

          endTime = clock();
          DrawPath(configPath);

          double timeForAlgorithm = (endTime-startTime)/(double)CLOCKS_PER_SEC;

          cout << "Time for computing the path: " << timeForAlgorithm << endl;

          //        WriteStuffToFile(timeForAlgorithm, timeForSmoothing, tree->getSize(), configPath.size(), configPath2.size());

          ExecuteTrajectory(configPath);
          cout << "Path found!" << endl;
          return true;
        }

        vector<NodePtr> neighbors = GetNeighbors(currentNode);
        for(NodePtr neighbor : neighbors){
          neighbor->setGCost(currentNode->getGCost() + UnweightedDistance(currentNode, neighbor));
          neighbor->setHCost(UnweightedDistance(neighbor, _goalNode));
          neighbor->setFCost(neighbor->getGCost() + neighbor->getHCost());

          multiset<NodePtr>::iterator it1 = FindInOpenSet(neighbor);
          if(it1 != _openSet.end()){
            NodePtr nodeInOpenSet = *it1;
            if(nodeInOpenSet->getFCost() < neighbor->getFCost())
              continue;
            else
              _openSet.erase(it1);
          }

          vector<NodePtr>::iterator it2 = FindInClosedSet(neighbor);
          if(it2 != _closedSet.end()){
            NodePtr nodeInClosedSet = *it2;
            if(nodeInClosedSet->getFCost() < neighbor->getFCost())
              continue;
            else
              _closedSet.erase(it2);
          }

          _openSet.insert(neighbor);
        }

        _closedSet.push_back(currentNode);
      }
    }
    cout << "Path doesn't exist." << endl;
    return false;
  }

  /*--------------------------------------------------------------------------------AStar--------------------------------------------------------------------*/



  /*--------------------------------------------------------------------------------ARAStar--------------------------------------------------------------------*/

  bool ARAStar(ostream& sout, istream& sin){
    InitAStar(sout, sin);
    string status;

    _epsilon = 2;
    _epsilonDash = 0;
    _epsilonDelta = 0.2;

    _goalNode->setGCost(9999);
    _goalNode->setFCost(_goalNode->getGCost());
    _startNode->setGCost(0);
    _startNode->setFCost(_goalNode->getGCost());

    ImprovePath();
    updateEpsilon();

    while(_epsilonDash > 1){
        cout << "Current Epsilon.. " << _epsilon << endl;
        _epsilon -= _epsilonDelta;
        _closedSet.clear();
        ImprovePath();
        updateEpsilon();

    }

    cout << "path length :" << _pathAStar.size() << endl;
    cout << "................ARA Ending................." << endl;
    return true;
  }

  /*--------------------------------------------------------------------------------ARAStar--------------------------------------------------------------------*/


  /*--------------------------------------------------------------------------------ANAStar--------------------------------------------------------------------*/

  bool ANAStar(ostream& sout, istream& sin){
    _G_ANA = 99999;
    _E_ANA = 99999;
    InitANAStar(sout, sin);
    cout << "Inside ANA Star .. " << _openSet_ANA.size() << endl;

    while(_openSet_ANA.size() != 0){
        cout << "Im here" << endl;
        ImproveSolution();
        cout << "Current Optimal Solution _E_ANA :: " << _E_ANA << endl;
        updateOpenList();
    }

    cout << "Ana Finished " << endl;
    return true;
  }

  /*--------------------------------------------------------------------------------ANAStar--------------------------------------------------------------------*/


  bool Test(ostream& sout, istream& sin){
    NodePtr node1(new Node());
    NodePtr node2(new Node());
    NodePtr node3(new Node());

    node1->setFCost(5);
    node1->setConfiguration(vector<dReal>{1, 1, 1});

//    node2->setFCost(2);
//    node2->setConfiguration(vector<dReal>{2, 2, 2});
//
//    node3->setFCost(7);
//    node3->setConfiguration(vector<dReal>{3, 3, 3});
//
//    _openSet.insert(node1);
//    _openSet.insert(node2);
//    _openSet.insert(node3);

//    for(int i = 0; i < 3; ++i){
//      NodePtr node = *_openSet.begin();
//      _openSet.erase(_openSet.begin());
//      cout << node->getFCost() << endl;
//    }

//    NodePtr nodecheck(new Node(vector<dReal>{1, 1, 1}, nullptr));
////    nodecheck->setFCost(5);
////    auto it = _openSet.find(nodecheck);
//    for(auto node : _openSet){
//      if(*node == *nodecheck){
//        cout << "found" << node->getFCost() << endl;
//        break;
//      }
//    }
//    if(it != _openSet.end()){
//      NodePtr node = *it;
//      cout << "found" << node->getFCost() << endl;
//    }else
//      cout << "not found\n";

    vector<NodePtr> neighbors = GetNeighbors(node1);
    for(auto n : neighbors){
      for(size_t i = 0; i < n->getConfiguration().size(); ++i){
        cout << n->getConfiguration().at(i) << " ";
      }
      cout << endl;
    }

    cout << "number of neighbors: " << neighbors.size() << endl;
    return true;
  }

  /* Initializes the members by calling the input parser. */
  void Init(ostream& so, istream& si){
    _penv->GetRobots(_robots);
    _robot = _robots.at(0);

    srand(time(NULL));
    _robot->GetActiveDOFValues(_startConfig);
    _goalConfig = GetInputAsVector(so, si);
    _robot->GetActiveDOFLimits(_activeLowerLimits, _activeUpperLimits);
//    assert(_goalConfig.size() == 7 && "goalConfig should be of size 7!");
//    assert(_startConfig.size() == 7 && "startConfig wasn't size 7 :(");

    _activeDOFRanges.reserve(_activeLowerLimits.size());
    for(size_t i = 0; i < _activeLowerLimits.size(); ++i){
      cout << "lower: " << _activeLowerLimits[i] << " upper: " << _activeUpperLimits[i] << endl;
      if(i == 3){// || i == 6){
        _activeUpperLimits[i] = M_PI;
        _activeLowerLimits[i] = -M_PI;
      }
      _activeDOFRanges[i] = _activeUpperLimits[i]-_activeLowerLimits[i];
    }

//    _dofWeights = {3.17104, 2.75674, 2.2325, 1.78948, 0, 0.809013, 0};
    _dofWeights = {1, 1, 1};

    // Root and Goal nodes
    _startNode = NodePtr(new Node(_startConfig, nullptr));
    _goalNode = NodePtr(new Node(_goalConfig, nullptr));
    startTime = clock();
  }

  void InitAStar(ostream& so, istream& si){
    // Get robot
    _penv->GetRobots(_robots);
    _robot = _robots.at(0);

    _robot->GetActiveDOFValues(_startConfig);

    _goalConfig = GetInputAsVector(so, si);

    _startNode = NodePtr(new Node(_startConfig, nullptr));
    _goalNode = NodePtr(new Node(_goalConfig, nullptr));

    _openSet.insert(_startNode);
    startTime = clock();
  }

  void InitANAStar(ostream& so, istream& si){
    // Get robot
    _penv->GetRobots(_robots);
    _robot = _robots.at(0);
    _robot->GetActiveDOFValues(_startConfig);
    _goalConfig = GetInputAsVector(so, si);

    _startNode = NodePtr(new Node(_startConfig, nullptr));
    //_startNode->setEKey(_G_ANA);

    _goalNode = NodePtr(new Node(_goalConfig, nullptr));
    _openSet_ANA.insert(_startNode);
    startTime = clock();
  }

  /*Updates Epsilon for ARA | ANA */
  void updateEpsilon(){
        _openSet.insert(_inconsSet.begin(), _inconsSet.end());
        NodePtr tempNode = *_openSet.begin();
        dReal tempVal = currentNode->getFCost()/tempNode->getFCost();
       _epsilonDash = min(_epsilon, tempVal);
  }

  /*Improve Path function for ARA | ANA takes care for inconsistent list */
  void ImprovePath(){
      //cout << "Im here also :: " << _openSet_ANA.size() << endl;
      while(_openSet.size() != 0){
        //    cout << "Inside Improve Path :: " << _openSet_ANA.size() << endl;
            currentNode = *_openSet.begin();
            _openSet.erase(_openSet.begin());

            DrawPoint(currentNode);
            count_print++;

            if(CheckCollision(currentNode))
                continue;
            else{
                if(UnweightedDistance(currentNode, _goalNode) < STEP_SIZE){
                   // cout << "Current Path Length: : ---- " << _closedSet.size() << endl;

                    _goalNode->setParentNode(currentNode);
                    vector<NodePtr> path = GetAStarPath();
                    _pathAStar = path;

                    vector< vector<dReal> > configPath;
                    configPath.reserve(path.size());
                    for(NodePtr pnode : path)
                        configPath.push_back(pnode->getConfiguration());

                    endTime = clock();
                    DrawPath(configPath);

                    return;
                }

            vector<NodePtr> neighbors = GetNeighbors(currentNode);
            for(NodePtr neighbor : neighbors){
                    neighbor->setGCost(currentNode->getGCost() + UnweightedDistance(currentNode, neighbor));
                    neighbor->setHCost(UnweightedDistance(neighbor, _goalNode));
                    neighbor->setFCost(neighbor->getGCost() + _epsilon * neighbor->getHCost());

                    multiset<NodePtr>::iterator it1 = FindInOpenSet(neighbor);
                    if(it1 != _openSet.end()){
                        NodePtr nodeInOpenSet = *it1;
                        if(nodeInOpenSet->getFCost() < neighbor->getFCost())
                            continue;
                        else
                            _openSet.erase(it1);
                     }

                    //if in closed set
                    vector<NodePtr>::iterator it2 = FindInClosedSet(neighbor);
                    if(it2 != _closedSet.end()){
                        NodePtr nodeInClosedSet = *it2;

                       if(nodeInClosedSet->getFCost() > neighbor->getFCost()){
                            //_inconsSet.insert(neighbor);
                            continue;
                        }else{
                            multiset<NodePtr>::iterator it3 = FindInInconSet(neighbor);
                            if(it3 != _inconsSet.end()){
                                _inconsSet.erase(it3);
                                _inconsSet.insert(neighbor);
                            }else{
                                _inconsSet.insert(neighbor);
                            }
                            continue;
                        }
                    }


                    _openSet.insert(neighbor);
            }

            _closedSet.push_back(currentNode);
            }
        }
        cout << "Path doesn't exist." << endl;
        return;
  }

  /*Improve path for ANAStar */
  void ImproveSolution(){

    while(_openSet_ANA.size() != 0){
        currentNode = *_openSet_ANA.begin();
        _openSet_ANA.erase(_openSet_ANA.begin());
        DrawPoint(currentNode);
        cout << "_openSet_ANA :: " << _openSet_ANA.size() << endl;
        if(CheckCollision(currentNode))
            continue;
        else{

            if(currentNode->getEKey() < _E_ANA){
                cout << "_E_ANA :: " << _E_ANA << endl;
                _E_ANA = currentNode->getEKey();
            }
            if(UnweightedDistance(currentNode, _goalNode) < STEP_SIZE){
                _G_ANA = currentNode->getGCost();
                _goalNode->setParentNode(currentNode);
                vector<NodePtr> path = GetAStarPath();
                _pathAStar = path;

                vector< vector<dReal> > configPath;
                configPath.reserve(path.size());
                for(NodePtr pnode : path)
                    configPath.push_back(pnode->getConfiguration());

                endTime = clock();
                DrawPath(configPath);
                return;
            }

            vector<NodePtr> neighbors = GetNeighbors(currentNode);
            for(NodePtr neighbor : neighbors){
                neighbor->setGCost(currentNode->getGCost() + UnweightedDistance(currentNode, neighbor));
                neighbor->setHCost(UnweightedDistance(neighbor, _goalNode));
                neighbor->setFCost(neighbor->getGCost() + _epsilon * neighbor->getHCost());
                float temp_h = neighbor->getHCost();
                float temp_g = neighbor->getGCost();
                neighbor->setEKey(_G_ANA - temp_g / temp_h);

                multiset<NodePtr>::iterator it1 = FindInOpenSet_ANA(neighbor);
                if(it1 != _openSet_ANA.end()){
                    NodePtr nodeInOpenSet = *it1;
                    if(nodeInOpenSet->getFCost() < neighbor->getFCost())
                        continue;
                    else
                        _openSet_ANA.erase(it1);
                }

                if(neighbor->getGCost() + neighbor->getHCost() < _G_ANA){
                    _openSet_ANA.insert(neighbor);
                }
            }

        }
    }

     cout << "Path does not exist " <<  _openSet_ANA.size() << endl;
     return;
  }

  void updateOpenList(){
        multiset<NodePtr>::iterator it;
        for(it = _openSet_ANA.begin(); it != _openSet_ANA.end(); ++it){
            NodePtr temp = *it;
            if(temp->getFCost() > _G_ANA){
                _openSet_ANA.erase(it++);
            }
        }
  }


  /* Returns a random node without any goal bias. */
  NodePtr CreateRandomNode(){
    vector<dReal> randomConfig(_activeLowerLimits.size());
    NodePtr randomNode(new Node());

    do{
      for(size_t i = 0; i < _activeLowerLimits.size(); ++i)
        randomConfig[i] = static_cast<dReal>((RandomNumberGenerator()/100 * (_activeDOFRanges[i])) + _activeLowerLimits[i]);
      randomNode->setConfiguration(randomConfig);
    }while(CheckCollision(randomNode));

    return randomNode;
  }

  /* Returns a random node with any goal bias. */
  NodePtr CreateRandomNodeWithBias(){
    /*The idea for goal biasing was referenced from the Internet.*/
    vector<dReal> randomConfig(_activeLowerLimits.size());
    NodePtr randomNode(new Node());

    if(RandomNumberGenerator() <= GOAL_BIAS){
      return _goalNode;
    }else{
      do{
        for(size_t i = 0; i < _activeLowerLimits.size(); ++i){
            randomConfig[i] = static_cast<dReal>((RandomNumberGenerator()/100 * (_activeDOFRanges[i])) + _activeLowerLimits[i]);
        }
        randomNode->setConfiguration(randomConfig);
      }while(CheckCollision(randomNode));

      return randomNode;
    }
  }

  /* Returns a random node with gaussian bias, with mean as path from astar_variant. */
  NodePtr CreateNodeWithGaussianBias(){
    /* TODO : change form of _closedSet as required by RRT */
    NodePtr gaussianNode(new Node());
    default_random_engine generator;

    //cout << _pathAStar.size() << endl;

    if(RandomNumberGenerator() <= GOAL_BIAS || _pathAStar.empty()){
        //cout << "Biased From Goal" << endl;
        return _goalNode;

    }else{
        cout << "Following Astar Path " << endl;
        do{
                NodePtr meanNode = _pathAStar.back();
                _pathAStar.pop_back();
                vector<dReal> meanConfig = meanNode->getConfiguration();
                vector<dReal> nextConfig;

                //x
                normal_distribution<float> distribution_x(meanConfig[0],_gaussVarX);
                nextConfig.push_back(distribution_x(generator));
                //y
                normal_distribution<float> distribution_y(meanConfig[1],_gaussVarY);
                nextConfig.push_back(distribution_y(generator));
                //z
                normal_distribution<float> distribution_z(meanConfig[2],_gaussVarZ);
                nextConfig.push_back(distribution_z(generator));

                NodePtr randomNode = CreateRandomNode();
                vector<dReal> randomConfig = randomNode->getConfiguration();

                for(size_t i = 3; i < _activeLowerLimits.size(); ++i){
                    nextConfig.push_back(randomConfig[i]);
                }

                gaussianNode->setConfiguration(nextConfig);

                cout << "Mean Config : " << meanConfig[0] << " " << meanConfig[1] << " " << meanConfig[2] << " " << meanConfig.size() << " " << "Next Config : " << nextConfig[0] << " " << nextConfig[1] << " " << nextConfig[2] << " " << nextConfig[3] ;//<< nextConfig.size() << " " << _activeLowerLimits.size() << " "<< _activeUpperLimits.size() <<  endl;

            }while(CheckCollision(gaussianNode) && !_pathAStar.empty());

            return gaussianNode;

        }

 }




  /* Returns a random number between, and including, 0 and 99.*/
  float RandomNumberGenerator(){
    return rand() % 100;
  }

  /* Checks collision of the robot with the environment and itself and returns true if there is any collision detected. */
  bool CheckCollision(NodePtr node){
    _robot->SetActiveDOFValues(node->getConfiguration());
    bool check1 = _penv->CheckCollision(_robot);
    bool check2 = _penv->CheckSelfCollision(_robot);
    _robot->SetActiveDOFValues(_startConfig);
    return check1 || check2;
  }

  /* Extends one step towards the given node. */
  string Extend(TreePtr tree, NodePtr node){
    NodePtr nearestNode = NearestNode(tree, node);
    NodePtr newNode;
    try{
     newNode = NewStep(nearestNode, node);
    }catch(int &a){
      return "Trapped";
    }

    if(InLimits(newNode) && !CheckCollision(newNode)){
      newNode->setParentNode(nearestNode);
      tree->addNode(newNode);
      if(UnweightedDistance(newNode, node) <= STEP_SIZE){
        node->setParentNode(newNode);
        tree->addNode(node);
        return "Reached";
      }else
        return "Advanced";
    }else
      return "Trapped";
  }

  /* Tries to connect the tree to the given node. */
  string Connect(TreePtr tree, NodePtr node){
    string status;
    NodePtr nearestNode = NearestNode(tree, node);
    NodePtr start = nearestNode;
    do{
      NodePtr newNode;
      try{
          newNode = NewStep(nearestNode, node);
         }catch(int &a){
           return "Trapped";
         }
      if(InLimits(newNode) && !CheckCollision(newNode)){
        newNode->setParentNode(nearestNode);
        tree->addNode(newNode);
        nearestNode = newNode;
        if(UnweightedDistance(newNode, node) <= STEP_SIZE){
          node->setParentNode(newNode);
          tree->addNode(node);
          if(UnweightedDistance(node, _goalNode) <= STEP_SIZE)
            return "GoalReached";
          else
            return "Reached";
        }else
          status = "Advanced";
      }else{
        return "Trapped";}
    }while(status == "Advanced");
    return status;
  }

  /* Extend method for RRTStar. */
  string RRTStarExtend(TreePtr tree, NodePtr node){
    NodePtr nearestNode = NearestNode(tree, node);
    NodePtr newNode;
    try{
      newNode = NewStep(nearestNode, node);
    }catch(int &a){
      return "Trapped";
    }
    newNode->setGCost(nearestNode->getGCost() + STEP_SIZE);

    if(InLimits(newNode) && !CheckCollision(newNode)){
      newNode->setParentNode(nearestNode);
      tree->addNode(newNode);

      NodePtr minNode = nearestNode;
      vector<NodePtr> nearNodes = Near(tree, newNode);
      for(auto near : nearNodes){
        dReal possibleCost = near->getGCost() + UnweightedDistance(near, newNode);
        if(possibleCost < newNode->getGCost()){
          newNode->setGCost(possibleCost);
          minNode = near;
        }
      }
      newNode->setParentNode(minNode);

      for(auto near : nearNodes){
        if(near->getGCost() > newNode->getGCost() + UnweightedDistance(near, newNode))
          near->setParentNode(newNode);
      }

      if(UnweightedDistance(newNode, node) <= STEP_SIZE){
        node->setGCost(newNode->getGCost() + UnweightedDistance(newNode, node));
        node->setParentNode(newNode);
        tree->addNode(node);
        if(UnweightedDistance(node, _goalNode) <= STEP_SIZE)
          return "GoalReached";
        else
          return "Reached";
      }else
        return "Advanced";
    }else
      return "Trapped";
  }

  /* Checks if the configuration of the node is in DOF limits. */
  bool InLimits(NodePtr node){
    vector<dReal> config = node->getConfiguration();
    for(size_t i = 0; i < config.size(); ++i){
      if(config[i] < _activeLowerLimits[i] || config[i] > _activeUpperLimits[i])
        return false;
    }
    return true;
  }

  /* Parses the input from the python script and returns the goal config. */
  vector<dReal> GetInputAsVector(ostream& sout, istream& sinput){
    char input[INPUT_SIZE];
    vector<dReal> goalConfig;
    try{
      vector<string> temp;
      sinput.getline(input, INPUT_SIZE);
      utils::TokenizeString(input, "[ ,]", temp);
      for(string s : temp)
        goalConfig.push_back(atof(s.c_str()));
    }catch(exception &e){
      cout << e.what() << endl;
    }
    return goalConfig;
  }

  /* Generates and executes a given configuration path. */
  void ExecuteTrajectory(vector< vector<dReal> > &configPath){
    EnvironmentMutex& lock = _penv->GetMutex();
    lock.lock();
    TrajectoryBasePtr traj = RaveCreateTrajectory(_penv);
    traj->Init(_robot->GetActiveConfigurationSpecification());


    for(vector<dReal> config : configPath)
      traj->Insert(0, config);
    traj->Insert(0, _startConfig);

    planningutils::RetimeActiveDOFTrajectory(traj, _robot);

    _robot->GetController()->SetPath(traj);

    lock.unlock();
  }

  /* Returns a new node in the direction of the second node at a step_size distance from the first node. */
  NodePtr NewStep(NodePtr from, NodePtr to){
    vector<dReal> unitVector;
    vector<dReal> fromConfig = from->getConfiguration();
    vector<dReal> toConfig = to->getConfiguration();
    dReal mag = UnweightedDistance(from, to);
    if(mag == 0)
      throw 0;
    for(size_t i=0; i < fromConfig.size(); ++i){
      unitVector.push_back((toConfig[i] - fromConfig[i])/mag);
    }
    vector<dReal> newConfig;
    for(size_t i=0; i < fromConfig.size(); ++i){
      newConfig.push_back(fromConfig[i] + (unitVector[i]*STEP_SIZE));
    }
    NodePtr newNode(new Node());
    newNode->setConfiguration(newConfig);
    DrawPoint(newNode);
    return newNode;
  }

  /* Calculates and returns the distance between two nodes taking into consideration the DOF weights. */
  dReal WeightedDistance(NodePtr node1, NodePtr node2){
      dReal distanceSquared = 0;
      vector<dReal> node1Config = node1->getConfiguration();
      vector<dReal> node2Config = node2->getConfiguration();

      // Weighted distance
      for(size_t i = 0; i < node1Config.size(); ++i){
        distanceSquared += pow((node1Config[i] - node2Config[i])*_dofWeights[i], 2);
      }

      return sqrt(distanceSquared);
  }

  /* Calculates and returns the distance between two nodes. */
  dReal UnweightedDistance(NodePtr node1, NodePtr node2){
        dReal distanceSquared = 0;
        vector<dReal> node1Config = node1->getConfiguration();
        vector<dReal> node2Config = node2->getConfiguration();

        //for(size_t i = 0; i < node1Config.size(); ++i){
        for(size_t i = 0; i < 3; ++i){
          distanceSquared += pow((node1Config[i] - node2Config[i]), 2);
        }

        return sqrt(distanceSquared);
    }

  /* Returns the nearest node in the tree to the node. */
  NodePtr NearestNode(TreePtr tree, NodePtr node){
      dReal lowestDistance = numeric_limits<double>::max();
      NodePtr closestNode;
      for(NodePtr n : tree->getAllNodes()){
        dReal distance = WeightedDistance(n, node);
        if(distance <= lowestDistance){
          lowestDistance = distance;
          closestNode = n;
        }
      }
      return closestNode;
  }

  /* Returns all the nodes in the tree within a step_size radius of the given node. */
  vector<NodePtr> Near(TreePtr tree, NodePtr node){
    vector<NodePtr> nearNodes;
    for(NodePtr n : tree->getAllNodes()){
      dReal distance = UnweightedDistance(n, node);
      if(distance <= STEP_SIZE)
        nearNodes.push_back(n);
    }
    return nearNodes;

  }

  /* Smooths the path generated by RRT using a predefined number of iterations. */
  void ShortcutSmoothing(vector<NodePtr>& priorPath){

    for(int k = 0; k < SMOOTHING_ITERATIONS; ++k){
//      WritePathLengthToFile(k, priorPath.size());
      float rand1 = RandomNumberGenerator()/100;
      float rand2 = RandomNumberGenerator()/100;

      int index1 = static_cast<int>(rand1 * priorPath.size());
      int index2 = static_cast<int>(rand2 * priorPath.size());

      if(index1 == index2)
        continue;

      NodePtr node1 = priorPath[index1];
      NodePtr temp = node1;
      NodePtr node2 = priorPath[index2];
      TreePtr tempTree(new NodeTree());
      tempTree->addNode(node1);

      string status;
      // connect node1 and node2
      do{
        NodePtr newNode = NewStep(node1, node2);

        if(InLimits(newNode) && !CheckCollision(newNode)){
          newNode->setParentNode(node1);
          tempTree->addNode(newNode);
          node1 = newNode;
          if(UnweightedDistance(newNode, node2) <= STEP_SIZE){
            node2->setParentNode(newNode);
            tempTree->addNode(node2);
            status = "Reached";
          }else
            status = "Advanced";
        }else
          status = "Trapped";
      }while(status == "Advanced");

      if(status == "Reached"){
        vector<NodePtr> shorter = tempTree->getAllNodes();

        vector<NodePtr>::iterator it1 = find(priorPath.begin(), priorPath.end(), temp);
        vector<NodePtr>::iterator it2 = find(priorPath.begin(), priorPath.end(), node2);

        if(it1 == priorPath.end() || it2 == priorPath.begin() || it1+1 >= it2-1){
          continue;
        }
        priorPath.erase(it1+1, it2-1);
        priorPath.insert(find(priorPath.begin(), priorPath.end(), temp)+1, shorter.begin()+1, shorter.end()-1);
      }else
        continue;

    }
  }

  /* Returns an iterator to the node in the open set. Returns end if not found. */
  multiset<NodePtr>::iterator FindInOpenSet(NodePtr node){
    multiset<NodePtr>::iterator it;
    for(it = _openSet.begin(); it != _openSet.end(); ++it){
      if(**it == *node)
        return it;
    }
    return it;
  }

  /* Returns an iterator to the node in the open set. Returns end if not found. */
  multiset<NodePtr>::iterator FindInOpenSet_ANA(NodePtr node){
    multiset<NodePtr>::iterator it;
    for(it = _openSet_ANA.begin(); it != _openSet_ANA.end(); ++it){
      if(**it == *node)
        return it;
    }
    return it;
  }

  /* Returns an iterator to the node in the closed set. Returns end if not found. */
  vector<NodePtr>::iterator FindInClosedSet(NodePtr node){
    vector<NodePtr>::iterator it;
    for(it = _closedSet.begin(); it != _closedSet.end(); ++it){
      if(**it == *node)
        return it;
    }
    return it;
  }

 /* Returns an iterator to the node in the INCONS set. Returns end if not found. */
  multiset<NodePtr>::iterator FindInInconSet(NodePtr node){
    multiset<NodePtr>::iterator it;
    for(it = _inconsSet.begin(); it != _inconsSet.end(); ++it){
      if(**it == *node)
        return it;
    }
    return it;
  }

  /* Returns 8-connected neighbors to the given node, as a vector of NodePtrs. Only for 3D. */
  vector<NodePtr> GetNeighbors(NodePtr node){
    vector<dReal> config = node->getConfiguration();
    vector<dReal> operations = {-STEP_SIZE, 0, STEP_SIZE};
    vector<NodePtr> neighbors;

    for(dReal o1 : operations){
      for(dReal o2 : operations){
        for(dReal o3 : operations){
          if(o1 != 0 || o2 != 0 || o3 != 0){
            vector<dReal> newConfig = {config[0] + o1, config[1] + o2, config[2] + o3};
            NodePtr n(new Node(newConfig, node));
            neighbors.push_back(n);
          }
        }
      }
    }
    return neighbors;
  }

  /* Find the path from the parents. */
  vector<NodePtr> GetAStarPath(){

    vector<NodePtr> path;
    NodePtr final = _goalNode;
    path.push_back(final);
    while(final->getParentNode() != nullptr){
      final = final->getParentNode();
      path.push_back(final);
    }
    cout << "Path length" << path.size() << endl;
    return path;
  }

  /* Draws a path of configurations. */
  void DrawPath(vector< vector<dReal> >& path, string color){
    _robot->SetActiveManipulator("leftarm");
    RobotBase::ManipulatorPtr manipulator = _robot->GetActiveManipulator();

    vector<float> raveColor;
    if(color == "red")
      raveColor = {1, 0, 0, 1};
    if(color == "blue")
      raveColor = {0, 0, 1, 1};


    for(vector<dReal> config : path){
      vector<float> point;
      _robot->SetActiveDOFValues(config);
      Transform t = manipulator->GetEndEffectorTransform();
      RaveVector<dReal> translation = t.trans;
      point.push_back(translation.x);
      point.push_back(translation.y);
      point.push_back(translation.z);
      _handles.push_back(_penv->plot3(&point[0], 1, 1, 6, &raveColor[0], 0, true));
    }
  }

  /* Draws a path of configurations. */
  void DrawPath(vector< vector<dReal> >& path){
    vector<float> raveColor = {1, 0, 0, 1};

    for(vector<dReal> config : path){
      vector<float> point;
      point.push_back(config[0]);
      point.push_back(config[1]);
      point.push_back(config[2]);
      _handles.push_back(_penv->plot3(&point[0], 1, 1, 3, &raveColor[0], 0, true));
    }
  }

  /* Draws a single point at the node. For AStar. */
  void DrawPoint(NodePtr node){
    vector<float> raveColor = {0, 0, 1, 1};
    vector<float> point;

    vector<dReal> config = node->getConfiguration();
    for(size_t i = 0; i < 3; ++i)
      point.push_back(config.at(i));
    _handles.push_back(_penv->plot3(&point[0], 1, 1, 3.1, &raveColor[0], 0, true));
  }

  /* Methods for writing data to files. Can be generalized. */
  void WriteDurationsToFile(dReal bias, double algo, double smoothing){
    ofstream file;
    file.open("computation_times.csv", ios_base::app);
    file << std::fixed << std::setprecision(2) << bias << "," << algo << "," << smoothing << "\n";
  }

  void WriteDurationToFile(dReal bias, double algo){
      ofstream file;
      file.open("connect_times_for_biases.csv", ios_base::app);
      file << std::fixed << std::setprecision(2) << bias << "," << algo << "\n";
    }

  void WritePathLengthToFile(int num, int size){
      ofstream file;
      file.open("smoothing_path_length.csv", ios_base::app);
      file << std::fixed << std::setprecision(2) << num << "," << size << "\n";
    }

  void WriteStuffToFile(double algo, double smoothing, int nodes, int unsmoothed, int smoothed){
      ofstream file;
      file.open("5stuff.csv", ios_base::app);
      file << std::fixed << std::setprecision(2) << algo << "," << smoothing << "," << nodes << "," << unsmoothed << "," << smoothed << "\n";
    }

  void WriteBiStuffToFile(double algo, double smoothing, int nodes, int unsmoothed){
        ofstream file;
        file.open("extra_stuff.csv", ios_base::app);
        file << std::fixed << std::setprecision(2) << algo+smoothing << "," << nodes << "," << unsmoothed << "\n";
      }

private:
  EnvironmentBasePtr _penv;
  vector<dReal> _startConfig;
  vector<dReal> _goalConfig;
  vector<dReal> _activeLowerLimits;
  vector<dReal> _activeUpperLimits;
  vector<RobotBasePtr> _robots;
  RobotBasePtr _robot;
  NodePtr _startNode;
  NodePtr _goalNode;
  NodePtr currentNode;
  vector<dReal> _activeDOFRanges;
  vector<dReal> _dofWeights;
  vector<GraphHandlePtr> _handles;
  string red = "red";
  string blue = "blue";
  clock_t startTime;
  clock_t endTime;

  dReal _epsilon;
  dReal _epsilonDash;
  dReal _epsilonDelta;
  vector<NodePtr> _pathAStar;

  dReal _gaussianFactor = 50;
  float _gaussVarX = 1;
  float _gaussVarY = 1;
  float _gaussVarZ = 1;

  float _G_ANA;
  float _E_ANA;

  int count_print = 0;

  struct NodePriority{
    bool operator()(const NodePtr left, const NodePtr right) const{
      return left->getFCost() < right->getFCost();
    }
  };

  struct NodePriority_ANA{
    bool operator()(const NodePtr left, const NodePtr right) const{
      return left->getEKey() < right->getEKey();
    }
  };

  char pause;

  multiset<NodePtr, NodePriority> _inconsSet;
  multiset<NodePtr, NodePriority> _openSet;

  multiset<NodePtr, NodePriority_ANA> _openSet_ANA;

  vector<NodePtr> _closedSet;
  dReal STEP_SIZE;
  dReal GOAL_BIAS;
  };


// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
  if( type == PT_Module && interfacename == "rrt_module" ) {
    return InterfaceBasePtr(new rrt_module(penv,sinput));
  }

  return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
  info.interfacenames[PT_Module].push_back("rrt_module");

}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}

