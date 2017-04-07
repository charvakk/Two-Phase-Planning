/*
 * 2phase_planning.h
 *
 *  Created on: Mar 28, 2017
 *      Author: Charvak Kondapalli || Aditya Gupta
 */
#include <openrave/openrave.h>
#include <openrave/plugin.h>
#include <openrave/planningutils.h>

#include <boost/bind.hpp>
#include <boost/algorithm/string/classification.hpp> // Include boost::for is_any_of
#include <boost/algorithm/string/split.hpp> // Include for boost::split

#include <iostream>
#include <string.h>
#include <tuple>
#include <ctime>
#include <cstdio>

#include "2phase_planning.h"
#include <algorithm>
#include <limits>
#include <math.h>
#include <openrave/utils.h>
#include <openrave/planningutils.h>
#include <ctime>
#include <iostream>
#include <fstream>

using namespace std;
using namespace OpenRAVE;


#define COMPUTATION_TIME 35000
#define STEP_SIZE 0.4
dReal GOAL_BIAS = 31;
#define INPUT_SIZE 52
#define CLOSE_ENOUGH 0.2
#define SMOOTHING_ITERATIONS 200

/***************ARA STAR DEFINES*************************************/
#define STEP_COST 0.1 // used for gCost || in A-STAR
#define STEP_SIZE_ASTAR 0.5
#define _EPSILON_ 1;

typedef boost::shared_ptr<Node> NodePtr;
typedef boost::shared_ptr<NodeTree> TreePtr;

/* Node method implementations.*/
int Node::sm_nodeCounter = 0;


Node::Node(vector<dReal> configuration, NodePtr parent):
m_ffCost(0),
m_fgCost(0),
m_fhCost(0),
m_pparentNode(parent),
m_vdconfiguration(configuration)
{
m_iid = ++sm_nodeCounter;
}

Node::Node():
m_ffCost(0),
m_fgCost(0),
m_fhCost(0),
m_pparentNode(nullptr),
m_vdconfiguration()
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
  dReal sum=0;
  for(size_t i = 0; i < m_vdconfiguration.size(); ++i){
    // if(m_vdconfiguration[i] != other.getConfiguration()[i])
      // return false;
    sum = sum + m_vdconfiguration[i] - other.getConfiguration()[i];
  }
  if(sum < 0.06)
    return true;
  else
    return false;
  // return true;
}

// bool Node::operator !=(Node& other){
//   return !this->operator ==(other);
// }

//CHANGED this 
float Node::getFCost() const{
  return m_fgCost + m_fhCost;
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

//removes multiple occourances of tha value
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
  rrt_module(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv) {
    _penv = penv;
    
    RegisterCommand("MyCommand",boost::bind(&rrt_module::MyCommand,this,_1,_2),
                    "ASTAR Implimentation.");
  
  }
  virtual ~rrt_module() {}

  /*--------------------------------------------------------------------------------AStar--------------------------------------------------------------------*/
  bool MyCommand(ostream& sout, istream& sin){

   cout << " ASTAR ARA Started ...." <<  endl;
   InitAStar(sout, sin);

   printConfig("start",_startConfig);
   printConfig("goal",_goalConfig);
   _startNode->setHCost(calculate_H(_startNode->getConfiguration(), _goalNode->getConfiguration()));
   _startNode->setGCost(0);
   _currentNode = _startNode;
   int count = 0;
   float tempCost1=0;
   // printConfig(_currentNode);
   // //open set already has start node from InitAStar
   while(_openSet.size() != 0){
   // while(count < 1000){
    count++;
    
    // test(count);
     //also checks if goal is reached or not
     cout << "------------------------>" <<  count  << "<------------------------"<< endl;
     _currentNode = getNearestNode();
     // printConfig("NearestNode :: ",_currentNode->getConfiguration());

     if(_currentNode == nullptr){
      cout << "goal reached" << endl;
      break;
     }

     // cout << "Test" << endl;
     //_robot->SetActiveDOFValues(_goalConfig);
     // cout << "Test2" << endl;
     // test(1);
     getValidNeibhour(_currentNode);
     // test(2);
     tempCost1 = _currentNode->getGCost() + tempCost1;
     // printConfig("")
     // cout << "Current G Cost --> " << _currentNode->getGCost() << endl;
     // test(3);
     // cout << "tempCost1 "  << tempCost1 << endl;
     // test(1);
     // cout <<"-----------------------------NEIBHOURS------------------------------" << endl; 
     // for(size_t i=0 ; i<_neibhourConfig.size() ; i++){
      // cout <<  " i_"<< i<< " ::: "<< _neibhourConfig[i][0] <<  "  "<< _neibhourConfig[i][1] <<"  "<< _neibhourConfig[i][2] << endl;//<< "  "<<_neibhourConfig[i][3] << endl;
      // }
     removeNode(_currentNode);
     // printConfig("next_config",_currentNode->getConfiguration()) ;
     // cout << "_currentNode->getFCost() : "<< _currentNode->getFCost() <<  endl;

     // _currentNode->setHCost(calculate_H(_currentNode->getConfiguration(),_goalConfig));

     if( 0.2 > _currentNode->getHCost() ){
      cout << "goal reached" << endl;
      break;
     }
     
     // test(2);
     // cout <<  _closedSet.size() << endl;
     _closedSet.push_back(_currentNode);
     // cout <<  _closedSet.size() << endl;
     // cout <<"-----------------------------CLOSED------------------------------" << endl; 
     // for(size_t i=0 ; i<_closedSet.size() ; i++){
      // cout <<  " i_"<< i << "    ::: "<< _closedSet[i]->getConfiguration()[0] <<  "  "<< _closedSet[i]->getConfiguration()[1] <<"  "<< _closedSet[i]->getConfiguration()[2] << endl;
      // }
     // test(3);
     // cout << _neibhourConfig.size() << endl;

      for(size_t i=0 ; i <  _neibhourConfig.size() ; i++){


        float tempCost = _neibhourConfig[i][3] + tempCost1;
        // cout  << " tempCost->" << tempCost <<endl;

        if(checkPass(tempCost, _neibhourConfig[i]) == 2){
          // cout << "oooo i->" << i << " size->"<<_neibhourConfig.size()<< " tempCost->" << tempCost <<endl;
           //add in openSet
           boost::shared_ptr<Node> newNode(new Node(_neibhourConfig[i], _currentNode));// = new Node(_neibhourConfig[i], _currentNode);
           
           newNode->setGCost(tempCost);
           newNode->setHCost(calculate_H(_neibhourConfig[i], _goalConfig));
           _openSet.push_back(newNode);

        }
        // if(checkPass(tempCost, _neibhourConfig[i]) == 1){
          //previous node only

        // }// }else{
        //   continue;
        // }
      }
    } 

    return true;
  }
  /* ........astar functions............................................... */
  void test(int a){
    cout << "test :" << a << endl;
  }
  //print function
  void printConfig(string s,vector<dReal> a){
    cout << s <<" : ";
    for(size_t i=0 ; i<a.size()-1 ; i++){
      cout<<" " << a[i] << " . ";
    }cout <<endl;
  }

  //eucleadean hueristic
  float calculate_H(vector<dReal> a, vector<dReal> b){
    float hValue;
    for(size_t i=0 ; i< a.size()-1 ; i++){
      hValue += pow((a[i] - b[i]), 2);
    }
    hValue = sqrt(hValue) * _EPSILON_;
    return hValue;
  }

  //get next node from oenSet
  NodePtr getNearestNode(){
    vector<NodePtr>::iterator it = _openSet.begin();//min_element(_openSet.begin(), _openSet.end(), compare_f);
    vector<NodePtr>::iterator it1 = _openSet.begin();;
    float tempF=99,min=10000;
    NodePtr temp;
    int index;
    while(it != _openSet.end()){
      // min = 99;
      tempF = (*it)->getFCost();
      // for(size_t i=0 ; i<_neibhourConfig.size() ; i++){
      // cout <<  " i " <<distance(_openSet.begin(), it)<< " : "<< (*it)->getConfiguration()[0] <<  "  "<< (*it)->getConfiguration()[1] <<"  "<< (*it)->getConfiguration()[2] << " === "<< tempF << endl;//<< endl;
      // }cd
      // cout 
    // }

      if(tempF < min){

        index = distance(_openSet.begin(), it);
        // cout << "min " <<min <<" tempF " <<tempF <<" hi..." << index <<  endl;
        temp = (*it);
        min = tempF;
      }
      
      ++it;
    }
    // cout << "index...." << index << "||  min ...."  << min << endl;
    //check if its GOAL or not
    // cout << "Nearest Config : :: " ;
    printConfig("Nearest Config",temp->getConfiguration());
    // cout << "Goal Config :::" ;
    // printConfig(_goalNode->getConfiguration());
    dReal test=0;

    for(size_t ii=0; ii<_goalConfig.size()-1 ; ii++){
      test = test + abs(temp->getConfiguration()[ii] -_goalConfig[ii]);
    }
    // cout <<  "Distance from Goal :"<< test << endl;

    if(test < 0.001)//temp->getConfiguration() ==_goalNode->getConfiguration())
    {  
      cout << "GOAL FOUND..." << endl;
      return nullptr;
    }else{
      // cout << "." << endl;
      return (temp); 

    }
  }


  // bool compare_f()
  // { return lhs->getFCost() < rhs->getFCost(); }

  //get the valid neibhour
  void getValidNeibhour(NodePtr a){

    // cout << "1Get Valid Neibhour ....." << endl;
    NodePtr temp;

    vector<dReal> tempConfig;
    int num=0;
    _neibhourConfig.clear();
    // cout << "3Get Valid Neibhour ....." << endl;
    float move[3]={-STEP_SIZE_ASTAR,0,STEP_SIZE_ASTAR};
    for(int i = 0; i < 3 ; i ++){

      for(int j = 0; j < 3 ; j ++){
        for(int k = 0; k < 3 ; k ++){
              // cout << i <<"....." << j << "....." << k << endl;    
              //remove [0, 0 , 0] config
              
              if(i*j*k != 1){
                num++;

                tempConfig.clear();
                tempConfig.push_back(a->getConfiguration()[0] + move[i]);
                tempConfig.push_back(a->getConfiguration()[1] + move[j]);
                tempConfig.push_back(a->getConfiguration()[2] + move[k]);
                float tempval=0;
                tempval = (abs(move[i]) + abs(move[j]) + abs(move[k]));
                tempConfig.push_back(tempval);

                if(!checkValid(tempConfig, tempval)){
                  _neibhourConfig.push_back(tempConfig);

                }

              }
        }  
      }  
    }
   }

   bool checkValid(vector<dReal> tempConfig, float ass){

    // printConfig("checkValid",a);
    
     _robot->SetActiveDOFValues(tempConfig);
     bool flag = _penv->CheckCollision(_robot);
     _robot->SetActiveDOFValues(_startConfig);
    
    //check if this config is present
     for(size_t i=0 ; i<_openSet.size() ; i++){
        vector<dReal> temp = _openSet[i]->getConfiguration();
        int count1 = 0;
        for(size_t j=0 ;j<3 ; j++){
          if(temp[j] == tempConfig[j])
            count1++;

        }
        if(count1 == 3){
          // printConfig("_openSet[i]->getConfiguration()  ",_openSet[i]->getConfiguration()) ;
          // printConfig("  tempConfig ",tempConfig);// tempConfig << endl;

          float tt =  _openSet[i]->getGCost() + ass;
          _openSet[i]->setGCost(tt);
          flag = true;
        }
     }

     return (flag);
   } 

   //remove current node from openSet
   void removeNode(NodePtr a){
      _openSet.erase(remove(_openSet.begin(), _openSet.end(), a), _openSet.end());
  }

  int checkPass(float a, vector<dReal> b){

    int flag = 1;
    int flag1 = 1;
    int result;

    for(size_t i=0 ; i<_closedSet.size() ; i++){

      flag1 = 1;
      
      // printConfig("b->",_closedSet[i]);

      for(size_t j=0; j < b.size()-1; j++){
        
        if(b[j] != _closedSet[i]->getConfiguration()[j])
          flag1 = 0;

      }

      // cout << "-";
      if(flag1){
        // cout << 'h' << endl;
        // printConfig("_closedSet",_closedSet[i]->getConfiguration());
        // printConfig("a->",b); 
        if(a < _closedSet[i]->getGCost()){

           // void removeNode(NodePtr a){
              // _closedSet.erase(remove(_closedSet.begin(), _closedSet.end(), a), _openSet.end());
           // }
          
          // _closedSet.erase(_closedSet.begin() + i);
          _closedSet[i]->setGCost(a);
          result = 1;
          
        }else{

          result = 0;

        }
      
      }else{
      
          result = 2;
        }
    } 
    // cout << endl;
    // bool final;
    // if(flag)
    //   final = true;
    // else
    //   final = false;

      return result;
  }

  /* ........................................................................... */


  /* Initializes the members by calling the input parser. */
  void Init(ostream& so, istream& si){
    _penv->GetRobots(_robots);
    _robot = _robots.at(0);

    srand(time(NULL));
    _robot->GetActiveDOFValues(_startConfig);
    _goalConfig = GetInputAsVector(so, si);
    _robot->GetActiveDOFLimits(_activeLowerLimits, _activeUpperLimits);
    assert(_goalConfig.size() == 7 && "goalConfig should be of size 7!");
    assert(_startConfig.size() == 7 && "startConfig wasn't size 7 :(");

    _activeDOFRanges.reserve(_activeLowerLimits.size());
    for(size_t i = 0; i < _activeLowerLimits.size(); ++i){
      if(i == 4 || i == 6){
        _activeUpperLimits[i] = M_PI;
        _activeLowerLimits[i] = -M_PI;
      }
      _activeDOFRanges[i] = _activeUpperLimits[i]-_activeLowerLimits[i];
    }

    _dofWeights = {3.17104, 2.75674, 2.2325, 1.78948, 0, 0.809013, 0};

    // Root and Goal nodes
    _startNode = NodePtr(new Node(_startConfig, nullptr));
    _goalNode = NodePtr(new Node(_goalConfig, nullptr));
    startTime = clock();
  }

  void InitAStar(ostream& so, istream& si){
    _penv->GetRobots(_robots);
    _robot = _robots.at(0);
    // cout << "Inside Astar Initialiser.." << endl;
    //srand(time(NULL));
    // _robot->GetActiveDOFValues(_startConfig);
    _startConfig.push_back(0);
    _startConfig.push_back(0);
    _startConfig.push_back(5);
    _startConfig.push_back(0);

    // cout << "1Inside Astar Initialiser.." << endl;

    _goalConfig = GetInputAsVector(so, si);
    // cout << "2Inside Astar Initialiser.." << endl;

   // Initialize start and goal nodes
    _startNode = NodePtr(new Node(_startConfig, nullptr));
    // cout << "3Inside Astar Initialiser.." << endl;

    _goalNode = NodePtr(new Node(_goalConfig, nullptr));
    // cout << "4Inside Astar Initialiser.." << endl;

    _openSet.push_back(_startNode);
    // cout << "5Inside Astar Initialiser.." << endl;

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
    NodePtr newNode = NewStep(nearestNode, node);
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
      NodePtr newNode = NewStep(nearestNode, node);

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

  /* Checks if the configuration of the node is in DOF limits. */
  bool InLimits(NodePtr node){
    vector<dReal> config = node->getConfiguration();
    for(size_t i = 0; i < config.size(); ++i){
      if(config[i] < _activeLowerLimits[i] || config[i] > _activeUpperLimits[i])
        return false;
    }
    return true;
  }

  /*Parses the input from the python script and returns the goal config. */
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

        for(size_t i = 0; i < node1Config.size(); ++i){
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

  /* Returns the node with the lowest f cost in the open set */
  NodePtr FindLowestF(){
    float lowest = numeric_limits<float>::max();
    NodePtr bestNode = nullptr;

    for(NodePtr node : _openSet){
      if(node->getFCost() < lowest){
        lowest = node->getFCost();
        bestNode = node;
      }
    }
    return bestNode;
  }

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
  NodePtr _currentNode;

  vector<dReal> _activeDOFRanges;
  vector<dReal> _dofWeights;
  vector<GraphHandlePtr> _handles;
  string red = "red";
  string blue = "blue";
  clock_t startTime;
  clock_t endTime;

  vector<NodePtr> _closedSet;
  vector<NodePtr> _openSet;
  vector<NodePtr> _neibhourSet;

  vector< vector<dReal> > _neibhourConfig;
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

