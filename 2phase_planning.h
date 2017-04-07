/*
 * 2phase_planning.h
 *
 *  Created on: Mar 28, 2017
 *      Author: Charvak Kondapalli || Aditya Gupta
 */

#ifndef PHASE_PLANNING_H_
#define PHASE_PLANNING_H_

#include <boost/bind.hpp>
#include <openrave/plugin.h>
#include <openrave/openrave.h>
#include <tr1/unordered_map>

using namespace std;
using namespace OpenRAVE;

/* Helper class for a Node. */
class Node {
  typedef boost::shared_ptr<Node> NodePtr;
private:
  static int sm_nodeCounter;
  /* ID of the node. */
  int m_iid;

  /* Configuration of the robot at the node. */
  vector<dReal> m_vdconfiguration;

  /* Pointer to the parent node. */
  NodePtr m_pparentNode;

  /* Costs for A* variants. f = g + h */
  float m_ffCost=99, m_fgCost=99, m_fhCost=99;

public:

  Node(vector<dReal> configuration, NodePtr parent);

  Node();

  ~Node();

  const vector<dReal>& getConfiguration() const;

  void setConfiguration(const vector<dReal>& configuration);

  const NodePtr getParentNode() const;

  void setParentNode(const NodePtr parentNode);

  int getId() const;

  bool operator ==(Node& other);

  // bool operator !=(Node& other);

  float getFCost() const;

  void setFCost(float fCost);

  float getGCost() const;

  void setGCost(float gCost);

  float getHCost() const;

  void setHCost(float hCost);
};


/*--------------------------------------------------------------------------------------------------------*/


/* Tree of nodes explored. */
class NodeTree {
  typedef boost::shared_ptr<Node> NodePtr;
private:
  /* Pointers to all the nodes explored. */
  vector<NodePtr> m_vpnodes;

public:
  NodePtr getMostRecentNode() const;

  /* Adds a node pointer to the list of nodes explored. */
  bool addNode(NodePtr p_node);

  /* Deletes a node from the list. */
  void deleteNode(NodePtr node);

  /* Returns a node from the list of the given ID.
   * Throws 0 if no node with given ID exists. */
  NodePtr getNode(int id);

  /* Returns the path from the root to the node of given ID, including the node. */
  vector<NodePtr> getPathTo(int id);

  vector<NodePtr>& getAllNodes();

  size_t getSize();
};

#endif /* PHASE_PLANNING_H_ */
