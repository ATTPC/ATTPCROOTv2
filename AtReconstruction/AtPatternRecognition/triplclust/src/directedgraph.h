//
// directedgraph.h
//     Class for directed graph and building directed MST.
//
// Author:  Christoph Dalitz
// Date:    2022-11-07
// License: see ../LICENSE
//
/** @file */

#ifndef DIRECTEDGRAPH_H
#define DIRECTEDGRAPH_H

#include "pointcloud.h"

#include <algorithm>
#include <iostream>
#include <map>
#include <vector>

#include "kdtree/kdtree.hpp"

class Node;

/**
 * @brief A weighted, directed Edge.
 *
 */
struct Edge {
   Node *begin;
   Node *end;
   double cost;
   Edge()
   {
      begin = NULL;
      end = NULL;
      cost = 0.0;
   }
   Edge(Node *b, Node *e, double c)
   {
      begin = b;
      end = e;
      cost = c;
   }
   bool operator<(const Edge &a) const { return cost < a.cost; }
   bool operator==(const Edge &a) const { return (cost == a.cost) && (begin == a.begin) && (end == a.end); }
};

/**
 * @brief A Node containing a vector of incoming and outgoing Edges, the Coordinates and the index.
 *
 */
class Node {
private:
   std::vector<Edge> _in;
   std::vector<Edge> _out;

   /**
    * @brief Removes _in edge by Iterator.
    *
    * @param it
    * @return * std::vector<Edge>::iterator
    */
   std::vector<Edge>::iterator remove_in_edge(std::vector<Edge>::iterator it) { return _in.erase(it); }
   void add_in(Node *new_node, double cost) { _in.push_back(Edge(new_node, this, cost)); }
   bool depthSearch(size_t depth)
   {
      Edge e;
      if (depth > 0) {
         for (std::vector<Edge>::iterator iterator = _in.begin(); iterator != _in.end(); ++iterator) {
            e = *iterator;
            if (e.begin->depthSearch(--depth)) {
               return true;
            }
         }
         return false;
      } else {
         return true;
      }
   }
   bool isBranchRoot() { return _in.size() > 1; }

   // has to return: sum and n because the edge we look at itself should not be included in mean neighbourhood
   // return value is: N and sum
   std::pair<size_t, double> get_neighbourhood();

public:
   size_t index;
   double x; // X-Coordinate
   double y; // Y-Coordinate
   double z; // Z-Coordinate

   Node()
   {
      this->index = 0;
      this->x = 0.0;
      this->y = 0.0;
      this->z = 0.0;
   }
   Node(size_t _index, double _x, double _y, double _z)
   {
      this->index = _index;
      this->x = _x;
      this->y = _y;
      this->z = _z;
   }
   size_t getnEdge() { return _in.size(); }
   void add_edge(Node *new_node, double cost)
   {
      _out.push_back(Edge(this, new_node, cost));
      new_node->add_in(this, cost);
   }

   /**
    * @brief Removes an out edge by its end.
    *
    * @param Node * n
    */
   void remove_out_edge(Node *n)
   {
      bool found = false;
      for (std::vector<Edge>::iterator it = _out.begin(); (it != _out.end()) && (!found); it++) {
         if (it->end == n) {
            found = true;
            _out.erase(it);
         }
      }
   }

   size_t
   removeBranchEdges(size_t min_depth, std::vector<Edge> *debug_removed_edges, std::vector<Node *> *segment_heads);

   size_t removeLongEdges(std::vector<Edge> *debug_removed_edges, std::vector<Node *> *segment_heads);

   // get Index of next Nodes (write into result)
   void get_next(std::vector<size_t> *result, bool all = false);
};

/**
 * @brief Graph that organizes in a Minimum-Spanning-Tree. The structure is contained in a std::map of Indexes and
 * Nodes.
 *
 *
 */
class Graph {
private:
   std::map<size_t, Node> graph;      // reason for map: KDTree only returns index as nearest neighbour
   std::vector<size_t> indices;       // keep track of index of Node in original, complete pointcloud
   std::vector<Node *> subtree_roots; // keep track of roots of subtrees after removing edges

   std::vector<Edge> removed_long_edges;   // debugging: All removed long Edges
   std::vector<Edge> removed_branch_edges; // debugging: All removed branch Edges

   void constructMST();              // Edmonds Algorithm, simplified
   Kdtree::KdTree constructKDTree(); // Helper Function of constructMST()

public:
   // take Pointcloud and indices and construct a MST
   /**
    * @brief Construct a new MST Graph object.
    *
    * @param cloud
    * @param indices
    */
   Graph(PointCloud &cloud, std::vector<size_t> indices);
   // f==true : write to file with name filename
   // f==false : write to stdout
   void generateGnuplot(bool f = true, std::string filename = "debug-file.gnuplot");
   // remove edges with depth > depth
   bool removeBranches(size_t depth);
   // remove edges with length greater than "Vielfaches" von Nachbarschaft
   void removeLongEdges();

   std::vector<std::vector<size_t>> assignNewCurveIds();

   // update begins of subtrees
   void addSegmentHead(Node *new_seg) { subtree_roots.push_back(new_seg); }

   size_t getnEdge()
   {
      size_t result = 0;
      size_t id;
      for (std::vector<size_t>::iterator iterator = indices.begin(); iterator != indices.end(); ++iterator) {
         id = *iterator;
         result += graph.at(id).getnEdge();
      }
      return result;
   }
};

// only search for nodes on the right hand side of search point
struct Predicate : public Kdtree::KdNodePredicate {
   int a;
   Predicate(size_t index) { a = index; }
   bool operator()(const Kdtree::KdNode &kn) const { return a < kn.index; }
};

#endif
