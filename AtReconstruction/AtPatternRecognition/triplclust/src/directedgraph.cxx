//
// directedgraph.cpp
//     Class for directed graph and building directed MST.
//
// Author:  Christoph Dalitz
// Date:    2022-11-07
// License: see ../LICENSE
//
/** @file */

#include "directedgraph.h"

#include "util.h"
#include <assert.h> /* assert */
#include <math.h>   /* sqrt */

#include <algorithm>

/**
 * @brief Return the euklidian distance of two Kdtree::CoordPoints
 *
 * @param a
 * @param b
 * @return double
 */
double euklidian_distance(Kdtree::CoordPoint a, Kdtree::CoordPoint b)
{
   return sqrt(((a[0] - b[0]) * (a[0] - b[0])) + ((a[1] - b[1]) * (a[1] - b[1])) + ((a[2] - b[2]) * (a[2] - b[2])));
}

/**
 * @brief Construct a new Graph:: Graph object. This includes building an MST.
 * @param cloud
 * @param indices
 */
Graph::Graph(PointCloud &cloud, std::vector<size_t> _indices)
{
   // check input
   assert(cloud.size() > 0);
   assert(_indices.size() > 0);
   assert(std::set<size_t>(_indices.begin(), _indices.end()).size() == _indices.size()); // indices are unique
   assert((indices.back() < cloud.size()));                                              // checks range of indices

   this->indices = indices;

   // insert Nodes without Edges. Nodes have x,y,z Coordinates and the corresponding index
   for (unsigned int i = 0; i < indices.size(); i++) {
      graph.insert(std::pair<size_t, Node>(
         indices[i], Node(indices[i], cloud[indices[i]].x, cloud[indices[i]].y, cloud[indices[i]].z)));
   }

   // Add first subtree: the first Point by time and the last Point by Index
   this->subtree_roots.push_back(&graph.at(indices[indices.size() - 1]));

   // construct MST by adding Edges
   constructMST();
}

/**
 * @brief Constructs a KDTree. Requires the graph and indices attribute to be set
 *
 * @return Kdtree::KdTree
 */
Kdtree::KdTree Graph::constructKDTree()
{
   // build kdtree
   Kdtree::KdNodeVector nodes;
   size_t index;
   for (std::vector<size_t>::iterator iterator = indices.begin(); iterator != indices.end(); ++iterator) {
      index = *iterator;
      Kdtree::CoordPoint p = {graph.at(index).x, graph.at(index).y, graph.at(index).z};
      nodes.push_back(Kdtree::KdNode(p, NULL, index));
   }
   Kdtree::KdTree kdtree(&nodes);
   return kdtree;
}

/**
 * @brief Constructs MST with Chu-Lui/Edmond's algorithm. Simplified, because we assume no cycles are possible due to
 * strict ordering.
 *
 */
void Graph::constructMST()
{
   // build kdtree
   Kdtree::KdTree kdtree = constructKDTree();

   double cost;
   size_t nn_idx;
   size_t last = indices.back();
   size_t index;
   // for vertex in graph do
   for (std::vector<size_t>::iterator iterator = indices.begin(); iterator != indices.end(); ++iterator) {
      index = *iterator;
      // no outgoing edges to be added at last Node
      if (index != last) {
         Kdtree::CoordPoint p = {graph.at(index).x, graph.at(index).y, graph.at(index).z};
         Kdtree::KdNodeVector neighbors;
         Predicate predicate = Predicate(index);
         std::vector<double> distances;
         kdtree.k_nearest_neighbors(p, 1, &neighbors, &distances, &predicate);
         // nearest neighbor
         if (neighbors.size() == 0) {
            std::cout << "neighbors is empty!!!" << std::endl;
         }
         nn_idx = neighbors.front().index;
         Kdtree::CoordPoint nn = {graph.at(nn_idx).x, graph.at(nn_idx).y, graph.at(nn_idx).z};

         // add Edge
         cost = sqrt(distances.front());
         graph[index].add_edge(&graph[neighbors.front().index], cost);
      }
   }
}

/**
 * @brief Calls removeBranchEdges for every Node in the Graph.
 * Returns true when at least one branch was removed. Else returns false.
 * @param depth
 * @return true
 * @return false
 */
bool Graph::removeBranches(size_t depth)
{
   size_t removed = 0;
   size_t idx;
   for (std::vector<size_t>::iterator iterator = indices.begin(); iterator != indices.end(); ++iterator) {
      idx = *iterator;
      removed += graph.at(idx).removeBranchEdges(depth, &removed_branch_edges, &subtree_roots);
   }
   if (removed > 0) {
      return true;
   }
   return false;
}

/**
 * @brief Calls removeLongEdges for every Node in the Graph.
 *
 */
void Graph::removeLongEdges()
{
   size_t removed = 0;
   size_t idx;
   for (std::vector<size_t>::iterator iterator = indices.begin(); iterator != indices.end(); ++iterator) {
      idx = *iterator;
      removed += graph.at(idx).removeLongEdges(&removed_long_edges, &subtree_roots);
   }
}

/**
 * @brief Does a breath-first search starting at each of the Segment Heads to assign new Curve IDs.
 *
 * @return std::vector< std::vector<size_t> >
 */
std::vector<std::vector<size_t>> Graph::assignNewCurveIds()
{
   int current_id = 0;
   std::vector<std::vector<size_t>> result(subtree_roots.size());
   std::vector<size_t> open_list;
   std::vector<size_t> closed_list;
   std::vector<size_t> temp;
   size_t v;
   Node *node_ptr;
   size_t i;
   // for segment Head
   for (std::vector<Node *>::iterator iterator = subtree_roots.begin(); iterator != subtree_roots.end(); ++iterator) {
      node_ptr = *iterator;
      open_list.clear();
      closed_list.clear();
      closed_list.push_back(node_ptr->index);
      open_list.push_back(node_ptr->index);
      result[current_id].push_back(node_ptr->index);
      while (!open_list.empty()) {
         v = open_list.front();
         open_list.erase(open_list.begin());
         temp.clear();
         graph.at(v).get_next(&temp, true);
         for (std::vector<size_t>::iterator iterator2 = temp.begin(); iterator2 != temp.end(); ++iterator2) {
            i = *iterator2;
            if (std::find(closed_list.begin(), closed_list.end(), i) == closed_list.end()) { // if not in closed list
               closed_list.push_back(i);
               result[current_id].push_back(i);
               open_list.push_back(i);
            }
         }
      }
      current_id++;
   }
   return result;
}

/**
 * @brief Generates two Gnuplot files in the calling directory: One with the filename
 * [filename_prefix]_edgeRemoval.gnuplot and another with the filename [filename_prefix]_segmentation.gnuplot
 * [filename_prefix]_edgeRemoval.gnuplot:
 * * Demonstrates the Edges that are removed either because they are long or because they are head of a branch.
 * * The color of the points corresponds to the index to show the order of the points. The new roots of the resulting
 * subtrees are also shown.
 * @param f
 * @param filename_prefix
 */
void Graph::generateGnuplot(bool f, std::string filename_prefix)
{

   // option for graph output:
   // 1: Remove Edges Plot
   // 2: Cluster-Ids Plot

   if (f) {
      // demo edge Removal in MST
      std::ofstream outfile1(filename_prefix + "_edgeRemoval.gnuplot");
      std::vector<size_t> idx;
      size_t index;
      size_t edge_to;
      Edge e;
      Edge edge;
      Node *node_ptr;
      if (outfile1) {
         outfile1 << "set palette rgb 34,35,36;\n";
         // outfile<<"splot '-' using 1:2:3 with points palette; pause -1;\n";
         outfile1 << "splot '-' using 1:2:3 with points palette pointtype 7 ps 1, '-' with lines lc 'black' title "
                     "'Kept', '-' with lines lc 'green' title 'Removed (Too long)', '-' with lines lc 'magenta' title "
                     "'Removed (Is Branch)', '-' with points lc 'web-blue' pointtype 1 ps 2 title 'Head of Segment'; "
                     "pause -1;\n"; //, '-' using 1:2:3 with points lc 'blue' ps 1, '-' with lines lc 'blue' title
                                    //'Removal Candidate'; pause -1;\n";
         for (std::vector<size_t>::iterator iterator = indices.begin(); iterator != indices.end(); ++iterator) {
            index = *iterator;
            outfile1 << graph.at(index).x << " " << graph.at(index).y << " " << graph.at(index).z << " " << index
                     << "\n";
         }
         outfile1 << "e\n";
         for (std::vector<size_t>::iterator iterator = indices.begin(); iterator != indices.end(); ++iterator) {
            index = *iterator;
            graph[index].get_next(&idx);
            for (std::vector<size_t>::iterator iterator2 = idx.begin(); iterator2 != idx.end(); ++iterator2) {
               edge_to = *iterator2;
               outfile1 << graph.at(index).x << " " << graph.at(index).y << " " << graph.at(index).z << "\n";
               outfile1 << graph.at(edge_to).x << " " << graph.at(edge_to).y << " " << graph.at(edge_to).z << "\n";
               outfile1 << "\n\n";
            }
         }
         outfile1 << "e\n";
         for (std::vector<Edge>::iterator iterator = this->removed_long_edges.begin();
              iterator != this->removed_long_edges.end(); ++iterator) {
            edge = *iterator;
            outfile1 << edge.begin->x << " " << edge.begin->y << " " << edge.begin->z << "\n";
            outfile1 << edge.end->x << " " << edge.end->y << " " << edge.end->z << "\n";
            outfile1 << "\n\n";
         }
         outfile1 << "e\n";
         for (std::vector<Edge>::iterator iterator = this->removed_branch_edges.begin();
              iterator != this->removed_branch_edges.end(); ++iterator) {
            edge = *iterator;
            outfile1 << edge.begin->x << " " << edge.begin->y << " " << edge.begin->z << "\n";
            outfile1 << edge.end->x << " " << edge.end->y << " " << edge.end->z << "\n";
            outfile1 << "\n\n";
         }
         outfile1 << "e\n";
         for (std::vector<Node *>::iterator iterator = this->subtree_roots.begin();
              iterator != this->subtree_roots.end(); ++iterator) {
            node_ptr = *iterator;
            outfile1 << node_ptr->x << " " << node_ptr->y << " " << node_ptr->z << "\n";
         }
         outfile1.close();
      }
      // demo segmentation
      std::ofstream outfile(filename_prefix + "_segmentation.gnuplot");
      std::vector<std::vector<size_t>> result;
      result = assignNewCurveIds();
      if (outfile) {
         outfile << "splot '-' with points lc 'black' pt 1, ";
         for (unsigned int cluster = 0; cluster < result.size(); cluster++) {
            outfile << "'-' with points pt 7 title 'Cluster " << std::to_string(cluster) + "', ";
         }
         outfile << "'-' with points lc 'orange' pt 2 title 'Segment Heads'; pause -1;\n";
         for (std::vector<size_t>::iterator iterator = indices.begin(); iterator != indices.end(); ++iterator) {
            index = *iterator;
            outfile << graph.at(index).x << " " << graph.at(index).y << " " << graph.at(index).z << "\n";
         }
         outfile << "e\n";
         for (unsigned int cluster = 0; cluster < result.size(); cluster++) {
            for (std::vector<size_t>::iterator iterator = result[cluster].begin(); iterator != result[cluster].end();
                 ++iterator) {
               index = *iterator;
               outfile << graph.at(index).x << " " << graph.at(index).y << " " << graph.at(index).z << "\n";
            }
            outfile << "e\n";
         }
         for (std::vector<Node *>::iterator iterator = subtree_roots.begin(); iterator != subtree_roots.end();
              ++iterator) {
            node_ptr = *iterator;
            outfile << node_ptr->x << " " << node_ptr->y << " " << node_ptr->z << "\n";
         }
         outfile << "e\n";
         outfile.close();
      }
   }
}

/**
 * @brief Does a breadth-first search until the max_depth (4). Returns the count of the Edges and the sum of the costs
 * of earch encountered edge. pair : N edges, Sum of the Cost of the Edges
 * @return std::pair<size_t, double>
 */
std::pair<size_t, double> Node::get_neighbourhood()
{
   int max_depth = 4; // 4-er Nachbarschaft
   double sum = 0;
   double N = 0;
   // open_list is queue
   std::vector<Node *> open_list;
   std::vector<Node *> closed_list;
   Node *v = this;
   int level = 0;
   Edge e;

   // null Idee von: https://stackoverflow.com/questions/31247634/how-to-keep-track-of-depth-in-breadth-first-search

   open_list.push_back(this);
   open_list.push_back(NULL);
   while ((!open_list.empty()) && (level < max_depth)) {
      v = open_list.front(); // queueify
      open_list.erase(open_list.begin());
      if (v == NULL) {
         level++;
         // if(open_list.back() == NULL) break;        // changed
         open_list.push_back(NULL);
      } else {
         closed_list.push_back(v);
         for (std::vector<Edge>::iterator iterator = v->_out.begin(); iterator != v->_out.end(); ++iterator) {
            e = *iterator;
            if (std::find(closed_list.begin(), closed_list.end(), e.end) == closed_list.end()) {
               sum += e.cost;
               N++;
               open_list.push_back(e.end);
            }
         }
         for (std::vector<Edge>::iterator iterator = v->_in.begin(); iterator != v->_in.end(); ++iterator) {
            e = *iterator;
            if (std::find(closed_list.begin(), closed_list.end(), e.begin) == closed_list.end()) {
               sum += e.cost;
               N++;
               open_list.push_back(e.begin);
            }
         }
      }
   }
   return std::pair<size_t, double>(N, sum);
}

/**
 * @brief Removes the longest Edge(s) leading into a branch of a depth of at least min_depth. Adds the next Node as a
 * new Segment Head and the Edge to a debug vector. Returns count of removed Edges.
 * @param min_depth
 * @param debug_removed_edges
 * @param segment_heads
 * @return size_t
 */
size_t
Node::removeBranchEdges(size_t min_depth, std::vector<Edge> *debug_removed_edges, std::vector<Node *> *segment_heads)
{

   size_t counter = 0;
   std::vector<Edge> removal_candidates;
   Edge i;
   Edge i2;

   if (this->isBranchRoot()) { // check if this node is Root of a branch (has > 1 Incoming Edge)
      for (std::vector<Edge>::iterator it = _in.begin(); it != _in.end(); it++) { // for every edge:
         if (it->begin->depthSearch(min_depth)) {                                 // check depth of branch
            // if depth > min_depth
            counter++;
            removal_candidates.push_back(*it);
         }
      }
      if (counter >= 2) { // is this Node Root at least 2 long Branches?
         std::vector<Edge>::iterator iteratorMinElement =
            std::min_element(removal_candidates.begin(), removal_candidates.end());
         removal_candidates.erase(iteratorMinElement);
         for (std::vector<Edge>::iterator iterator = removal_candidates.begin(); iterator != removal_candidates.end();
              ++iterator) {
            i = *iterator;
            debug_removed_edges->push_back(i);
         }
         std::vector<Edge> temp_in = _in;
         _in.clear();
         for (std::vector<Edge>::iterator iterator = temp_in.begin(); iterator != temp_in.end(); ++iterator) {
            i2 = *iterator;
            if (!(std::find(removal_candidates.begin(), removal_candidates.end(), i2) != removal_candidates.end())) {
               _in.push_back(i2);
            } else {
               i.begin->remove_out_edge(this);
               segment_heads->push_back(i2.begin);
            }
         }
         temp_in.clear();
         removal_candidates.clear();
         return counter - 1;
      }
   }
   return 0;
}

/**
 * @brief Calls get_neighbourhood and then checks for every incoming edge e with weight w:
 * * get the mean weight length of the neighbourhood edges (without e): n_mean
 * * if w > (n_mean*4.5):
 * * * edge should be removed due to being long
 * * * add the edge to the debug vector
 * * * add the begin of the edge to the segment head vector
 * * * remove edge e
 * @param debug_removed_edges: is potentially added to
 * @param segment_heads: is potentially added to
 * @return size_t: how many incoming edges to this Node were removed
 */
size_t Node::removeLongEdges(std::vector<Edge> *debug_removed_edges, std::vector<Node *> *segment_heads)
{
   std::pair<size_t, double> n_sum = get_neighbourhood();
   double cost_sum = n_sum.second;
   int n_edges = n_sum.first;
   size_t counter = 0;
   double n_mean = 0;
   for (std::vector<Edge>::iterator it = _in.begin(); it != _in.end();) { // for every edge, do
      n_mean = (cost_sum - it->cost) / (n_edges - 1);
      if (it->cost > (n_mean * 4.5)) {        // if cost of edge > "Vielfaches" (4.5) of mean Neighbourhood Costs.
         debug_removed_edges->push_back(*it); // add edge to removed_edges in Graph object, for gnuplot debug file
         segment_heads->push_back(it->begin);
         it->begin->remove_out_edge(this);
         it = remove_in_edge(it); // remove the edge.
         counter++;
      } else {
         it++;
      }
   }
   return counter;
}

/**
 * @brief gets the indexes of the connected Nodes, either just outgoing edges or also incoming edges if full=true
 *
 * @return result
 * @param full: true if you want indexes of both incoming and outgoing edges
 */
void Node::get_next(std::vector<size_t> *result, bool full)
{
   (*result).clear();
   Edge edge;
   for (std::vector<Edge>::iterator iterator = _out.begin(); iterator != _out.end(); ++iterator) {
      edge = *iterator;
      (*result).push_back(edge.end->index);
   }
   if (full) {
      for (std::vector<Edge>::iterator iterator = _in.begin(); iterator != _in.end(); ++iterator) {
         edge = *iterator;
         (*result).push_back(edge.begin->index);
      }
   }
}
