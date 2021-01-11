/**
 * Copyright 2020, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Jingnan Shi, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 */

#include "teaser/graph.h"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>

using namespace std;

vector<int> teaser::MaxCliqueSolver::findMaxClique(teaser::Graph graph) {

  const auto all_vertices = graph.getVertices();
  using BoostGraph = boost::adjacency_list <boost::vecS, boost::vecS, boost::undirectedS>;
  BoostGraph bg(all_vertices.size());
  for (const auto& i : all_vertices) {
    const auto& c_edges = graph.getEdges(i);
    for (int j : c_edges)
    {
      boost::add_edge(i, j, bg);
    }
  }

  std::vector<int> component(all_vertices.size());
  int num = boost::connected_components(bg, &component[0]);

  // find max component
  std::vector<int> comp_size(num, 0);
  for (int c : component)
  {
    comp_size[c]++;
  }

  int comp_max = std::distance(begin(comp_size), std::max_element(begin(comp_size), end(comp_size)));

  std::vector<int> C;
  C.reserve(all_vertices.size());
  for (int i = 0; i < component.size(); ++i)
  {
    if (component[i] == comp_max)
    {
      C.push_back(i);
    }
  }

  return C;
}
