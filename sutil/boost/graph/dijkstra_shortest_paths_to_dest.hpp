//=======================================================================
// Copyright 1997, 1998, 1999, 2000 University of Notre Dame.
// Authors: Andrew Lumsdaine, Lie-Quan Lee, Jeremy G. Siek
//
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//=======================================================================
//
//
// Revision History:
//   04 April 2001: Added named parameter variant. (Jeremy Siek)
//   01 April 2001: Modified to use new <boost/limits.hpp> header. (JMaddock)
//


/*
   dijkstra_shortest_paths edit the the source to dest vertex
   edit by syhkiller@163.com
   all version will not initialize distance map and color map and precursor map
   if user did not provide distance map, it will use lazy property by default
   return inf
 */
#ifndef BOOST_GRAPH_DIJKSTRA_TO_DEST_HPP
#define BOOST_GRAPH_DIJKSTRA_TO_DEST_HPP

#include <functional>
#include <boost/limits.hpp>
#include <boost/graph/named_function_params.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/relax.hpp>
#include <boost/pending/indirect_cmp.hpp>
#include <boost/graph/exception.hpp>
#include <boost/pending/relaxed_heap.hpp>
#include <boost/graph/overloading.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/graph/detail/d_ary_heap.hpp>
#include <boost/graph/two_bit_color_map.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/type_traits.hpp>
#include <boost/concept/assert.hpp>

#ifdef BOOST_GRAPH_DIJKSTRA_TESTING
#  include <boost/pending/mutable_queue.hpp>
#endif // BOOST_GRAPH_DIJKSTRA_TESTING
#include  <boost/graph/dijkstra_shortest_paths.hpp>
#include  <unordered_map>
#include    "breadth_first_search_to_dest.hpp"
#include    "lazy_associative_property_map.hpp"
namespace boost {

  // Call breadth first search with default color map.
  template <class Graph, class SourceInputIter, class DijkstraVisitor,
            class PredecessorMap, class DistanceMap,
            class WeightMap, class IndexMap, class Compare, class Combine,
            class DistZero>
  inline void
  dijkstra_shortest_paths_to_dest_no_init
    (const Graph& g,
     SourceInputIter s_begin, SourceInputIter s_end,
     typename graph_traits<Graph>::vertex_descrpitor d,
     PredecessorMap predecessor, DistanceMap distance, WeightMap weight,
     IndexMap index_map,
     Compare compare, Combine combine, DistZero zero,
     DijkstraVisitor vis)
  {
    typedef
      detail::default_color_map_generator<Graph, IndexMap>
      ColorMapHelper;
    typedef typename ColorMapHelper::type ColorMap;
    ColorMap color =
      ColorMapHelper::build(g, index_map);
    dijkstra_shortest_paths_no_init( g, s_begin, s_end, d, predecessor, distance, weight,
      index_map, compare, combine, zero, vis,
        color);
  }

  // Call breadth first search with default color map.
  template <class Graph, class DijkstraVisitor,
            class PredecessorMap, class DistanceMap,
            class WeightMap, class IndexMap, class Compare, class Combine,
            class DistZero>
  inline void
  dijkstra_shortest_paths_to_dest_no_init
    (const Graph& g,
     typename graph_traits<Graph>::vertex_descriptor s,
     typename graph_traits<Graph>::vertex_descriptor d,
     PredecessorMap predecessor, DistanceMap distance, WeightMap weight,
     IndexMap index_map,
     Compare compare, Combine combine, DistZero zero,
     DijkstraVisitor vis)
  {
    dijkstra_shortest_paths_no_init(g, &s, &s + 1, d, predecessor, distance,
                                    weight, index_map, compare, combine, zero,
                                    vis);
  }

  // Call breadth first search
  template <class Graph, class SourceInputIter, class DijkstraVisitor,
            class PredecessorMap, class DistanceMap,
            class WeightMap, class IndexMap, class Compare, class Combine,
            class DistZero, class ColorMap>
  inline void
  dijkstra_shortest_paths_to_dest_no_init
    (const Graph& g,
     SourceInputIter s_begin, SourceInputIter s_end,
     typename graph_traits<Graph>::vertex_descriptor d,
     PredecessorMap predecessor, DistanceMap distance, WeightMap weight,
     IndexMap index_map,
     Compare compare, Combine combine, DistZero zero,
     DijkstraVisitor vis, ColorMap color)
  {
    typedef indirect_cmp<DistanceMap, Compare> IndirectCmp;
    IndirectCmp icmp(distance, compare);

    typedef typename graph_traits<Graph>::vertex_descriptor Vertex;

#ifdef BOOST_GRAPH_DIJKSTRA_TESTING
    if (!dijkstra_relaxed_heap) {
      typedef mutable_queue<Vertex, std::vector<Vertex>, IndirectCmp, IndexMap>
        MutableQueue;

      MutableQueue Q(num_vertices(g), icmp, index_map);
      detail::dijkstra_bfs_visitor<DijkstraVisitor, MutableQueue, WeightMap,
        PredecessorMap, DistanceMap, Combine, Compare>
      bfs_vis(vis, Q, weight, predecessor, distance, combine, compare, zero);

      breadth_first_visit_to_dest(g, s_begin, s_end, d, Q, bfs_vis, color);
      return;
    }
#endif // BOOST_GRAPH_DIJKSTRA_TESTING

#ifdef BOOST_GRAPH_DIJKSTRA_USE_RELAXED_HEAP
    typedef relaxed_heap<Vertex, IndirectCmp, IndexMap> MutableQueue;
    MutableQueue Q(num_vertices(g), icmp, index_map);
#else // Now the default: use a d-ary heap
      boost::scoped_array<std::size_t> index_in_heap_map_holder;
      typedef
        detail::vertex_property_map_generator<Graph, IndexMap, std::size_t>
        IndexInHeapMapHelper;
      typedef typename IndexInHeapMapHelper::type IndexInHeapMap;
      IndexInHeapMap index_in_heap =
        IndexInHeapMapHelper::build(g, index_map, index_in_heap_map_holder);
      typedef d_ary_heap_indirect<Vertex, 4, IndexInHeapMap, DistanceMap, Compare>
        MutableQueue;
      MutableQueue Q(distance, index_in_heap, compare);
#endif // Relaxed heap

    detail::dijkstra_bfs_visitor<DijkstraVisitor, MutableQueue, WeightMap,
      PredecessorMap, DistanceMap, Combine, Compare>
        bfs_vis(vis, Q, weight, predecessor, distance, combine, compare, zero);

    breadth_first_visit_to_dest(g, s_begin, s_end, d, Q, bfs_vis, color);
  }

  // Call breadth first search
  template <class Graph, class DijkstraVisitor,
            class PredecessorMap, class DistanceMap,
            class WeightMap, class IndexMap, class Compare, class Combine,
            class DistZero, class ColorMap>
  inline void
  dijkstra_shortest_paths_to_dest_no_init
    (const Graph& g,
     typename graph_traits<Graph>::vertex_descriptor s,
     typename graph_traits<Graph>::vertex_descriptor d,
     PredecessorMap predecessor, DistanceMap distance, WeightMap weight,
     IndexMap index_map,
     Compare compare, Combine combine, DistZero zero,
     DijkstraVisitor vis, ColorMap color)
  {
    dijkstra_shortest_paths_to_dest_no_init(g, &s, &s + 1, d, predecessor, distance,
                                    weight, index_map, compare, combine,
                                    zero, vis, color);
  }

  // Initialize distances and call breadth first search with default color map
  template <class VertexListGraph, class SourceInputIter, class DijkstraVisitor,
            class PredecessorMap, class DistanceMap,
            class WeightMap, class IndexMap, class Compare, class Combine,
            class DistInf, class DistZero, typename T, typename Tag, 
            typename Base>
  inline void
  dijkstra_shortest_paths_to_dest
    (const VertexListGraph& g,
     SourceInputIter s_begin, SourceInputIter s_end,
     typename graph_traits<VertexListGraph>::vertex_descriptor d,
     PredecessorMap predecessor, DistanceMap distance, WeightMap weight,
     IndexMap index_map,
     Compare compare, Combine combine, DistInf inf, DistZero zero,
     DijkstraVisitor vis,
     const bgl_named_params<T, Tag, Base>&
     BOOST_GRAPH_ENABLE_IF_MODELS_PARM(VertexListGraph,vertex_list_graph_tag))
  {
    boost::two_bit_color_map<IndexMap> color(num_vertices(g), index_map);
    dijkstra_shortest_paths_to_dest(g, s_begin, s_end,d, predecessor, distance, weight,
                            index_map, compare, combine, inf, zero, vis,
                            color);
  }

  // Initialize distances and call breadth first search with default color map
  template <class VertexListGraph, class DijkstraVisitor,
            class PredecessorMap, class DistanceMap,
            class WeightMap, class IndexMap, class Compare, class Combine,
            class DistInf, class DistZero, typename T, typename Tag, 
            typename Base>
  inline void
  dijkstra_shortest_paths_to_dest
    (const VertexListGraph& g,
     typename graph_traits<VertexListGraph>::vertex_descriptor s,
     typename graph_traits<VertexListGraph>::vertex_descriptor d,
     PredecessorMap predecessor, DistanceMap distance, WeightMap weight,
     IndexMap index_map,
     Compare compare, Combine combine, DistInf inf, DistZero zero,
     DijkstraVisitor vis,
     const bgl_named_params<T, Tag, Base>&
     BOOST_GRAPH_ENABLE_IF_MODELS_PARM(VertexListGraph,vertex_list_graph_tag))
  {
    dijkstra_shortest_paths_to_dest(g, &s, &s + 1, d, predecessor, distance, weight,
                            index_map, compare, combine, inf, zero, vis);
  }

  // Initialize distances and call breadth first search
  template <class VertexListGraph, class SourceInputIter, class DijkstraVisitor,
            class PredecessorMap, class DistanceMap,
            class WeightMap, class IndexMap, class Compare, class Combine,
            class DistInf, class DistZero, class ColorMap>
  inline void
  dijkstra_shortest_paths_to_dest
    (const VertexListGraph& g,
     SourceInputIter s_begin, SourceInputIter s_end,
     typename graph_traits<VertexListGraph>::vertex_descriptor d,
     PredecessorMap predecessor, DistanceMap distance, WeightMap weight,
     IndexMap index_map,
     Compare compare, Combine combine, DistInf inf, DistZero zero,
     DijkstraVisitor vis, ColorMap color)
  {
    typedef typename property_traits<ColorMap>::value_type ColorValue;
    typedef color_traits<ColorValue> Color;
    /*typename graph_traits<VertexListGraph>::vertex_iterator ui, ui_end;
    for (boost::tie(ui, ui_end) = vertices(g); ui != ui_end; ++ui) {
      //vis.initialize_vertex(*ui, g);
      //put(distance, *ui, inf);
      //put(predecessor, *ui, *ui);
      //put(color, *ui, Color::white());
    }*/
    for (SourceInputIter it = s_begin; it != s_end; ++it) {
      put(distance, *it, zero);
    }

    dijkstra_shortest_paths_to_dest_no_init(g, s_begin, s_end, d, predecessor, distance,
                            weight, index_map, compare, combine, zero, vis,
                            color);
  }

  // Initialize distances and call breadth first search
  template <class VertexListGraph, class DijkstraVisitor,
            class PredecessorMap, class DistanceMap,
            class WeightMap, class IndexMap, class Compare, class Combine,
            class DistInf, class DistZero, class ColorMap>
  inline void
  dijkstra_shortest_paths_to_dest
    (const VertexListGraph& g,
     typename graph_traits<VertexListGraph>::vertex_descriptor s,
     typename graph_traits<VertexListGraph>::vertex_descriptor d,
     PredecessorMap predecessor, DistanceMap distance, WeightMap weight,
     IndexMap index_map,
     Compare compare, Combine combine, DistInf inf, DistZero zero,
     DijkstraVisitor vis, ColorMap color)
  {
    dijkstra_shortest_paths_to_dest(g, &s, &s + 1, d, predecessor, distance, weight,
                            index_map, compare, combine, inf, zero,
                            vis, color);
  }

  // Initialize distances and call breadth first search
  template <class VertexListGraph, class SourceInputIter,
            class DijkstraVisitor,
            class PredecessorMap, class DistanceMap,
            class WeightMap, class IndexMap, class Compare, class Combine,
            class DistInf, class DistZero>
  inline void
  dijkstra_shortest_paths_to_dest
    (const VertexListGraph& g,
     SourceInputIter s_begin, SourceInputIter s_end,
     typename graph_traits<VertexListGraph>::vertex_descriptor d,
     PredecessorMap predecessor, DistanceMap distance, WeightMap weight,
     IndexMap index_map,
     Compare compare, Combine combine, DistInf inf, DistZero zero,
     DijkstraVisitor vis)
  {
    dijkstra_shortest_paths_to_dest(g, s_begin, s_end, d, predecessor, distance,
                            weight, index_map,
                            compare, combine, inf, zero, vis,
                            no_named_parameters());
  }

  // Initialize distances and call breadth first search
  template <class VertexListGraph, class DijkstraVisitor,
            class PredecessorMap, class DistanceMap,
            class WeightMap, class IndexMap, class Compare, class Combine,
            class DistInf, class DistZero>
  inline void
  dijkstra_shortest_paths_to_dest
    (const VertexListGraph& g,
     typename graph_traits<VertexListGraph>::vertex_descriptor s,
     typename graph_traits<VertexListGraph>::vertex_descriptor d,
     PredecessorMap predecessor, DistanceMap distance, WeightMap weight,
     IndexMap index_map,
     Compare compare, Combine combine, DistInf inf, DistZero zero,
     DijkstraVisitor vis)
  {
    std::cout << __LINE__ << std::endl;
    dijkstra_shortest_paths_to_dest(g, &s, &s + 1, predecessor, distance,
                            weight, index_map,
                            compare, combine, inf, zero, vis);
  }

  namespace detail {

    // Handle defaults for PredecessorMap and
    // Distance Compare, Combine, Inf and Zero
    template <class VertexListGraph, class DistanceMap, class WeightMap,
              class IndexMap, class Params>
    inline void
    dijkstra_to_dest_dispatch2
      (const VertexListGraph& g,
       typename graph_traits<VertexListGraph>::vertex_descriptor s,
       typename graph_traits<VertexListGraph>::vertex_descriptor d,
       DistanceMap distance, WeightMap weight, IndexMap index_map,
       const Params& params)
    {
      // Default for predecessor map
      dummy_property_map p_map;

      typedef typename property_traits<DistanceMap>::value_type D;
      D inf = choose_param(get_param(params, distance_inf_t()),
                           (std::numeric_limits<D>::max)());

      dijkstra_shortest_paths_to_dest
        (g, s, d,
         choose_param(get_param(params, vertex_predecessor), p_map),
         distance, weight, index_map,
         choose_param(get_param(params, distance_compare_t()),
                      std::less<D>()),
         choose_param(get_param(params, distance_combine_t()),
                      closed_plus<D>(inf)),
         inf,
         choose_param(get_param(params, distance_zero_t()),
                      D()),
         choose_param(get_param(params, graph_visitor),
                      make_dijkstra_visitor(null_visitor())),
         params);
    }

    template <class VertexListGraph, class DistanceMap, class WeightMap,
              class IndexMap, class Params>
    inline void
    dijkstra_to_dest_dispatch1
      (const VertexListGraph& g,
       typename graph_traits<VertexListGraph>::vertex_descriptor s,
       typename graph_traits<VertexListGraph>::vertex_descriptor d,
       DistanceMap distance, WeightMap weight, IndexMap index_map,
       const Params& params)
    {
      // Default for distance map
      typedef typename property_traits<WeightMap>::value_type D;
      D inf = choose_param(get_param(params, distance_inf_t()),
                           (std::numeric_limits<D>::max)());

      typename std::vector<D>::size_type
        n = is_default_param(distance) ? num_vertices(g) : 1;
        std::unordered_map<typename graph_traits<VertexListGraph>::vertex_descriptor , D> distance_map;
        distance_map.reserve(n);

        //std::vector<D> distance_map(n);
      detail::dijkstra_to_dest_dispatch2
        (g, s, d, choose_param(distance, make_lazy_property_map(distance_map, inf)),
       // (g, s, choose_param(distance, make_iterator_property_map
        //                      (distance_map.begin(), index_map,
         //                             distance_map[0])),
         weight, index_map, params);
    }

  } // namespace detail

  // Named Parameter Variant
  template <class VertexListGraph, class Param, class Tag, class Rest>
  inline void
  dijkstra_shortest_paths_to_dest
    (const VertexListGraph& g,
     typename graph_traits<VertexListGraph>::vertex_descriptor s,
     typename graph_traits<VertexListGraph>::vertex_descriptor d,
     const bgl_named_params<Param,Tag,Rest>& params)
  {
    // Default for edge weight and vertex index map is to ask for them
    // from the graph.  Default for the visitor is null_visitor.
    detail::dijkstra_to_dest_dispatch1
      (g, s, d,
       get_param(params, vertex_distance),
       choose_const_pmap(get_param(params, edge_weight), g, edge_weight),
       choose_const_pmap(get_param(params, vertex_index), g, vertex_index),
       params);
  }
} // namespace boost

#endif // BOOST_GRAPH_DIJKSTRA_TO_DEST_HPP
