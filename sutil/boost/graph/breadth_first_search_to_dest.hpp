//
//=======================================================================
// Copyright 1997, 1998, 1999, 2000 University of Notre Dame.
// Authors: Andrew Lumsdaine, Lie-Quan Lee, Jeremy G. Siek
//
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//=======================================================================
//
#ifndef BOOST_GRAPH_BREADTH_FIRST_SEARCH_TO_DEST_HPP
#define BOOST_GRAPH_BREADTH_FIRST_SEARCH_TO_DEST_HPP

/*
  Breadth First Search Algorithm (Cormen, Leiserson, and Rivest p. 470)
*/

/*

  bfs when first meet destination vertex will stop,
  after examine vertex

  [note] both breadth_first_visit_to_dest and breadth_first_search_to_dest
  will not initialize the color map
  edit by syhkiller@163.com
*/
#include <boost/config.hpp>
#include <vector>
#include <boost/pending/queue.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/visitors.hpp>
#include <boost/graph/named_function_params.hpp>
#include <boost/graph/overloading.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/two_bit_color_map.hpp>
#include <boost/concept/assert.hpp>

#ifdef BOOST_GRAPH_USE_MPI
#include <boost/graph/distributed/concepts.hpp>
#endif // BOOST_GRAPH_USE_MPI
//usr the struct defined in bfs
#include <boost/graph/breadth_first_search.hpp>
namespace boost {

  // Multiple-source version
  template <class IncidenceGraph, class Buffer, class BFSVisitor,
            class ColorMap, class SourceIterator>
  void breadth_first_visit_to_dest
    (const IncidenceGraph& g,
     SourceIterator sources_begin, SourceIterator sources_end, 
     typename graph_traits<IncidenceGraph>::vertex_descriptor dest,
     Buffer& Q, BFSVisitor vis, ColorMap color)
  {
    BOOST_CONCEPT_ASSERT(( IncidenceGraphConcept<IncidenceGraph> ));
    typedef graph_traits<IncidenceGraph> GTraits;
    typedef typename GTraits::vertex_descriptor Vertex;
    BOOST_CONCEPT_ASSERT(( BFSVisitorConcept<BFSVisitor, IncidenceGraph> ));
    BOOST_CONCEPT_ASSERT(( ReadWritePropertyMapConcept<ColorMap, Vertex> ));
    typedef typename property_traits<ColorMap>::value_type ColorValue;
    typedef color_traits<ColorValue> Color;
    typename GTraits::out_edge_iterator ei, ei_end;

    for (; sources_begin != sources_end; ++sources_begin) {
      Vertex s = *sources_begin;
      put(color, s, Color::gray());           vis.discover_vertex(s, g);
      Q.push(s);
    }
    while (! Q.empty()) {
      Vertex u = Q.top(); Q.pop();            vis.examine_vertex(u, g);
      if ( u == dest)   break;
      for (boost::tie(ei, ei_end) = out_edges(u, g); ei != ei_end; ++ei) {
        Vertex v = target(*ei, g);            vis.examine_edge(*ei, g);
        ColorValue v_color = get(color, v);
        if (v_color == Color::white()) {      vis.tree_edge(*ei, g);
          put(color, v, Color::gray());       vis.discover_vertex(v, g);
          Q.push(v);
        } else {                              vis.non_tree_edge(*ei, g);
          if (v_color == Color::gray())       vis.gray_target(*ei, g);
          else                                vis.black_target(*ei, g);
        }
      } // end for
      put(color, u, Color::black());          vis.finish_vertex(u, g);
    } // end while
  } // breadth_first_visit

  // Single-source version
  template <class IncidenceGraph, class Buffer, class BFSVisitor,
            class ColorMap>
  void breadth_first_visit_to_dest
    (const IncidenceGraph& g,
     typename graph_traits<IncidenceGraph>::vertex_descriptor s,
     typename graph_traits<IncidenceGraph>::vertex_descriptor d,
     Buffer& Q, BFSVisitor vis, ColorMap color)
  {
    typename graph_traits<IncidenceGraph>::vertex_descriptor sources[1] = {s};
    breadth_first_visit_to_dest(g, sources, sources + 1, d, Q, vis, color);
  }


  template <class VertexListGraph, class SourceIterator,
            class Buffer, class BFSVisitor,
            class ColorMap>
  void breadth_first_search_to_dest
    (const VertexListGraph& g,
     SourceIterator sources_begin, SourceIterator sources_end,
     typename graph_traits<VertexListGraph>::vertex_descriptor d,
     Buffer& Q, BFSVisitor vis, ColorMap color)
  {
      /*
    // Initialization
    typedef typename property_traits<ColorMap>::value_type ColorValue;
    typedef color_traits<ColorValue> Color;
    typename boost::graph_traits<VertexListGraph>::vertex_iterator i, i_end;
    for (boost::tie(i, i_end) = vertices(g); i != i_end; ++i) {
      vis.initialize_vertex(*i, g);
      put(color, *i, Color::white());
    }*/
    breadth_first_visit_to_dest(g, sources_begin, sources_end, d, Q, vis, color);
  }

  template <class VertexListGraph, class Buffer, class BFSVisitor,
            class ColorMap>
  void breadth_first_search_to_dest
    (const VertexListGraph& g,
     typename graph_traits<VertexListGraph>::vertex_descriptor s,
     typename graph_traits<VertexListGraph>::vertex_descriptor d,
     Buffer& Q, BFSVisitor vis, ColorMap color)
  {
    typename graph_traits<VertexListGraph>::vertex_descriptor sources[1] = {s};
    breadth_first_search_to_dest(g, sources, sources + 1, d, Q, vis, color);
  }

  namespace detail {

    template <class VertexListGraph, class ColorMap, class BFSVisitor,
      class P, class T, class R>
    void bfs2dest_helper
      (VertexListGraph& g,
       typename graph_traits<VertexListGraph>::vertex_descriptor s,
       typename graph_traits<VertexListGraph>::vertex_descriptor d,
       ColorMap color,
       BFSVisitor vis,
       const bgl_named_params<P, T, R>& params,
       boost::mpl::false_)
    {
      typedef graph_traits<VertexListGraph> Traits;
      // Buffer default
      typedef typename Traits::vertex_descriptor Vertex;
      typedef boost::queue<Vertex> queue_t;
      queue_t Q;
      breadth_first_search_to_dest
        (g, s, d,
         choose_param(get_param(params, buffer_param_t()), boost::ref(Q)).get(),
         vis, color);
    }

#ifdef BOOST_GRAPH_USE_MPI
    template <class DistributedGraph, class ColorMap, class BFSVisitor,
              class P, class T, class R>
    void bfs2dest_helper
      (DistributedGraph& g,
       typename graph_traits<DistributedGraph>::vertex_descriptor s,
       typename graph_traits<DistributedGraph>::vertex_descriptor d,
       ColorMap color,
       BFSVisitor vis,
       const bgl_named_params<P, T, R>& params,
       boost::mpl::true_);
#endif // BOOST_GRAPH_USE_MPI

    //-------------------------------------------------------------------------
    // Choose between default color and color parameters. Using
    // function dispatching so that we don't require vertex index if
    // the color default is not being used.

    template <class ColorMap>
    struct bfs2dest_dispatch {
      template <class VertexListGraph, class P, class T, class R>
      static void apply
      (VertexListGraph& g,
       typename graph_traits<VertexListGraph>::vertex_descriptor s,
       typename graph_traits<VertexListGraph>::vertex_descriptor d,
       const bgl_named_params<P, T, R>& params,
       ColorMap color)
      {
        bfs2dest_helper
          (g, s,d, color,
           choose_param(get_param(params, graph_visitor),
                        make_bfs_visitor(null_visitor())),
           params,
           boost::mpl::bool_<
             boost::is_base_and_derived<
               distributed_graph_tag,
               typename graph_traits<VertexListGraph>::traversal_category>::value>());
      }
    };

    template <>
    struct bfs2dest_dispatch<param_not_found> {
      template <class VertexListGraph, class P, class T, class R>
      static void apply
      (VertexListGraph& g,
       typename graph_traits<VertexListGraph>::vertex_descriptor s,
       typename graph_traits<VertexListGraph>::vertex_descriptor d,
       const bgl_named_params<P, T, R>& params,
       param_not_found)
      {
        null_visitor null_vis;

        bfs2dest_helper
          (g, s, d,
           make_two_bit_color_map
           (num_vertices(g),
            choose_const_pmap(get_param(params, vertex_index),
                              g, vertex_index)),
           choose_param(get_param(params, graph_visitor),
                        make_bfs_visitor(null_vis)),
           params,
           boost::mpl::bool_<
             boost::is_base_and_derived<
               distributed_graph_tag,
               typename graph_traits<VertexListGraph>::traversal_category>::value>());
      }
    };

  } // namespace detail

#if 1
  // Named Parameter Variant
  template <class VertexListGraph, class P, class T, class R>
  void breadth_first_search_to_dest
    (const VertexListGraph& g,
     typename graph_traits<VertexListGraph>::vertex_descriptor s,
     typename graph_traits<VertexListGraph>::vertex_descriptor d,
     const bgl_named_params<P, T, R>& params)
  {
    // The graph is passed by *const* reference so that graph adaptors
    // (temporaries) can be passed into this function. However, the
    // graph is not really const since we may write to property maps
    // of the graph.
    VertexListGraph& ng = const_cast<VertexListGraph&>(g);
    typedef typename get_param_type< vertex_color_t, bgl_named_params<P,T,R> >::type C;
    detail::bfs2dest_dispatch<C>::apply(ng, s, d, params,
                                   get_param(params, vertex_color));
  }
#endif


  // This version does not initialize colors, user has to.

  template <class IncidenceGraph, class P, class T, class R>
  void breadth_first_visit_to_dest
    (const IncidenceGraph& g,
     typename graph_traits<IncidenceGraph>::vertex_descriptor s,
     typename graph_traits<IncidenceGraph>::vertex_descriptor d,
     const bgl_named_params<P, T, R>& params)
  {
    // The graph is passed by *const* reference so that graph adaptors
    // (temporaries) can be passed into this function. However, the
    // graph is not really const since we may write to property maps
    // of the graph.
    IncidenceGraph& ng = const_cast<IncidenceGraph&>(g);

    typedef graph_traits<IncidenceGraph> Traits;
    // Buffer default
    typedef typename Traits::vertex_descriptor vertex_descriptor;
    typedef boost::queue<vertex_descriptor> queue_t;
    queue_t Q;

    breadth_first_visit_to_dest
      (ng, s, d,
       choose_param(get_param(params, buffer_param_t()), boost::ref(Q)).get(),
       choose_param(get_param(params, graph_visitor),
                    make_bfs_visitor(null_visitor())),
       choose_pmap(get_param(params, vertex_color), ng, vertex_color)
       );
  }
} // namespace boost


#endif // BOOST_GRAPH_BREADTH_FIRST_SEARCH_HPP

