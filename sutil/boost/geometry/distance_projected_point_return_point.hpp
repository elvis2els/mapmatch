// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2008-2014 Bruno Lalande, Paris, France.
// Copyright (c) 2008-2014 Barend Gehrels, Amsterdam, the Netherlands.
// Copyright (c) 2009-2014 Mateusz Loskot, London, UK.

// This file was modified by Oracle on 2014.
// Modifications copyright (c) 2014, Oracle and/or its affiliates.

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle
// Contributed and/or modified by Adam Wulkiewicz, on behalf of Oracle

// edit by syhkiller@163.com, the template file is
// boost/geometry/strategies/cartesian/distance_projected_point_ax.hpp

// Parts of Boost.Geometry are redesigned from Geodan's Geographic Library
// (geolib/GGL), copyright (c) 1995-2010 Geodan, Amsterdam, the Netherlands.

// Use, modification and distribution is subject to the Boost Software License,
// Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef BOOST_GEOMETRY_STRATEGIES_CARTESIAN_DISTANCE_PROJECTED_POINT_RETURN_POINT_HPP
#define BOOST_GEOMETRY_STRATEGIES_CARTESIAN_DISTANCE_PROJECTED_POINT_RETURN_POINT_HPP


#include <algorithm>

#include <boost/concept_check.hpp>
#include <boost/mpl/if.hpp>
#include <boost/type_traits.hpp>

#include <boost/geometry/core/access.hpp>
#include <boost/geometry/core/point_type.hpp>

#include <boost/geometry/algorithms/convert.hpp>
#include <boost/geometry/arithmetic/arithmetic.hpp>
#include <boost/geometry/arithmetic/dot_product.hpp>

#include <boost/geometry/strategies/tags.hpp>
#include <boost/geometry/strategies/distance.hpp>
#include <boost/geometry/strategies/default_distance_result.hpp>
#include <boost/geometry/strategies/cartesian/distance_pythagoras.hpp>
#include <boost/geometry/strategies/cartesian/distance_projected_point.hpp>

#include <boost/geometry/util/select_coordinate_type.hpp>

// Helper geometry (projected point on line)
#include <boost/geometry/geometries/point.hpp>


namespace boost {
namespace geometry
{


namespace strategy {
namespace distance
{


#ifndef DOXYGEN_NO_DETAIL
namespace detail
{

template <typename T, typename P>
struct projected_point_return_result {
    typedef T value_type;
    T distance;
    P point;
    std::pair<T&, P&> to_pair() {
        return { distance, point};
    }
    std::pair<T const&, P const&> to_pair()const {
        return {distance, point};
    }
    projected_point_return_result(T const& c = T(0)): distance(c) {}

    friend inline bool operator<(projected_point_return_result const& left,
                                 projected_point_return_result const& right)
    {
        return left.distance < right.distance;
    }

    friend inline bool operator==(projected_point_return_result const& left,
                                  projected_point_return_result const& right) {
        return left.distance == right.distance;
    }
};

template <typename Distance>
class projected_point_return_result_less
{
public:
    inline bool operator()(Distance const& left, Distance const& right) const
    {
        return left < right;
    }
};

template
<
    typename CalculationType = void,
    typename Strategy = pythagoras<CalculationType>
    >
class projected_point_return_point
{
public :
    template <typename Point, typename PointOfSegment>
    struct calculation_type
        : public projected_point<CalculationType, Strategy>
          ::template calculation_type<Point, PointOfSegment>
    {};

    template <typename Point, typename PointOfSegment>
    struct result_type
    {
        typedef projected_point_return_result
        <
        typename calculation_type<Point, PointOfSegment>::type,
                 Point
                 > type;
    };

public :

    template <typename Point, typename PointOfSegment>
    inline typename result_type<Point, PointOfSegment>::type
    apply(Point const& p, PointOfSegment const& p1, PointOfSegment const& p2) const
    {
        assert_dimension_equal<Point, PointOfSegment>();

        typedef typename calculation_type<Point, PointOfSegment>::type calculation_type;

        // A projected point of points in Integer coordinates must be able to be
        // represented in FP.
        typedef model::point
        <
        calculation_type,
        dimension<PointOfSegment>::value,
        typename coordinate_system<PointOfSegment>::type
        > fp_point_type;

        // For convenience
        typedef fp_point_type fp_vector_type;
        typename result_type<Point, PointOfSegment>::type result;

        /*
            Algorithm [p: (px,py), p1: (x1,y1), p2: (x2,y2)]
            VECTOR v(x2 - x1, y2 - y1)
            VECTOR w(px - x1, py - y1)
            c1 = w . v
            c2 = v . v
            b = c1 / c2
            RETURN POINT(x1 + b * vx, y1 + b * vy)
        */

        // v is multiplied below with a (possibly) FP-value, so should be in FP
        // For consistency we define w also in FP
        fp_vector_type v, w, projected;

        geometry::convert(p2, v);
        geometry::convert(p, w);
        geometry::convert(p1, projected);
        subtract_point(v, projected);
        subtract_point(w, projected);

        Strategy strategy;
        boost::ignore_unused_variable_warning(strategy);

        calculation_type const zero = calculation_type();
        calculation_type const c1 = dot_product(w, v);
        if (c1 <= zero)
        {
            result.distance = strategy.apply(p, p1);
            geometry::convert(p1, result.point);
            return result;
        }
        calculation_type const c2 = dot_product(v, v);
        if (c2 <= c1)
        {
            result.distance = strategy.apply(p, p2);
            geometry::convert(p2, result.point);
            return result;
        }

        // See above, c1 > 0 AND c2 > c1 so: c2 != 0
        calculation_type const b = c1 / c2;

        multiply_value(v, b);
        add_point(projected, v);

        result.distance = strategy.apply(p, projected);
        geometry::convert(projected, result.point );
        return result;
    }
};

} // namespace detail
#endif // DOXYGEN_NO_DETAIL

#ifndef DOXYGEN_NO_STRATEGY_SPECIALIZATIONS
namespace services
{

template <typename CalculationType, typename Strategy>
struct tag<detail::projected_point_return_point<CalculationType, Strategy> >
{
    typedef strategy_tag_distance_point_segment type;
};


template <typename CalculationType, typename Strategy, typename P, typename PS>
struct return_type<detail::projected_point_return_point<CalculationType, Strategy>, P, PS>
{
    typedef typename detail::projected_point_return_point<CalculationType, Strategy>
    ::template result_type<P, PS>::type type;
};


template <typename CalculationType, typename Strategy>
struct comparable_type<detail::projected_point_return_point<CalculationType, Strategy> >
{
    // Define a projected_point strategy with its underlying point-point-strategy
    // being comparable
    typedef detail::projected_point_return_point
    <
    CalculationType,
    typename comparable_type<Strategy>::type
    > type;
};


template <typename CalculationType, typename Strategy>
struct get_comparable<detail::projected_point_return_point<CalculationType, Strategy> >
{
    typedef typename comparable_type
    <
    detail::projected_point_return_point<CalculationType, Strategy>
    >::type comparable_type;
public :
    static inline comparable_type apply(detail::projected_point_return_point<CalculationType, Strategy> const& )
    {
        return comparable_type();
    }
};


template <typename CalculationType, typename Strategy, typename P, typename PS>
struct result_from_distance<detail::projected_point_return_point<CalculationType, Strategy>, P, PS>
{
private :
    typedef typename return_type<detail::projected_point_return_point<CalculationType, Strategy>, P, PS>::type return_type;
public :
    template <typename T>
    static inline return_type apply(detail::projected_point_return_point<CalculationType, Strategy> const& , T const& value)
    {
        Strategy s;
        return_type ret;
        ret.distance = result_from_distance<Strategy, P, PS>::apply(s, value.distance );
        geometry::convert(result_from_distance<Strategy, P, PS>::apply(s, value.point ), ret.point );
        return ret;
    }
};


} // namespace services
#endif // DOXYGEN_NO_STRATEGY_SPECIALIZATIONS

template
<
    typename CalculationType = void,
    typename Strategy = pythagoras<CalculationType>
    >
using projected_point_return_point = detail::projected_point_return_point<CalculationType, Strategy> ;

}
} // namespace strategy::distance


}
} // namespace boost::geometry

#endif // BOOST_GEOMETRY_STRATEGIES_CARTESIAN_DISTANCE_PROJECTED_POINT_RETURN_POINT_HPP
