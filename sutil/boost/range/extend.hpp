#ifndef  RANGE_EXTEND_HPP
#define  RANGE_EXTEND_HPP

#include  <boost/range.hpp>
#include  <boost/range/adaptors.hpp>
#include  <boost/iterator.hpp>
#include  <boost/iterator/zip_iterator.hpp>
#include  <boost/iterator/counting_iterator.hpp>
#include  <boost/algorithm/cxx11/all_of.hpp>
#include  <boost/algorithm/cxx11/any_of.hpp>
#include  <boost/algorithm/cxx11/copy_if.hpp>
#include  <type_traits>

namespace boost {
struct with_prepare {};
struct with_cleanup {};


template<typename InputIterator, typename BinaryFunction>
BinaryFunction foreach_adjacent(InputIterator first, InputIterator last, BinaryFunction fun) {
    InputIterator second = std::next(first, 1);
    while (first != last and second != last) {
        fun(*first++, *second++);
    }
    return fun;
}

template<typename SinglePassRange, typename BinaryFunction>
inline BinaryFunction foreach_adjacent(SinglePassRange && rng, BinaryFunction fun) {
    return foreach_adjacent(boost::begin(std::forward<SinglePassRange>(rng)), boost::end(std::forward<SinglePassRange>(rng)), fun);
}

template<typename Tag, typename InputIterator, typename UnaryFunction, typename BinaryFunction>
typename std::enable_if <
std::is_same<with_prepare, Tag>::value,
    BinaryFunction >::type

foreach_adjacent(InputIterator first, InputIterator last, UnaryFunction prepare, BinaryFunction fun) {
    if (first != last)
        prepare(*first);
    InputIterator second = std::next(first, 1);
    while (first != last and second != last) {
        fun(*first++, *second++);
    }
    return fun;
}


template<typename Tag, typename SinglePassRange, typename UnaryFunction, typename BinaryFunction>
inline typename std::enable_if <
std::is_same<with_prepare, Tag>::value,
    BinaryFunction >::type
foreach_adjacent(SinglePassRange&& rng, UnaryFunction prepare, BinaryFunction fun) {
    return foreach_adjacent<Tag>(boost::begin(std::forward<SinglePassRange>(rng)), boost::end(std::forward<SinglePassRange>(rng)),
                                 prepare, fun
                                );
}


template<typename Tag, typename InputIterator, typename UnaryFunction, typename BinaryFunction>
typename std::enable_if <
std::is_same<with_cleanup, Tag>::value,
    UnaryFunction
    >::type

foreach_adjacent(InputIterator first, InputIterator last, BinaryFunction fun, UnaryFunction cleanup) {
    InputIterator second = std::next(first, 1);
    if (second == last)
        cleanup(*first);
    while (first != last and second != last) {
        fun(*first++, *second++);
        if (second == last)
            cleanup(*first);
    }
    return cleanup;
}

template<typename Tag, typename SinglePassRange, typename UnaryFunction, typename BinaryFunction>
typename std::enable_if <
std::is_same<with_cleanup, Tag>::value,
    UnaryFunction
    >::type

inline foreach_adjacent(SinglePassRange&& rng, BinaryFunction fun, UnaryFunction cleanup) {
    return foreach_adjacent<Tag>(boost::begin(std::forward<SinglePassRange>(rng)), boost::end(std::forward<SinglePassRange>(rng)),
                                 fun, cleanup);
}

template<typename InputIterator, typename OutputIterator, typename BinaryFunction>
OutputIterator copy_adjacent(InputIterator first, InputIterator last, OutputIterator d_first, BinaryFunction fun) {
    InputIterator second = std::next(first, 1);
    while (first != last and second != last) {
        *d_first++ = fun(*first++, *second++);
    }
    return d_first;
}

template<typename SinglePassRange, typename OutputIterator, typename BinaryFunction>
inline OutputIterator copy_adjacent(SinglePassRange&& rng, OutputIterator d_first, BinaryFunction fun) {
    return copy_adjacent(boost::begin(std::forward<SinglePassRange>(rng)), boost::end(std::forward<SinglePassRange>(rng)),
                         d_first, fun);
}

template<typename Tag, typename InputIterator, typename OutputIterator, typename BinaryFunction, typename UnaryFunction>
typename std::enable_if <
std::is_same<Tag, with_prepare>::value,
    OutputIterator >::type
copy_adjacent(InputIterator first, InputIterator last, OutputIterator d_first, UnaryFunction prepare, BinaryFunction fun) {
    InputIterator second = std::next(first, 1);
    if (first != last)
        *d_first++ = prepare(*first);
    while (first != last and second != last) {
        *d_first++ = fun(*first++, *second++);
    }
    return d_first;
}


template<typename Tag, typename SinglePassRange, typename OutputIterator, typename BinaryFunction, typename UnaryFunction>
inline typename std::enable_if <
std::is_same<Tag, with_prepare>::value,
    OutputIterator >::type
copy_adjacent(SinglePassRange&& rng, OutputIterator d_first, UnaryFunction prepare, BinaryFunction fun) {
    return copy_adjacent<Tag>(boost::begin(std::forward<SinglePassRange>(rng)), boost::end(std::forward<SinglePassRange>(rng)),
                              d_first, prepare, fun);
}


template<typename Tag, typename InputIterator, typename OutputIterator, typename BinaryFunction, typename UnaryFunction>
typename std::enable_if <
std::is_same<Tag, with_cleanup>::value,
    OutputIterator >::type
copy_adjacent(InputIterator first, InputIterator last, OutputIterator d_first, BinaryFunction fun, UnaryFunction cleanup) {
    InputIterator second = std::next(first, 1);
    if (second == last)
        *d_first++ = cleanup(*first);
    while (first != last and second != last) {
        *d_first++ = fun(*first++, *second++);
        if (second == last) {
            *d_first++ = cleanup(*first);
        }
    }
    return d_first;
}

template<typename Tag, typename SinglePassRange, typename OutputIterator, typename BinaryFunction, typename UnaryFunction>
inline typename std::enable_if <
std::is_same<Tag, with_cleanup>::value,
    OutputIterator >::type
copy_adjacent(SinglePassRange&& rng, OutputIterator d_first, BinaryFunction fun, UnaryFunction cleanup) {
    return copy_adjacent<Tag>(boost::begin(std::forward<SinglePassRange>(rng)), boost::end(std::forward<SinglePassRange>(rng)),
                              d_first, fun, cleanup);
}



#define MAKE_ZIP_RANGE_BODY \
boost::make_iterator_range(\
            boost::make_zip_iterator(\
                std::make_tuple(boost::begin(ranges)...)\
                ),\
            boost::make_zip_iterator( \
                std::make_tuple(boost::end(ranges)...)\
                )\
            )

//template<typename ...Ranges>
//auto zip_range(Ranges&&... ranges)-> decltype(MAKE_ZIP_RANGE_BODY) {
//    return MAKE_ZIP_RANGE_BODY;
//}

//template<typename ...Ranges>
//struct zip_range:
//    public iterator_range<
//        zip_iterator<
//            tuple<
//                typename range_iterator<typename std::remove_reference<Ranges>::type>::type...
//            >
//        >
//    >{
//        typedef iterator_range<
//        zip_iterator<
//            tuple<
//                typename range_iterator<typename std::remove_reference<Ranges>::type>::type...
//            >
//        >
//    >base_t;
//        zip_range(Ranges&&... ranges):base_t(
//                boost::make_zip_iterator(
//                    boost::make_tuple(boost::begin(ranges)...)
//                    ),
//                boost::make_zip_iterator(
//                    boost::make_tuple(boost::end(ranges)...)
//                    )
//                )
//        {}
//    };

template<typename ...Ranges>
boost::iterator_range <
boost::zip_iterator <
boost::tuple <
typename boost::range_iterator<typename std::remove_reference<Ranges>::type>::type...
>
>
>
inline ziped(Ranges&&... ranges) {
    return  boost::make_iterator_range(
                boost::make_zip_iterator(
                    boost::make_tuple(boost::begin(ranges)...)
                ),
                boost::make_zip_iterator(
                    boost::make_tuple( boost::end(ranges)...)
                )
            );
}

template<typename ForwardRange>
struct adjacented_range:
    public iterator_range <
    zip_iterator <
    tuple<BOOST_DEDUCED_TYPENAME range_iterator<ForwardRange>::type,
    BOOST_DEDUCED_TYPENAME range_iterator<ForwardRange>::type
    >
    >
    > {

    typedef iterator_range<zip_iterator<tuple<BOOST_DEDUCED_TYPENAME range_iterator<ForwardRange>::type,
            BOOST_DEDUCED_TYPENAME range_iterator<ForwardRange>::type>>> base_t;


    adjacented_range(ForwardRange& rng): base_t(
            boost::make_zip_iterator(boost::make_tuple(boost::begin(rng), boost::begin(rng) + 1)),
            boost::make_zip_iterator(boost::make_tuple(boost::end(rng) - 1, boost::end(rng)))) {}

};

namespace detail {
struct adjacented_forwarder {};
template<typename ForwardRange>
adjacented_range<ForwardRange>
operator|(ForwardRange& rng, adjacented_forwarder const&) {
    return adjacented_range<ForwardRange>(rng);
}

template<typename ForwardRange>
adjacented_range<const ForwardRange>
operator|(ForwardRange const& rng, adjacented_forwarder const&) {
    return adjacented_range<const ForwardRange>(rng);
}

}//endof namespace detail

detail::adjacented_forwarder const adjacented = detail::adjacented_forwarder();

template<typename ForwardRange>
adjacented_range<ForwardRange> adjacent(ForwardRange& rng) {
    return rng | adjacented;
}
template<typename ForwardRange>
adjacented_range<const ForwardRange> adjacent(ForwardRange const& rng) {
    return rng | adjacented;
}

template<typename SinglePassRange>
struct enumerate_range:
    public iterator_range <
    zip_iterator <
    tuple<counting_iterator<int>, BOOST_DEDUCED_TYPENAME  range_iterator<SinglePassRange>::type>
    >
    > {
    typedef iterator_range <
    zip_iterator <
    tuple<counting_iterator<int>,  BOOST_DEDUCED_TYPENAME range_iterator<SinglePassRange>::type>
    > > base_t;

    enumerate_range(SinglePassRange& rng, int start = 0): base_t(
            boost::make_zip_iterator(
                boost::make_tuple(
                    boost::make_counting_iterator(start), boost::begin(rng)
                )),
            boost::make_zip_iterator(
                boost::make_tuple(
                    boost::make_counting_iterator(start  + (int)size(rng)), boost::end(rng)
                ))
        ) {}

};
template<typename SinglePassRange>
enumerate_range<SinglePassRange>
enumerate(SinglePassRange & rng, int s = 0) {
    return enumerate_range<SinglePassRange>(rng, s);
}

template<typename SinglePassRange>
enumerate_range<SinglePassRange const>
enumerate(SinglePassRange const& rng, int s = 0) {
    return enumerate_range<SinglePassRange const>(rng , s);
}

namespace detail {
struct enumerated_forwarder { int start; };
struct enumerate_forwarder_generator {
    enumerated_forwarder operator()(int s)const {
        enumerated_forwarder fwd;
        fwd.start = s;
        return fwd;
    }
};

template<typename SinglePassRange>
enumerate_range<SinglePassRange const> operator|(SinglePassRange const& rng, enumerated_forwarder const& fwd) {
    return enumerate(rng, fwd.start);
}

template<typename SinglePassRange>
enumerate_range<SinglePassRange > operator|(SinglePassRange & rng, enumerated_forwarder const& fwd) {
    return enumerate(rng, fwd.start);
}

template<typename SinglePassRange>
enumerate_range<SinglePassRange const> operator|(SinglePassRange const& rng, enumerate_forwarder_generator const&) {
    return enumerate(rng);
}
template<typename SinglePassRange>
enumerate_range<SinglePassRange> operator|(SinglePassRange & rng, enumerate_forwarder_generator const&) {
    return enumerate(rng);
}
}

detail::enumerate_forwarder_generator const enumerated = detail::enumerate_forwarder_generator();
}//endof namespace boost
#endif  /*RANGE_EXTEND_HPP*/
