#ifndef  BOOST_DATE_TIME_FORMAT_HPP
#define  BOOST_DATE_TIME_FORMAT_HPP

#include  <boost/date_time.hpp>
#include  <sstream>

template<typename DateTimeType>
struct date_time_facet_traits{
    typedef void input_facet_type;
    typedef void facet_type;
};
template<>
struct date_time_facet_traits<boost::gregorian::date>{
    typedef boost::gregorian::date_input_facet input_facet_type;
    typedef boost::gregorian::date_facet facet_type;
};
template<>
struct date_time_facet_traits<boost::posix_time::ptime>{
    typedef boost::posix_time::time_input_facet input_facet_type;
    typedef boost::posix_time::time_facet facet_type;
};

template<typename DateTimeType>
std::string to_format_string(DateTimeType const& dt, char const* format){
    std::stringstream ss;
    ss.imbue(std::locale(ss.getloc(), new typename date_time_facet_traits<DateTimeType>::facet_type(format)));
    ss << dt;
    return ss.str();
}

template<typename DateTimeType>
DateTimeType from_format_string(std::string const& str, char const* format){
    DateTimeType d;
    std::stringstream ss(str);
    ss.imbue(std::locale(ss.getloc(), new typename date_time_facet_traits<DateTimeType>::input_facet_type(format)));
    ss >> d;
    return d;
}

#endif  /*BOOST_DATE_TIME_FORMAT_HPP*/
