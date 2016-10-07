#ifndef  VISITOR_ADAPTOR_HPP
#define  VISITOR_ADAPTOR_HPP

#include  <boost/graph/visitors.hpp>

namespace boost {

template<typename C, typename Tag>
struct VisitorWithEventFilter : base_visitor<VisitorWithEventFilter<C, Tag> > {
    typedef Tag event_filter;
    C c;

    VisitorWithEventFilter(C c) : c(c) {
    }

    VisitorWithEventFilter(VisitorWithEventFilter const &o) : c(o.c) {
    }

    template<typename... Arg>
    void operator()(Arg &&... arg) {
        c(std::forward<Arg>(arg)...);
    }
};

template<typename Tag, typename Fun>
VisitorWithEventFilter<Fun, Tag>
adapt_visitor(Fun f, Tag tg = Tag()) {
    return VisitorWithEventFilter<Fun, Tag>(f);
}

}



#endif  /*VISITOR_ADAPTOR_HPP*/
