#ifndef  KEY_VISITOR_HPP
#define  KEY_VISITOR_HPP

#include  <utility>
#include  <memory>

template<typename T>
struct key {
    typedef T value_type;
    typedef T base_type;
    T& operator()(T& t)const {
        return t;
    }

    const T& operator()(const T& t)const {
        return t;
    }
};

template<typename V, typename C>
struct key<V C::*> {
    typedef V value_type;
    typedef C base_type;
    V C::* member;

    key(V C::* m): member(m) {}

    V& operator()(C& c)const {
        return c.*member;
    }
    V const& operator()(C const& c)const {
        return c.*member;
    }

    V& operator()(C* c)const {
        return c->*member;
    }
    V const& operator()(C const* c)const {
        return c->*member;
    }

    V& operator()(std::shared_ptr<C> ptr)const {
        return (*this)(ptr.get());
    }

    V const& operator()(std::shared_ptr<C const> ptr)const {
        return (*this)(ptr.get());
    }

    V& operator()(std::weak_ptr<C> ptr)const {
        return (*this)(ptr.get());
    }
    V const& operator()(std::weak_ptr<C const> ptr)const {
        return (*this)(ptr.get());
    }

};

template<typename R, typename C>
struct key<R(C::*)()> {
    typedef R value_type;
    typedef C base_type;
    R(C::*mem_fun)();
    key(R (C::*mf)()): mem_fun(mf) {}


    R operator()(C& c)const {
        return (c.*mem_fun)();
    }
    R operator()(C* c)const {
        return (c->*mem_fun)();
    }
    R operator()(std::shared_ptr<C> ptr)const {
        return (*this)(*ptr);
    }
    R operator()(std::weak_ptr<C> ptr)const {
        return (*this)(*ptr);
    }
};

template<typename R, typename C>
struct key<R(C::*)()const> {
    typedef R value_type;
    R(C::*mem_fun)()const;
    key(R (C::*mf)()const): mem_fun(mf) {}
    R operator()(C const& c)const {
        return (c.*mem_fun)();
    }
    R operator()(C const* c)const {
        return (c->*mem_fun)();
    }
    R operator()(std::shared_ptr<C const> ptr)const {
        return (*this)(*ptr);
    }
    R operator()(std::shared_ptr<C> ptr)const {
        return (*this)(*ptr);
    }
    R operator()(std::weak_ptr<C const> ptr)const {
        return (*this)(*ptr);
    }
    R operator()(std::weak_ptr<C> ptr)const {
        return (*this)(*ptr);
    }
};


template<typename V, typename C>
key<V C::*> key_of(V C::* m) {
    return key<V C::*>(m);
}

template<typename R, typename C>
key<R(C::*)()> key_of(R(C::*m)()) {
    return key<R(C::*)()>(m);
}

template<typename R, typename C>
key<R(C::*)()const> key_of(R(C::*m)()const) {
    return key<R(C::*)()const>(m);
}

template<typename BinaryFun, typename Key>
struct binary {
    BinaryFun binary_fun;
    Key key;
    binary(Key const& k): key(k) {}
    binary(BinaryFun const& f, Key const& k): binary_fun(f), key(k) {}
    template<typename T1, typename T2>
    auto operator()(T1&& a, T2&& b)->decltype(binary_fun(key(std::forward<T1>(a)), key(std::forward<T2>(b)))) {
        return binary_fun(key(std::forward<T1>(a)), key(std::forward<T2>(b)));
    }
    template<typename T, typename V>
    auto operator()(T&& a, V&& b)->decltype(binary_fun(key(std::forward<T>(a)), std::forward<V>(b))) {
        return binary_fun(key(std::forward<T>(a)), std::forward<V>(b));
    }
    template<typename V, typename T>
    auto operator()(V&& a, T&& b)->decltype(binary_fun(std::forward<V>(a), key(std::forward<T>(b)))) {
        return binary_fun(std::forward<V>(a), key(std::forward<T>(b)));
    }
};

template<template<typename E> class TemplateBinaryFun, typename V, typename C>
binary<TemplateBinaryFun<V>, key<V C::*> > binary_of(V C::* m) {
    key<V C::* > k(m);
    return binary<TemplateBinaryFun<V>, key<V C::* > >(k);
}

template<template<typename E> class TemplateBinaryFun, typename R, typename C>
binary<TemplateBinaryFun<R>, key<R (C::*)() > > binary_of(R (C::* m)()) {
    key<R(C::*)() > k(m);
    return binary<TemplateBinaryFun<R>, key<R(C::*)() > >(k);
}

template<template<typename E> class TemplateBinaryFun, typename R, typename C>
binary<TemplateBinaryFun<R>, key<R (C::*)()const > > binary_of(R (C::* m)()const) {
    key<R(C::*)()const > k(m);
    return binary<TemplateBinaryFun<R>, key<R(C::*)()const > >(k);
}


template<typename BinaryFun, typename V, typename C>
binary<BinaryFun, key<V C::*> > binary_of(BinaryFun fun, V C::* m) {
    key<V C::* > k(m);
    return binary<BinaryFun, key<V C::* > >(fun, k);
}

template<typename BinaryFun, typename R, typename C>
binary<BinaryFun, key<R (C::*)() > > binary_of(BinaryFun fun, R (C::* m)()) {
    key<R(C::*)() > k(m);
    return binary<BinaryFun, key<R(C::*)() > >(fun, k);
}

template<typename BinaryFun, typename R, typename C>
binary<BinaryFun, key<R (C::*)()const > > binary_of(BinaryFun fun, R (C::* m)()const) {
    key<R(C::*)()const > k(m);
    return binary<BinaryFun, key<R(C::*)()const > >(fun, k);
}


template<typename UnaryFun, typename Key>
struct unary {
    UnaryFun unary_fun;
    Key key;
    unary(Key const& k): key(k) {}
    unary(UnaryFun const& f, Key const& k): unary_fun(f), key(k) {}
    template<typename T>
    auto operator()(T&& a)->decltype(unary_fun(key(std::forward<T>(a)))) {
        return unary_fun(key(std::forward<T>(a)));
    }
};

template<template<typename E> class TemplateUnaryFun, typename V, typename C>
unary<TemplateUnaryFun<V>, key<V C::*> > unary_of(V C::* m) {
    key<V C::* > k(m);
    return unary<TemplateUnaryFun<V>, key<V C::* > >(k);
}

template<template<typename E> class TemplateUnaryFun, typename R, typename C>
unary<TemplateUnaryFun<R>, key<R (C::*)() > > unary_of(R (C::* m)()) {
    key<R(C::*)() > k(m);
    return unary<TemplateUnaryFun<R>, key<R(C::*)() > >(k);
}

template<template<typename E> class TemplateUnaryFun, typename R, typename C>
unary<TemplateUnaryFun<R>, key<R (C::*)()const > > unary_of(R (C::* m)()const) {
    key<R(C::*)()const > k(m);
    return unary<TemplateUnaryFun<R>, key<R(C::*)()const > >(k);
}


template<typename UnaryFun, typename V, typename C>
unary<UnaryFun, key<V C::*> > unary_of(UnaryFun fun, V C::* m) {
    key<V C::* > k(m);
    return unary<UnaryFun, key<V C::* > >(fun, k);
}

template<typename UnaryFun, typename R, typename C>
unary<UnaryFun, key<R (C::*)() > > unary_of(UnaryFun fun, R (C::* m)()) {
    key<R(C::*)() > k(m);
    return unary<UnaryFun, key<R(C::*)() > >(fun, k);
}

template<typename UnaryFun, typename R, typename C>
unary<UnaryFun, key<R (C::*)()const > > unary_of(UnaryFun fun, R (C::* m)()const) {
    key<R(C::*)()const > k(m);
    return unary<UnaryFun, key<R(C::*)()const > >(fun, k);
}


#endif  /*KEY_VISITOR_HPP*/
