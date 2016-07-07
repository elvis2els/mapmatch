#ifndef  LAZY_ASSOCIATIVE_PROPERTY_MAP_HPP
#define  LAZY_ASSOCIATIVE_PROPERTY_MAP_HPP

#include  <boost/property_map/property_map.hpp>
namespace boost {
/// \bref 惰性管理容器属性表
    template<typename Map>
    class lazy_associative_property_map {
    public:
        typedef typename Map::key_type key_type;
        typedef typename Map::mapped_type value_type;

        inline lazy_associative_property_map(Map const &m, value_type const &lazy_value)
                : _map(const_cast<Map &>(m)), _lazy_value(lazy_value) {
        }

        inline lazy_associative_property_map(lazy_associative_property_map const &pm)
                : _map(const_cast<Map &>(pm._map)),
                  _lazy_value(pm._lazy_value) {
        }

        inline value_type const &get(key_type const &k) const {
            if (_map.count(k)) {
                return _map.at(k);
            }
            return _lazy_value;
        }

        inline void put(key_type const &k, value_type const &value) {
            _map[k] = value;
        }

        inline value_type &at(key_type const &k) {
            if (_map.count(k) == 0) {
                _map[k] = _lazy_value;
            }
            return _map[k];
        }

        inline value_type const &at(key_type const &k) const {
            return get(k);
        }

        inline value_type const &operator[](key_type const &k) const {
            return at(k);
        }

        inline value_type &operator[](key_type const &k) {
            return at(k);
        }

    private:
        Map &_map;
        value_type _lazy_value;
    };

    template<typename M>
    struct property_traits<lazy_associative_property_map<M> > {
        typedef typename lazy_associative_property_map<M>::value_type value_type;
        typedef value_type &reference;
        typedef typename lazy_associative_property_map<M>::key_type key_type;
        typedef read_write_property_map_tag category;
    };

    template<typename Map>
    lazy_associative_property_map<Map> make_lazy_property_map(Map const &m, typename Map::mapped_type const &lazy) {
        return lazy_associative_property_map<Map>(m, lazy);
    }


//for some did not support koeing lookup
    template<typename M>
    inline void put(lazy_associative_property_map<M> &m,
            typename lazy_associative_property_map<M>::key_type const &k,
            typename lazy_associative_property_map<M>::value_type const &v) {
        m.put(k, v);
    }

    template<typename M>
    inline typename lazy_associative_property_map<M>::value_type const &
    get(lazy_associative_property_map<M> const &m,
            typename lazy_associative_property_map<M>::key_type const &k) {
        return m.get(k);
    }

    template<typename M>
    inline typename lazy_associative_property_map<M>::value_type const &
    at(lazy_associative_property_map<M> const &m,
            typename lazy_associative_property_map<M>::key_type const &k) {
        return m.at(k);
    }

    template<typename M>
    inline typename lazy_associative_property_map<M>::value_type &
    at(lazy_associative_property_map<M> &m,
            typename lazy_associative_property_map<M>::key_type const &k) {
        return m.at(k);
    }
}
#endif  /*LAZY_ASSOCIATIVE_PROPERTY_MAP_HPP*/
