#ifndef  COPY_ON_WRITE_HPP
#define  COPY_ON_WRITE_HPP

#include  <atomic>
#include  <utility>
struct entity_init_tag {};
entity_init_tag construct_entity;

template<typename Implementation>
class COWPImpl final {
public:
    typedef Implementation value_type;
private:
    struct Impl {
        Impl(): ref(1) {}

        Impl(Impl const& other): ref(1), entity(other.entity) {}

        template<typename... Args>
        explicit Impl(entity_init_tag, Args&&... args): ref(1), entity(std::forward<Args>(args)...) {}


        std::atomic_int ref;
        Implementation entity;
    }* pimpl;
    void detach() {
        Impl * tmp = new Impl(*pimpl);
        -- pimpl->ref;
        if (pimpl->ref.load() == 0) {
            delete pimpl;
        }
        pimpl = tmp;
    }

public:
    COWPImpl(): pimpl(new Impl) {}

    template<typename... Args>
    explicit COWPImpl(entity_init_tag, Args&&... args): pimpl(new Impl(construct_entity, std::forward<Args>(args)...) ) {}

    explicit COWPImpl(std::nullptr_t): pimpl(nullptr) {}

    COWPImpl(COWPImpl const& other) {
        if (other.pimpl ) {
            other.pimpl->ref.fetch_add(1);
            pimpl = other.pimpl;
        }
    }

    COWPImpl(COWPImpl&& other) {
        pimpl = other.pimpl;
        other.pimpl = nullptr;
    }

    COWPImpl& operator=(COWPImpl const& other) {
        if ( & other != this) {
            if ( other.pimpl )  {
                other.pimpl->ref.fetch_add(1);
                this->~COWPImpl();
                pimpl = other.pimpl;
            } else {
                this->~COWPImpl();
                pimpl = nullptr;
            }
        }
        return *this;
    }

    COWPImpl& operator=(COWPImpl&& other) {
        if ( this != & other ) {
            this->~COWPImpl();
            pimpl = other.pimpl;
            other.pimpl = nullptr;
        }
        return *this;
    }

    COWPImpl& operator=(std::nullptr_t) {
        this->~COWPImpl();
        pimpl = nullptr;
        return *this;
    }

    bool null()const {
        return pimpl == nullptr;
    }

    operator void* ()const {
        return pimpl;
    }

    bool operator ! () const {
        return ! null();
    }

    Implementation const* readonly()const {
        if ( not pimpl) return nullptr;
        return  & pimpl->entity;
    }

    Implementation* modifyable() {
        if (pimpl and pimpl->ref.load() > 1) {
            detach();
        }
        return & pimpl->entity;
    }

    Implementation* operator->() {
        return modifyable();
    }

    Implementation const* operator->()const {
        return readonly();
    }
    int reference_count()const {
        return pimpl->ref.load();
    }

    ~COWPImpl() {
        if ( pimpl ) {
            if (  -- pimpl->ref == 0 ) {
                delete pimpl;
            }
        }
    }
};

#endif  /*COPY_ON_WRITE_HPP*/
