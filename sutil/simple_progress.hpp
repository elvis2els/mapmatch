#ifndef  SIMAPLE_PROGRESS_H
#define  SIMAPLE_PROGRESS_H


#include <cstdio>
#include <boost/timer.hpp>
class SimpleProgress
{
public:
    SimpleProgress(size_t total, double up = 0.1)
        :_total(total),_cur(0),_update(up)
    {
    }
    void add()
    {
        advance(1);
    }

    void advance(int n)
    {
        _cur += n;
        if ( _cur < 0 ) _cur = 0;
        if ( _timer.elapsed() > _update )
        {
            printf("\033[Kprogress:%5.2f%%\r", (double)_cur / _total * 100.0);
            fflush(stdout);
            _timer.restart();
        }
    }

    SimpleProgress& operator++()
    {
        add();
        return *this;
    }
    SimpleProgress operator++(int)
    {
        SimpleProgress tmp(*this);
        add();
        return tmp;
    }

    SimpleProgress& operator+=(int n)
    {
        advance(n);
        return *this;
    }

    SimpleProgress& operator-=(int n)
    {
        advance(n);
        return *this;
    }

    void done()
    {
        _cur = _total;
        printf("\033[Kprogress:%5.2f%%\n", (double)_cur / _total * 100.0);
    }

private:
       size_t _total;
       int _cur;
       boost::timer _timer;
       double _update;
};

#endif  /*SIMAPLE_PROGRESS_H*/
