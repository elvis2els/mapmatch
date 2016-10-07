#ifndef  SIMPLEGUARD_HPP
#define  SIMPLEGUARD_HPP

#include <functional>
class SimpleGuard
{
public:
	template<typename Fun>
	SimpleGuard(Fun fun): _fun(fun) {}
	~SimpleGuard() { _fun();}
private:
	std::function<void(void)> _fun;
};
#endif  /*SIMPLEGUARD_HPP*/
