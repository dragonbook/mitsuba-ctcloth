#ifndef __DEBUG_H__
#define __DEBUG_H__

#define VAL(x) std::cout << #x << ": " << x << '\n';
#define REF(x) std::cout << #x << ": " << x.toString() << '\n';
#define PTR(x) std::cout << #x << ": " << x->toString() << '\n';

#endif

