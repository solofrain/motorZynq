#ifndef _SPINLOCK_H_
#define _SPINLOCK_H_

#include <atomic>

class spinlock
{
private:
    std::atomic<bool> locked_ {false};

public:
    void lock();
    void unlock();
};

#endif
