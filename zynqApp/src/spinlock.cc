/************************************************************
 *
 * Project: 3FI detector
 *
 * Module: spin lock.
 *
 * Revision:
 *   . v0.1  6/7/2024
 *
 * Author: Ji Li <liji@bnl.gov>
 *
 ************************************************************/
#include <iostream>
#include <atomic>

#include "spinlock.h"

void spinlock::lock()
{
    while( locked_.exchange( true, std::memory_order_acquire) );
}

void spinlock::unlock()
{
    locked_.store( false, std::memory_order_release );
}
