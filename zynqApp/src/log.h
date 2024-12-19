#pragma once

#include <iostream>
#include <iomanip>

//#define TRACE_REG
//#define TRACE_SET_POS
#define TRACE_MOVE
//#define TRACE_STOP
#define TRACE_POLL

template< typename... Ts >
void trace_reg( Ts&&... ts )
{
#ifdef TRACE_REG
    ( std::cout << ... << std::forward<Ts>( ts )) << '\n';
#endif
}

template< typename... Ts >
void trace_set_pos( Ts&&... ts )
{
#ifdef TRACE_SET_POS
    ( std::cout << ... << std::forward<Ts>( ts )) << '\n';
#endif
}

template< typename... Ts >
void trace_move( Ts&&... ts )
{
#ifdef TRACE_MOVE
    ( std::cout << ... << std::forward<Ts>( ts )) << '\n';
#endif
}

template< typename... Ts >
void trace_stop( Ts&&... ts )
{
#ifdef TRACE_STOP
    ( std::cout << ... << std::forward<Ts>( ts )) << '\n';
#endif
}

template< typename... Ts >
void trace_poll( Ts&&... ts )
{
#ifdef TRACE_POLL
    ( std::cout << ... << std::forward<Ts>( ts )) << '\n';
#endif
}


inline auto cout_hex = [](int a) {
    return std::hex << a << std::dec;
};
