/**
 * Copyright 2017
 * 
 * This file is part of On-line POMDP Planning Toolkit (OPPT).
 * OPPT is free software: you can redistribute it and/or modify it under the terms of the 
 * GNU General Public License published by the Free Software Foundation, 
 * either version 2 of the License, or (at your option) any later version.
 * 
 * OPPT is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along with OPPT. 
 * If not, see http://www.gnu.org/licenses/.
 */
#ifndef _OPPT_EVENT_HPP_
#define _OPPT_EVENT_HPP_
#include <boost/signals2/signal.hpp>

namespace oppt
{
namespace event
{

template<typename T>
class EventT
{
public:
    EventT() = default;
    
    void connect(const std::function<T> &subscriber) {
        sig_.connect(subscriber);
    }
    
    void operator()() {
        sig_();
    }
    
    template<typename P1>
    void operator()(const P1 &p1) {
        sig_(p1);
    }
    
    template<typename P1, typename P2>
    void operator()(const P1 &p1, const P2 &p2) {
        sig_(p1, p2);
    }
    
    template<typename P1, typename P2, typename P3>
    void operator()(const P1 &p1, const P2 &p2, const P3 &p3) {
        sig_(p1, p2, p3);
    }
    
private:
    boost::signals2::signal<T> sig_;

};

class Events {
    public: 
        static EventT<void (const unsigned int&, const unsigned int&)> stepFinished;
	
	static EventT<void (const unsigned int&)> runFinished;
    
};

}

}

#endif
