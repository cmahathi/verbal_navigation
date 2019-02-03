/**
 * \file  bwi_directions_generator.h
 * \brief  Converts a set of poses (assumed to be a path on a map) into
 *         instructions in English for navigating said path.
 *
 * \author Connor Sheehan
 *

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the <organization> nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 *
 *
 *
 **/

#ifndef BWI_DIRECTIONS_GENERATOR
#define BWI_DIRECTIONS_GENERATOR

#include "ros/ros.h"
#include "verbal_navigation/MapInfo.h"

namespace bwi_directions_generator {
    
    class BwiDirectionsGenerator {
        public:
        
        BwiDirectionsGenerator();

        // We can probably just make a method that builds and returns a MapInfo
        // with the appropriate parameters to get our path. Because the logical_translator
        // only accepts one map floor at a time we will have to find some way to provide
        // directions that cross floors.
        
    };
}

#endif
