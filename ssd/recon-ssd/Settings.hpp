/*
Copyright (c) 2013, Daniel Moreno
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
      
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
      
    * Neither the name of the Brown University nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission. 

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR 
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef __SETTINGS_HPP__
#define __SETTINGS_HPP__

#include <string>
#include "octree/SSDData.hpp"

class Settings
{
public:
    bool        debug;
    bool        manifold;
    bool        computeColor;
    Real        bboxExpansionFactor;
    size_t      octreeLevels;
    size_t      octreeLevelStart;
    size_t      samplesPerNode;
    Real        isoLevel;
    Real        weight0;
    Real        weight1;
    Real        weight2;
    Real        weight0_color;
    Real        weight1_color;
    Real        solverTolerance;
    Real        solverTolerance_color;
    size_t      minIterations;
    size_t      maxIterations;
    bool        fast;
    bool        polygons;
    bool        writeBinary;
    std::string inFile;
    std::string outFile;

    Settings() :
        debug(false),
        manifold(false),
        computeColor(false),
        bboxExpansionFactor(Real(1.10)),
        octreeLevels(8),
        octreeLevelStart(1),
        samplesPerNode(1),
        isoLevel(Real(0.0)),
        weight0(Real(1.0)),
        weight1(Real(1.0)),
        weight2(Real(1.0)),
        weight0_color(Real(1.0)),
        weight1_color(Real(0.1)),
        solverTolerance(Real(1.0e-8)),
        solverTolerance_color(Real(1.0e-4)),
        minIterations(1),
        maxIterations(300),
        fast(false),
        polygons(false),
        writeBinary(true),
        inFile(),
        outFile()
    {
    }
};

#endif //__SETTINGS_HPP__