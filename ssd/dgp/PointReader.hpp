/*
Copyright (c) 2013, Fatih Calakli and Daniel Moreno
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

#pragma once //SRM added based on stackoverflow comment no idea what it does
#ifndef __POINTREADER_HPP__
#define __POINTREADER_HPP__

#include <string>
#include <functional>
#include <unordered_map>

#include "Mesh/DataStructures/MeshModelInterface/meshmodelinterface.h" //For mesh

namespace PointReader
{
    template <typename PointContainer> bool load(const char * filename, PointContainer & points);
    void check_properties(const char * filename, std::unordered_map<std::string, bool> & properties);

    template <typename PointContainer> bool load_ply(const char * filename, PointContainer & points);
    template <typename PointContainer> bool load_npt(const char * filename, PointContainer & points);
    template <typename PointContainer> bool load_bpa(const char * filename, PointContainer & points);
    //New template for MeshModelInterface
    template <typename PointContainer> bool load_fromMMI( CSIRO::Mesh::MeshModelInterface& mesh, PointContainer & points);

    void check_properties_ply(const char * filename, std::unordered_map<std::string, bool> & properties);
    void check_properties_npt(const char * filename, std::unordered_map<std::string, bool> & properties);
    void check_properties_bpa(const char * filename, std::unordered_map<std::string, bool> & properties);

};

#include "PointReader.inl"

#endif // __POINTREADER_HPP__