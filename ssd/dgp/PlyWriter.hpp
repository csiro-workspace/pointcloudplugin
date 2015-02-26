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
#ifndef __PLYWRITER_HPP__
#define __PLYWRITER_HPP__

#include <vector>
#include "Mesh/DataStructures/MeshModelInterface/meshmodelinterface.h" //For mesh

namespace PlyWriter 
{
    enum PlyFlags {PlyPoints = 0x00, PlyColors = 0x01, PlyNormals = 0x02, PlyBinary = 0x04, PlyPlane = 0x08, PlyFaces = 0x10, PlyTexture = 0x20};

    template <typename PointType>
    bool write(const std::string & filename, std::vector<PointType> const& vertices, std::vector<std::vector<size_t> > const& faces, unsigned flags = PlyPoints);
    //Write to mminterface
    template <typename PointType>
    bool write_toMMI(CSIRO::Mesh::MeshModelInterface& mesh, std::vector<PointType> const& vertices, std::vector<std::vector<size_t> > const& faces, bool color);

};

#include "PlyWriter.inl"

#endif //__PLYWRITER_HPP__