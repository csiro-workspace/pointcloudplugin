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

//This code is mainly adapted from Thomas Lewiner's code. 

/**
 * \file    morton.hpp
 * \author  Thomas Lewiner   <tomlew@puc-rio.br>
 * \author  Rener Pereira de Castro <rener@mat.puc-rio.br>
 * \author  Matmidia Lab, Math Dept, PUC-Rio
 * \date    10/01/2010
 *
 * \brief  Morton codes manipulation
 *
 * Morton codes manipulation
*/
//_____________________________________________________________________________

#ifndef __MORTON_HPP__
#define __MORTON_HPP__

#include "ssd/dgp/inttypes.h"

namespace morton
{
    uint64_t make_key(size_t i, size_t j, size_t k, size_t L);
    void split_key(uint64_t key, size_t &i, size_t &j, size_t &k, size_t &L);
    size_t key_level(uint64_t key);

    inline void children(uint64_t key, uint64_t children[8]) 
    {
        key <<= 3;
        children[0] = key|0; children[1] = key|1; children[2] = key|2; children[3] = key|3;
        children[4] = key|4; children[5] = key|5; children[6] = key|6; children[7] = key|7;
    }

    uint64_t addition_locate_code(uint64_t const& N, uint64_t const& DN);
    void vertices_keys(uint64_t cell_key, uint64_t vertices[8], size_t MAX_LEVEL);
    
    inline uint64_t middle_vertex(uint64_t v0, uint64_t v1)
    {
        return (addition_locate_code(v0, v1)>>3);
    }

    void neighbor_keys(uint64_t key, const size_t * direction, uint64_t * neighbors);
};

#endif // __MORTON_HPP__