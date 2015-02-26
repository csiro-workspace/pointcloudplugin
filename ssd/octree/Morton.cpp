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
 * \file    morton.cpp
 * \author  Thomas Lewiner   <tomlew@puc-rio.br>
 * \author  Rener Pereira de Castro <rener@mat.puc-rio.br>
 * \author  Matmidia Lab, Math Dept, PUC-Rio
 * \date    10/01/2010
 *
 * \brief  Morton codes manipulation
 *
 * Morton codes manipulation
*/

#include "Morton.hpp"

#include <cmath>
#include <climits>

namespace morton
{
    //_____________________________________________________________________________
    // Contraction and Dilatation Masks
    static const uint64_t three_2 = 9 ;
    static const uint64_t three_1 = 3 ;
    static const uint64_t three_0 = 1 ;
    static const uint64_t shift_2a = 18, shift_2b = 36 ;
    static const uint64_t shift_1a =  6, shift_1b = 12 ;
    static const uint64_t shift_0a =  2, shift_0b =  4 ;
    static const uint64_t dilate_mask_2 = 0x7FC0000FF80001FFLL ;  // bits 1 from 0-9, 27-36 and 54-63
    static const uint64_t dilate_mask_1 = 0x01C0E070381C0E07LL ;  // bits 1 from 0-3, 9-12, 18-21,...
    static const uint64_t dilate_mask_0 = 0x9249249249249249LL ;  // bits 1 at 0,3,6,9,12,...
    static const uint64_t dilate_tz     = 0x4924924924924924LL ;  // bits 1 at 2,5,8,11,14,...    rigth to left
    static const uint64_t dilate_ty     = 0x2492492492492492LL ;  // bits 1 at 1,4,7,10,13,...
    static const uint64_t dilate_tx     = 0x9249249249249249LL ;  // bits 1 at 0,3,6,9,12,...  
    static const uint64_t dilate_t1     = 0xB6DB6DB6DB6DB6DBLL ;  // bits 0 at 2,5,8,11,14,...    = ~tz
    static const uint64_t dilate_t2     = 0xDB6DB6DB6DB6DB6DLL ;  // bits 0 at 1,4,7,10,13,...    = ~ty
    static const uint64_t dilate_t3     = 0x6DB6DB6DB6DB6DB6LL ;  // bits 0 at 0,3,6,9,12,...     = ~tx

    //_____________________________________________________________________________
    // Step Directions Find Neighbours                                                  (dz,dy,dx)
    static const uint64_t dir_neigh[27] = { 0xFFFFFFFFFFFFFFFFLL , // Direction (-1,-1,-1)   0
                                            0x6DB6DB6DB6DB6DB6LL , // Direction (-1,-1, 0)   1
                                            0x6DB6DB6DB6DB6DB7LL , // Direction (-1,-1, 1)   2
                                            0xDB6DB6DB6DB6DB6DLL , // Direction (-1, 0,-1)   3
                                            0x4924924924924924LL , // Direction (-1, 0, 0)   4 [4]
                                            0x4924924924924925LL , // Direction (-1, 0, 1)   5
                                            0xDB6DB6DB6DB6DB6FLL , // Direction (-1, 1,-1)   6
                                            0x4924924924924926LL , // Direction (-1, 1, 0)   7
                                            0x4924924924924927LL , // Direction (-1, 1, 1)   8
                                            0xB6DB6DB6DB6DB6DBLL , // Direction ( 0,-1,-1)   9
                                            0x2492492492492492LL , // Direction ( 0,-1, 0)  10 [2]
                                            0x2492492492492493LL , // Direction ( 0,-1, 1)  11
                                            0x9249249249249249LL , // Direction ( 0, 0,-1)  12 [0]
                                            0x0000000000000000LL , // Direction ( 0, 0, 0)  13
                                            0x0000000000000001LL , // Direction ( 0, 0, 1)  14 [1]
                                            0x924924924924924BLL , // Direction ( 0, 1,-1)  15
                                            0x0000000000000002LL , // Direction ( 0, 1, 0)  16 [3]
                                            0x0000000000000003LL , // Direction ( 0, 1, 1)  17
                                            0xB6DB6DB6DB6DB6DFLL , // Direction ( 1,-1,-1)  18
                                            0x2492492492492496LL , // Direction ( 1,-1, 0)  19
                                            0x2492492492492497LL , // Direction ( 1,-1, 1)  20
                                            0x924924924924924DLL , // Direction ( 1, 0,-1)  21
                                            0x0000000000000004LL , // Direction ( 1, 0, 0)  22 [5]
                                            0x0000000000000005LL , // Direction ( 1, 0, 1)  23
                                            0x924924924924924FLL , // Direction ( 1, 1,-1)  24
                                            0x0000000000000006LL , // Direction ( 1, 1, 0)  25
                                            0x0000000000000007LL , // Direction ( 1, 1, 1)  26
                                        };
};

uint64_t morton::make_key(size_t i, size_t j, size_t k, size_t L)
{
    uint64_t bz = k;
    uint64_t by = j;
    uint64_t bx = i;
    
    if (L>three_2)
    {
        bz = ( bz | (bz << shift_2a) | (bz << shift_2b) ) & dilate_mask_2 ;
        by = ( by | (by << shift_2a) | (by << shift_2b) ) & dilate_mask_2 ;
        bx = ( bx | (bx << shift_2a) | (bx << shift_2b) ) & dilate_mask_2 ;
    }
    if (L>three_1)
    {
        bz = ( bz | (bz << shift_1a) | (bz << shift_1b) ) & dilate_mask_1 ;
        by = ( by | (by << shift_1a) | (by << shift_1b) ) & dilate_mask_1 ;
        bx = ( bx | (bx << shift_1a) | (bx << shift_1b) ) & dilate_mask_1 ;
    }
    if (L>three_0)
    {
        bz = ( bz | (bz << shift_0a) | (bz << shift_0b) ) & dilate_mask_0 ;
        by = ( by | (by << shift_0a) | (by << shift_0b) ) & dilate_mask_0 ;
        bx = ( bx | (bx << shift_0a) | (bx << shift_0b) ) & dilate_mask_0 ;
    }
    return (static_cast<uint64_t>(1)<<(3*L)) | (bz<<2) | (by<<1) | bx;
}

void morton::split_key(uint64_t key, size_t &i, size_t &j, size_t &k, size_t &L)
{
    size_t level = key_level(key) ;
    uint64_t bz( (key >> 2) & dilate_mask_0 );
    uint64_t by( (key >> 1) & dilate_mask_0 );
    uint64_t bx( (   key  ) & dilate_mask_0 );

    if (level>three_0)
    {
        bz = ( bz | (bz >> shift_0a) | (bz >> shift_0b) ) & dilate_mask_1 ;
        by = ( by | (by >> shift_0a) | (by >> shift_0b) ) & dilate_mask_1 ;
        bx = ( bx | (bx >> shift_0a) | (bx >> shift_0b) ) & dilate_mask_1 ;

        if (level>three_1)
        {
            bz = ( bz | (bz >> shift_1a) | (bz >> shift_1b) ) & dilate_mask_2 ;
            by = ( by | (by >> shift_1a) | (by >> shift_1b) ) & dilate_mask_2 ;
            bx = ( bx | (bx >> shift_1a) | (bx >> shift_1b) ) & dilate_mask_2 ;

            if (level>three_2)
            {
                bz = bz | (bz >> shift_2a) | (bz >> shift_2b) ;
                by = by | (by >> shift_2a) | (by >> shift_2b) ;
                bx = bx | (bx >> shift_2a) | (bx >> shift_2b) ;
            }
        }
    }

    size_t length_mask = (1<<level)-1;
    bz &= length_mask ;
    by &= length_mask ;
    bx &= length_mask ;

    //return Taubin integer indices
    k = static_cast<size_t>(bz);
    j = static_cast<size_t>(by);
    i = static_cast<size_t>(bx);
    L = static_cast<size_t>(level);
}

size_t morton::key_level(uint64_t key)
{   // octree depth level associated to a morton key ( = (bit length - 1) / 3 )
    static const float lg2_3 = 1.f / (std::log(2.f)*3.f) ;
    return static_cast<size_t>(std::floor(std::log(static_cast<float>(key)) * lg2_3));
}

uint64_t morton::addition_locate_code(uint64_t const& N, uint64_t const& DN)
{   // addition in dilated integer
    return (
          ( ((N | dilate_t1) + (DN & dilate_tz)) & dilate_tz ) |
          ( ((N | dilate_t2) + (DN & dilate_ty)) & dilate_ty ) |
          ( ((N | dilate_t3) + (DN & dilate_tx)) & dilate_tx )
          );
}

void morton::vertices_keys(uint64_t cell_key, uint64_t vertices[8], size_t MAX_LEVEL)
{   // generate a list of 8 vertices keys
    const size_t cell_level = key_level(cell_key);
    const uint64_t cell_root = uint64_t(1)<<(3*cell_level);
    const uint64_t vertex_root = uint64_t(1)<<(3*(MAX_LEVEL+1));
    const uint64_t kwr = cell_key^cell_root; //key without root
    const size_t shift = 3*(MAX_LEVEL-cell_level);

    //vertices
    vertices[0] = (addition_locate_code(kwr, 0) << shift)|vertex_root;
    vertices[1] = (addition_locate_code(kwr, 1) << shift)|vertex_root;
    vertices[2] = (addition_locate_code(kwr, 2) << shift)|vertex_root;
    vertices[3] = (addition_locate_code(kwr, 3) << shift)|vertex_root;
    vertices[4] = (addition_locate_code(kwr, 4) << shift)|vertex_root;
    vertices[5] = (addition_locate_code(kwr, 5) << shift)|vertex_root;
    vertices[6] = (addition_locate_code(kwr, 6) << shift)|vertex_root;
    vertices[7] = (addition_locate_code(kwr, 7) << shift)|vertex_root;
}

void morton::neighbor_keys(uint64_t key, const size_t * direction, uint64_t * neighbors)
{
    size_t nbits_neigh = 3*key_level(key);
    for (size_t i=0; direction[i]<SIZE_MAX; ++i)
    {
        neighbors[i] = addition_locate_code(key, dir_neigh[direction[i]]);
        if ( (neighbors[i]>>nbits_neigh) != 1 )
        {
            neighbors[i] = MAX_VALUE;
        }
    }
}
