/*
Copyright (c) 2011, Gabriel Taubin
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

#ifndef _BBox_h_
#define _BBox_h_

#include <cstdlib>
#include <vector>

class BBox
{
  std::vector<double> _min;
  std::vector<double> _max;

public:
  BBox(int d);
  ~BBox();

  inline int getDimension(void) const {return static_cast<int>(_min.size());}
  
  inline void setMax(size_t i, double value) {_max[i] = value;}
  inline double getMax(int i) const {return _max[i];}
  
  inline void setMin(size_t i, double value)  {_min[i] = value;}
  inline double getMin(size_t i) const {return _min[i];}

  inline double getCenter(size_t i) const {return 0.5*(_min[i]+_max[i]);}
  
  void setSide(size_t i, double value);
  inline double getSide(size_t i) const {return (_max[i]-_min[i]);}

  void expand(double factor);
};

#endif // _BBox_h_
