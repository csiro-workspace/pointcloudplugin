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

#ifndef __SIMPLEHASH_HPP__
#define __SIMPLEHASH_HPP__

#include <vector>
#include <algorithm>
#include <functional>
#include <climits>
#include <cassert>

template <size_t bits, size_t shift> class BitHash
{
    uint64_t mask;
public:
    BitHash() : mask(~(MAX_VALUE<<bits)) {}
    size_t operator() (uint64_t key) const {return static_cast<size_t>((key>>shift)&mask);}
};

template <typename Key, typename Data_, typename HashFunctor> class SimpleHash
{
    typedef std::pair<Key,Data_> Pair;
    typedef std::vector<Pair> Bucket;
    typedef std::vector<Bucket> HashType;

    struct KeyComparator : public std::binary_function<Key,Pair,bool> {
        bool operator() (Key key, const Pair & pair) const {return (pair.first==key);}
    };

    HashType _hash;
    size_t _count;
    HashFunctor hashf; 

public:
    //exported definitions
    typedef Data_ Data;
    typedef SimpleHash<Key,Data,HashFunctor> MyType;
    typedef std::pair<Key,Data*> Item;
    
    SimpleHash(size_t size, Data const& invalid_value) : INVALID(invalid_value), _hash(size), _count(0) {}

    inline bool access(Key key, const Data * new_data, Data * old_data, bool createIfNotFound = false)
    {   // Allows to determine if a node exists, retrieve its current data, and set new data with a single search
        Bucket & bucket = _hash[hashf(key)];
        typename Bucket::iterator iter = std::find_if(bucket.begin(), bucket.end(), std::bind1st(KeyComparator(), key));
        if (iter==bucket.end())
        {   //not found
            if (!createIfNotFound) {return false;}
            bucket.push_back(Pair(key, INVALID));   //of course not thread safe
            iter = bucket.end()-1;
            ++_count;
        }
        if (old_data)
        {   //save existing data
            *old_data = iter->second;
        }
        if (new_data)
        {   //set new data
            iter->second = *new_data;
        }
        return true;    //found
    }
    inline Item get(Key key, bool * found = NULL, bool createIfNotFound = false)
    {   //retrieve an item
        Bucket & bucket = _hash[hashf(key)];
        typename Bucket::iterator iter = std::find_if(bucket.begin(), bucket.end(), std::bind1st(KeyComparator(), key));
        bool was_found = (iter!=bucket.end());
        if (found)
        {   
            *found = was_found;
        }
        if (!was_found)
        {
            if (!createIfNotFound) 
            {   //return invalid item
                return Item(MAX_VALUE, static_cast<Data*>(NULL));
            }
            //create
            bucket.push_back(Pair(key, INVALID));   //of course not thread safe
            iter = bucket.end()-1;
            ++_count;
        }
        return Item(key, &iter->second);
    }
    //inline void insert(Key key, Data const& data) {access(key, &data, NULL, true);}
    inline Item insert(Key key, Data const& data)
    {   //insert (or set) item
        Bucket & bucket = _hash[hashf(key)];
        typename Bucket::iterator iter = std::find_if(bucket.begin(), bucket.end(), std::bind1st(KeyComparator(), key));
        if (iter==bucket.end())
        {   //not found
            bucket.push_back(Pair(key, data));   //of course not thread safe
            iter = bucket.end()-1;
            ++_count;
        }
        else
        {   //set new data
            iter->second = data;
        }
        return Item(key, &iter->second);
    }
    inline void remove(Key key)
    {
        Bucket & bucket = _hash[hashf(key)];
        typename Bucket::iterator iter = std::find_if(bucket.begin(), bucket.end(), std::bind1st(KeyComparator(), key));
        if (iter!=bucket.end()) {bucket.erase(iter); --_count;}
    }
    inline bool exists(Key key) const {return const_cast<MyType*>(this)->access(key, NULL, NULL);} //use get
    inline void clear(void) {std::fill(_hash.begin(), _hash.end(), Bucket()); _count=0;}
    inline size_t size(void) const {return _count;}

    //debug
    void get_stats(double & mean, size_t max[2], size_t count[2]) const
    {
        max[0] = 0; max[1] = 0;
        count[0] = 0; count[1] = 0;
        mean = 0.0;
        for (typename HashType::const_iterator iter=_hash.begin(); iter!=_hash.end(); ++iter)
        {   //search the max
            if (iter->size()>max[0])
            {
                max[1] = max[0];
                max[0] = iter->size();
            }
            else if (iter->size()>max[1] && iter->size()<max[0])
            {
                max[1] = iter->size();
            }
            mean += iter->size();
        }
        for (typename HashType::const_iterator iter=_hash.begin(); iter!=_hash.end(); ++iter)
        {   //count
            if (iter->size()==max[0])
            {
                ++count[0];
            }
            if (iter->size()==max[1])
            {
                ++count[1];
            }
        }
        mean /= _hash.size();
    }

    //Simple iterator
    class iterator
    {
        friend MyType;
        Bucket * _bucket;
        Bucket * _bucket_end;
        Pair * _pair;
        Pair * _pair_end;
    public:
        iterator(MyType * instance, bool end = false) :
            _bucket((instance->_hash.begin()!=instance->_hash.end()?&*(instance->_hash.begin()):NULL)), _bucket_end(_bucket+instance->_hash.size()), 
            _pair((_bucket->begin()!=_bucket->end()?&*(_bucket->begin()):NULL)), _pair_end(_pair+_bucket->size())
        {
            if (end)
            {
                _bucket = _bucket_end;
                _pair_end = ((_bucket_end-1)->begin()!=(_bucket_end-1)->end()?&*((_bucket_end-1)->begin()):NULL) + (_bucket_end-1)->size();
                _pair = _pair_end;
            }
            else if (_bucket->size()==0)
            {   //first bucket empty, advance the iterator
                ++(*this);
            }
        }
        iterator & operator++()
        {
            while (_bucket!=_bucket_end)
            {
                if (_pair==_pair_end)
                {   //next bucket
                    ++_bucket;
                    if (_bucket==_bucket_end)
                    {   //finish
                        break;
                    }
                    else
                    {   //begin the new bucket
                        _pair = (_bucket->begin()!=_bucket->end()?&*(_bucket->begin()):NULL);
                        _pair_end = _pair+_bucket->size();
                    }
                }
                else
                {   //increment current bucket
                    ++_pair;
                }
                if (_pair!=_pair_end)
                {   //ok
                    break;
                }
            }
            return (*this);
        }
        inline Item operator*() const {return Item(_pair->first, &_pair->second);}
        //inline Item * operator->() const {return &Item(_pair->first, &_pair->second);}
        inline bool operator==(const iterator & rhs) const {return (_bucket==rhs._bucket) && (_pair==rhs._pair);}
        inline bool operator!=(const iterator & rhs) const {return (_bucket!=rhs._bucket) || (_pair!=rhs._pair);}
    };

    iterator begin(void) {return iterator(this);}
    iterator end(void) {return iterator(this, true);}
    
    const Data INVALID;
};

#endif // __SIMPLEHASH_HPP__
