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

#include <string>
#include <iostream>
#include <istream>
#include <sstream>
#include <fstream>

//SRM add mesh nodes interface
#include "Mesh/DataStructures/MeshModelInterface/meshnodesinterface.h"

#include "ply.h"

#include "PointXYZ.hpp"
#include "PointXYZNormal.hpp"
#include "PointXYZRGBNormal.hpp"

namespace PointReader
{
    //vertex type
    typedef struct PlyVertex {
        float x,y,z;
        float nx,ny,nz;
        unsigned char r,g,b;
        void *other_props;       
    } PlyVertex;

    //: list of property information for a vertex
    static PlyProperty vert_props[] = { 
      {"x",  Float32, Float32, offsetof(PlyVertex,x),  0, 0, 0, 0},
      {"y",  Float32, Float32, offsetof(PlyVertex,y),  0, 0, 0, 0},
      {"z",  Float32, Float32, offsetof(PlyVertex,z),  0, 0, 0, 0},
      {"nx", Float32, Float32, offsetof(PlyVertex,nx), 0, 0, 0, 0},
      {"ny", Float32, Float32, offsetof(PlyVertex,ny), 0, 0, 0, 0},
      {"nz", Float32, Float32, offsetof(PlyVertex,nz), 0, 0, 0, 0},
      {"diffuse_red",  Uint8,   Uint8,   offsetof(PlyVertex,r),  0, 0, 0, 0},
      {"diffuse_green",  Uint8,   Uint8,   offsetof(PlyVertex,g),  0, 0, 0, 0},
      {"diffuse_blue",  Uint8,   Uint8,   offsetof(PlyVertex,b),  0, 0, 0, 0},
      {"red",  Uint8,   Uint8,   offsetof(PlyVertex,r),  0, 0, 0, 0},
      {"green",  Uint8,   Uint8,   offsetof(PlyVertex,g),  0, 0, 0, 0},
      {"blue",  Uint8,   Uint8,   offsetof(PlyVertex,b),  0, 0, 0, 0},
    };

    PointXYZ & set_point(PointXYZ & p, PlyVertex const& v)
    {
        p.x = v.x; p.y = v.y; p.z = v.z;
        return p;
    }

    PointXYZNormal & set_point(PointXYZNormal & p, PlyVertex const& v)
    {
        p.x  = v.x ; p.y  = v.y ; p.z  = v.z;
        p.nx = v.nx; p.ny = v.ny; p.nz = v.nz;
        return p;
    }

    PointXYZRGBNormal & set_point(PointXYZRGBNormal & p, PlyVertex const& v)
    {
        p.x  = v.x ; p.y  = v.y ; p.z  = v.z ;
        p.nx = v.nx; p.ny = v.ny; p.nz = v.nz;
        p.r = v.r/255.f; p.g = v.g/255.f; p.b = v.b/255.f; p.a = 1.f;
        return p;
    }
}

template <typename PointContainer> bool PointReader::load(const char * filename, PointContainer & points)
{
    std::string file(filename);
    size_t pos = file.find_last_of(".");
    std::string ext = file.substr(pos+1);
    if (pos+1<file.npos)
    {
        if (ext=="ply") 
        {
            return load_ply(filename, points);
        }
        if (ext=="npts" || ext=="npt") 
        {
            return load_npt(filename, points);
        }
        if (ext=="bpa") 
        {
            return load_bpa(filename, points);
        }
    }
    return false;
}

void PointReader::check_properties(const char * filename, std::unordered_map<std::string, bool> & properties)
{
    std::string file(filename);
    size_t pos = file.find_last_of(".");
    std::string ext = file.substr(pos+1);
    if (pos+1<file.npos)
    {
        if (ext=="ply") 
        {
            return check_properties_ply(filename, properties);
        }
        if (ext=="npts" || ext=="npt") 
        {
            return check_properties_npt(filename, properties);
        }
        if (ext=="bpa") 
        {
            return check_properties_bpa(filename, properties);
        }
    }
}

template <typename PointContainer> bool PointReader::load_ply(const char * filename, PointContainer & points)
{
    PointContainer point_list;

    //open
    FILE * fp = fopen(filename, "rb");
    if (!fp)
    {   //open error
        return false;
    }

    PlyFile * file = read_ply(fp);
    if (!file || file->num_elem_types == 0)
    {   //header parse error, or no elements
        fclose(fp);
        return false;
    }

    // go through the elements and read vertices
    for (int i=0; i<file->num_elem_types; ++i) 
    {
        // prepare to read the i'th list of elems
        int elem_count;
        char * elem_name = setup_element_read_ply(file, i, &elem_count);
        if (equal_strings(elem_name, "vertex")) 
        {   // vertex started

            bool has_colors = false;
            bool has_normals = false;
            for (int j=0; j<file->elems[i]->nprops; ++j) 
            {
                if (equal_strings(file->elems[i]->props[j]->name, "diffuse_red")) { has_colors = true; }
                if (equal_strings(file->elems[i]->props[j]->name, "red")) { has_colors = true; }
                if (equal_strings(file->elems[i]->props[j]->name, "nx")) { has_normals = true; }	  
            }

            ////////////////////////////////////////////// read vertex started
            PlyVertex vertex;
            memset(&vertex, 0, sizeof(PlyVertex));
            int n_vertex_props = has_colors ? 12 : 6;
            for (int j=0; j<n_vertex_props; ++j)
            {
                setup_property_ply(file, &vert_props[j]);
            }

            PlyOtherProp * vert_other = get_other_properties_ply(file, offsetof(PlyVertex, other_props));
            point_list.resize(elem_count);
            for (int j=0; j<elem_count; ++j) 
            {
                //PointContainer::value_type & p = point_list[j];
                get_element_ply(file, static_cast<void*>(&vertex));
                set_point(point_list[j], vertex);
            }

            if (vert_other)
            {
                free(vert_other);
            }
        }
        else  
        {   /* all non-vertex and non-face elements are grabbed here */
            get_other_element_ply(file);
        }

        if (elem_name)
        {
            free(elem_name);
        }
    }   //for elements

    //close and free file
    fclose(fp);
    free_ply(file);

    //this is the optimal way of creating a new vector and freeing old data
    points.swap(point_list);

    return true;
}

void PointReader::check_properties_ply(const char * filename, std::unordered_map<std::string, bool> & properties)
{
    //open
    FILE * fp = fopen(filename, "rb");
    if (!fp)
    {   //open error
        return;
    }

    PlyFile * file = read_ply(fp);
    if (!file || file->num_elem_types == 0)
    {   //header parse error, or no elements
        fclose(fp);
        return;
    }

    // go through the elements and read vertices
    for (int i=0; i<file->num_elem_types; ++i) 
    {
        // prepare to read the i'th list of elems
        int elem_count;
        char * elem_name = setup_element_read_ply(file, i, &elem_count);
        if (equal_strings(elem_name, "vertex")) 
        {   // vertex started

            for (int j=0; j<file->elems[i]->nprops; ++j) 
            {
                for (std::unordered_map<std::string, bool>::iterator iter=properties.begin(); iter!=properties.end(); ++iter)
                {
                    if (equal_strings(file->elems[i]->props[j]->name, iter->first.c_str()))
                    {
                        iter->second = true;
                    }
                }  
            }
        }
        else  
        {   /* all non-vertex and non-face elements are grabbed here */
            get_other_element_ply(file);
        }

        if (elem_name)
        {
            free(elem_name);
        }
    }   //for elements

    //close and free file
    fclose(fp);
    free_ply(file);
}

template <typename PointContainer> bool PointReader::load_npt(const char * filename, PointContainer & points)
{
    PointContainer point_list;

    //open
    std::filebuf fb;
    if (!fb.open(filename, std::ios::in|std::ios::binary))
    {
        return false;
    }

    std::istream is(&fb);
    while (is)
    {
        std::string line;
        std::getline(is, line);
        std::istringstream lines(line);

        PlyVertex vertex;
        lines >> vertex.x;
        lines >> vertex.y;
        lines >> vertex.z;
        lines >> vertex.nx;
        lines >> vertex.ny;
        lines >> vertex.nz;

        typename PointContainer::value_type p;
        set_point(p, vertex);
        point_list.push_back(p);
    }

    //close
    fb.close();

    //this is the optimal way of creating a new vector and freeing old data
    points.swap(point_list);

    return true;
}

void PointReader::check_properties_npt(const char * filename, std::unordered_map<std::string, bool> & properties)
{
    const char * prop[] = {"x", "y", "z", "nx", "ny", "nz", NULL};

    const char ** p = prop;
    while (*p)
    {
        std::string s(*p);
        std::unordered_map<std::string, bool>::iterator iter = properties.find(s);
        if (iter!=properties.end())
        {
            iter->second = true;
        }
        ++p;
    }
}

template <typename PointContainer> bool PointReader::load_bpa(const char * filename, PointContainer & points)
{
    PointContainer point_list;

    //open
    FILE * fp = fopen(filename, "rb");
    if (!fp)
    {   //open error
        return false;
    }

    double pn[6];
    while (fread(&pn, sizeof(double), 6, fp)==6)
    {
        PlyVertex vertex;
        vertex.x  = static_cast<float>(pn[0]);
        vertex.y  = static_cast<float>(pn[1]);
        vertex.z  = static_cast<float>(pn[2]);
        vertex.nx = static_cast<float>(pn[3]);
        vertex.ny = static_cast<float>(pn[4]);
        vertex.nz = static_cast<float>(pn[5]);

        typename PointContainer::value_type p;
        set_point(p, vertex);
        point_list.push_back(p);
    }

   //close
    fclose(fp);

    //this is the optimal way of creating a new vector and freeing old data
    points.swap(point_list);

    return true;
}

void PointReader::check_properties_bpa(const char * filename, std::unordered_map<std::string, bool> & properties)
{
    check_properties_npt(filename, properties);
}
//SRM New template for MMI
template <typename PointContainer> bool PointReader::load_fromMMI(CSIRO::Mesh::MeshModelInterface& mesh, PointContainer & points)
{
    PointContainer point_list;

    //Get Nodes and rgba state + normals
    CSIRO::Mesh::MeshNodesInterface& inputNodes = mesh.getNodes();
    if (!inputNodes.hasState("normal"))
    {
        std::cout << QString("ERROR: Nodes do not have state named normal, exiting") + "\n";
        return false;
    }
    const CSIRO::Mesh::NodeStateHandle& normalState = inputNodes.getStateHandle("normal");
    if (!inputNodes.hasState("RGBA"))
    {
        std::cout << QString("WARNING: Nodes do not have a RGBA state, can not do color SSD") + "\n";
    }
    const CSIRO::Mesh::NodeStateHandle& rgbaState = inputNodes.getStateHandle("RGBA");
    //Iterate through nodes
    CSIRO::Mesh::MeshNodesInterface::const_iterator nIter = inputNodes.begin();
    
    for(; nIter != inputNodes.end(); ++nIter)
    {
        //Get node
        CSIRO::Mesh::Vector3d n,pos;
        pos = inputNodes.getPosition(*nIter);
        inputNodes.getState(*nIter,normalState, n);
        int cols;
        PlyVertex vertex;
        vertex.x = pos.x;
        vertex.y = pos.y;
        vertex.z = pos.z;
        vertex.nx = n.x;
        vertex.ny = n.y;
        vertex.nz = n.z;
        if (inputNodes.hasState("RGBA"))
        {
        inputNodes.getState(*nIter,rgbaState, cols);
        int tway = cols >> 24;
        vertex.b = cols >> 16;
        vertex.g = cols >> 8;
        vertex.r = cols;
        /*vertex.b = cols >> 24;
        vertex.g = cols >> 16;
        vertex.r = cols >> 8;*/

        }
        typename PointContainer::value_type p;
        set_point(p,vertex);
        point_list.push_back(p);
    }
    
    
    //this is the optimal way of creating a new vector and freeing old data
    points.swap(point_list);
    std::cout << QString("Input point list length: %1").arg(points.size()) + "\n";
    return true;
}

