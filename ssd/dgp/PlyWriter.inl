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


#include <fstream>

//SRM add mesh nodes & elements interface
#include "Mesh/DataStructures/MeshModelInterface/meshnodesinterface.h"
#include "Mesh/DataStructures/MeshModelInterface/meshelementsinterface.h"

#include "dgp/PointXYZRGB.hpp"

namespace PlyWriter
{
    template <typename PointType> void write_point(PointType const& p, std::ofstream & outfile, bool flagBinary)
    {
        float x = static_cast<float>(p.x), y = static_cast<float>(p.y), z = static_cast<float>(p.z);
        if (flagBinary)
        {
            outfile.write(reinterpret_cast<const char *>(&x), sizeof(float));
            outfile.write(reinterpret_cast<const char *>(&y), sizeof(float));
            outfile.write(reinterpret_cast<const char *>(&z), sizeof(float));
        }
        else
        {
            outfile << x << " " << y << " "  << z;
            outfile << std::endl;
        }
    }

    template <> void write_point<PointXYZRGB>(PointXYZRGB const& p, std::ofstream & outfile, bool flagBinary)
    {
        float x = static_cast<float>(p.x), y = static_cast<float>(p.y), z = static_cast<float>(p.z);
        uint8_t b = static_cast<uint8_t>(p.b*255.f + 0.5f), g = static_cast<uint8_t>(p.g*255.f + 0.5f), r = static_cast<uint8_t>(p.r*255.f + 0.5f), a = static_cast<uint8_t>(p.a*255.f + 0.5f);
        if (flagBinary)
        {
            outfile.write(reinterpret_cast<const char *>(&x), sizeof(float));
            outfile.write(reinterpret_cast<const char *>(&y), sizeof(float));
            outfile.write(reinterpret_cast<const char *>(&z), sizeof(float));

            outfile.write(reinterpret_cast<const char *>(&r), sizeof(uint8_t));
            outfile.write(reinterpret_cast<const char *>(&g), sizeof(uint8_t));
            outfile.write(reinterpret_cast<const char *>(&b), sizeof(uint8_t));
            outfile.write(reinterpret_cast<const char *>(&a), sizeof(uint8_t));
        }
        else
        {
            outfile << x << " " << y << " "  << z
                    << " " << static_cast<int>(r) << " " << static_cast<int>(g) << " " << static_cast<int>(b) << " " << static_cast<int>(a)
                    << std::endl;
        }
    }
    
    //MMI points
    template <typename PointType> void add_node(PointType const& p, CSIRO::Mesh::MeshNodesInterface& nodes, std::vector<CSIRO::Mesh::NodeHandle>& nLookup, CSIRO::Mesh::NodeStateHandle& colHandle)
    {
        double x = static_cast<double>(p.x);
        double y = static_cast<double>(p.y);
        double z = static_cast<double>(p.z);
        CSIRO::Mesh::NodeHandle an = nodes.add(CSIRO::Mesh::Vector3d(p.x, p.y, p.z));
        nLookup.push_back(an);
    }
    template <> void add_node<PointXYZRGB>(PointXYZRGB const& p, CSIRO::Mesh::MeshNodesInterface& nodes, std::vector<CSIRO::Mesh::NodeHandle>& nLookup, CSIRO::Mesh::NodeStateHandle& colHandle)
    {
        double x = static_cast<double>(p.x);
        double y = static_cast<double>(p.y);
        double z = static_cast<double>(p.z);
        //RGBA
        uint rgba = 0;
        uint8_t b = static_cast<uint8_t>(p.b*255.f + 0.5f), g = static_cast<uint8_t>(p.g*255.f + 0.5f), r = static_cast<uint8_t>(p.r*255.f + 0.5f), a = static_cast<uint8_t>(p.a*255.f + 0.5f);
        rgba |= a << 24; //alpha channel to full
        rgba |= b << 16;
        rgba |= g << 8;
        rgba |= r;
        int alph = static_cast<int> (rgba);
        CSIRO::Mesh::NodeHandle an = nodes.add(CSIRO::Mesh::Vector3d(p.x, p.y, p.z));
        nodes.setState(an,colHandle,alph);
        nLookup.push_back(an);
    }
};

template <typename PointType>
bool PlyWriter::write(const std::string & filename, std::vector<PointType> const& vertices, std::vector<std::vector<size_t> > const& faces, unsigned flags)
{
    const bool flagBinary  = (flags&PlyBinary )>0;
    const bool flagFaces   = (flags&PlyFaces  )>0;
    const bool flagNormals = (flags&PlyNormals)>0;
    const bool flagColors  = (flags&PlyColors )>0;

    std::ofstream outfile;
    std::ios::openmode mode = std::ios::out|std::ios::trunc|(flagBinary?std::ios::binary:static_cast<std::ios::openmode>(0));
    outfile.open(filename.c_str(), mode);
    if (!outfile.is_open())
    {
        return false;
    }

    const char * format_header = (flagBinary? "binary_little_endian 1.0" : "ascii 1.0");
    outfile << "ply" << std::endl 
            << "format " << format_header << std::endl 
            << "comment ssd generated" << std::endl 
            << "element vertex " << vertices.size() << std::endl 
            << "property float x" << std::endl 
            << "property float y" << std::endl 
            << "property float z" << std::endl;
    if (flagNormals)
    {
        outfile << "property float nx" << std::endl 
                << "property float ny" << std::endl 
                << "property float nz" << std::endl;
    }
    if (flagColors)
    {
        outfile << "property uchar red" << std::endl 
                << "property uchar green" << std::endl 
                << "property uchar blue" << std::endl 
                << "property uchar alpha" << std::endl;
    }
    outfile << "element face " << (flagFaces ? faces.size() : 0) << std::endl 
            << "property list uchar int vertex_indices" << std::endl 
            << "end_header" << std::endl ;

    for (typename std::vector<PointType>::const_iterator iter=vertices.begin(); iter!=vertices.end(); ++iter)
    {
        write_point(*iter, outfile, flagBinary);
    }

    if (flagFaces)
    {
        for (std::vector<std::vector<size_t> >::const_iterator iter1=faces.begin(); iter1!=faces.end(); ++iter1)
        {
            std::vector<size_t> const& face = (*iter1);
            if (flagBinary)
            {
                unsigned char size = static_cast<unsigned char>(iter1->size()&0xff);
                outfile.write(reinterpret_cast<const char *>(&size), sizeof(unsigned char));
            }
            else
            {
                outfile << iter1->size();
            }

            for (std::vector<size_t>::const_iterator iter2=face.begin(); iter2!=face.end(); ++iter2)
            {
                if (flagBinary)
                {
                    int index = static_cast<int>(*iter2);
                    outfile.write(reinterpret_cast<const char *>(&index), sizeof(int));
                }
                else
                {
                    outfile << " " << (*iter2);
                }
            }
            if (!flagBinary)
            {
                outfile << std::endl;
            }
        }
    }

    outfile.close();
    return true;
}
//Write to MMI
template <typename PointType>
bool PlyWriter::write_toMMI(CSIRO::Mesh::MeshModelInterface& mesh, std::vector<PointType> const& vertices, std::vector<std::vector<size_t> > const& faces, bool color)
{
    CSIRO::Mesh::MeshNodesInterface& outNodes = mesh.getNodes();
    CSIRO::Mesh::MeshElementsInterface& tris = mesh.getElements(CSIRO::Mesh::ElementType::Tri::getInstance());
    CSIRO::Mesh::MeshElementsInterface& quads = mesh.getElements(CSIRO::Mesh::ElementType::Quad::getInstance());
    CSIRO::Mesh::NodeStateHandle rgbaHandle;
    std::vector<CSIRO::Mesh::NodeHandle> nodeLookup;
    std::cout << QString("No. Vertices: %1 No. Faces: %2").arg(vertices.size()).arg(faces.size()) + "\n";
    if (color)
    {
        int a = 0;
        rgbaHandle = outNodes.addState("RGBA", a);
    }
    //Add nodes
    for (typename std::vector<PointType>::const_iterator iter = vertices.begin(); iter != vertices.end(); ++iter)
    {
        add_node(*iter,outNodes,nodeLookup,rgbaHandle);
    }
        
    //std::cout << QString("Node lookup size is %1").arg(nodeLookup.size()) + "\n";
    //Add faces
    int ntri = 0;
    int nquad = 0;
    for (std::vector<std::vector<size_t>>::const_iterator elistIter = faces.begin(); elistIter != faces.end(); ++elistIter)
    {
        std::vector<size_t> const& face = (*elistIter);
        std::vector<CSIRO::Mesh::NodeHandle> nList;
        for (std::vector<size_t>::const_iterator flistIter = face.begin(); flistIter != face.end(); ++flistIter)
        {
            int index = static_cast<int>(*flistIter);
            nList.push_back(nodeLookup[index]);//Add node handle to nodehandlelist
            //std::cout << QString("Index is %1").arg(index) + "\n";
        }
        if (elistIter->size() == 3)
        {
            tris.add(nList[0],nList[1],nList[2]);
            ++ntri;
        }
        if (elistIter->size() == 4)
        {
            quads.add(nList[0],nList[1],nList[2],nList[3]);
            ++nquad;
        }
    }
    std::cout << QString("Added %1 tris, %2 quads").arg(ntri).arg(nquad) + "\n";
    return true;
}
