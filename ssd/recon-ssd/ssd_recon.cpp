/*
Copyright (c) 2011, Fatih Calakli and Gabriel Taubin
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

#include <string.h>
#include <ssd/dgp/PointReader.hpp>
#include <ssd/dgp/PlyWriter.hpp>

#include "run_ssd.hpp"

void printVerstion(FILE* fp);
void fprintOptions(FILE* fp, Settings const& D);
void print_usage(Settings const& D);
void parseCommandLine(int argc, char **argv, Settings & D);

inline static void error(const char *msg) 
{
    fprintf(stdout,"ssd | ERROR | %s\n",(msg)?msg:"");
    exit(0);
}

template <typename PointContainer> bool load_points(Settings const& D, PointContainer & points);
template <typename PointContainer> void save_mesh(Settings const& D, PointContainer const& vertices, std::vector<std::vector<size_t> > const& faces);

//////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    printVerstion(stdout);

    ////////////////////////////////////////////////////////////////////////
    // parse command line

    Settings D;
    parseCommandLine(argc, argv, D);

    if (D.debug)
    {
        fprintf(stdout,"SSD {\n");
        fprintf(stdout,"\n");
        fprintOptions(stdout,D);
    }
    fprintf(stdout,"\n");
    fprintf(stdout,"   input file: \"%s\"\n",D.inFile.c_str());
    fprintf(stdout,"  output file: \"%s\"\n",D.outFile.c_str());
    fprintf(stdout,"\n");

    size_t outPos = D.outFile.find_last_of(".");
    std::string outExt = D.outFile.substr(outPos+1);
    if (outExt!="ply")
    {
        fprintf(stdout,"\n    Output format not supported\n");
        return 1;
    }

    //check file properties
    std::unordered_map<std::string, bool> properties;
    properties["x"] = false; properties["y"] = false; properties["z"] = false;
    properties["nx"] = false; properties["ny"] = false; properties["nz"] = false;
    properties["red"] = false; properties["blue"] = false; properties["green"] = false;
    properties["diffuse_red"] = false; properties["diffuse_blue"] = false; properties["diffuse_green"] = false;

    PointReader::check_properties(D.inFile.c_str(), properties);
    if (!properties["x"] || !properties["y"] || !properties["z"])
    {
        fprintf(stdout,"\n    ERROR: no point data found.\n");
        return 1;
    }
    if (!properties["nx"] || !properties["ny"] || !properties["nz"])
    {
        fprintf(stdout,"\n    ERROR: point normals are required.\n");
        return 1;
    }
    if (D.computeColor
        && !(properties["red"] && properties["blue"] && properties["green"])
        && !(properties["diffuse_red"] && properties["diffuse_blue"] && properties["diffuse_green"]))
    {
        fprintf(stdout,"\n    ERROR: color data not found.\n");
        return 1;
    }

    //run ssd
    if (D.computeColor)
    {
        OctreeBundle<SSDColorType> bundle(D);
        if (load_points(D, bundle.points))
        {
            std::vector<typename SSDColorType::OutPointType> vertices;
            std::vector<std::vector<size_t> > faces;
            run_ssd(D, bundle, vertices, faces);
            save_mesh(D, vertices, faces);
        }
    }
    else
    {
        OctreeBundle<SSDType> bundle(D);
        if (load_points(D, bundle.points))
        {
            std::vector<typename SSDType::OutPointType> vertices;
            std::vector<std::vector<size_t> > faces;
            run_ssd(D, bundle, vertices, faces);
            save_mesh(D, vertices, faces);
        }
    }
    exit(0);
    return 0;
}

template <typename PointContainer> bool load_points(Settings const& D, PointContainer & points)
{
    ////////////////////////////////////////////////////////////////////////
    // read data points from 'inFile'

    if(D.debug)
    {
        fprintf(stdout,"  reading oriented points {\n");
        fprintf(stdout,"    inFile = \"%s\"\n", D.inFile.c_str());
        fflush(stdout);
    }

    if (!PointReader::load(D.inFile.c_str(), points))
    {
        fprintf(stdout,"\n    File load failed\n");
        return false;
    }

    size_t nPoints = points.size();

    if (D.debug) 
    {
        fprintf(stdout,"    nPts   = %lu\n", nPoints);
        fprintf(stdout,"  }\n");
        fprintf(stdout,"\n");
        fflush(stdout);
    }

    if (nPoints<=0)
    {
        if (D.debug) 
        {
            fprintf(stdout,"  cannot continue without points\n");
            fprintf(stdout,"}\n");
            fflush(stdout);
        }
        return false;
    }

    return true;
}

template <typename PointContainer> void save_mesh(Settings const& D, PointContainer const& vertices, std::vector<std::vector<size_t> > const& faces)
{
    //////////////////////////////////////////////////////////////////////
    // save mesh to 'outFile'

    if (D.debug) 
    {
        fprintf(stdout,"  saving output file {\n");
        fprintf(stdout,"    outFile = \"%s\"\n",D.outFile.c_str());
        fflush(stdout);
    }

    if(D.debug) fprintf(stdout,"    PLY\n");
    int ply_flags = PlyWriter::PlyPoints|PlyWriter::PlyFaces;
    if (D.writeBinary)
    {
        ply_flags |= PlyWriter::PlyBinary;
    }
    if (D.computeColor)
    {
        ply_flags |= PlyWriter::PlyColors;
    }
    PlyWriter::write(D.outFile, vertices, faces, ply_flags);

    if(D.debug) 
    {
        fprintf(stdout,"  }\n");
        fprintf(stdout,"\n");
        fflush(stdout);
    }

    //////////////////////////////////////////////////////////////////////
    if(D.debug)
    {
        fprintf(stdout,"}\n\n");
        fflush(stdout);
    }
}

void printVerstion(FILE* fp)
{
  fprintf(fp, "SSD Version %d.%d -- http://mesh.brown.edu/ssd/\n", SSD_VER_MAJOR, SSD_VER_MINOR);
}

void fprintOptions(FILE* fp, Settings const& D) 
{
    fprintf(fp,"     -d|-debug               [%s]\n", (D.debug?"true":"false"));
    fprintf(fp,"     -c|-computeColor        [%s]\n", (D.computeColor?"true":"false"));
    fprintf(fp,"   -bbe|-bboxExpansionFactor <%lf>\n", D.bboxExpansionFactor);
    fprintf(fp,"    -oL|-octreeLevels        <%lu>\n", D.octreeLevels);
    fprintf(fp,"   -oLs|-octreeLevelStart    <%lu>\n", D.octreeLevelStart);
    fprintf(fp,"   -spn|-samplesPerNode      <%lu>\n", D.samplesPerNode);
    fprintf(fp,"     -l|-isoLevel            <%lf>\n", D.isoLevel);
    fprintf(fp,"   -tol|-solverTolerance     <%1.e>\n", D.solverTolerance);
    fprintf(fp,"  -tolc|-solverToleranceColor<%1.e>\n", D.solverTolerance_color);
    fprintf(fp," -minIt|-maxNumOfIterations  <%lu>\n", D.minIterations);
    fprintf(fp," -maxIt|-maxNumOfIterations  <%lu>\n", D.maxIterations);
    fprintf(fp,"     -w|-weights             <%lf> <%lf> <%lf>\n", D.weight0, D.weight1, D.weight2);
    fprintf(fp,"    -wc|-weightsColor        <%lf> <%lf>\n", D.weight0_color, D.weight1_color);
    fprintf(fp,"       |-fast                <%s>\n", (D.fast?"true":"false"));
    fprintf(fp,"     -p|-polygons            <%s>\n", (D.polygons?"true":"false"));
    fprintf(fp,"     -a|-ascii               <%s>\n", (D.writeBinary?"false":"true"));
}

void print_usage(Settings const& D) 
{
    fprintf(stdout,"USAGE\n");
    fprintf(stdout,"  SSD [options] inFile outFile\n");
    fprintOptions(stdout, D);
}

void parseCommandLine(int argc, char **argv, Settings & D) 
{
    if (argc==1)
    {
        print_usage(D);
        exit(0);
    }

    for (int i=1; i<argc; ++i) 
    {
        if (!strcmp(argv[i],"-h") || !strcmp(argv[i],"-help")) { print_usage(D); exit(0); } 
        else if (!strcmp(argv[i],"-d")   || !strcmp(argv[i],"-debug")) { D.debug = !D.debug; }
        else if (!strcmp(argv[i],"-c")   || !strcmp(argv[i],"-computeColor")) { D.computeColor = !D.computeColor; } 
        else if (!strcmp(argv[i],"-bbe") || !strcmp(argv[i],"-bboxExpansionFactor")) 
        {   
            if ((++i)>=argc || sscanf(argv[i],"%lf",&(D.bboxExpansionFactor))<1) { error("parsing -bboxExpansionFactor"); }
        } 
        else if (!strcmp(argv[i],"-oL")  || !strcmp(argv[i],"-octreeLevels")) 
        {
            if ((++i)>=argc || sscanf(argv[i],"%d",&(D.octreeLevels))<1) { error("parsing -octreeLevels"); }
        } 
        else if (!strcmp(argv[i],"-oLs") || !strcmp(argv[i],"-octreeLevelStart")) 
        {
            if ((++i)>=argc || sscanf(argv[i],"%d",&(D.octreeLevelStart))<1) { error("parsing -octreeLevelStart"); }
        } 
        else if (!strcmp(argv[i],"-spn") || !strcmp(argv[i],"-samplesPerNode")) 
        {
            if ((++i)>=argc || sscanf(argv[i],"%d",&(D.samplesPerNode))<1) { error("parsing -samplesPerNode"); }
        }
        else if (!strcmp(argv[i],"-l")   || !strcmp(argv[i],"-isoLevel")) 
        {
            if ((++i)>=argc || sscanf(argv[i],"%lf",&(D.isoLevel))<1) { error("parsing -isoLevel"); }
        } 
        else if (!strcmp(argv[i],"-w")   || !strcmp(argv[i],"-weights")) 
        {
            if ((++i)>=argc || sscanf(argv[i],"%lf",&(D.weight0))<=0) { error("parsing -weights"); }
            if ((++i)>=argc || sscanf(argv[i],"%lf",&(D.weight1))<=0) { error("parsing -weights"); }
            if ((++i)>=argc || sscanf(argv[i],"%lf",&(D.weight2))<=0) { error("parsing -weights"); } 
        }
        else if (!strcmp(argv[i],"-wc")   || !strcmp(argv[i],"-weightsColor")) 
        {
            if ((++i)>=argc || sscanf(argv[i],"%lf",&(D.weight0_color))<=0) { error("parsing -weightsColor"); }
            if ((++i)>=argc || sscanf(argv[i],"%lf",&(D.weight1_color))<=0) { error("parsing -weightsColor"); }
        }
        else if (!strcmp(argv[i],"-tol") || !strcmp(argv[i],"-solverTolerance")) 
        {
            if ((++i)>=argc || sscanf(argv[i],"%lf",&(D.solverTolerance))>1.0) { error("parsing -solverTolerance"); }
        }
        else if (!strcmp(argv[i],"-tolc") || !strcmp(argv[i],"-solverToleranceColor")) 
        {
            if ((++i)>=argc || sscanf(argv[i],"%lf",&(D.solverTolerance_color))>1.0) { error("parsing -solverToleranceColor"); }
        }
        else if (!strcmp(argv[i],"-minIt") || !strcmp(argv[i],"-minNumOfIterations")) 
        {
            if ((++i)>=argc || sscanf(argv[i],"%u",&(D.minIterations))<0) { error("parsing -minNumOfIterations"); }
        } 
        else if (!strcmp(argv[i],"-maxIt") || !strcmp(argv[i],"-maxNumOfIterations")) 
        {
            if ((++i)>=argc || sscanf(argv[i],"%u",&(D.maxIterations))<0) { error("parsing -maxNumOfIterations"); }
        } 
        else if (!strcmp(argv[i],"-fast")) { D.fast = true; }
        else if (!strcmp(argv[i],"-p")   || !strcmp(argv[i],"-polygons")) { D.polygons = true; }
        else if (!strcmp(argv[i],"-a")   || !strcmp(argv[i],"-ascii"))    { D.writeBinary = false; }
        else if (argv[i][0]=='-') { error("unknown option"); } 
        else if (D.inFile.size()==0) { D.inFile = std::string(argv[i]); } 
        else if (D.outFile.size()==0) { D.outFile = std::string(argv[i]); }
    }
  
    if (D.inFile.size()==0)                 { error("no inFile");  }
    if (D.outFile.size()==0)                { error("no outFile"); }
    if (D.octreeLevels<1)                   { error("levels<1");   }
    if (D.octreeLevelStart>D.octreeLevels)  { error("octreeLevelStart>octreeLevels"); }
}
