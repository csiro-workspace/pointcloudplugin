Point Cloud plugin for CSIRO Workspace
======================================

This project is a plugin for the [CSIRO Workspace](https://research.csiro.au/workspace/) scientific workflow framework. Its purpose is to expose existing open source libraries including [PCL](http://pointclouds.org/), [libLAS](http://www.liblas.org/), [LASzip](http://www.laszip.org/) and [SSD Surface Reconstruction](http://mesh.brown.edu/ssd/software.html) for use in Workspace to support point cloud processing and visualisation workflows. 

This plugin is very early in development and only exposes a small percentage of data structures and algorithms from the listed libraries. Feedback and contributions are more than welcome to help expand its capabilities. 

Contributors
------------
- Matt Bolger - 
Computational Software and Visualisation, CSIRO Digital Productivity, Clayton VIC
- Stuart Mead -
Risk Frontiers, Dept. of Environmental Sciences, Macquarie University, North Ryde NSW and CSIRO Digital Productivity, Clayton VIC

Compiling and using the plugin
------------------------------
1. Download and install [CSIRO Workspace](https://research.csiro.au/workspace/download/)
2. Download and install [PCL](http://pointclouds.org/) (or build a version yourself)
3. Checkout this repository
4. If you want LAS/LAZ support checkout [libLAS](http://www.liblas.org/) and [LASzip](http://www.laszip.org/) in sibling directories to this project's source
5. Launch CMake from Workspace's Development menu to configure and generate the project. This needs to be done from within Workspace rather than running CMake directly as some key environment variables get setup.
   * If Boost detection fails as part of finding PCL during the CMake Configure, explicitly add a CMake variable for Boost_INCLUDE_DIR pointing to the version of boost being used by PCL (eg. C:/Program Files/PCL 1.6.0/3rdParty/Boost/include for the Windows All-in-one installer). This prevents the Boost version that ships with Workspace for Python support causing issues. 
6. Compile the project
7. Copy Install\installAreas\installArea-PointClouds.txt from your build directory to Workspace's installAreas directory
8. Restart Workspace and a PointCloud group should be available in the Workspace operation catalogue

About CSIRO Workspace
---------------------
Workspace is a powerful scientific application framework and workflow editor which has been in development for over 7 years. Originally designed to construct workflows for scientists in the computational fluid dynamics space, Workspace recognises the importance of interactivity, visualisation, scalability, and ease-of-use in a scientific workflow. It ships with out of the box support for things like mesh processing, interactive 3D visualisation, plotting, networking and database access and is designed to be easily extended through plugins. More than just a workflow editor, Workspace offers a natural path from early code development as part of a research workflow right through to the development of standalone applications for deployment to a collaborator or external client.

Workspace is developed by the Computational Modelling and Simulation Group of Australia’s Commonwealth Scientific and Industrial Research Organisation (CSIRO). Workspace has been developed with support from CSIRO eResearch, Computational and Simulation Sciences and the Digital Productivity Flagship.

For more details or to contact the team see the [Workspace research page](https://research.csiro.au/workspace/).

License
-------
CSIRO Open Source Software License Agreement 
(variation of the BSD / MIT License)

Copyright (c) 2015, Commonwealth Scientific and Industrial Research 
Organisation (CSIRO) ABN 41 687 119 230.

All rights reserved. CSIRO is willing to grant you a license to this Workspace 
Point Cloud Plugin on the following terms, except where otherwise indicated for
third party material.

Redistribution and use of this software in source and binary forms, with or 
without modification, are permitted provided that the following conditions are
met:
• Redistributions of source code must retain the above copyright notice, this 
  list of conditions and the following disclaimer.
• Redistributions in binary form must reproduce the above copyright notice, 
  this list of conditions and the following disclaimer in the documentation 
  and/or other materials provided with the distribution.
• Neither the name of CSIRO nor the names of its contributors may be used to
  endorse or promote products derived from this software without specific prior
  written permission of CSIRO.

EXCEPT AS EXPRESSLY STATED IN THIS AGREEMENT AND TO THE FULL EXTENT PERMITTED BY APPLICABLE LAW, THE SOFTWARE IS PROVIDED "AS-IS". CSIRO MAKES NO REPRESENTATIONS, WARRANTIES OR CONDITIONS OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO ANY REPRESENTATIONS, WARRANTIES OR CONDITIONS REGARDING THE CONTENTS OR ACCURACY OF THE SOFTWARE, OR OF TITLE, MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, NON-INFRINGEMENT, THE ABSENCE OF LATENT OR OTHER DEFECTS, OR THE PRESENCE OR ABSENCE OF ERRORS, WHETHER OR NOT DISCOVERABLE.
TO THE FULL EXTENT PERMITTED BY APPLICABLE LAW, IN NO EVENT SHALL CSIRO BE LIABLE ON ANY LEGAL THEORY (INCLUDING, WITHOUT LIMITATION, IN AN ACTION FOR BREACH OF CONTRACT, NEGLIGENCE OR OTHERWISE) FOR ANY CLAIM, LOSS, DAMAGES OR OTHER LIABILITY HOWSOEVER INCURRED.  WITHOUT LIMITING THE SCOPE OF THE PREVIOUS SENTENCE THE EXCLUSION OF LIABILITY SHALL INCLUDE: LOSS OF PRODUCTION OR OPERATION TIME, LOSS, DAMAGE OR CORRUPTION OF DATA OR RECORDS; OR LOSS OF ANTICIPATED SAVINGS, OPPORTUNITY, REVENUE, PROFIT OR GOODWILL, OR OTHER ECONOMIC LOSS; OR ANY SPECIAL, INCIDENTAL, INDIRECT, CONSEQUENTIAL, PUNITIVE OR EXEMPLARY DAMAGES, ARISING OUT OF OR IN CONNECTION WITH THIS AGREEMENT, ACCESS OF THE SOFTWARE OR ANY OTHER DEALINGS WITH THE SOFTWARE, EVEN IF CSIRO HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH CLAIM, LOSS, DAMAGES OR OTHER LIABILITY.
APPLICABLE LEGISLATION SUCH AS THE AUSTRALIAN CONSUMER LAW MAY APPLY REPRESENTATIONS, WARRANTIES, OR CONDITIONS, OR IMPOSES OBLIGATIONS OR LIABILITY ON CSIRO THAT CANNOT BE EXCLUDED, RESTRICTED OR MODIFIED TO THE FULL EXTENT SET OUT IN THE EXPRESS TERMS OF THIS CLAUSE ABOVE "CONSUMER GUARANTEES".  TO THE EXTENT THAT SUCH CONSUMER GUARANTEES CONTINUE TO APPLY, THEN TO THE FULL EXTENT PERMITTED BY THE APPLICABLE LEGISLATION, THE LIABILITY OF CSIRO UNDER THE RELEVANT CONSUMER GUARANTEE IS LIMITED (WHERE PERMITTED AT CSIRO’S OPTION) TO ONE OF FOLLOWING REMEDIES OR SUBSTANTIALLY EQUIVALENT REMEDIES:
(a) THE REPLACEMENT OF THE SOFTWARE, THE SUPPLY OF EQUIVALENT SOFTWARE, OR SUPPLYING RELEVANT SERVICES AGAIN;
(b) THE REPAIR OF THE SOFTWARE;
(c) THE PAYMENT OF THE COST OF REPLACING THE SOFTWARE, OF ACQUIRING EQUIVALENT SOFTWARE, HAVING THE RELEVANT SERVICES SUPPLIED AGAIN, OR HAVING THE SOFTWARE REPAIRED.
IN THIS CLAUSE, CSIRO INCLUDES ANY THIRD PARTY AUTHOR OR OWNER OF ANY PART OF THE SOFTWARE OR MATERIAL DISTRIBUTED WITH IT.  CSIRO MAY ENFORCE ANY RIGHTS ON BEHALF OF THE RELEVANT THIRD PARTY.

As a condition of this license, you agree that where you make any adaptations, modifications, further developments, or additional features available to CSIRO or the public in connection with your access to the Software, you do so on the terms of the BSD 3-Clause License template, a copy available at: http://opensource.org/licenses/BSD-3-Clause.

Third Party Components
----------------------
The following third party components are distributed with the Software. You agree to comply with the license terms for these components as part of accessing the Software. Other third party software may also be identified in separate files distributed with the Software.

SMOOTH SIGNED DISTANCE SURFACE RECONSTRUCTION SOFTWARE (VERSION 3.0)
Copyright (c) 2011, Fatih Calakli and Gabriel Taubin
See <source dir>/ssd/license.txt for full details

This project is designed to be built against a number of existing open source libraries but developers should acquire the source for these projects themselves.

* http://pointclouds.org/
* https://github.com/libLAS/libLAS
* https://github.com/LASzip/LASzip
