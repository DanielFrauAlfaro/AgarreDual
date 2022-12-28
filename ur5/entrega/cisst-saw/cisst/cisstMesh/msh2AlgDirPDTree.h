/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */
// ****************************************************************************
//
//    Copyright (c) 2014, Seth Billings, Russell Taylor, Johns Hopkins University
//    All rights reserved.
//
//    Redistribution and use in source and binary forms, with or without
//    modification, are permitted provided that the following conditions are
//    met:
//
//    1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
//    2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
//    3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
//    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//    A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//    HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//    DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// ****************************************************************************

#ifndef _msh2AlgDirPDTree_h
#define _msh2AlgDirPDTree_h

#include <cisstMesh/mshForwardDeclarations.h>
#include <cisstMesh/msh2DirPDTreeNode.h>

// Alwasy include last!
#include <cisstMesh/mshExport.h>

class CISST_EXPORT msh2AlgDirPDTree
{
    //
    // This is the base class for a family of Directional PD Tree search algorithms
    //

    //--- Algorithm Parameters ---//

 public:

    msh2DirPDTreeBase  *pDirTree;   // the PD tree

    //--- Algorithm Methods ---//

 public:

    // constructor
    msh2AlgDirPDTree(msh2DirPDTreeBase *pDirTree);

    // destructor
    virtual ~msh2AlgDirPDTree() {}


    //--- PD Tree Interface Methods ---//

    // fast check if a node might contain a datum having smaller match error
    //  than the error bound
    virtual int NodeMightBeCloser(const vct2 &sample, const vct2 &sampleNorm,
                                  msh2DirPDTreeNode const *node,
                                  double ErrorBound) = 0;

    // fast check if a datum might have smaller match error than error bound
    virtual int DatumMightBeCloser(const vct2 &sample, const vct2 &sampleNorm,
                                   int datum,
                                   double ErrorBound) = 0;

    // finds the point on this datum with lowest match error
    //  and returns the match error and closest point
    virtual double FindClosestPointOnDatum(const vct2 &sample, const vct2 &sampleNorm,
                                           vct2 &closest, vct2 &closestNorm,
                                           int datum) = 0;

};

#endif
