// Copyright (c) 2018, Danilo Ferreira. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef FEM_SOLVER
#define FEM_SOLVER

#include <femObjectData.h>

#include <maya/MPxNode.h>
#include <maya/MObject.h>
#include <maya/MTypeId.h>
#include <maya/MString.h>
#include <maya/MStatus.h>
#include <maya/MPlug.h>
#include <maya/MPlugArray.h>
#include <maya/MDataBlock.h>
#include <maya/MVector.h>
#include <maya/MComputation.h>

class FEMSolver : public MPxNode {
public:
    static MObject enableObject;
    static MObject startTimeObject;
    static MObject currentTimeObject;
    static MObject substepsObject;
    static MObject gravityXObject;
    static MObject gravityYObject;
    static MObject gravityZObject;
    static MObject gravityObject;
    static MObject maximumIterationsObject;
    static MObject currentStateObject;
    static MObject outputStateObject;

    static const MTypeId id;
    static const MString typeName;

    MComputation computation;

    FEMSolver();
    virtual ~FEMSolver();

    static void * creator();
    static MStatus initialize();
    virtual void postConstructor();
    virtual MStatus	setDependentsDirty(const MPlug &, MPlugArray &);
    virtual MStatus compute(const MPlug &, MDataBlock &);

private:
    void simulateSubstep(FEMObjectData *, const MVector &, double, int) const;
};

#endif