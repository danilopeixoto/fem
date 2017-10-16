// Copyright (c) 2017, Danilo Peixoto. All rights reserved.
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

#ifndef FEM_OBJECT
#define FEM_OBJECT

#include "femObjectData.h"

#include <maya/MPxNode.h>
#include <maya/MObject.h>
#include <maya/MTypeId.h>
#include <maya/MString.h>
#include <maya/MStatus.h>
#include <maya/MPlug.h>
#include <maya/MPlugArray.h>
#include <maya/MDataBlock.h>

class FEMObject : public MPxNode {
public:
    static MObject enableObject;
    static MObject passiveObject;
    static MObject densityObject;
    static MObject poissonsRatioObject;
    static MObject youngsModulusObject;
    static MObject massDampingObject;
    static MObject stiffnessDampingObject;
    static MObject minimumYieldStrengthObject;
    static MObject maximumYieldStrengthObject;
    static MObject creepRateObject;
    static MObject frictionObject;
    static MObject initialVelocityXObject;
    static MObject initialVelocityYObject;
    static MObject initialVelocityZObject;
    static MObject initialVelocityObject;
    static MObject initialAngularVelocityXObject;
    static MObject initialAngularVelocityYObject;
    static MObject initialAngularVelocityZObject;
    static MObject initialAngularVelocityObject;
    static MObject startTimeObject;
    static MObject currentTimeObject;
    static MObject inputMeshObject;
    static MObject surfaceNodesObject;
    static MObject volumeNodesObject;
    static MObject matrixObject;
    static MObject outputMeshObject;
    static MObject nextStateObject;
    static MObject currentStateObject;

    static const MTypeId id;
    static const MString typeName;

    FEMObject();
    virtual ~FEMObject();

    static void * creator();
    static MStatus initialize();
    virtual MStatus	setDependentsDirty(const MPlug &, MPlugArray &);
    virtual MStatus compute(const MPlug &, MDataBlock &);

private:
    FEMParameters parameters;

    void updateOutputMesh(const FEMObjectData *, MObject &) const;
};

#endif