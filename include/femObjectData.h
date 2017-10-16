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

#ifndef FEM_OBJECT_DATA
#define FEM_OBJECT_DATA

#include <maya/MPxData.h>
#include <maya/MTypeId.h>
#include <maya/MString.h>
#include <maya/MObject.h>
#include <maya/MVector.h>
#include <maya/MIntArray.h>

#include <opentissue/math/math.h>
#include <opentissue/fem/fem.h>

typedef opentissue::math::Types<double, unsigned int> FEMMathTypes;
typedef FEMMathTypes::vector_type FEMVector;
typedef opentissue::fem::Mesh<FEMMathTypes> FEMTetrahedralMesh;

struct FEMParameters {
    bool enable;
    bool passive;

    double density;
    double poissonsRatio;
    double youngsModulus;
    double massDamping;
    double stiffnessDamping;
    double minimumYieldStrength;
    double maximumYieldStrength;
    double creepRate;
    double friction;

    MVector initialVelocity;
    MVector initialAngularVelocity;

    MObject meshObject;
    MObject surfaceNodesObject;
    MObject volumeNodesObject;
    MObject matrixObject;

    bool updateParameters;
    bool updateMesh;

    FEMParameters();
    FEMParameters(const FEMParameters &);
    ~FEMParameters();
};

class FEMObjectData : public MPxData {
public:
    static const MTypeId id;
    static const MString typeName;

    FEMObjectData();
    FEMObjectData(const MPxData &);
    virtual ~FEMObjectData();

    virtual MTypeId typeId() const;
    virtual MString name() const;

    static void * creator();
    virtual	void copy(const MPxData &);

    FEMObjectData & reset();

    FEMObjectData & initialize(FEMParameters &);
    FEMObjectData & update(FEMParameters &, const MPxData &);

    FEMTetrahedralMesh * getTetrahedralMesh();
    const FEMTetrahedralMesh * getTetrahedralMesh() const;
    MIntArray * getSurfaceNodes();
    const MIntArray * getSurfaceNodes() const;

    bool isEnable() const;
    bool isPassive() const;

    double getMassDamping() const;
    double getStiffnessDamping() const;
    double getFriction() const;

private:
    FEMTetrahedralMesh * tetrahedralMesh;
    MIntArray * surfaceNodes;

    bool enable;
    bool passive;

    double massDamping;
    double stiffnessDamping;
    double friction;

    FEMObjectData & allocate();
    FEMObjectData & deallocate();
};

#endif