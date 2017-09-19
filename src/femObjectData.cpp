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

#include "femObjectData.h"

#include <maya/MFnMesh.h>
#include <maya/MFnIntArrayData.h>
#include <maya/MPoint.h>

FEMParameters::FEMParameters() {
    enable = true;
    passive = false;

    density = 0;
    poissonsRatio = 0;
    youngsModulus = 0;
    massDamping = 0;
    stiffnessDamping = 0;
    minimumYieldStrength = 0;
    maximumYieldStrength = 0;
    creepRate = 0;
    friction = 0;
}
FEMParameters::FEMParameters(const FEMParameters & parameters) {
    if (this != &parameters) {
        enable = parameters.enable;
        passive = parameters.passive;

        density = parameters.density;
        poissonsRatio = parameters.poissonsRatio;
        youngsModulus = parameters.youngsModulus;
        massDamping = parameters.massDamping;
        stiffnessDamping = parameters.stiffnessDamping;
        minimumYieldStrength = parameters.minimumYieldStrength;
        maximumYieldStrength = parameters.maximumYieldStrength;
        creepRate = parameters.creepRate;
        friction = parameters.friction;

        initialVelocity = parameters.initialVelocity;
        initialAngularVelocity = parameters.initialAngularVelocity;

        meshObject = parameters.meshObject;
        surfaceNodesObject = parameters.surfaceNodesObject;
        volumeNodesObject = parameters.volumeNodesObject;
    }
}
FEMParameters::~FEMParameters() {}

const MTypeId FEMObjectData::id(0x00128580);
const MString FEMObjectData::typeName("femObjectData");

FEMObjectData::FEMObjectData() {
    tetrahedralMesh = new TetrahedralMesh;
    surfaceNodes = new MIntArray;

    enable = true;
    passive = false;

    massDamping = 0;
    stiffnessDamping = 0;
    friction = 0;
}
FEMObjectData::FEMObjectData(const MPxData & source) {
    tetrahedralMesh = new TetrahedralMesh;
    surfaceNodes = new MIntArray;

    copy(source);
}
FEMObjectData::FEMObjectData(const FEMParameters & parameters) {
    tetrahedralMesh = new TetrahedralMesh;
    this->surfaceNodes = new MIntArray;

    initialize(parameters);
}
FEMObjectData::~FEMObjectData() {
    if (tetrahedralMesh != nullptr)
        delete tetrahedralMesh;

    if (surfaceNodes != nullptr)
        delete surfaceNodes;
}

MTypeId FEMObjectData::typeId() const {
    return id;
}
MString FEMObjectData::name() const {
    return typeName;
}

void * FEMObjectData::creator() {
    return new FEMObjectData;
}
void FEMObjectData::copy(const MPxData & source) {
    const FEMObjectData & objectData = (const FEMObjectData &)source;

    if (this != &objectData) {
        *tetrahedralMesh = *objectData.getTetrahedralMesh();
        *surfaceNodes = *objectData.getSurfaceNodes();

        enable = objectData.isEnable();
        passive = objectData.isPassive();

        massDamping = objectData.getMassDamping();
        stiffnessDamping = objectData.getStiffnessDamping();
        friction = objectData.getFriction();
    }
}

FEMObjectData & FEMObjectData::initialize(const FEMParameters & parameters) {
    MFnMesh mesh(parameters.meshObject);

    MPoint point;

    for (int i = 0; i < mesh.numVertices(); i++) {
        mesh.getPoint(i, point);
        tetrahedralMesh->insert(Vector(point.x, point.y, point.z));
    }

    MFnIntArrayData intArrayData;

    intArrayData.setObject(parameters.surfaceNodesObject);
    *this->surfaceNodes = intArrayData.array();

    intArrayData.setObject(parameters.volumeNodesObject);
    MIntArray vn = intArrayData.array();

    unsigned int i = 0;

    while (i < vn.length())
        tetrahedralMesh->insert(vn[i++], vn[i++], vn[i++], vn[i++]);

    enable = parameters.enable;
    passive = parameters.passive;

    massDamping = parameters.massDamping;
    stiffnessDamping = parameters.stiffnessDamping;
    friction = parameters.friction;

    if (enable) {
        opentissue::fem::initialize(*tetrahedralMesh, parameters.density,
            parameters.poissonsRatio, parameters.youngsModulus,
            parameters.minimumYieldStrength, parameters.maximumYieldStrength,
            parameters.creepRate);

        if (passive)
            opentissue::fem::set_fixed(*tetrahedralMesh, true);
        else {
            const MVector & v = parameters.initialVelocity;
            const MVector & a = parameters.initialAngularVelocity;

            opentissue::fem::apply_velocity(*tetrahedralMesh, Vector(v.x, v.y, v.z));
            opentissue::fem::apply_angular_velocity(*tetrahedralMesh, Vector(a.x, a.y, a.z));
        }
    }

    return *this;
}
FEMObjectData & FEMObjectData::update(const FEMParameters & parameters,
    const MPxData & source) {
    const FEMObjectData & objectData = (const FEMObjectData &)source;

    *tetrahedralMesh = *objectData.getTetrahedralMesh();
    *surfaceNodes = *objectData.getSurfaceNodes();

    enable = parameters.enable;
    passive = parameters.passive;

    massDamping = parameters.massDamping;
    stiffnessDamping = parameters.stiffnessDamping;
    friction = parameters.friction;

    if (enable) {
        opentissue::fem::initialize(*tetrahedralMesh, parameters.density,
            parameters.poissonsRatio, parameters.youngsModulus,
            parameters.minimumYieldStrength, parameters.maximumYieldStrength,
            parameters.creepRate);

        if (passive)
            opentissue::fem::set_fixed(*tetrahedralMesh, true);
        else
            opentissue::fem::set_fixed(*tetrahedralMesh, false);
    }

    return *this;
}

TetrahedralMesh * FEMObjectData::getTetrahedralMesh() {
    return tetrahedralMesh;
}
const TetrahedralMesh * FEMObjectData::getTetrahedralMesh() const {
    return tetrahedralMesh;
}
MIntArray * FEMObjectData::getSurfaceNodes() {
    return surfaceNodes;
}
const MIntArray * FEMObjectData::getSurfaceNodes() const {
    return surfaceNodes;
}

bool FEMObjectData::isEnable() const {
    return enable;
}
bool FEMObjectData::isPassive() const {
    return passive;
}

double FEMObjectData::getMassDamping() const {
    return massDamping;
}
double FEMObjectData::getStiffnessDamping() const {
    return stiffnessDamping;
}
double FEMObjectData::getFriction() const {
    return friction;
}