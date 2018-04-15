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

#include <femObjectData.h>

#include <maya/MPoint.h>
#include <maya/MFnIntArrayData.h>
#include <maya/MFnMesh.h>

FEMParameters::FEMParameters() {
    enable = true;
    passive = false;

    density = 1000.0;
    poissonsRatio = 0;
    youngsModulus = 1.0e5;
    massDamping = 2.0;
    stiffnessDamping = 0;
    minimumYieldStrength = 0;
    maximumYieldStrength = 0;
    creepRate = 0;
    friction = 0.5;

    updateParameters = false;
    updateMesh = false;
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
        matrix = parameters.matrix;

        updateParameters = parameters.updateParameters;
        updateMesh = parameters.updateMesh;
    }
}
FEMParameters::~FEMParameters() {}

const MTypeId FEMObjectData::id(0x00128580);
const MString FEMObjectData::typeName("femObjectData");

FEMObjectData::FEMObjectData() {
    allocate();

    enable = true;
    passive = false;

    massDamping = 2.0;
    stiffnessDamping = 0;
    friction = 0.5;
}
FEMObjectData::FEMObjectData(const MPxData & source) {
    allocate();
    copy(source);
}
FEMObjectData::~FEMObjectData() {
    deallocate();
}

MTypeId FEMObjectData::typeId() const {
    return id;
}
MString FEMObjectData::name() const {
    return typeName;
}

void * FEMObjectData::creator() {
    return new FEMObjectData();
}
void FEMObjectData::copy(const MPxData & source) {
    const FEMObjectData & objectData = (const FEMObjectData &)source;

    if (this != &objectData) {
        *tetrahedralMesh = *objectData.getTetrahedralMesh();
        surfaceNodes->copy(*objectData.getSurfaceNodes());

        enable = objectData.isEnable();
        passive = objectData.isPassive();

        massDamping = objectData.getMassDamping();
        stiffnessDamping = objectData.getStiffnessDamping();
        friction = objectData.getFriction();
    }
}

MStatus FEMObjectData::readASCII(const MArgList & argumentList, unsigned int & index) {
    return MS::kSuccess;
}
MStatus FEMObjectData::readBinary(std::istream & istream, unsigned int length) {
    return MS::kSuccess;
}
MStatus FEMObjectData::writeASCII(std::ostream & ostream) {
    ostream << enable << ' ' << passive << ' ' << tetrahedralMesh->size_tetrahedra();

    return ostream.fail() ? MS::kFailure : MS::kSuccess;
}
MStatus FEMObjectData::writeBinary(std::ostream & ostream) {
    ostream.write((const char *)&enable, sizeof(bool));

    if (!ostream.fail())
        ostream.write((const char *)&passive, sizeof(bool));

    if (!ostream.fail()) {
        size_t elementCount = tetrahedralMesh->size_tetrahedra();
        ostream.write((const char *)&elementCount, sizeof(size_t));
    }

    return ostream.fail() ? MS::kFailure : MS::kSuccess;
}

FEMObjectData & FEMObjectData::reset() {
    deallocate();
    allocate();

    enable = true;
    passive = false;

    massDamping = 2.0;
    stiffnessDamping = 0;
    friction = 0.5;

    return *this;
}

FEMObjectData & FEMObjectData::initialize(FEMParameters & parameters) {
    enable = parameters.enable;
    passive = parameters.passive;

    massDamping = parameters.massDamping;
    stiffnessDamping = parameters.stiffnessDamping;
    friction = parameters.friction;

    MFnMesh mesh(parameters.meshObject);
    MPoint point;

    for (int i = 0; i < mesh.numVertices(); i++) {
        mesh.getPoint(i, point);
        point *= parameters.matrix;

        tetrahedralMesh->insert(FEMVector(point.x, point.y, point.z));
    }

    MFnIntArrayData arrayData;

    arrayData.setObject(parameters.surfaceNodesObject);
    surfaceNodes->copy(arrayData.array());

    arrayData.setObject(parameters.volumeNodesObject);

    for (unsigned int i = 0; i < arrayData.length() / 4; i++) {
        tetrahedralMesh->insert(arrayData[i * 4], arrayData[i * 4 + 1],
            arrayData[i * 4 + 2], arrayData[i * 4 + 3]);
    }

    if (passive)
        opentissue::fem::set_fixed(*tetrahedralMesh, true);
    else {
        opentissue::fem::initialize(*tetrahedralMesh, parameters.density,
            parameters.poissonsRatio, parameters.youngsModulus,
            parameters.minimumYieldStrength, parameters.maximumYieldStrength,
            parameters.creepRate);

        const MVector & v = parameters.initialVelocity;
        const MVector & a = parameters.initialAngularVelocity;

        opentissue::fem::set_velocity(*tetrahedralMesh, FEMVector(v.x, v.y, v.z));
        opentissue::fem::set_angular_velocity(*tetrahedralMesh, FEMVector(a.x, a.y, a.z));
    }

    return *this;
}
FEMObjectData & FEMObjectData::update(FEMParameters & parameters, const MPxData & source) {
    if (parameters.updateMesh)
        return initialize(parameters);

    copy(source);

    if (parameters.updateParameters) {
        enable = parameters.enable;
        passive = parameters.passive;

        massDamping = parameters.massDamping;
        stiffnessDamping = parameters.stiffnessDamping;
        friction = parameters.friction;

        opentissue::fem::set_fixed(*tetrahedralMesh, passive);

        if (!passive) {
            opentissue::fem::update_parameters(*tetrahedralMesh, parameters.density,
                parameters.poissonsRatio, parameters.youngsModulus,
                parameters.minimumYieldStrength, parameters.maximumYieldStrength,
                parameters.creepRate);
        }
    }

    return *this;
}

FEMTetrahedralMesh * FEMObjectData::getTetrahedralMesh() {
    return tetrahedralMesh;
}
const FEMTetrahedralMesh * FEMObjectData::getTetrahedralMesh() const {
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

FEMObjectData & FEMObjectData::allocate() {
    tetrahedralMesh = new FEMTetrahedralMesh();
    surfaceNodes = new MIntArray();

    return *this;
}
FEMObjectData & FEMObjectData::deallocate() {
    if (tetrahedralMesh != nullptr)
        delete tetrahedralMesh;

    if (surfaceNodes != nullptr)
        delete surfaceNodes;

    return *this;
}