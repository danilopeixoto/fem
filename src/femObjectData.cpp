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
        boundaryVolumesObject = parameters.boundaryVolumesObject;
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

    collisionObject->setFriction(friction);
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
        boundaryVolumes->copy(*objectData.getBoundaryVolumes());
        collisionObject->createShape(tetrahedralMesh, surfaceNodes, boundaryVolumes);

        enable = objectData.isEnable();
        passive = objectData.isPassive();

        massDamping = objectData.getMassDamping();
        stiffnessDamping = objectData.getStiffnessDamping();
        friction = objectData.getFriction();

        collisionObject->setFriction(friction);
    }
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
    tetrahedralMesh->clear();
    surfaceNodes->clear();
    boundaryVolumes->clear();
    collisionObject->deleteShape();

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

    arrayData.setObject(parameters.boundaryVolumesObject);
    boundaryVolumes->copy(arrayData.array());

    arrayData.setObject(parameters.volumeNodesObject);

    for (unsigned int i = 0; i < arrayData.length() / 4; i++) {
        tetrahedralMesh->insert(arrayData[i * 4], arrayData[i * 4 + 1],
            arrayData[i * 4 + 2], arrayData[i * 4 + 3]);
    }

    collisionObject->createShape(tetrahedralMesh, surfaceNodes, boundaryVolumes);
    collisionObject->setFriction(friction);

    if (passive) {
        opentissue::fem::set_fixed(*tetrahedralMesh, true);
        opentissue::fem::reset_world_coordinate(*tetrahedralMesh);
    }
    else {
        opentissue::fem::initialize(*tetrahedralMesh, parameters.density,
            parameters.poissonsRatio, parameters.youngsModulus,
            parameters.minimumYieldStrength, parameters.maximumYieldStrength,
            parameters.creepRate);

        const MVector & velocity = parameters.initialVelocity;
        const MVector & angularVelocity = parameters.initialAngularVelocity;

        opentissue::fem::set_velocity(*tetrahedralMesh,
            FEMVector(velocity.x, velocity.y, velocity.z));
        opentissue::fem::set_angular_velocity(*tetrahedralMesh,
            FEMVector(angularVelocity.x, angularVelocity.y, angularVelocity.z));
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
MIntArray * FEMObjectData::getBoundaryVolumes() {
    return boundaryVolumes;
}
const MIntArray * FEMObjectData::getBoundaryVolumes() const {
    return boundaryVolumes;
}
FEMCollisionObject * FEMObjectData::getCollisionObject() {
    return collisionObject;
}
const FEMCollisionObject * FEMObjectData::getCollisionObject() const {
    return collisionObject;
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

void FEMObjectData::allocate() {
    tetrahedralMesh = new FEMTetrahedralMesh();
    surfaceNodes = new MIntArray();
    boundaryVolumes = new MIntArray();
    collisionObject = new FEMCollisionObject();
}

void FEMObjectData::deallocate() {
    if (tetrahedralMesh != nullptr) {
        delete tetrahedralMesh;
        tetrahedralMesh = nullptr;
    }

    if (surfaceNodes != nullptr) {
        delete surfaceNodes;
        surfaceNodes = nullptr;
    }

    if (boundaryVolumes != nullptr) {
        delete boundaryVolumes;
        boundaryVolumes = nullptr;
    }

    if (collisionObject != nullptr) {
        delete collisionObject;
        collisionObject = nullptr;
    }
}