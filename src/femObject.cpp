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

#include "femObject.h"

#include <maya/MFnNumericAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnNumericData.h>
#include <maya/MFnMeshData.h>
#include <maya/MFnIntArrayData.h>
#include <maya/MFnPluginData.h>
#include <maya/MDataHandle.h>
#include <maya/MFnMesh.h>
#include <maya/MIntArray.h>
#include <maya/MPointArray.h>
#include <maya/MItMeshVertex.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MString.h>
#include <maya/MGlobal.h>

MObject FEMObject::enableObject;
MObject FEMObject::densityObject;
MObject FEMObject::poissonsRatioObject;
MObject FEMObject::youngsModulusObject;
MObject FEMObject::massDampingObject;
MObject FEMObject::stiffnessDampingObject;
MObject FEMObject::minimumYieldStrengthObject;
MObject FEMObject::maximumYieldStrengthObject;
MObject FEMObject::creepRateObject;
MObject FEMObject::frictionObject;
MObject FEMObject::passiveObject;
MObject FEMObject::initialVelocityXObject;
MObject FEMObject::initialVelocityYObject;
MObject FEMObject::initialVelocityZObject;
MObject FEMObject::initialVelocityObject;
MObject FEMObject::initialAngularVelocityXObject;
MObject FEMObject::initialAngularVelocityYObject;
MObject FEMObject::initialAngularVelocityZObject;
MObject FEMObject::initialAngularVelocityObject;
MObject FEMObject::startTimeObject;
MObject FEMObject::currentTimeObject;
MObject FEMObject::inputMeshObject;
MObject FEMObject::surfaceNodesObject;
MObject FEMObject::volumeNodesObject;
MObject FEMObject::matrixObject;
MObject FEMObject::outputMeshObject;
MObject FEMObject::nextStateObject;
MObject FEMObject::currentStateObject;

const MTypeId FEMObject::id(0x00128582);
const MString FEMObject::typeName("femObject");

FEMObject::FEMObject() {}
FEMObject::~FEMObject() {}

void * FEMObject::creator() {
    return new FEMObject;
}
MStatus FEMObject::initialize() {
    MFnNumericAttribute numericAttribute;
    MFnUnitAttribute unitAttribute;
    MFnTypedAttribute typedAttribute;
    MFnMatrixAttribute matrixAttribute;

    enableObject = numericAttribute.create("enable", "e", MFnNumericData::kBoolean, true);
    numericAttribute.setStorable(true);
    numericAttribute.setKeyable(true);

    densityObject = numericAttribute.create("density", "d", MFnNumericData::kDouble, 1000.0);
    numericAttribute.setMin(1.0e-3);
    numericAttribute.setSoftMax(2000.0);
    numericAttribute.setStorable(true);
    numericAttribute.setKeyable(true);

    poissonsRatioObject = numericAttribute.create("poissonsRatio", "pr", MFnNumericData::kDouble, 0);
    numericAttribute.setSoftMin(-10.0);
    numericAttribute.setSoftMax(10);
    numericAttribute.setStorable(true);
    numericAttribute.setKeyable(true);

    youngsModulusObject = numericAttribute.create("youngsModulus", "ym", MFnNumericData::kDouble, 1.0e5);
    numericAttribute.setMin(0);
    numericAttribute.setSoftMax(1.0e6);
    numericAttribute.setStorable(true);
    numericAttribute.setKeyable(true);

    massDampingObject = numericAttribute.create("massDamping", "md", MFnNumericData::kDouble, 0.5);
    numericAttribute.setMin(0);
    numericAttribute.setSoftMax(1.0);
    numericAttribute.setStorable(true);
    numericAttribute.setKeyable(true);

    stiffnessDampingObject = numericAttribute.create("stiffnessDamping", "sd",
        MFnNumericData::kDouble, 0.1);
    numericAttribute.setMin(0);
    numericAttribute.setSoftMax(1.0);
    numericAttribute.setStorable(true);
    numericAttribute.setKeyable(true);

    minimumYieldStrengthObject = numericAttribute.create("minimumYieldStrength", "miny",
        MFnNumericData::kDouble, 0);
    numericAttribute.setMin(0);
    numericAttribute.setMax(1.0);
    numericAttribute.setStorable(true);
    numericAttribute.setKeyable(true);

    maximumYieldStrengthObject = numericAttribute.create("maximumYieldStrength", "maxy",
        MFnNumericData::kDouble, 0);
    numericAttribute.setMin(0);
    numericAttribute.setMax(1.0);
    numericAttribute.setStorable(true);
    numericAttribute.setKeyable(true);

    creepRateObject = numericAttribute.create("creepRate", "c", MFnNumericData::kDouble, 0);
    numericAttribute.setMin(0);
    numericAttribute.setMax(1.0);
    numericAttribute.setStorable(true);
    numericAttribute.setKeyable(true);

    frictionObject = numericAttribute.create("friction", "f", MFnNumericData::kDouble, 0.5);
    numericAttribute.setMin(0);
    numericAttribute.setMax(1.0);
    numericAttribute.setStorable(true);
    numericAttribute.setKeyable(true);

    passiveObject = numericAttribute.create("passive", "p", MFnNumericData::kBoolean, false);
    numericAttribute.setStorable(true);
    numericAttribute.setKeyable(true);

    initialVelocityXObject = numericAttribute.create("initialVelocityX", "vx", MFnNumericData::kDouble, 0);
    numericAttribute.setStorable(true);
    numericAttribute.setKeyable(true);

    initialVelocityYObject = numericAttribute.create("initialVelocityY", "vy", MFnNumericData::kDouble, 0);
    numericAttribute.setStorable(true);
    numericAttribute.setKeyable(true);

    initialVelocityZObject = numericAttribute.create("initialVelocityZ", "vz", MFnNumericData::kDouble, 0);
    numericAttribute.setStorable(true);
    numericAttribute.setKeyable(true);

    initialVelocityObject = numericAttribute.create("initialVelocity", "v",
        initialVelocityXObject, initialVelocityYObject, initialVelocityZObject);
    numericAttribute.setDefault(0, 0, 0);
    numericAttribute.setStorable(true);
    numericAttribute.setKeyable(true);

    initialAngularVelocityXObject = numericAttribute.create("initialAngularVelocityX", "ax",
        MFnNumericData::kDouble, 0);
    unitAttribute.setStorable(true);
    unitAttribute.setKeyable(true);

    initialAngularVelocityYObject = numericAttribute.create("initialAngularVelocityY", "ay",
        MFnNumericData::kDouble, 0);
    unitAttribute.setStorable(true);
    unitAttribute.setKeyable(true);

    initialAngularVelocityZObject = numericAttribute.create("initialAngularVelocityZ", "az",
        MFnNumericData::kDouble, 0);
    unitAttribute.setStorable(true);
    unitAttribute.setKeyable(true);

    initialAngularVelocityObject = numericAttribute.create("initialAngularVelocity", "a",
        initialAngularVelocityXObject, initialAngularVelocityYObject, initialAngularVelocityZObject);
    numericAttribute.setDefault(0, 0, 0);
    numericAttribute.setStorable(true);
    numericAttribute.setKeyable(true);

    startTimeObject = unitAttribute.create("startTime", "st", MFnUnitAttribute::kTime, 0);
    unitAttribute.setStorable(true);

    currentTimeObject = unitAttribute.create("currentTime", "t", MFnUnitAttribute::kTime, 0);
    unitAttribute.setStorable(true);

    inputMeshObject = typedAttribute.create("inputMesh", "in", MFnMeshData::kMesh);
    typedAttribute.setStorable(true);

    surfaceNodesObject = typedAttribute.create("surfaceNodes", "sn", MFnIntArrayData::kIntArray);
    typedAttribute.setStorable(true);

    volumeNodesObject = typedAttribute.create("volumeNodes", "vn", MFnIntArrayData::kIntArray);
    typedAttribute.setStorable(true);

    matrixObject = matrixAttribute.create("matrix", "m", MFnMatrixAttribute::kDouble);
    matrixAttribute.setStorable(true);

    outputMeshObject = typedAttribute.create("outputMesh", "out", MFnMeshData::kMesh);
    typedAttribute.setWritable(false);
    typedAttribute.setStorable(false);

    nextStateObject = typedAttribute.create("nextState", "ns", MFnPluginData::kPlugin);
    typedAttribute.setStorable(true);

    currentStateObject = typedAttribute.create("currentState", "cs", MFnPluginData::kPlugin);
    typedAttribute.setWritable(false);
    typedAttribute.setStorable(false);

    addAttribute(enableObject);
    addAttribute(densityObject);
    addAttribute(poissonsRatioObject);
    addAttribute(youngsModulusObject);
    addAttribute(massDampingObject);
    addAttribute(stiffnessDampingObject);
    addAttribute(minimumYieldStrengthObject);
    addAttribute(maximumYieldStrengthObject);
    addAttribute(creepRateObject);
    addAttribute(frictionObject);
    addAttribute(passiveObject);
    addAttribute(initialVelocityObject);
    addAttribute(initialAngularVelocityObject);
    addAttribute(startTimeObject);
    addAttribute(currentTimeObject);
    addAttribute(inputMeshObject);
    addAttribute(surfaceNodesObject);
    addAttribute(volumeNodesObject);
    addAttribute(matrixObject);
    addAttribute(outputMeshObject);
    addAttribute(nextStateObject);
    addAttribute(currentStateObject);

    attributeAffects(enableObject, currentStateObject);
    attributeAffects(densityObject, currentStateObject);
    attributeAffects(poissonsRatioObject, currentStateObject);
    attributeAffects(youngsModulusObject, currentStateObject);
    attributeAffects(massDampingObject, currentStateObject);
    attributeAffects(stiffnessDampingObject, currentStateObject);
    attributeAffects(minimumYieldStrengthObject, currentStateObject);
    attributeAffects(maximumYieldStrengthObject, currentStateObject);
    attributeAffects(creepRateObject, currentStateObject);
    attributeAffects(frictionObject, currentStateObject);
    attributeAffects(passiveObject, currentStateObject);
    attributeAffects(initialVelocityObject, currentStateObject);
    attributeAffects(initialAngularVelocityObject, currentStateObject);
    attributeAffects(startTimeObject, outputMeshObject);
    attributeAffects(startTimeObject, currentStateObject);
    attributeAffects(currentTimeObject, outputMeshObject);
    attributeAffects(currentTimeObject, currentStateObject);
    attributeAffects(inputMeshObject, currentStateObject);
    attributeAffects(surfaceNodesObject, currentStateObject);
    attributeAffects(volumeNodesObject, currentStateObject);
    attributeAffects(matrixObject, currentStateObject);
    attributeAffects(nextStateObject, outputMeshObject);
    attributeAffects(nextStateObject, currentStateObject);

    return MS::kSuccess;
}
MStatus FEMObject::compute(const MPlug & plug, MDataBlock & data) {
    MStatus status;

    if (plug != outputMeshObject && plug != currentStateObject)
        return MS::kUnknownParameter;

    MFnPluginData pluginData;
    pluginData.create(FEMObjectData::id);

    FEMObjectData * objectData = (FEMObjectData *)pluginData.data();

    MDataHandle currentStateHandle = data.outputValue(currentStateObject, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    currentStateHandle.set(objectData);

    MFnMeshData meshData;
    MObject outputMesh = meshData.create();

    MDataHandle outputMeshHandle = data.outputValue(outputMeshObject, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    outputMeshHandle.set(outputMesh);

    int startTime = data.inputValue(startTimeObject).asInt();
    int currentTime = data.inputValue(currentTimeObject).asInt();

    FEMParameters parameters;

    parameters.enable = data.inputValue(enableObject).asBool();
    parameters.density = data.inputValue(densityObject).asDouble();
    parameters.poissonsRatio = data.inputValue(poissonsRatioObject).asDouble();
    parameters.youngsModulus = data.inputValue(youngsModulusObject).asDouble();
    parameters.massDamping = data.inputValue(massDampingObject).asDouble();
    parameters.stiffnessDamping = data.inputValue(stiffnessDampingObject).asDouble();
    parameters.minimumYieldStrength = data.inputValue(minimumYieldStrengthObject).asDouble();
    parameters.maximumYieldStrength = data.inputValue(maximumYieldStrengthObject).asDouble();
    parameters.creepRate = data.inputValue(creepRateObject).asDouble();
    parameters.friction = data.inputValue(frictionObject).asDouble();
    parameters.passive = data.inputValue(passiveObject).asBool();
    parameters.initialVelocity = data.inputValue(initialVelocityObject).asVector();
    parameters.initialAngularVelocity = data.inputValue(initialAngularVelocityObject).asVector();

    MDataHandle surfaceNodesHandle = data.inputValue(surfaceNodesObject, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    parameters.surfaceNodesObject = surfaceNodesHandle.data();

    MDataHandle volumeNodesHandle = data.inputValue(volumeNodesObject, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    parameters.volumeNodesObject = volumeNodesHandle.data();

    if (parameters.passive || currentTime == startTime) {
        MDataHandle inputMeshHandle = data.inputValue(inputMeshObject, &status);
        CHECK_MSTATUS_AND_RETURN_IT(status);

        MDataHandle matrixHandle = data.inputValue(matrixObject, &status);
        CHECK_MSTATUS_AND_RETURN_IT(status);

        MObject meshObject = inputMeshHandle.asMesh();

        if (meshObject.isNull())
            return MS::kFailure;

        transformMesh(meshObject, matrixHandle.asMatrix());

        parameters.meshObject = meshObject;
        objectData->initialize(parameters);
    }
    else if (currentTime > startTime) {
        MDataHandle nextStateHandle = data.inputValue(nextStateObject, &status);
        CHECK_MSTATUS_AND_RETURN_IT(status);

        objectData->update(parameters, *nextStateHandle.asPluginData());
    }

    createOutputMesh(objectData, outputMesh);

    data.setClean(plug);

    return MS::kSuccess;
}

void FEMObject::createOutputMesh(const FEMObjectData * objectData,
    MObject & meshObject) const {
    const MIntArray * surfaceNodes = objectData->getSurfaceNodes();
    const TetrahedralMesh * tetrahedralMesh = objectData->getTetrahedralMesh();

    int pointCount = (int)tetrahedralMesh->size_nodes();
    unsigned int surfaceCount = surfaceNodes->length() / 3;

    MPointArray points;
    points.setLength(pointCount);

#pragma omp parallel for

    for (int i = 0; i < pointCount; i++) {
        const Vector & p = (tetrahedralMesh->node_begin() + i)->m_coord;

        points[i].x = p[0];
        points[i].y = p[1];
        points[i].z = p[2];
    }

    MFnMesh mesh;

    mesh.create(pointCount, surfaceCount, points,
        MIntArray(surfaceCount, 3), *surfaceNodes, meshObject);
}
void FEMObject::transformMesh(MObject & meshObject, const MMatrix & matrix) const {
    MFnMesh mesh(meshObject);

#pragma omp parallel for

    for (int i = 0; i < mesh.numVertices(); i++) {
        MPoint point;
        mesh.getPoint(i, point);

        mesh.setPoint(i, point * matrix);
    }
}