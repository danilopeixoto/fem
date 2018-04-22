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

#include <femObject.h>

#include <maya/MFnNumericAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnNumericData.h>
#include <maya/MFnMeshData.h>
#include <maya/MFnIntArrayData.h>
#include <maya/MFnPluginData.h>
#include <maya/MDataHandle.h>
#include <maya/MTime.h>
#include <maya/MFnMesh.h>
#include <maya/MIntArray.h>
#include <maya/MPointArray.h>

#define FEM_COMPARE_ATTR(attribute) (plug == attribute)

MObject FEMObject::enableObject;
MObject FEMObject::passiveObject;
MObject FEMObject::densityObject;
MObject FEMObject::poissonsRatioObject;
MObject FEMObject::youngsModulusObject;
MObject FEMObject::massDampingObject;
MObject FEMObject::stiffnessDampingObject;
MObject FEMObject::minimumYieldStrengthObject;
MObject FEMObject::maximumYieldStrengthObject;
MObject FEMObject::creepRateObject;
MObject FEMObject::frictionObject;
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

FEMObject::FEMObject() : updateInitialData(false) {}
FEMObject::~FEMObject() {}

void * FEMObject::creator() {
    return new FEMObject();
}
MStatus FEMObject::initialize() {
    MFnNumericAttribute numericAttribute;
    MFnUnitAttribute unitAttribute;
    MFnTypedAttribute typedAttribute;
    MFnMatrixAttribute matrixAttribute;

    enableObject = numericAttribute.create("enable", "e", MFnNumericData::kBoolean, true);
    numericAttribute.setStorable(true);
    numericAttribute.setKeyable(true);

    passiveObject = numericAttribute.create("passive", "p", MFnNumericData::kBoolean, false);
    numericAttribute.setStorable(true);
    numericAttribute.setKeyable(true);

    densityObject = numericAttribute.create("density", "d", MFnNumericData::kDouble, 1000.0);
    numericAttribute.setMin(1.0e-3);
    numericAttribute.setSoftMax(2000.0);
    numericAttribute.setStorable(true);
    numericAttribute.setKeyable(true);

    poissonsRatioObject = numericAttribute.create("poissonsRatio", "pr", MFnNumericData::kDouble, 0);
    numericAttribute.setMin(-0.499);
    numericAttribute.setMax(0.499);
    numericAttribute.setStorable(true);
    numericAttribute.setKeyable(true);

    youngsModulusObject = numericAttribute.create("youngsModulus", "ym", MFnNumericData::kDouble, 1.0e5);
    numericAttribute.setMin(0);
    numericAttribute.setSoftMax(1.0e5);
    numericAttribute.setStorable(true);
    numericAttribute.setKeyable(true);

    massDampingObject = numericAttribute.create("massDamping", "md", MFnNumericData::kDouble, 2.0);
    numericAttribute.setMin(0);
    numericAttribute.setSoftMax(10.0);
    numericAttribute.setStorable(true);
    numericAttribute.setKeyable(true);

    stiffnessDampingObject = numericAttribute.create("stiffnessDamping", "sd",
        MFnNumericData::kDouble, 0);
    numericAttribute.setMin(0);
    numericAttribute.setSoftMax(10.0);
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
    addAttribute(passiveObject);
    addAttribute(densityObject);
    addAttribute(poissonsRatioObject);
    addAttribute(youngsModulusObject);
    addAttribute(massDampingObject);
    addAttribute(stiffnessDampingObject);
    addAttribute(minimumYieldStrengthObject);
    addAttribute(maximumYieldStrengthObject);
    addAttribute(creepRateObject);
    addAttribute(frictionObject);
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

    attributeAffects(currentTimeObject, currentStateObject);
    attributeAffects(currentTimeObject, outputMeshObject);

    return MS::kSuccess;
}
MStatus	FEMObject::setDependentsDirty(const MPlug & plug, MPlugArray & plugArray) {
    if (FEM_COMPARE_ATTR(enableObject) || FEM_COMPARE_ATTR(passiveObject)
        || FEM_COMPARE_ATTR(densityObject) || FEM_COMPARE_ATTR(poissonsRatioObject)
        || FEM_COMPARE_ATTR(youngsModulusObject) || FEM_COMPARE_ATTR(massDampingObject)
        || FEM_COMPARE_ATTR(stiffnessDampingObject) || FEM_COMPARE_ATTR(minimumYieldStrengthObject)
        || FEM_COMPARE_ATTR(maximumYieldStrengthObject) || FEM_COMPARE_ATTR(creepRateObject)
        || FEM_COMPARE_ATTR(frictionObject))
        parameters.updateParameters = true;

    if (FEM_COMPARE_ATTR(inputMeshObject) || FEM_COMPARE_ATTR(surfaceNodesObject)
        || FEM_COMPARE_ATTR(volumeNodesObject) || FEM_COMPARE_ATTR(matrixObject))
        parameters.updateMesh = true;

    if (updateInitialData && (parameters.updateParameters || parameters.updateMesh)) {
        MObject nodeObject(thisMObject());

        MPlug currentStatePlug(nodeObject, currentStateObject);
        MPlug outputMeshPlug(nodeObject, outputMeshObject);

        plugArray.append(currentStatePlug);
        plugArray.append(outputMeshPlug);
    }

    return MPxNode::setDependentsDirty(plug, plugArray);
}
MStatus FEMObject::compute(const MPlug & plug, MDataBlock & data) {
    if (plug != outputMeshObject && plug != currentStateObject)
        return MS::kUnknownParameter;

    MStatus status;

    MDataHandle startTimeHandle = data.inputValue(startTimeObject, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MDataHandle currentTimeHandle = data.inputValue(currentTimeObject, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MTime startTime = startTimeHandle.asTime();
    MTime currentTime = currentTimeHandle.asTime();

    if (currentTime < startTime)
        return MS::kFailure;

    MDataHandle enableHandle = data.inputValue(enableObject, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    if (!enableHandle.asBool())
        return MS::kFailure;

    MDataHandle passiveHandle = data.inputValue(passiveObject, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MDataHandle densityHandle = data.inputValue(densityObject, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MDataHandle poissonsRatioHandle = data.inputValue(poissonsRatioObject, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MDataHandle youngsModulusHandle = data.inputValue(youngsModulusObject, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MDataHandle massDampingHandle = data.inputValue(massDampingObject, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MDataHandle stiffnessDampingHandle = data.inputValue(stiffnessDampingObject, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MDataHandle minimumYieldStrengthHandle = data.inputValue(minimumYieldStrengthObject, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MDataHandle maximumYieldStrengthHandle = data.inputValue(maximumYieldStrengthObject, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MDataHandle creepRateHandle = data.inputValue(creepRateObject, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MDataHandle frictionHandle = data.inputValue(frictionObject, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MDataHandle inputMeshHandle = data.inputValue(inputMeshObject, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MDataHandle surfaceNodesHandle = data.inputValue(surfaceNodesObject, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MDataHandle volumeNodesHandle = data.inputValue(volumeNodesObject, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MDataHandle matrixHandle = data.inputValue(matrixObject, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MDataHandle outputMeshHandle = data.outputValue(outputMeshObject, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MDataHandle currentStateHandle = data.outputValue(currentStateObject, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    parameters.enable = enableHandle.asBool();
    parameters.passive = passiveHandle.asBool();
    parameters.density = densityHandle.asDouble();
    parameters.poissonsRatio = poissonsRatioHandle.asDouble();
    parameters.youngsModulus = youngsModulusHandle.asDouble();
    parameters.massDamping = massDampingHandle.asDouble();
    parameters.stiffnessDamping = stiffnessDampingHandle.asDouble();
    parameters.minimumYieldStrength = minimumYieldStrengthHandle.asDouble();
    parameters.maximumYieldStrength = maximumYieldStrengthHandle.asDouble();
    parameters.creepRate = creepRateHandle.asDouble();
    parameters.friction = frictionHandle.asDouble();
    parameters.surfaceNodesObject = surfaceNodesHandle.data();
    parameters.volumeNodesObject = volumeNodesHandle.data();
    parameters.matrix = matrixHandle.asMatrix();
    parameters.meshObject = inputMeshHandle.asMesh();

    if (parameters.meshObject.isNull())
        return MS::kFailure;

    MObject outputMesh = outputMeshHandle.asMesh();
    FEMObjectData * currentStateData = (FEMObjectData *)currentStateHandle.asPluginData();

    if (outputMesh.isNull()) {
        MFnMeshData meshData;
        outputMesh = meshData.create();
    }

    MFnPluginData pluginData;

    if (currentStateData == nullptr) {
        pluginData.create(FEMObjectData::id);
        currentStateData = (FEMObjectData *)pluginData.data();
    }

    if (currentTime == startTime) {
        updateInitialData = true;

        MDataHandle initialVelocityHandle = data.inputValue(initialVelocityObject, &status);
        CHECK_MSTATUS_AND_RETURN_IT(status);

        MDataHandle initialAngularVelocityHandle = data.inputValue(initialAngularVelocityObject, &status);
        CHECK_MSTATUS_AND_RETURN_IT(status);

        parameters.initialVelocity = initialVelocityHandle.asVector();
        parameters.initialAngularVelocity = initialAngularVelocityHandle.asVector();

        currentStateData->initialize(parameters);
        updateOutputMesh(currentStateData, outputMesh);
    }
    else {
        updateInitialData = false;

        MDataHandle nextStateHandle = data.inputValue(nextStateObject, &status);
        CHECK_MSTATUS_AND_RETURN_IT(status);

        const FEMObjectData * nextStateData = (const FEMObjectData *)nextStateHandle.asPluginData();

        if (nextStateData == nullptr)
            return MS::kFailure;

        currentStateData->update(parameters, *nextStateData);
        updateOutputMesh(currentStateData, outputMesh);

        parameters.updateParameters = false;
        parameters.updateMesh = false;
    }

    outputMeshHandle.set(outputMesh);
    currentStateHandle.set(currentStateData);

    data.setClean(plug);

    return MS::kSuccess;
}

void FEMObject::updateOutputMesh(const FEMObjectData * objectData,
    MObject & meshObject) const {
    const MIntArray * surfaceNodes = objectData->getSurfaceNodes();
    const FEMTetrahedralMesh * tetrahedralMesh = objectData->getTetrahedralMesh();

    int pointCount = (int)tetrahedralMesh->size_nodes();
    int surfaceCount = surfaceNodes->length() / 3;

    MPointArray points;
    points.setLength(pointCount);

    for (int i = 0; i < pointCount; i++) {
        const FEMVector & p = tetrahedralMesh->const_node(i)->m_world_coord;

        points[i].x = p[0];
        points[i].y = p[1];
        points[i].z = p[2];
    }

    MFnMesh mesh;

    mesh.create(pointCount, surfaceCount, points,
        MIntArray(surfaceCount, 3), *surfaceNodes, meshObject);

    for (int i = 0; i < mesh.numEdges(); i++)
        mesh.setEdgeSmoothing(i, false);

    mesh.cleanupEdgeSmoothing();
}