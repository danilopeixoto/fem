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

#include <femPlugin.h>
#include <femSolver.h>

#include <maya/MFnNumericAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnNumericData.h>
#include <maya/MFnPluginData.h>
#include <maya/MDataHandle.h>
#include <maya/MArrayDataHandle.h>
#include <maya/MArrayDataBuilder.h>
#include <maya/MTime.h>

MObject FEMSolver::enableObject;
MObject FEMSolver::startTimeObject;
MObject FEMSolver::currentTimeObject;
MObject FEMSolver::substepsObject;
MObject FEMSolver::gravityXObject;
MObject FEMSolver::gravityYObject;
MObject FEMSolver::gravityZObject;
MObject FEMSolver::gravityObject;
MObject FEMSolver::maximumIterationsObject;
MObject FEMSolver::currentStateObject;
MObject FEMSolver::outputStateObject;

const MTypeId FEMSolver::id(0x00128583);
const MString FEMSolver::typeName("femSolver");

FEMSolver::FEMSolver() {}
FEMSolver::~FEMSolver() {}

void * FEMSolver::creator() {
    return new FEMSolver();
}
MStatus FEMSolver::initialize() {
    MFnNumericAttribute numericAttribute;
    MFnUnitAttribute unitAttribute;
    MFnTypedAttribute typedAttribute;

    enableObject = numericAttribute.create("enable", "e", MFnNumericData::kBoolean, true);
    numericAttribute.setStorable(true);
    numericAttribute.setKeyable(true);

    startTimeObject = unitAttribute.create("startTime", "st", MFnUnitAttribute::kTime, 1);
    unitAttribute.setStorable(true);

    currentTimeObject = unitAttribute.create("currentTime", "t", MFnUnitAttribute::kTime, 0);
    unitAttribute.setStorable(true);

    substepsObject = numericAttribute.create("substeps", "s", MFnNumericData::kInt, 2);
    numericAttribute.setMin(0);
    numericAttribute.setSoftMax(10);
    numericAttribute.setStorable(true);
    numericAttribute.setKeyable(true);

    gravityXObject = numericAttribute.create("gravityX", "gx", MFnNumericData::kDouble, 0);
    numericAttribute.setStorable(true);
    numericAttribute.setKeyable(true);

    gravityYObject = numericAttribute.create("gravityY", "gy", MFnNumericData::kDouble, -9.8);
    numericAttribute.setStorable(true);
    numericAttribute.setKeyable(true);

    gravityZObject = numericAttribute.create("gravityZ", "gz", MFnNumericData::kDouble, 0);
    numericAttribute.setStorable(true);
    numericAttribute.setKeyable(true);

    gravityObject = numericAttribute.create("gravity", "g", gravityXObject, gravityYObject, gravityZObject);
    numericAttribute.setDefault(0.0, -9.8, 0.0);
    numericAttribute.setStorable(true);
    numericAttribute.setKeyable(true);

    maximumIterationsObject = numericAttribute.create("maximumIterations", "mi",
        MFnNumericData::kInt, 10);
    numericAttribute.setMin(1);
    numericAttribute.setSoftMax(10);
    numericAttribute.setStorable(true);
    numericAttribute.setKeyable(true);

    currentStateObject = typedAttribute.create("currentState", "cs", MFnPluginData::kPlugin);
    typedAttribute.setStorable(true);
    typedAttribute.setArray(true);
    typedAttribute.setUsesArrayDataBuilder(true);

    outputStateObject = typedAttribute.create("outputState", "os", MFnPluginData::kPlugin);
    typedAttribute.setWritable(false);
    typedAttribute.setStorable(true);
    typedAttribute.setArray(true);
    typedAttribute.setUsesArrayDataBuilder(true);

    addAttribute(enableObject);
    addAttribute(startTimeObject);
    addAttribute(currentTimeObject);
    addAttribute(substepsObject);
    addAttribute(gravityObject);
    addAttribute(maximumIterationsObject);
    addAttribute(currentStateObject);
    addAttribute(outputStateObject);

    return MS::kSuccess;
}
void FEMSolver::postConstructor() {
    MPlug startTimePlug(thisMObject(), startTimeObject);
    startTimePlug.setMTime(FEMPlugin::getStartTime());
}
MStatus FEMSolver::setDependentsDirty(const MPlug & plug, MPlugArray & plugArray) {
    if (plug == currentStateObject) {
        MPlug outputStatePlug(thisMObject(), outputStateObject);

        plugArray.append(outputStatePlug);

        if (plug.isElement()) {
            MPlug objectStatePlug = outputStatePlug.elementByLogicalIndex(plug.logicalIndex());
            plugArray.append(objectStatePlug);
        }
        else {
            for (unsigned int i = 0; i < outputStatePlug.numElements(); i++) {
                MPlug objectStatePlug = outputStatePlug.elementByPhysicalIndex(i);
                plugArray.append(objectStatePlug);
            }
        }
    }

    return MPxNode::setDependentsDirty(plug, plugArray);
}
MStatus FEMSolver::compute(const MPlug & plug, MDataBlock & data) {
    if (plug != outputStateObject)
        return MS::kUnknownParameter;

    MStatus status;

    MDataHandle enableHandle = data.inputValue(enableObject, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    if (!enableHandle.asBool())
        return MS::kFailure;

    MDataHandle startTimeHandle = data.inputValue(startTimeObject);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MDataHandle currentTimeHandle = data.inputValue(currentTimeObject);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MTime startTime = startTimeHandle.asTime();
    MTime currentTime = currentTimeHandle.asTime();

    if (currentTime < startTime)
        return MS::kFailure;

    MDataHandle substepsHandle = data.inputValue(substepsObject);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MDataHandle gravityHandle = data.inputValue(gravityObject, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MDataHandle maximumIterationsHandle = data.inputValue(maximumIterationsObject, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MArrayDataHandle currentStateArrayHandle = data.inputArrayValue(currentStateObject, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MArrayDataHandle outputStateArrayHandle = data.outputArrayValue(outputStateObject, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    int substeps = substepsHandle.asInt();
    MVector gravity = gravityHandle.asVector();

    int maximumIterations = maximumIterationsHandle.asInt();

    int steps = substeps + 1;
    double timestep = 1.0 / (steps * FEMPlugin::getFramerate());

    unsigned int objectCount = currentStateArrayHandle.elementCount();
    MArrayDataBuilder arrayDataBuilder(outputStateArrayHandle.builder());

    computation.beginComputation();

    for (unsigned int i = 0; i < objectCount; i++) {
        if (computation.isInterruptRequested()) {
            computation.endComputation();
            return MS::kFailure;
        }

        MDataHandle currentStateHandle = currentStateArrayHandle.inputValue(&status);
        CHECK_MSTATUS_AND_RETURN_IT(status);

        MDataHandle outputStateHandle = arrayDataBuilder.addElement(i, &status);
        CHECK_MSTATUS_AND_RETURN_IT(status);

        FEMObjectData * objectData = (FEMObjectData *)currentStateHandle.asPluginData();

        if (objectData == nullptr)
            return MS::kFailure;

        if (objectData->isEnable()) {
            for (int s = 0; s < steps; s++) {
                if (computation.isInterruptRequested()) {
                    computation.endComputation();
                    return MS::kFailure;
                }

                if (!objectData->isPassive())
                    simulateSubstep(objectData, gravity, timestep, maximumIterations);
            }
        }

        outputStateHandle.set(objectData);
        currentStateArrayHandle.next();
    }

    outputStateArrayHandle.set(arrayDataBuilder);
    data.setClean(plug);

    computation.endComputation();

    return MS::kSuccess;
}

void FEMSolver::simulateSubstep(FEMObjectData * objectData,
    const MVector & gravity, double timestep, int maxIterations) const {
    FEMTetrahedralMesh * tetrahedralMesh = objectData->getTetrahedralMesh();

    opentissue::fem::apply_acceleration(*tetrahedralMesh,
        FEMVector(gravity.x, gravity.y, gravity.z));

    opentissue::fem::simulate(*tetrahedralMesh, objectData->getMassDamping(),
        objectData->getStiffnessDamping(), timestep, maxIterations);

    opentissue::fem::clear_external_forces(*tetrahedralMesh);
}