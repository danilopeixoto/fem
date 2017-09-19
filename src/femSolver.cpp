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

#include "femSolver.h"

#include <maya/MGlobal.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MFnNumericData.h>
#include <maya/MFnPluginData.h>
#include <maya/MDataHandle.h>
#include <maya/MArrayDataHandle.h>
#include <maya/MArrayDataBuilder.h>
#include <maya/MFnMesh.h>

MObject FEMSolver::enableObject;
MObject FEMSolver::startTimeObject;
MObject FEMSolver::currentTimeObject;
MObject FEMSolver::substepsObject;
MObject FEMSolver::gravityXObject;
MObject FEMSolver::gravityYObject;
MObject FEMSolver::gravityZObject;
MObject FEMSolver::gravityObject;
MObject FEMSolver::scaleObject;
MObject FEMSolver::maximumIterationsObject;
MObject FEMSolver::currentStateObject;
MObject FEMSolver::outputStateObject;

const MTypeId FEMSolver::id(0x00128583);
const MString FEMSolver::typeName("femSolver");

FEMSolver::FEMSolver() {}
FEMSolver::~FEMSolver() {}

void * FEMSolver::creator() {
    return new FEMSolver;
}
MStatus FEMSolver::initialize() {
    MFnNumericAttribute numericAttribute;
    MFnUnitAttribute unitAttribute;
    MFnTypedAttribute typedAttribute;
    MFnCompoundAttribute compoundAttribute;

    enableObject = numericAttribute.create("enable", "e", MFnNumericData::kBoolean, true);
    numericAttribute.setStorable(true);
    numericAttribute.setKeyable(true);

    startTimeObject = unitAttribute.create("startTime", "st", MFnUnitAttribute::kTime, 1);
    unitAttribute.setStorable(true);

    currentTimeObject = unitAttribute.create("currentTime", "t", MFnUnitAttribute::kTime, 0);
    unitAttribute.setStorable(true);

    substepsObject = numericAttribute.create("substeps", "s", MFnNumericData::kInt, 4);
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

    scaleObject = numericAttribute.create("scale", "sc", MFnNumericData::kDouble, 1.0);
    numericAttribute.setMin(1.0e-3);
    numericAttribute.setSoftMax(1.0);
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
    addAttribute(scaleObject);
    addAttribute(maximumIterationsObject);
    addAttribute(currentStateObject);
    addAttribute(outputStateObject);

    attributeAffects(enableObject, outputStateObject);
    attributeAffects(startTimeObject, outputStateObject);
    attributeAffects(currentTimeObject, outputStateObject);
    attributeAffects(substepsObject, outputStateObject);
    attributeAffects(gravityObject, outputStateObject);
    attributeAffects(scaleObject, outputStateObject);
    attributeAffects(maximumIterationsObject, outputStateObject);
    attributeAffects(currentStateObject, outputStateObject);

    return MS::kSuccess;
}
MStatus FEMSolver::compute(const MPlug & plug, MDataBlock & data) {
    MStatus status;

    if (plug != outputStateObject)
        return MS::kUnknownParameter;

    bool enable = data.inputValue(enableObject).asBool();

    if (!enable)
        return MS::kSuccess;

    int startTime = data.inputValue(startTimeObject).asInt();
    int currentTime = data.inputValue(currentTimeObject).asInt();

    if (currentTime < startTime)
        return MS::kSuccess;

    MArrayDataHandle currentStateArrayHandle = data.inputArrayValue(currentStateObject, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    if (currentStateArrayHandle.elementCount() == 0)
        return MS::kSuccess;

    int framerate = getFramerate();
    int substeps = data.inputValue(substepsObject).asInt() + 1;
    int maxIterations = data.inputValue(maximumIterationsObject).asInt();

    double timestep = 1.0 / (framerate * substeps);

    MVector gravity = data.inputValue(gravityObject).asVector();

    MArrayDataHandle outputStateArrayHandle = data.outputArrayValue(outputStateObject, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MArrayDataBuilder arrayDataBuilder = outputStateArrayHandle.builder();

    unsigned int index = 0;

    do {
        MDataHandle currentStateHandle = currentStateArrayHandle.inputValue(&status);
        CHECK_MSTATUS_AND_RETURN_IT(status);

        FEMObjectData * objectData = (FEMObjectData *)currentStateHandle.asPluginData();

        MDataHandle outputStateHandle = arrayDataBuilder.addElement(index++, &status);
        CHECK_MSTATUS_AND_RETURN_IT(status);

        outputStateHandle.set(objectData);

        if (objectData->isEnable()) {
            TetrahedralMesh * tetrahedralMesh = objectData->getTetrahedralMesh();

            for (int i = 0; i < substeps; i++) {
                if (!objectData->isPassive())
                    runSubstep(tetrahedralMesh, objectData, gravity, timestep, maxIterations);
            }

            double scale = data.inputValue(scaleObject).asDouble();

            if (currentTime == startTime && scale != 1.0)
                scaleTetrahedralMesh(tetrahedralMesh, scale);
        }
    } while (currentStateArrayHandle.next() != MS::kFailure);

    data.setClean(plug);

    return MS::kSuccess;
}

void FEMSolver::runSubstep(TetrahedralMesh * tetrahedralMesh, const FEMObjectData * objectData,
    const MVector & gravity, double timestep, int maxIterations) const {
    opentissue::fem::apply_acceleration(*tetrahedralMesh,
        Vector(gravity.x, gravity.y, gravity.z));

    opentissue::fem::simulate(*tetrahedralMesh, objectData->getMassDamping(),
        objectData->getStiffnessDamping(), timestep, maxIterations);
}
void FEMSolver::scaleTetrahedralMesh(TetrahedralMesh * tetrahedralMesh, double scale) const {
#pragma omp parallel for

    for (int i = 0; i < tetrahedralMesh->size_nodes(); i++) {
        TetrahedralMesh::node_iterator node = tetrahedralMesh->node(i);
        node->m_coord *= scale;
    }
}

int FEMSolver::getFramerate() const {
    int framerate = 24;

    MString result;
    MGlobal::executeCommand("currentUnit -query -time", result);

    if (result == "game")
        framerate = 15;
    else if (result == "film")
        framerate = 24;
    else if (result == "pal")
        framerate = 25;
    else if (result == "ntsc")
        framerate = 30;
    else if (result == "show")
        framerate = 48;
    else if (result == "palf")
        framerate = 50;
    else if (result == "ntscf")
        framerate = 60;
    else if (result.substitute("fps", "") != MS::kFailure)
        framerate = result.asInt();

    return framerate;
}