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

// Disable cleanup: for %%f in ("$(OutDir)\*") do if not "%%f" == "$(OutDir)\$(TargetFileName)" del /s /f /q "%%f"

#include <femPlugin.h>
#include <femSolver.h>

#include <maya/MFnNumericAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MFnNumericData.h>
#include <maya/MFnStringData.h>
#include <maya/MFnPluginData.h>
#include <maya/MDataHandle.h>
#include <maya/MArrayDataHandle.h>
#include <maya/MArrayDataBuilder.h>
#include <maya/MIntArray.h>
#include <maya/MTime.h>
#include <maya/MFnMesh.h>

#include <Alembic/Abc/OArchive.h>
#include <Alembic/Abc/OObject.h>
#include <Alembic/Abc/TypedArraySample.h>
#include <Alembic/AbcGeom/OPolyMesh.h>
#include <Alembic/AbcCoreOgawa/ReadWrite.h>

#include <string>

MObject FEMSolver::enableObject;
MObject FEMSolver::startTimeObject;
MObject FEMSolver::currentTimeObject;
MObject FEMSolver::substepsObject;
MObject FEMSolver::gravityXObject;
MObject FEMSolver::gravityYObject;
MObject FEMSolver::gravityZObject;
MObject FEMSolver::gravityObject;
MObject FEMSolver::maximumIterationsObject;
MObject FEMSolver::useDiskCacheObject;
MObject FEMSolver::cacheNameObject;
MObject FEMSolver::directoryObject;
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
    MFnCompoundAttribute compoundAttribute;

    MFnStringData stringData;

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

    useDiskCacheObject = numericAttribute.create("useDiskCache", "udc", MFnNumericData::kBoolean, false);
    numericAttribute.setStorable(true);
    numericAttribute.setKeyable(true);

    cacheNameObject = typedAttribute.create("cacheName", "cn", MFnStringData::kString,
        stringData.create(FEMPlugin::getCacheName()));
    typedAttribute.setStorable(true);

    directoryObject = typedAttribute.create("directory", "d", MFnStringData::kString,
        stringData.create(FEMPlugin::getCacheDirectory()));
    typedAttribute.setStorable(true);

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
    addAttribute(useDiskCacheObject);
    addAttribute(cacheNameObject);
    addAttribute(directoryObject);
    addAttribute(currentStateObject);
    addAttribute(outputStateObject);

    return MS::kSuccess;
}
void FEMSolver::postConstructor() {
    MObject nodeObject(thisMObject());

    MPlug startTimePlug(nodeObject, startTimeObject);
    startTimePlug.setMTime(MTime(FEMPlugin::getStartTime(), MTime::uiUnit()));

    MPlug directoryPlug(nodeObject, directoryObject);
    directoryPlug.setString(FEMPlugin::getCacheDirectory());
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

    MDataHandle useDiskCacheHandle = data.inputValue(useDiskCacheObject, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MDataHandle cacheNameHandle = data.inputValue(cacheNameObject, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MDataHandle directoryHandle = data.inputValue(directoryObject, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MArrayDataHandle currentStateArrayHandle = data.inputArrayValue(currentStateObject, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MArrayDataHandle outputStateArrayHandle = data.outputArrayValue(outputStateObject, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    int substeps = substepsHandle.asInt();
    MVector gravity = gravityHandle.asVector();

    int maximumIterations = maximumIterationsHandle.asInt();

    bool useDiskCache = useDiskCacheHandle.asBool();
    MString cacheName = cacheNameHandle.asString();
    MString directory = directoryHandle.asString();

    if (useDiskCache && (cacheName.length() == 0 || directory.length() == 0))
        return MS::kFailure;

    int steps = substeps + 1;
    double timestep = 1.0 / (steps * FEMPlugin::getFramerate());

    unsigned int objectCount = currentStateArrayHandle.elementCount();

    FEMFrameData frameData;
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

        if (useDiskCache)
            frameData.push_back(objectData);

        outputStateHandle.set(objectData);
        currentStateArrayHandle.next();
    }

    MString filename = cacheFilename(directory, cacheName, (unsigned int)currentTime.value());

    if (useDiskCache && !exportFrame(frameData, filename)) {
        computation.endComputation();
        return MS::kFailure;
    }

    outputStateArrayHandle.set(arrayDataBuilder);
    data.setClean(plug);

    computation.endComputation();

    return MS::kSuccess;
}

MString FEMSolver::cacheFilename(const MString & directory, const MString & basename,
    unsigned int frame) {
    return directory + "/" + basename + std::to_string(frame).c_str() + ".abc";
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
bool FEMSolver::exportFrame(const FEMFrameData & frameData, const MString & filename) {
    Alembic::Abc::OArchive archive(Alembic::AbcCoreOgawa::WriteArchive(), filename.asChar());

    if (!archive.valid())
        return false;

    Alembic::Abc::OObject rootObject(archive, Alembic::Abc::kTop);
    unsigned objectID = 0;

    for (const FEMObjectData * objectData : frameData) {
        if (computation.isInterruptRequested())
            return false;

        Alembic::AbcGeom::OPolyMesh polygonObject(rootObject, "object_" + std::to_string(objectID++));
        Alembic::AbcGeom::OPolyMeshSchema & mesh = polygonObject.getSchema();

        const MIntArray * surfaceNodes = objectData->getSurfaceNodes();
        const FEMTetrahedralMesh * tetrahedralMesh = objectData->getTetrahedralMesh();

        int pointCount = (int)tetrahedralMesh->size_nodes();
        int surfaceCount = surfaceNodes->length() / 3;

        FEMList<FEMFloatPoint> points(pointCount);
        FEMList<int> indices(surfaceNodes->length());
        FEMBoundingBox box;

        surfaceNodes->get(indices.data());

        for (int i = 0; i < pointCount; i++) {
            if (computation.isInterruptRequested())
                return false;

            const FEMVector & p = tetrahedralMesh->const_node(i)->m_world_coord;
            FEMFloatPoint & point = points[i];

            point.x = p[0];
            point.y = p[1];
            point.z = p[2];

            box.extendBy(point);
        }

        Alembic::AbcGeom::OPolyMeshSchema::Sample sample(
            Alembic::Abc::P3fArraySample(points),
            Alembic::Abc::Int32ArraySample(indices),
            Alembic::Abc::Int32ArraySample(FEMList<int>(surfaceCount, 3)));

        sample.setSelfBounds(box);
        mesh.set(sample);
    }

    return true;
}