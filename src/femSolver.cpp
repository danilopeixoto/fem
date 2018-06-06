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

#include <opentissue/fem/fem.h>

#include <LinearMath/btVector3.h>
#include <BulletCollision/NarrowPhaseCollision/btPersistentManifold.h>
#include <BulletCollision/NarrowPhaseCollision/btManifoldPoint.h>

#include <functional>

FEMContactIndexPair::FEMContactIndexPair() {}
FEMContactIndexPair::FEMContactIndexPair(unsigned int index0, unsigned int index1)
    : index0(index0), index1(index1) {}
FEMContactIndexPair::~FEMContactIndexPair() {}

FEMContactIndexPairHash::FEMContactIndexPairHash() {}
FEMContactIndexPairHash::~FEMContactIndexPairHash() {}

size_t FEMContactIndexPairHash::operator ()(const FEMContactIndexPair & contactIndexPair) const {
    size_t h0 = std::hash<unsigned int>()(contactIndexPair.index0);
    size_t h1 = std::hash<unsigned int>()(contactIndexPair.index1);

    return h0 ^ h1;
};

FEMContactIndexPairEqual::FEMContactIndexPairEqual() {}
FEMContactIndexPairEqual::~FEMContactIndexPairEqual() {}

bool FEMContactIndexPairEqual::operator ()(const FEMContactIndexPair & lhs,
    const FEMContactIndexPair & rhs) const {
    bool t0 = lhs.index0 == rhs.index0 || lhs.index0 == rhs.index1;
    bool t1 = lhs.index1 == rhs.index0 || lhs.index1 == rhs.index1;

    return t0 && t1;
};

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

    FEMFrameData frameData;
    frameData.reserve(currentStateArrayHandle.elementCount());

    FEMCollisionWorld * collisionWorld = new FEMCollisionWorld();

    computation.beginComputation();

    do {
        if (computation.isInterruptRequested()) {
            computation.endComputation();
            return MS::kFailure;
        }

        MDataHandle currentStateHandle = currentStateArrayHandle.inputValue(&status);
        CHECK_MSTATUS_AND_RETURN_IT(status);

        FEMObjectData * objectData = (FEMObjectData *)currentStateHandle.asPluginData();

        if (objectData == nullptr)
            return MS::kFailure;

        if (objectData->isEnable()) {
            collisionWorld->addCollisionObject(objectData->getCollisionObject());

            if (!objectData->isPassive())
                frameData.push_back(objectData);
        }
    } while (currentStateArrayHandle.next() == MS::kSuccess);

    for (int s = 0; s < steps; s++) {
        collisionWorld->performDiscreteCollisionDetection();

        status = performCollisionResponse(collisionWorld);
        CHECK_MSTATUS_AND_RETURN_IT(status);

        status = simulateTimestep(frameData, gravity, timestep, maximumIterations);
        CHECK_MSTATUS_AND_RETURN_IT(status);
    }

    delete collisionWorld;

    MArrayDataBuilder arrayDataBuilder(outputStateArrayHandle.builder());

    for (unsigned int i = 0; i < frameData.size(); i++) {
        if (computation.isInterruptRequested()) {
            computation.endComputation();
            return MS::kFailure;
        }

        MDataHandle outputStateHandle = arrayDataBuilder.addElement(i, &status);
        CHECK_MSTATUS_AND_RETURN_IT(status);

        outputStateHandle.set(frameData[i]);
    }

    outputStateArrayHandle.set(arrayDataBuilder);
    data.setClean(plug);

    computation.endComputation();

    return MS::kSuccess;
}

void FEMSolver::computeCollisionForces(FEMTriangleShape * triangleShape0,
    FEMTriangleShape * triangleShape1, double contactFriction) {
    FEMIntersectionRegion region;
    triangleShape0->calculateIntersectionRegion(triangleShape1, region);

    if (region.volume < FEM_EPSILON)
        return;

    btVector3 coordinates[2];
    triangleShape0->calculateVolumeCoordinates(region.centroid, coordinates[0]);
    triangleShape1->calculateVolumeCoordinates(region.centroid, coordinates[1]);

    btVector3 velocity[2];
    triangleShape0->calculateVelocity(coordinates[0], velocity[0]);
    triangleShape1->calculateVelocity(coordinates[1], velocity[1]);

    btVector3 relativeVelocity = velocity[1] - velocity[0];

    double velocityDotNormal = relativeVelocity.dot(region.normal);
    btVector3 normalForce = region.volume * velocityDotNormal * region.normal;

    btVector3 tangentialVelocity = relativeVelocity - velocityDotNormal * region.normal;
    double tangentialVelocityMagnitude = tangentialVelocity.length();

    if (tangentialVelocityMagnitude > FEM_EPSILON) {
        double frictionForceMagnitude = tangentialVelocityMagnitude * region.volume;
        double scaledNormalForceMagnitude = contactFriction * normalForce.length();

        if (frictionForceMagnitude > scaledNormalForceMagnitude)
            frictionForceMagnitude = scaledNormalForceMagnitude;

        btVector3 force = normalForce +
            (frictionForceMagnitude / tangentialVelocityMagnitude) * tangentialVelocity;

        triangleShape0->applyForce(coordinates[0], force);
        triangleShape1->applyForce(coordinates[1], -force);
    }
    else {
        triangleShape0->applyForce(coordinates[0], normalForce);
        triangleShape1->applyForce(coordinates[1], -normalForce);
    }
}

MStatus FEMSolver::performCollisionResponse(FEMCollisionWorld * collisionWorld) {
    int manifoldCount = collisionWorld->getDispatcher()->getNumManifolds();

    for (int i = 0; i < manifoldCount; i++) {
        FEMContactManifoldSet contactManifoldSet;

        btPersistentManifold * contactManifold =
            collisionWorld->getDispatcher()->getManifoldByIndexInternal(i);

        int contactCount = contactManifold->getNumContacts();

        for (int j = 0; j < contactCount; j++) {
            btManifoldPoint & contactPoint = contactManifold->getContactPoint(j);

            if (contactPoint.getDistance() < 0) {
                FEMContactIndexPair contactIndexPair(contactPoint.m_index0, contactPoint.m_index1);
                contactManifoldSet.insert(contactIndexPair);
            }
        }

        FEMCollisionObject * collisionObject0 = (FEMCollisionObject *)contactManifold->getBody0();
        FEMCollisionObject * collisionObject1 = (FEMCollisionObject *)contactManifold->getBody1();

        FEMCollisionShape * collisionShape0 = (FEMCollisionShape *)collisionObject0->getCollisionShape();
        FEMCollisionShape * collisionShape1 = (FEMCollisionShape *)collisionObject1->getCollisionShape();

        double contactFriction = 0.5 * (collisionObject0->getFriction() +
            collisionObject1->getFriction());

        FEMContactManifoldSet::const_iterator contactManifoldIterator(contactManifoldSet.cbegin());

        while (contactManifoldIterator != contactManifoldSet.cend()) {
            if (computation.isInterruptRequested()) {
                computation.endComputation();
                return MS::kFailure;
            }

            const FEMContactIndexPair & contactIndexPair = *(contactManifoldIterator++);

            FEMTriangleShape * triangleShape0 =
                (FEMTriangleShape *)collisionShape0->getChildShape(contactIndexPair.index0);
            FEMTriangleShape * triangleShape1 =
                (FEMTriangleShape *)collisionShape1->getChildShape(contactIndexPair.index1);

            computeCollisionForces(triangleShape0, triangleShape1, contactFriction);
        }
    }

    FEMIntersectionPairList intersections;

    for (int i = 0; i < collisionWorld->getNumCollisionObjects(); i++) {
        if (computation.isInterruptRequested()) {
            computation.endComputation();
            return MS::kFailure;
        }

        FEMCollisionObject * collisionObject =
            (FEMCollisionObject *)collisionWorld->getCollisionObjectArray()[i];

        FEMCollisionShape * collisionShape =
            (FEMCollisionShape *)collisionObject->getCollisionShape();

        double contactFriction = collisionObject->getFriction();

        collisionObject->calculateSelfIntersections(intersections);

        for (int j = 0; j < intersections.size(); j++) {
            const FEMIntersectionPair & intersectionPair = intersections[j];

            FEMTriangleShape * triangleShape0 =
                (FEMTriangleShape *)collisionShape->getChildShape(intersectionPair.first);
            FEMTriangleShape * triangleShape1 =
                (FEMTriangleShape *)collisionShape->getChildShape(intersectionPair.second);

            computeCollisionForces(triangleShape0, triangleShape1, contactFriction);
        }
    }

    return MS::kSuccess;
}
MStatus FEMSolver::simulateTimestep(FEMFrameData & frameData, const MVector & gravity,
    double timestep, int maximumIterations) {
    for (unsigned int i = 0; i < frameData.size(); i++) {
        if (computation.isInterruptRequested()) {
            computation.endComputation();
            return MS::kFailure;
        }

        FEMObjectData * objectData = frameData[i];
        FEMTetrahedralMesh * tetrahedralMesh = objectData->getTetrahedralMesh();

        opentissue::fem::apply_acceleration(*tetrahedralMesh,
            FEMVector(gravity.x, gravity.y, gravity.z));

        opentissue::fem::simulate(*tetrahedralMesh, objectData->getMassDamping(),
            objectData->getStiffnessDamping(), timestep, maximumIterations);

        opentissue::fem::clear_external_forces(*tetrahedralMesh);
    }

    return MS::kSuccess;
}