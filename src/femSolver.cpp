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

#include <LinearMath/btMinMax.h>
#include <BulletCollision/BroadphaseCollision/btDbvt.h>
#include <BulletCollision/NarrowPhaseCollision/btPersistentManifold.h>
#include <BulletCollision/NarrowPhaseCollision/btManifoldPoint.h>

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

FEMSolver::FEMSolver() {
    collisionWorld = std::make_shared<FEMCollisionWorld>();
}
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

        frameData.push_back(objectData);
    } while (currentStateArrayHandle.next() == MS::kSuccess);

    for (int s = 0; s < steps; s++) {
        collisionWorld->performDiscreteCollisionDetection();
        performCollisionResponse(frameData);
        simulateTimestep(frameData, gravity, timestep, maximumIterations);
    }

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

btVector3 FEMSolver::computeImpulse(
    const btVector3 & relativeVelocity, const btVector3 & normal,
    double friction, double inverseMass0, double inverseMass1,
    const btVector3 & distance0, const btVector3 & distance1,
    const btMatrix3x3 & inverseInertiaTensor0, const btMatrix3x3 & inverseInertiaTensor1) {
    double inverseMassSum = inverseMass0 + inverseMass1;
    double relativeVelocityDotNormal = relativeVelocity.dot(normal);

    double normalImpulseMagnitude = -relativeVelocityDotNormal /
        (inverseMassSum + ((inverseInertiaTensor0 * distance0.cross(normal)).cross(distance0) +
        (inverseInertiaTensor1 * distance1.cross(normal)).cross(distance1)).dot(normal));

    btVector3 normalImpulse = normalImpulseMagnitude * normal;

    btVector3 tangent = relativeVelocity - relativeVelocityDotNormal * normal;
    double tangentMagnitude = tangent.length();

    if (tangentMagnitude > FEM_EPSILON) {
        tangent /= tangentMagnitude;

        double frictionImpulseMagnitude = -relativeVelocity.dot(tangent) /
            (inverseMassSum +
            ((inverseInertiaTensor0 * distance0.cross(tangent)).cross(distance0) +
                (inverseInertiaTensor1 * distance1.cross(tangent)).cross(distance1)).dot(tangent));

        double scaledNormalImpulseMagnitude = normalImpulseMagnitude * friction;
        btClamp(frictionImpulseMagnitude, -scaledNormalImpulseMagnitude, scaledNormalImpulseMagnitude);

        return normalImpulse + frictionImpulseMagnitude * tangent;
    }

    return normalImpulse;
}

void FEMSolver::performCollisionResponse(FEMFrameData & frameData) {
    int manifoldCount = collisionWorld->getDispatcher()->getNumManifolds();

    for (int i = 0; i < manifoldCount; i++) {
        btPersistentManifold * contactManifold =
            collisionWorld->getDispatcher()->getManifoldByIndexInternal(i);

        FEMCollisionObject * collisionObject0 = (FEMCollisionObject *)contactManifold->getBody0();
        FEMCollisionObject * collisionObject1 = (FEMCollisionObject *)contactManifold->getBody1();

        FEMCollisionShape * collisionShape0 = (FEMCollisionShape *)collisionObject0->getCollisionShape();
        FEMCollisionShape * collisionShape1 = (FEMCollisionShape *)collisionObject1->getCollisionShape();

        double contactFriction = 0.5 * (collisionObject0->getFriction() +
            collisionObject1->getFriction());

        int contactCount = contactManifold->getNumContacts();

        for (int j = 0; j < contactCount; j++) {
            btManifoldPoint & contactPoint = contactManifold->getContactPoint(j);

            if (contactPoint.getDistance() < 0) {
                FEMTriangleShape * triangle0 =
                    (FEMTriangleShape *)collisionShape0->getChildShape(contactPoint.m_index0);
                FEMTriangleShape * triangle1 =
                    (FEMTriangleShape *)collisionShape1->getChildShape(contactPoint.m_index1);

                double & mass00 = triangle0->getNode(0)->m_mass;
                double & mass01 = triangle0->getNode(1)->m_mass;
                double & mass02 = triangle0->getNode(2)->m_mass;

                double & mass10 = triangle1->getNode(0)->m_mass;
                double & mass11 = triangle1->getNode(1)->m_mass;
                double & mass12 = triangle1->getNode(2)->m_mass;

                FEMVector & velocity00 = triangle0->getNode(0)->m_velocity;
                FEMVector & velocity01 = triangle0->getNode(1)->m_velocity;
                FEMVector & velocity02 = triangle0->getNode(2)->m_velocity;

                FEMVector & velocity10 = triangle1->getNode(0)->m_velocity;
                FEMVector & velocity11 = triangle1->getNode(1)->m_velocity;
                FEMVector & velocity12 = triangle1->getNode(2)->m_velocity;

                btVector3 barycentric[2];
                triangle0->calculateBarycentric(contactPoint.getPositionWorldOnA(), barycentric[0]);
                triangle1->calculateBarycentric(contactPoint.getPositionWorldOnB(), barycentric[1]);

                btScalar inverseMass0 = barycentric[0].dot(
                    btVector3(1.0 / mass00, 1.0 / mass01, 1.0 / mass02));
                btScalar inverseMass1 = barycentric[1].dot(
                    btVector3(1.0 / mass10, 1.0 / mass11, 1.0 / mass12));

                btMatrix3x3 velocity0(
                    velocity00[0], velocity01[0], velocity02[0],
                    velocity00[1], velocity01[1], velocity02[1],
                    velocity00[2], velocity01[2], velocity02[2]);

                btMatrix3x3 velocity1(
                    velocity10[0], velocity11[0], velocity12[0],
                    velocity10[1], velocity11[1], velocity12[1],
                    velocity10[2], velocity11[2], velocity12[2]);

                btMatrix3x3 inverseInertiaTensor[2];
                triangle0->calculateInverseInertiaTensor(inverseInertiaTensor[0]);
                triangle1->calculateInverseInertiaTensor(inverseInertiaTensor[1]);

                btVector3 distanceVector0 = contactPoint.getPositionWorldOnA() -
                    collisionShape0->getDynamicAabbTree()->m_root->volume.Center();
                btVector3 distanceVector1 = contactPoint.getPositionWorldOnB() -
                    collisionShape1->getDynamicAabbTree()->m_root->volume.Center();

                btVector3 relativeVelocity = velocity1 * barycentric[1] - velocity0 * barycentric[0];

                btVector3 impulse = computeImpulse(relativeVelocity,
                    contactPoint.m_normalWorldOnB, contactFriction,
                    inverseMass0, inverseMass1,
                    distanceVector0, distanceVector1,
                    inverseInertiaTensor[0], inverseInertiaTensor[1]);

                velocity0 -= FEMOuterProduct(impulse / inverseMass0, barycentric[0]);
                velocity1 += FEMOuterProduct(impulse / inverseMass1, barycentric[1]);

                velocity00[0] = velocity0[0][0];
                velocity00[1] = velocity0[1][0];
                velocity00[2] = velocity0[2][0];

                velocity01[0] = velocity0[0][1];
                velocity01[1] = velocity0[1][1];
                velocity01[2] = velocity0[2][1];

                velocity02[0] = velocity0[0][2];
                velocity02[1] = velocity0[1][2];
                velocity02[2] = velocity0[2][2];

                velocity10[0] = velocity1[0][0];
                velocity10[1] = velocity1[1][0];
                velocity10[2] = velocity1[2][0];

                velocity11[0] = velocity1[0][1];
                velocity11[1] = velocity1[1][1];
                velocity11[2] = velocity1[2][1];

                velocity12[0] = velocity1[0][2];
                velocity12[1] = velocity1[1][2];
                velocity12[2] = velocity1[2][2];
            }
        }
    }
}
void FEMSolver::simulateTimestep(FEMFrameData & frameData, const MVector & gravity,
    double timestep, int maximumIterations) {
    for (unsigned int i = 0; i < frameData.size(); i++) {
        FEMObjectData * objectData = frameData[i];
        FEMTetrahedralMeshSharedPointer tetrahedralMesh = objectData->getTetrahedralMesh();

        opentissue::fem::apply_acceleration(*tetrahedralMesh,
            FEMVector(gravity.x, gravity.y, gravity.z));

        opentissue::fem::simulate(*tetrahedralMesh, objectData->getMassDamping(),
            objectData->getStiffnessDamping(), timestep, maximumIterations);

        opentissue::fem::clear_external_forces(*tetrahedralMesh);
    }
}