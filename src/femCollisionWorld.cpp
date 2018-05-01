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

#include <femCollisionWorld.h>

#include <BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h>
#include <BulletCollision/CollisionDispatch/btCollisionDispatcher.h>
#include <BulletCollision/BroadphaseCollision/btDbvtBroadphase.h>
#include <BulletCollision/BroadphaseCollision/btBroadphaseProxy.h>

btMatrix3x3 FEMOuterProduct(const btVector3 & lhs, const btVector3 & rhs) {
    return btMatrix3x3(
        lhs.x() * rhs.x(), lhs.x() * rhs.y(), lhs.x() * rhs.z(),
        lhs.y() * rhs.x(), lhs.y() * rhs.y(), lhs.y() * rhs.z(),
        lhs.z() * rhs.x(), lhs.z() * rhs.y(), lhs.z() * rhs.z());
}

FEMTriangleShape::FEMTriangleShape() : btPolyhedralConvexShape() {
    m_shapeType = TRIANGLE_SHAPE_PROXYTYPE;

    beginIndex = 0;

    nodes[0] = nullptr;
    nodes[1] = nullptr;
    nodes[2] = nullptr;
}
FEMTriangleShape::FEMTriangleShape(unsigned int beginIndex,
    FEMNodePointer node0, FEMNodePointer node1, FEMNodePointer node2) : btPolyhedralConvexShape() {
    m_shapeType = TRIANGLE_SHAPE_PROXYTYPE;

    this->beginIndex = beginIndex;

    nodes[0] = node0;
    nodes[1] = node1;
    nodes[2] = node2;
}
FEMTriangleShape::~FEMTriangleShape() {}

const char * FEMTriangleShape::getName() const {
    return "FEMTriangleShape";
}
int FEMTriangleShape::getNumPreferredPenetrationDirections() const {
    return 2;
}
void FEMTriangleShape::getPreferredPenetrationDirection(
    int index, btVector3 & penetrationVector) const {
    calculateNormal(penetrationVector);

    if (index)
        penetrationVector *= btScalar(-1.0);
}
int FEMTriangleShape::getNumVertices() const {
    return 3;
}
int FEMTriangleShape::getNumEdges() const {
    return 3;
}
int	FEMTriangleShape::getNumPlanes() const {
    return 1;
}
void FEMTriangleShape::getVertex(int index, btVector3 & vertex) const {
    FEMVector & position = nodes[index]->m_world_coord;

    vertex.setX(position[0]);
    vertex.setY(position[1]);
    vertex.setZ(position[2]);
}
void FEMTriangleShape::getEdge(int index, btVector3 & vertex0, btVector3 & vertex1) const {
    getVertex(index, vertex0);
    getVertex((index + 1) % 3, vertex1);
}
void FEMTriangleShape::getPlane(btVector3 & normal, btVector3 & position, int index) const {
    calculateNormal(normal);
    getVertex(0, position);
}
void FEMTriangleShape::getAabb(const btTransform & transform,
    btVector3 & minimum, btVector3 & maximum) const {
    getAabbSlow(transform, minimum, maximum);
}

void FEMTriangleShape::batchedUnitVectorGetSupportingVertexWithoutMargin(
    const btVector3 * inputVectorArray, btVector3 * outputVectorArray, int vectorCount) const {
    btVector3 vertices[3];

    getVertex(0, vertices[0]);
    getVertex(1, vertices[1]);
    getVertex(2, vertices[2]);

    for (int i = 0; i < vectorCount; i++) {
        const btVector3 & direction = inputVectorArray[i];
        btVector3 dots = direction.dot3(vertices[0], vertices[1], vertices[2]);

        outputVectorArray[i] = vertices[dots.maxAxis()];
    }
}
bool FEMTriangleShape::isInside(const btVector3 & point, btScalar tolerance) const {
    btVector3 normal, position;
    getPlane(normal, position, 0);

    btScalar distance = point.dot(normal);
    btScalar offset = position.dot(normal);

    distance -= offset;

    if (distance >= -tolerance && distance <= tolerance) {
        for (unsigned int i = 0; i < 3; i++) {
            btVector3 vertex0, vertex1;
            getEdge(i, vertex0, vertex1);

            btVector3 edge = vertex1 - vertex0;
            btVector3 edgeNormal = edge.cross(normal);

            edgeNormal.normalize();

            btScalar edgeDistance = point.dot(edgeNormal);
            btScalar edgeOffset = vertex0.dot(edgeNormal);

            edgeDistance -= edgeOffset;

            if (edgeDistance < -tolerance)
                return false;
        }

        return true;
    }

    return false;
}

void FEMTriangleShape::setBeginIndex(unsigned int beginIndex) {
    this->beginIndex = beginIndex;
}
void FEMTriangleShape::setNode(unsigned int index, FEMNodePointer node) {
    nodes[index] = node;
}
unsigned int FEMTriangleShape::getBeginIndex() const {
    return beginIndex;
}
FEMNodePointer FEMTriangleShape::getNode(unsigned int index) {
    return nodes[index];
}
const FEMNodePointer FEMTriangleShape::getNode(unsigned int index) const {
    return nodes[index];
}

void FEMTriangleShape::calculateNormal(btVector3 & normal) const {
    btVector3 vertices[3];

    getVertex(0, vertices[0]);
    getVertex(1, vertices[1]);
    getVertex(2, vertices[2]);

    normal = (vertices[1] - vertices[0]).cross(vertices[2] - vertices[0]);
    normal.normalize();
}
void FEMTriangleShape::calculateBarycentric(const btVector3 & point, btVector3 & barycentric) const {
    const FEMVector & n0 = nodes[0]->m_world_coord;
    const FEMVector & n1 = nodes[1]->m_world_coord;
    const FEMVector & n2 = nodes[2]->m_world_coord;

    FEMVector v0 = n1 - n0;
    FEMVector v1 = n2 - n0;
    FEMVector v2(
        point.x() - n0[0],
        point.y() - n0[1],
        point.z() - n0[2]);

    btScalar inverseDet = 1.0 / (v0[0] * v1[1] - v1[0] * v0[1]);

    barycentric.setX(inverseDet * (v2[0] * v1[1] - v1[0] * v2[1]));
    barycentric.setY(inverseDet * (v0[0] * v2[1] - v2[0] * v0[1]));
    barycentric.setZ(1.0 - barycentric.x() - barycentric.y());
}
void FEMTriangleShape::calculateInverseInertiaTensor(btMatrix3x3 & inverseInertiaTensor) const {
    const FEMVector & n0 = nodes[0]->m_world_coord;
    const FEMVector & n1 = nodes[1]->m_world_coord;
    const FEMVector & n2 = nodes[2]->m_world_coord;

    btScalar twiceArea = opentissue::math::length(opentissue::math::cross(n1 - n0, n2 - n0));

    btScalar c0 = 12.0 / twiceArea;

    btScalar s0 = n0[0] * n0[0];
    btScalar s1 = n1[1] * n1[1];
    btScalar s2 = n2[2] * n2[2];

    inverseInertiaTensor[0][0] = c0 / (s1 + s2);
    inverseInertiaTensor[0][1] = 0;
    inverseInertiaTensor[0][2] = 0;

    inverseInertiaTensor[1][0] = 0;
    inverseInertiaTensor[1][1] = c0 / (s0 + s2);
    inverseInertiaTensor[1][2] = 0;

    inverseInertiaTensor[2][0] = 0;
    inverseInertiaTensor[2][1] = 0;
    inverseInertiaTensor[2][2] = c0 / (s0 + s1);
}
btVector3 FEMTriangleShape::localGetSupportingVertexWithoutMargin(
    const btVector3 & direction) const {
    btVector3 vertices[3];

    getVertex(0, vertices[0]);
    getVertex(1, vertices[1]);
    getVertex(2, vertices[2]);

    btVector3 dots = direction.dot3(vertices[0], vertices[1], vertices[2]);
    return vertices[dots.maxAxis()];
}

FEMCollisionShape::FEMCollisionShape() : btCompoundShape() {}
FEMCollisionShape::~FEMCollisionShape() {
    for (unsigned int i = 0; i < getNumChildShapes(); i++) {
        FEMTriangleShape * triangleShape = (FEMTriangleShape *)getChildShape(i);
        removeChildShapeByIndex(i);

        delete triangleShape;
    }
}

FEMCollisionObject::FEMCollisionObject() : btCollisionObject() {
    tetrahedralMesh = nullptr;
    surfaceNodes = nullptr;
}
FEMCollisionObject::FEMCollisionObject(FEMTetrahedralMeshPointer tetrahedralMesh,
    FEMIntegerArrayPointer surfaceNodes) : btCollisionObject() {
    createShape(tetrahedralMesh, surfaceNodes);
}
FEMCollisionObject::~FEMCollisionObject() {
    deleteShape();
}

FEMTetrahedralMeshPointer FEMCollisionObject::getTetrahedralMesh() {
    return tetrahedralMesh;
}
const FEMTetrahedralMeshPointer FEMCollisionObject::getTetrahedralMesh() const {
    return tetrahedralMesh;
}
FEMIntegerArrayPointer FEMCollisionObject::getSurfaceNodes() {
    return surfaceNodes;
}
const FEMIntegerArrayPointer FEMCollisionObject::getSurfaceNodes() const {
    return surfaceNodes;
}

void FEMCollisionObject::createShape(FEMTetrahedralMeshPointer tetrahedralMesh,
    FEMIntegerArrayPointer surfaceNodes) {
    deleteShape();

    FEMCollisionShape * collisionShape = new FEMCollisionShape();

    for (unsigned int i = 0; i < surfaceNodes->length() / 3; i++) {
        FEMTriangleShape * triangleShape = new FEMTriangleShape();

        for (unsigned int j = 0; j < 3; j++) {
            FEMTetrahedralMesh::node_iterator node = tetrahedralMesh->node((*surfaceNodes)[i * 3 + j]);
            triangleShape->setNode(j, &(*node));
        }

        collisionShape->addChildShape(btTransform(), triangleShape);
    }

    setCollisionShape(collisionShape);
}
void FEMCollisionObject::deleteShape() {
    FEMCollisionShape * collisionShape = (FEMCollisionShape *)getCollisionShape();

    if (collisionShape != nullptr) {
        delete collisionShape;

        setCollisionShape(nullptr);
        tetrahedralMesh = nullptr;
        surfaceNodes = nullptr;
    }
}

FEMCollisionWorld::FEMCollisionWorld() : btCollisionWorld(
    new btCollisionDispatcher(new btDefaultCollisionConfiguration()),
    new btDbvtBroadphase(), nullptr) {}
FEMCollisionWorld::~FEMCollisionWorld() {
    delete ((btCollisionDispatcher *)m_dispatcher1)->getCollisionConfiguration();
    delete m_dispatcher1;
    delete m_broadphasePairCache;
}