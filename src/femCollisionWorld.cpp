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

#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/Polygon_mesh_processing/self_intersections.h>

#include <cmath>

unsigned int FEMIntersectionRegion::triangleIndices[12] = { 0, 1, 3, 1, 2, 3, 0, 3, 2, 0, 2, 1 };

void FEMIntersectionRegion::tetrahedronToMesh(const FEMTetrahedron * tetrahedron,
    FEMSurfaceMesh & surfaceMesh) {
    for (unsigned int i = 0; i < 4; i++) {
        const FEMVector & vertex = tetrahedron->node(i)->m_world_coord;

        surfaceMesh.add_vertex(FEMMeshPoint(vertex[0], vertex[1], vertex[2]));

        surfaceMesh.add_face(
            FEMSurfaceMesh::Vertex_index(triangleIndices[i * 3]),
            FEMSurfaceMesh::Vertex_index(triangleIndices[i * 3 + 1]),
            FEMSurfaceMesh::Vertex_index(triangleIndices[i * 3 + 2]));
    }
}
void FEMIntersectionRegion::calculateTetrahedronNormal(const FEMTetrahedron * tetrahedron,
    FEMVectorList & normalList) {
    normalList.resize(4);

    for (unsigned int i = 0; i < 4; i++) {
        const FEMVector & vertex0 = tetrahedron->node(triangleIndices[i * 3])->m_world_coord;
        const FEMVector & vertex1 = tetrahedron->node(triangleIndices[i * 3 + 1])->m_world_coord;
        const FEMVector & vertex2 = tetrahedron->node(triangleIndices[i * 3 + 2])->m_world_coord;

        FEMVector normal = (vertex1 - vertex0) % (vertex2 - vertex0);

        normalList[i][0] = normal[0];
        normalList[i][1] = normal[1];
        normalList[i][2] = normal[2];
    }
}
bool FEMIntersectionRegion::collinear(const btVector3 & lhs, const btVector3 & rhs) {
    return std::abs(lhs.dot(rhs) - lhs.length() * rhs.length()) < FEM_EPSILON;
}

FEMIntersectionRegion::FEMIntersectionRegion() : volume(0) {}
FEMIntersectionRegion::~FEMIntersectionRegion() {}

void FEMIntersectionRegion::create(const FEMTetrahedron * tetrahedron0,
    const FEMTetrahedron * tetrahedron1) {
    FEMSurfaceMesh mesh0, mesh1, outputMesh;
    FEMVectorList tetrahedronNormalList, outputNormalList;

    tetrahedronToMesh(tetrahedron0, mesh0);
    tetrahedronToMesh(tetrahedron1, mesh1);

    calculateTetrahedronNormal(tetrahedron0, tetrahedronNormalList);

    CGAL::Polygon_mesh_processing::corefine_and_compute_intersection(mesh0, mesh1, outputMesh);

    volume = 0;
    centroid.setZero();
    normal.setZero();

    FEMSurfaceMesh::Face_iterator faceIterator(outputMesh.faces_begin());

    while (faceIterator != outputMesh.faces_end()) {
        FEMSurfaceMesh::Halfedge_index edgeIndex(outputMesh.halfedge(*faceIterator));

        const FEMMeshPoint & point0 = outputMesh.point(outputMesh.target(edgeIndex));
        edgeIndex = outputMesh.next(edgeIndex);

        const FEMMeshPoint & point1 = outputMesh.point(outputMesh.target(edgeIndex));
        edgeIndex = outputMesh.next(edgeIndex);

        const FEMMeshPoint & point2 = outputMesh.point(outputMesh.target(edgeIndex));

        btVector3 vertex0(point0[0], point0[1], point0[2]);
        btVector3 vertex1(point1[0], point1[1], point1[2]);
        btVector3 vertex2(point2[0], point2[1], point2[2]);

        btVector3 n = (vertex1 - vertex0).cross(vertex2 - vertex0);
        outputNormalList.push_back(n);

        volume += n.dot(vertex0);

        btVector3 a = vertex0 + vertex1;
        btVector3 b = vertex1 + vertex2;
        btVector3 c = vertex2 + vertex0;

        centroid[0] += n[0] * (a[0] * a[0] + b[0] * b[0] + c[0] * c[0]);
        centroid[1] += n[1] * (a[1] * a[1] + b[1] * b[1] + c[1] * c[1]);
        centroid[2] += n[2] * (a[2] * a[2] + b[2] * b[2] + c[2] * c[2]);

        faceIterator++;
    }

    for (size_t i = 0; i < tetrahedronNormalList.size(); i++) {
        const btVector3 & tetrahedronNormal = tetrahedronNormalList[i];

        for (size_t j = 0; j < outputNormalList.size(); j++) {
            if (collinear(tetrahedronNormal, outputNormalList[j])) {
                normal += tetrahedronNormal;
                break;
            }
        }
    }

    volume /= 6.0;

    if (volume != 0)
        centroid /= 48.0 * volume;

    normal.normalize();
}

FEMTriangleShape::FEMTriangleShape() : btPolyhedralConvexShape() {
    m_shapeType = CUSTOM_POLYHEDRAL_SHAPE_TYPE;
    tetrahedron = nullptr;
}
FEMTriangleShape::FEMTriangleShape(int node0, int node1, int node2, FEMTetrahedron * tetrahedron)
    : btPolyhedralConvexShape() {
    m_shapeType = CUSTOM_POLYHEDRAL_SHAPE_TYPE;
    setTriangle(node0, node1, node2, tetrahedron);
}
FEMTriangleShape::~FEMTriangleShape() {}

void FEMTriangleShape::setTriangle(int node0, int node1, int node2, FEMTetrahedron * tetrahedron) {
    nodes[0] = node0;
    nodes[1] = node1;
    nodes[2] = node2;

    this->tetrahedron = tetrahedron;
}
int FEMTriangleShape::getNodeIndex(int index) const {
    return nodes[index];
}
FEMTetrahedron * FEMTriangleShape::getTetrahedron() {
    return tetrahedron;
}
const FEMTetrahedron * FEMTriangleShape::getTetrahedron() const {
    return tetrahedron;
}

const char * FEMTriangleShape::getName() const {
    return "FEMTriangleShape";
}
int FEMTriangleShape::getNumPreferredPenetrationDirections() const {
    return 2;
}
void FEMTriangleShape::getPreferredPenetrationDirection(int index,
    btVector3 & penetrationVector) const {
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
    const FEMVector & position = tetrahedron->global_index_node(nodes[index])->m_world_coord;

    vertex[0] = position[0];
    vertex[1] = position[1];
    vertex[2] = position[2];
}
void FEMTriangleShape::getEdge(int index, btVector3 & vertex0, btVector3 & vertex1) const {
    getVertex(index, vertex0);
    getVertex((index + 1) % 3, vertex1);
}
void FEMTriangleShape::getPlane(btVector3 & normal, btVector3 & position, int index) const {
    calculateNormal(normal);
    getVertex(index, position);
}
void FEMTriangleShape::getAabb(const btTransform & transform,
    btVector3 & minimum, btVector3 & maximum) const {
    minimum.setValue(FEM_INFINITY, FEM_INFINITY, FEM_INFINITY);
    maximum.setValue(-FEM_INFINITY, -FEM_INFINITY, -FEM_INFINITY);

    btVector3 vertex;

    for (int i = 0; i < 3; i++) {
        getVertex(i, vertex);

        minimum.setMin(vertex);
        maximum.setMax(vertex);
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

void FEMTriangleShape::calculateNormal(btVector3 & normal) const {
    btVector3 vertices[3];

    getVertex(0, vertices[0]);
    getVertex(1, vertices[1]);
    getVertex(2, vertices[2]);

    normal = (vertices[1] - vertices[0]).cross(vertices[2] - vertices[0]);
    normal.normalize();
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

void FEMTriangleShape::calculateIntersectionRegion(const FEMTriangleShape * triangleShape,
    FEMIntersectionRegion & region) const {
    region.create(tetrahedron, triangleShape->getTetrahedron());
}
void FEMTriangleShape::calculateVolumeCoordinates(const btVector3 & point,
    btVector3 & coordinates) const {
    FEMVector p(point[0], point[1], point[2]);

    const FEMVector & p0 = tetrahedron->i()->m_world_coord;
    const FEMVector & p1 = tetrahedron->j()->m_world_coord;
    const FEMVector & p2 = tetrahedron->k()->m_world_coord;
    const FEMVector & p3 = tetrahedron->m()->m_world_coord;

    double inv6V = 1.0 / (6.0 * tetrahedron->m_volume);

    FEMVector e0(p0 - p);
    FEMVector e1(p1 - p);
    FEMVector e2(p2 - p);
    FEMVector e3(p3 - p);

    coordinates[0] = (e1 * (e2 % e3)) * inv6V;
    coordinates[1] = (e2 * (e0 % e3)) * inv6V;
    coordinates[2] = (e3 * (e0 % e1)) * inv6V;
    coordinates[3] = 1.0 - coordinates[0] - coordinates[1] - coordinates[2];
}
void FEMTriangleShape::calculateVelocity(const btVector3 & coordinates, btVector3 & velocity) {
    velocity.setZero();

    for (int i = 0; i < 4; i++) {
        const FEMVector & nodeVelocity = tetrahedron->node(i)->m_velocity;

        velocity[0] += coordinates[i] * nodeVelocity[0];
        velocity[1] += coordinates[i] * nodeVelocity[1];
        velocity[2] += coordinates[i] * nodeVelocity[2];
    }
}
void FEMTriangleShape::applyForce(const btVector3 & coordinates, const btVector3 & force) {
    for (int i = 0; i < 4; i++) {
        FEMVector & nodeForce = tetrahedron->node(i)->m_f_external;

        nodeForce[0] += coordinates[i] * force[0];
        nodeForce[1] += coordinates[i] * force[1];
        nodeForce[2] += coordinates[i] * force[2];
    }
}

FEMCollisionShape::FEMCollisionShape() : btCompoundShape() {}
FEMCollisionShape::~FEMCollisionShape() {
    for (int i = 0; i < getNumChildShapes(); i++) {
        FEMTriangleShape * triangleShape = (FEMTriangleShape *)getChildShape(i);
        removeChildShapeByIndex(i);

        delete triangleShape;
    }
}

FEMCollisionObject::FEMCollisionObject() : btCollisionObject() {
    tetrahedralMesh = nullptr;
    surfaceNodes = nullptr;
    boundaryVolumes = nullptr;
}
FEMCollisionObject::FEMCollisionObject(FEMTetrahedralMesh * tetrahedralMesh,
    MIntArray * surfaceNodes, MIntArray * boundaryVolumes) : btCollisionObject() {
    createShape(tetrahedralMesh, surfaceNodes, boundaryVolumes);
}
FEMCollisionObject::~FEMCollisionObject() {
    deleteShape();
}

FEMTetrahedralMesh * FEMCollisionObject::getTetrahedralMesh() {
    return tetrahedralMesh;
}
const FEMTetrahedralMesh * FEMCollisionObject::getTetrahedralMesh() const {
    return tetrahedralMesh;
}
MIntArray * FEMCollisionObject::getSurfaceNodes() {
    return surfaceNodes;
}
const MIntArray * FEMCollisionObject::getSurfaceNodes() const {
    return surfaceNodes;
}
MIntArray * FEMCollisionObject::getBoundaryVolumes() {
    return boundaryVolumes;
}
const MIntArray * FEMCollisionObject::getBoundaryVolumes() const {
    return boundaryVolumes;
}

void FEMCollisionObject::createShape(FEMTetrahedralMesh * tetrahedralMesh,
    MIntArray * surfaceNodes, MIntArray * boundaryVolumes) {
    FEMCollisionShape * collisionShape = new FEMCollisionShape();

    for (unsigned int i = 0; i < surfaceNodes->length() / 3; i++) {
        int node0 = (*surfaceNodes)[i * 3];
        int node1 = (*surfaceNodes)[i * 3 + 1];
        int node2 = (*surfaceNodes)[i * 3 + 2];

        int tetrahedronIndex = (*boundaryVolumes)[i];

        FEMTetrahedron & tetrahedron = *tetrahedralMesh->tetrahedron(tetrahedronIndex);
        FEMTriangleShape * triangleShape = new FEMTriangleShape(node0, node1, node2, &tetrahedron);

        collisionShape->addChildShape(btTransform(), triangleShape);
    }

    setCollisionShape(collisionShape);

    this->tetrahedralMesh = tetrahedralMesh;
    this->surfaceNodes = surfaceNodes;
    this->boundaryVolumes = boundaryVolumes;
}
void FEMCollisionObject::deleteShape() {
    FEMCollisionShape * collisionShape = (FEMCollisionShape *)getCollisionShape();

    if (collisionShape != nullptr) {
        delete collisionShape;

        setCollisionShape(nullptr);

        tetrahedralMesh = nullptr;
        surfaceNodes = nullptr;
        boundaryVolumes = nullptr;
    }
}

void FEMCollisionObject::calculateSelfIntersections(FEMIntersectionPairList & intersections) const {
    FEMSurfaceMesh surfaceMesh;

    for (size_t i = 0; i < tetrahedralMesh->size_nodes(); i++) {
        const FEMVector & vertex = tetrahedralMesh->node(i)->m_world_coord;
        surfaceMesh.add_vertex(FEMMeshPoint(vertex[0], vertex[1], vertex[2]));
    }

    for (unsigned int i = 0; i < surfaceNodes->length() / 3; i++) {
        surfaceMesh.add_face(
            FEMSurfaceMesh::Vertex_index((*surfaceNodes)[i * 3]),
            FEMSurfaceMesh::Vertex_index((*surfaceNodes)[i * 3 + 1]),
            FEMSurfaceMesh::Vertex_index((*surfaceNodes)[i * 3 + 2]));
    }

    intersections.clear();
    CGAL::Polygon_mesh_processing::self_intersections(surfaceMesh, std::back_inserter(intersections));
}

FEMCollisionWorld::FEMCollisionWorld() : btCollisionWorld(
    new btCollisionDispatcher(new btDefaultCollisionConfiguration()),
    new btDbvtBroadphase(), nullptr) {}
FEMCollisionWorld::~FEMCollisionWorld() {
    btCollisionWorld::~btCollisionWorld();

    delete ((btCollisionDispatcher *)m_dispatcher1)->getCollisionConfiguration();
    delete m_dispatcher1;
    delete m_broadphasePairCache;
}

void FEMCollisionWorld::updateAabbs() {
    for (int i = 0; i < getNumCollisionObjects(); i++) {
        btCollisionObject * collisionObject = getCollisionObjectArray()[i];

        if (getForceUpdateAllAabbs() || collisionObject->isActive()) {
            FEMCollisionShape * collisionShape = (FEMCollisionShape *)collisionObject->getCollisionShape();
            collisionShape->recalculateLocalAabb();

            updateSingleAabb(collisionObject);
        }
    }
}