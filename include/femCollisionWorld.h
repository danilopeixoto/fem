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

#ifndef FEM_COLLISION_WORLD
#define FEM_COLLISION_WORLD

#include <maya/MIntArray.h>

#include <opentissue/math/math.h>
#include <opentissue/fem/fem.h>

#include <LinearMath/btScalar.h>
#include <LinearMath/btVector3.h>
#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btTransform.h>
#include <BulletCollision/CollisionShapes/btPolyhedralConvexShape.h>
#include <BulletCollision/CollisionShapes/btCompoundShape.h>
#include <BulletCollision/CollisionDispatch/btCollisionObject.h>
#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>

typedef opentissue::math::Types<double, unsigned int> FEMMathTypes;
typedef FEMMathTypes::vector_type FEMVector;
typedef opentissue::fem::Mesh<FEMMathTypes> FEMTetrahedralMesh;
typedef FEMTetrahedralMesh::node_type FEMNode;
typedef FEMTetrahedralMesh::tetrahedron_type FEMTetrahedron;

typedef FEMNode * FEMNodePointer;
typedef FEMTetrahedralMesh * FEMTetrahedralMeshPointer;
typedef MIntArray * FEMIntegerArrayPointer;
typedef MComputation * FEMComputationPointer;

btMatrix3x3 FEMOuterProduct(const btVector3 &, const btVector3 &);

class FEMTriangleShape : public btPolyhedralConvexShape {
public:
    BT_DECLARE_ALIGNED_ALLOCATOR();

    FEMTriangleShape();
    FEMTriangleShape(unsigned int, FEMNodePointer, FEMNodePointer, FEMNodePointer);
    virtual ~FEMTriangleShape();

    virtual const char * getName() const;
    virtual int getNumPreferredPenetrationDirections() const;
    virtual void getPreferredPenetrationDirection(int, btVector3 &) const;
    virtual int getNumVertices() const;
    virtual int getNumEdges() const;
    virtual int	getNumPlanes() const;
    virtual void getVertex(int, btVector3 &) const;
    virtual void getEdge(int, btVector3 &, btVector3 &) const;
    virtual void getPlane(btVector3 &, btVector3 &, int) const;
    virtual void getAabb(const btTransform &, btVector3 &, btVector3 &) const;

    virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(
        const btVector3 *, btVector3 *, int) const;
    virtual	bool isInside(const btVector3 &, btScalar) const;

    void setBeginIndex(unsigned int);
    void setNode(unsigned int, FEMNodePointer);
    unsigned int getBeginIndex() const;
    FEMNodePointer getNode(unsigned int);
    const FEMNodePointer getNode(unsigned int) const;

    void calculateNormal(btVector3 &) const;
    void calculateBarycentric(const btVector3 &, btVector3 &) const;
    void calculateInverseInertiaTensor(btMatrix3x3 &) const;
    btVector3 localGetSupportingVertexWithoutMargin(const btVector3 &) const;

private:
    unsigned int beginIndex;
    FEMNodePointer nodes[3];
};

class FEMCollisionShape : public btCompoundShape {
public:
    BT_DECLARE_ALIGNED_ALLOCATOR();

    FEMCollisionShape();
    virtual ~FEMCollisionShape();
};

class FEMCollisionObject : public btCollisionObject {
public:
    BT_DECLARE_ALIGNED_ALLOCATOR();

    FEMCollisionObject();
    FEMCollisionObject(FEMTetrahedralMeshPointer, FEMIntegerArrayPointer);
    virtual ~FEMCollisionObject();

    FEMTetrahedralMeshPointer getTetrahedralMesh();
    const FEMTetrahedralMeshPointer getTetrahedralMesh() const;
    FEMIntegerArrayPointer getSurfaceNodes();
    const FEMIntegerArrayPointer getSurfaceNodes() const;

    void createShape(FEMTetrahedralMeshPointer, FEMIntegerArrayPointer);
    void deleteShape();

private:
    FEMTetrahedralMeshPointer tetrahedralMesh;
    FEMIntegerArrayPointer surfaceNodes;
};

class FEMCollisionWorld : public btCollisionWorld {
public:
    BT_DECLARE_ALIGNED_ALLOCATOR();

    FEMCollisionWorld();
    ~FEMCollisionWorld();
};

#endif