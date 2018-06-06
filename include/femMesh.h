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

#ifndef FEM_MESH
#define FEM_MESH

#include <maya/MPxNode.h>
#include <maya/MObject.h>
#include <maya/MTypeId.h>
#include <maya/MString.h>
#include <maya/MStatus.h>
#include <maya/MPlug.h>
#include <maya/MDataBlock.h>
#include <maya/MIntArray.h>
#include <maya/MPointArray.h>
#include <maya/MComputation.h>

#include <openvdb/openvdb.h>
#include <openvdb/tools/MeshToVolume.h>
#include <openvdb/tools/VolumeToMesh.h>

#include <unordered_set>

typedef openvdb::Vec3d FEMPoint;
typedef openvdb::Vec3I FEMTriangle;
typedef openvdb::Vec4I FEMQuad;
typedef openvdb::FloatGrid FEMFloatGrid;
typedef openvdb::math::Transform FEMTransform;
typedef openvdb::tools::PolygonPool FEMPolygon;
typedef openvdb::tools::VolumeToMesh FEMVolumeMesher;
typedef openvdb::tools::PointList FEMPointList;
typedef openvdb::tools::PolygonPoolList FEMPolygonList;

class FEMMeshDataAdapter {
public:
    FEMMeshDataAdapter(MPointArray &, MIntArray &, double);

    size_t pointCount() const;
    size_t polygonCount() const;
    size_t vertexCount(size_t) const;

    void getIndexSpacePoint(size_t, size_t, FEMPoint &) const;

    const FEMTransform & getTransform() const;

private:
    static const size_t three;

    size_t numberPoints;
    size_t numberTriangles;

    MPointArray * points;
    MIntArray * triangles;

    FEMTransform::Ptr transform;
};

struct FEMTriangleFace {
    unsigned int triangleIndex;
    unsigned int index0, index1, index2;

    FEMTriangleFace();
    FEMTriangleFace(unsigned int, unsigned int, unsigned int);
    FEMTriangleFace(unsigned int, unsigned int, unsigned int, unsigned int);
    ~FEMTriangleFace();
};

struct FEMTriangleFaceHash {
    FEMTriangleFaceHash();
    ~FEMTriangleFaceHash();

    size_t operator ()(const FEMTriangleFace &) const;
};

struct FEMTriangleFaceEqual {
    FEMTriangleFaceEqual();
    ~FEMTriangleFaceEqual();

    bool operator ()(const FEMTriangleFace &, const FEMTriangleFace &) const;
};

typedef std::unordered_set<FEMTriangleFace, FEMTriangleFaceHash, FEMTriangleFaceEqual> FEMTriangleSet;

class FEMMesh : public MPxNode {
public:
    static MObject volumeElementScaleObject;
    static MObject useVoxelSizeObject;
    static MObject voxelSizeObject;
    static MObject inputMeshObject;
    static MObject outputMeshObject;
    static MObject surfaceNodesObject;
    static MObject volumeNodesObject;
    static MObject boundaryVolumesObject;

    static const MTypeId id;
    static const MString typeName;

    MComputation computation;

    FEMMesh();
    virtual ~FEMMesh();

    static void * creator();
    static MStatus initialize();
    virtual MStatus compute(const MPlug &, MDataBlock &);

private:
    class Interrupter {
    public:
        Interrupter(MComputation *);
        ~Interrupter();

        void start(const char * = nullptr);
        void end();

        bool wasInterrupted(int = -1);

    private:
        MComputation * computation;
    };

    MStatus tetrahedralize(MObject &, MIntArray &, MIntArray &, MIntArray &, double, double);
    double computeAverageSize(MObject &);
};

#endif