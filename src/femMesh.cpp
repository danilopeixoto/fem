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

#include <femMesh.h>

#include <maya/MFnNumericAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnNumericData.h>
#include <maya/MFnMeshData.h>
#include <maya/MFnIntArrayData.h>
#include <maya/MDataHandle.h>
#include <maya/MPoint.h>
#include <maya/MVector.h>
#include <maya/MFnMesh.h>
#include <maya/MItMeshVertex.h>

#include <nglib.h>

#include <cmath>
#include <functional>

FEMTriangleFace::FEMTriangleFace() {}
FEMTriangleFace::FEMTriangleFace(unsigned int index0, unsigned int index1, unsigned int index2)
    : index0(index0), index1(index1), index2(index2) {}
FEMTriangleFace::FEMTriangleFace(unsigned int triangleIndex,
    unsigned int index0, unsigned int index1, unsigned int index2)
    : triangleIndex(triangleIndex), index0(index0), index1(index1), index2(index2) {}
FEMTriangleFace::~FEMTriangleFace() {}

FEMTriangleFaceHash::FEMTriangleFaceHash() {}
FEMTriangleFaceHash::~FEMTriangleFaceHash() {}

size_t FEMTriangleFaceHash::operator ()(const FEMTriangleFace & triangle) const {
    size_t h0 = std::hash<unsigned int>()(triangle.index0);
    size_t h1 = std::hash<unsigned int>()(triangle.index1);
    size_t h2 = std::hash<unsigned int>()(triangle.index2);

    return (h0 ^ h1) ^ h2;
}

FEMTriangleFaceEqual::FEMTriangleFaceEqual() {}
FEMTriangleFaceEqual::~FEMTriangleFaceEqual() {}

bool FEMTriangleFaceEqual::operator ()(const FEMTriangleFace & lhs, const FEMTriangleFace & rhs) const {
    bool t0 = lhs.index0 == rhs.index0 || lhs.index0 == rhs.index1 || lhs.index0 == rhs.index2;
    bool t1 = lhs.index1 == rhs.index0 || lhs.index1 == rhs.index1 || lhs.index1 == rhs.index2;
    bool t2 = lhs.index2 == rhs.index0 || lhs.index2 == rhs.index1 || lhs.index2 == rhs.index2;

    return t0 && t1 && t2;
}

const size_t FEMMeshDataAdapter::three = 3;

FEMMeshDataAdapter::FEMMeshDataAdapter(MPointArray & points, MIntArray & triangles,
    double voxelSize) {
    numberPoints = points.length();
    numberTriangles = triangles.length() / three;

    this->points = &points;
    this->triangles = &triangles;

    transform = FEMTransform::createLinearTransform(voxelSize);

    for (unsigned int i = 0; i < numberPoints; i++) {
        MPoint & point = (*this->points)[i];
        FEMPoint transformedPoint(point.x, point.y, point.z);

        transformedPoint = transform->worldToIndex(transformedPoint);

        point.x = transformedPoint.x();
        point.y = transformedPoint.y();
        point.z = transformedPoint.z();
    }
}

size_t FEMMeshDataAdapter::pointCount() const {
    return numberPoints;
}
size_t FEMMeshDataAdapter::polygonCount() const {
    return numberTriangles;
}
size_t FEMMeshDataAdapter::vertexCount(size_t vertexIndex) const {
    return three;
}

void FEMMeshDataAdapter::getIndexSpacePoint(size_t polygonIndex, size_t vertexIndex,
    FEMPoint & position) const {
    const MPoint & point = (*points)[(*triangles)[polygonIndex * three + vertexIndex]];

    position[0] = point.x;
    position[1] = point.y;
    position[2] = point.z;
}

const FEMTransform & FEMMeshDataAdapter::getTransform() const {
    return *transform;
}

MObject FEMMesh::volumeElementScaleObject;
MObject FEMMesh::useVoxelSizeObject;
MObject FEMMesh::voxelSizeObject;
MObject FEMMesh::inputMeshObject;
MObject FEMMesh::outputMeshObject;
MObject FEMMesh::surfaceNodesObject;
MObject FEMMesh::volumeNodesObject;
MObject FEMMesh::boundaryVolumesObject;

const MTypeId FEMMesh::id(0x00128581);
const MString FEMMesh::typeName("femMesh");

FEMMesh::Interrupter::Interrupter(MComputation * computation) : computation(computation) {}
FEMMesh::Interrupter::~Interrupter() {}

bool FEMMesh::Interrupter::wasInterrupted(int percent) {
    return computation->isInterruptRequested();
}

void FEMMesh::Interrupter::start(const char * name) {}
void FEMMesh::Interrupter::end() {}

FEMMesh::FEMMesh() {}
FEMMesh::~FEMMesh() {}

void * FEMMesh::creator() {
    return new FEMMesh();
}
MStatus FEMMesh::initialize() {
    MFnNumericAttribute numericAttribute;
    MFnTypedAttribute typedAttribute;

    volumeElementScaleObject = numericAttribute.create("volumeElementScale", "s",
        MFnNumericData::kDouble, 1.0);
    numericAttribute.setMin(1.0e-3);
    numericAttribute.setMax(1.0);
    numericAttribute.setStorable(true);
    numericAttribute.setKeyable(true);

    useVoxelSizeObject = numericAttribute.create("useVoxelSize", "uvs", MFnNumericData::kBoolean, false);
    numericAttribute.setStorable(true);
    numericAttribute.setKeyable(true);

    voxelSizeObject = numericAttribute.create("voxelSize", "vs", MFnNumericData::kDouble, 1.0);
    numericAttribute.setMin(1.0e-3);
    numericAttribute.setStorable(true);
    numericAttribute.setKeyable(true);

    inputMeshObject = typedAttribute.create("inputMesh", "in", MFnMeshData::kMesh);
    typedAttribute.setStorable(true);

    outputMeshObject = typedAttribute.create("outputMesh", "out", MFnMeshData::kMesh);
    typedAttribute.setWritable(false);
    typedAttribute.setStorable(false);

    surfaceNodesObject = typedAttribute.create("surfaceNodes", "sn", MFnIntArrayData::kIntArray);
    typedAttribute.setWritable(false);
    typedAttribute.setStorable(false);

    volumeNodesObject = typedAttribute.create("volumeNodes", "vn", MFnIntArrayData::kIntArray);
    typedAttribute.setWritable(false);
    typedAttribute.setStorable(false);

    boundaryVolumesObject = typedAttribute.create("boundaryVolumes", "bv", MFnIntArrayData::kIntArray);
    typedAttribute.setWritable(false);
    typedAttribute.setStorable(false);

    addAttribute(volumeElementScaleObject);
    addAttribute(useVoxelSizeObject);
    addAttribute(voxelSizeObject);
    addAttribute(inputMeshObject);
    addAttribute(outputMeshObject);
    addAttribute(surfaceNodesObject);
    addAttribute(volumeNodesObject);
    addAttribute(boundaryVolumesObject);

    attributeAffects(volumeElementScaleObject, outputMeshObject);
    attributeAffects(volumeElementScaleObject, surfaceNodesObject);
    attributeAffects(volumeElementScaleObject, volumeNodesObject);
    attributeAffects(volumeElementScaleObject, boundaryVolumesObject);
    attributeAffects(useVoxelSizeObject, outputMeshObject);
    attributeAffects(useVoxelSizeObject, surfaceNodesObject);
    attributeAffects(useVoxelSizeObject, volumeNodesObject);
    attributeAffects(useVoxelSizeObject, boundaryVolumesObject);
    attributeAffects(voxelSizeObject, outputMeshObject);
    attributeAffects(voxelSizeObject, surfaceNodesObject);
    attributeAffects(voxelSizeObject, volumeNodesObject);
    attributeAffects(voxelSizeObject, boundaryVolumesObject);
    attributeAffects(inputMeshObject, outputMeshObject);
    attributeAffects(inputMeshObject, surfaceNodesObject);
    attributeAffects(inputMeshObject, volumeNodesObject);
    attributeAffects(inputMeshObject, boundaryVolumesObject);

    return MS::kSuccess;
}
MStatus FEMMesh::compute(const MPlug & plug, MDataBlock & data) {
    if (plug != outputMeshObject && plug != surfaceNodesObject
        && plug != volumeNodesObject)
        return MS::kUnknownParameter;

    MStatus status;

    MDataHandle volumeElementScaleHandle = data.inputValue(volumeElementScaleObject, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MDataHandle inputMeshHandle = data.inputValue(inputMeshObject, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MDataHandle useVoxelSizeHandle = data.inputValue(useVoxelSizeObject, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MDataHandle voxelSizeHandle = data.inputValue(voxelSizeObject, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MDataHandle outputMeshHandle = data.outputValue(outputMeshObject);
    MDataHandle surfaceNodesHandle = data.outputValue(surfaceNodesObject);
    MDataHandle volumeNodesHandle = data.outputValue(volumeNodesObject);
    MDataHandle boundaryVolumesHandle = data.outputValue(boundaryVolumesObject);

    double volumeElementScale = volumeElementScaleHandle.asDouble();

    bool useVoxelSize = useVoxelSizeHandle.asBool();
    double voxelSize = voxelSizeHandle.asDouble();

    outputMeshHandle.set(inputMeshHandle.asMesh());
    MObject meshObject = outputMeshHandle.asMesh();

    if (meshObject.isNull())
        return MS::kFailure;

    MIntArray surfaceNodes, volumeNodes, boundaryVolumes;

    status = tetrahedralize(meshObject, surfaceNodes, volumeNodes, boundaryVolumes,
        volumeElementScale, useVoxelSize ? voxelSize : 0);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MFnIntArrayData intArrayData;

    surfaceNodesHandle.set(intArrayData.create(surfaceNodes));
    volumeNodesHandle.set(intArrayData.create(volumeNodes));
    boundaryVolumesHandle.set(intArrayData.create(boundaryVolumes));

    data.setClean(plug);

    return MS::kSuccess;
}

MStatus FEMMesh::tetrahedralize(MObject & meshObject, MIntArray & surfaceNodes,
    MIntArray & volumeNodes, MIntArray & boundaryVolumes,
    double volumeElementScale, double voxelSize) {
    computation.beginComputation();

    MStatus status;
    MFnMesh mesh(meshObject);

    MPointArray vertexArray;
    mesh.getPoints(vertexArray);

    MIntArray triangleCounts, triangleVertices;
    mesh.getTriangles(triangleCounts, triangleVertices);

    triangleCounts.setLength(0);

    int triangleCount = triangleVertices.length() / 3;

    mesh.create(vertexArray.length(), triangleCount, vertexArray,
        MIntArray(triangleCount, 3), triangleVertices, meshObject);

    double scale = volumeElementScale * computeAverageSize(meshObject);

    if (scale == 0) {
        computation.endComputation();
        return MS::kFailure;
    }

    Interrupter interrupter(&computation);

    FEMMeshDataAdapter meshDataAdapter(vertexArray, triangleVertices,
        voxelSize == 0 ? 0.125 * scale : voxelSize);

    FEMFloatGrid::Ptr grid = openvdb::tools::meshToVolume<FEMFloatGrid>
        (interrupter, meshDataAdapter, meshDataAdapter.getTransform());

    if (computation.isInterruptRequested()) {
        computation.endComputation();
        return MS::kFailure;
    }

    vertexArray.setLength(0);
    triangleVertices.setLength(0);

    FEMVolumeMesher volumeMesher;
    volumeMesher(*grid);

    if (computation.isInterruptRequested() || volumeMesher.pointListSize() == 0) {
        computation.endComputation();
        return MS::kFailure;
    }

    nglib::STLGeometry * surfaceMesh = nglib::STLNewGeometry();
    nglib::Mesh * volumeMesh = nglib::NewMesh();

    nglib::MeshingParameters parameters;
    parameters.min_element_size = 0.25 * scale;
    parameters.max_element_size = parameters.min_element_size * 1.1;
    parameters.second_order = false;

    FEMPointList & points = volumeMesher.pointList();
    FEMPolygonList & polygons = volumeMesher.polygonPoolList();

    for (size_t i = 0; i < volumeMesher.polygonPoolListSize(); i++) {
        if (computation.isInterruptRequested()) {
            computation.endComputation();
            return MS::kFailure;
        }

        const FEMPolygon & polygon = polygons[i];

        for (size_t j = 0; j < polygon.numTriangles(); j++) {
            if (computation.isInterruptRequested()) {
                computation.endComputation();
                return MS::kFailure;
            }

            const FEMTriangle & triangle = polygon.triangle(j);

            const FEMPoint & v0 = points[triangle[0]];
            const FEMPoint & v1 = points[triangle[1]];
            const FEMPoint & v2 = points[triangle[2]];

            nglib::STLAddTriangle(surfaceMesh, (double *)v0.asPointer(),
                (double *)v2.asPointer(), (double *)v1.asPointer());
        }

        for (size_t j = 0; j < polygon.numQuads(); j++) {
            if (computation.isInterruptRequested()) {
                computation.endComputation();
                return MS::kFailure;
            }

            const FEMQuad & quad = polygon.quad(j);

            const FEMPoint & v0 = points[quad[0]];
            const FEMPoint & v1 = points[quad[1]];
            const FEMPoint & v2 = points[quad[2]];
            const FEMPoint & v3 = points[quad[3]];

            nglib::STLAddTriangle(surfaceMesh, (double *)v0.asPointer(),
                (double *)v2.asPointer(), (double *)v1.asPointer());

            nglib::STLAddTriangle(surfaceMesh, (double *)v0.asPointer(),
                (double *)v3.asPointer(), (double *)v2.asPointer());
        }
    }

    points.reset();
    polygons.reset();

    nglib::Status result = nglib::STLInitializeGeometry(surfaceMesh,
        volumeMesh, &parameters);

    if (computation.isInterruptRequested() || result != nglib::Status::Success) {
        computation.endComputation();
        return MS::kFailure;
    }

    result = nglib::STLGenerateSurfaceMesh(surfaceMesh, volumeMesh, &parameters);

    if (computation.isInterruptRequested() || result != nglib::Status::Success) {
        computation.endComputation();
        return MS::kFailure;
    }

    result = nglib::GenerateVolumeMesh(volumeMesh, &parameters);

    if (computation.isInterruptRequested() || result != nglib::Status::Success) {
        computation.endComputation();
        return MS::kFailure;
    }

    int pointCount = nglib::GetPointCount(volumeMesh);
    int surfaceCount = nglib::GetSurfaceCount(volumeMesh);
    int volumeCount = nglib::GetVolumeCount(volumeMesh);

    vertexArray.setLength(pointCount);
    surfaceNodes.setLength(surfaceCount * 3);
    volumeNodes.setLength(volumeCount * 4);
    boundaryVolumes.setLength(surfaceCount);

    for (int i = 0; i < pointCount; i++) {
        if (computation.isInterruptRequested()) {
            computation.endComputation();
            return MS::kFailure;
        }

        double point[3];

        nglib::GetPoint(volumeMesh, i, point);

        vertexArray.set(i, point[0], point[1], point[2]);
    }

    FEMTriangleSet boundaryTriangles;

    for (int i = 0; i < surfaceCount; i++) {
        if (computation.isInterruptRequested()) {
            computation.endComputation();
            return MS::kFailure;
        }

        int triangle[3];

        nglib::GetSurfaceElement(volumeMesh, i, triangle);

        for (int j = 0; j < 3; j++)
            surfaceNodes[i * 3 + j] = triangle[j];

        boundaryTriangles.insert(FEMTriangleFace(i, triangle[0], triangle[1], triangle[2]));
    }

    FEMTriangleSet triangleIndices;

    for (int i = 0; i < volumeCount; i++) {
        if (computation.isInterruptRequested()) {
            computation.endComputation();
            return MS::kFailure;
        }

        int tetrahedron[4];

        nglib::GetVolumeElement(volumeMesh, i, tetrahedron);

        volumeNodes[i * 4] = tetrahedron[0];
        volumeNodes[i * 4 + 1] = tetrahedron[1];
        volumeNodes[i * 4 + 2] = tetrahedron[3];
        volumeNodes[i * 4 + 3] = tetrahedron[2];

        FEMTriangleFace triangles[4];

        triangles[0] = FEMTriangleFace(tetrahedron[0], tetrahedron[1], tetrahedron[2]);
        triangles[1] = FEMTriangleFace(tetrahedron[1], tetrahedron[3], tetrahedron[2]);
        triangles[2] = FEMTriangleFace(tetrahedron[0], tetrahedron[2], tetrahedron[3]);
        triangles[3] = FEMTriangleFace(tetrahedron[0], tetrahedron[3], tetrahedron[1]);

        for (unsigned int j = 0; j < 4; j++) {
            const FEMTriangleFace & triangle = triangles[j];

            triangleIndices.insert(triangle);

            FEMTriangleSet::iterator triangleIterator(boundaryTriangles.find(triangle));

            if (triangleIterator != boundaryTriangles.end()) {
                boundaryVolumes[triangleIterator->triangleIndex] = i;
                boundaryTriangles.erase(triangleIterator);
            }
        }
    }

    triangleCount = triangleIndices.size();
    triangleVertices.setLength(triangleCount * 3);

    FEMTriangleSet::const_iterator it = triangleIndices.begin();

    for (int i = 0; i < triangleCount; i++) {
        if (computation.isInterruptRequested()) {
            computation.endComputation();
            return MS::kFailure;
        }

        const FEMTriangleFace & triangle = *it++;

        triangleVertices[i * 3] = triangle.index0;
        triangleVertices[i * 3 + 1] = triangle.index1;
        triangleVertices[i * 3 + 2] = triangle.index2;
    }

    triangleIndices.clear();

    nglib::DeleteMesh(volumeMesh);
    nglib::STLDeleteGeometry(surfaceMesh);

    mesh.create(pointCount, triangleCount, vertexArray,
        MIntArray(triangleCount, 3), triangleVertices, meshObject);

    for (int i = 0; i < mesh.numEdges(); i++) {
        if (computation.isInterruptRequested()) {
            computation.endComputation();
            return MS::kFailure;
        }

        mesh.setEdgeSmoothing(i, false);
    }

    mesh.cleanupEdgeSmoothing();
    computation.endComputation();

    return MS::kSuccess;
}
double FEMMesh::computeAverageSize(MObject & meshObject) {
    MItMeshVertex vertexIt(meshObject);
    MPoint centroid;

    for (; !vertexIt.isDone(); vertexIt.next()) {
        if (computation.isInterruptRequested())
            return 0;

        centroid += vertexIt.position();
    }

    int vertexCount = vertexIt.count();

    if (vertexCount == 0)
        return 0;

    centroid.x /= vertexCount;
    centroid.y /= vertexCount;
    centroid.z /= vertexCount;

    double squaredDistanceSum = 0;

    for (vertexIt.reset(); !vertexIt.isDone(); vertexIt.next()) {
        if (computation.isInterruptRequested())
            return 0;

        MVector distance = vertexIt.position() - centroid;
        squaredDistanceSum += distance * distance;
    }

    return 2.0 * std::sqrt(squaredDistanceSum / vertexCount);
}