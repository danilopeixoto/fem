// Netgen Library
//
// Library interface to the Netgen meshing kernel.
//
// Author:	Joachim Schoeberl
// Date:	7 May, 2000
//
// Philippose - 14 February, 2009
// Modifications for creating a dynamic library in Windows.
//
// Philippose - 27 June, 2009
// Do not locally redefine "mparam", this is a global object.
//
// Philippose - 13 September, 2010
// Added a couple more parameters into the meshing parameters.

#include "nglib.h"

#include <mystdlib.h>
#include <linalg.hpp>
#include <stlgeom.hpp>
#include <meshing.hpp>

namespace netgen {
    extern DLL_HEADER MeshingParameters mparam;
}

NGLIB_NAMESPACE_BEGIN

int index_offset;
ostream nullOutputStream(nullptr);

netgen::Array<netgen::STLReadTriangle> STLTriangleList;

MeshingParameters::MeshingParameters() {
    resetParameters();
}

void MeshingParameters::resetParameters() {
    local_element_size = true;

    min_element_size = 0;
    max_element_size = 1000.0;

    grading = 0.3;

    elements_per_edge = 2.0;
    elements_per_curve = 2.0;

    second_order = false;

    element_size_filename = nullptr;

    optimization_steps = 3;

    invert_surface = false;
    invert_volume = false;

    check_overlaps = true;
    check_overlapping_boundary = true;
}

void MeshingParameters::transferParameters() {
    netgen::mparam.uselocalh = local_element_size;

    netgen::mparam.minh = min_element_size;
    netgen::mparam.maxh = max_element_size;

	netgen::mparam.grading = netgen::max2(grading, 1.0e-3);

    netgen::mparam.segmentsperedge = elements_per_edge;
    netgen::mparam.curvaturesafety = elements_per_curve;

    netgen::mparam.secondorder = second_order;

    if (element_size_filename != nullptr)
        netgen::mparam.meshsizefilename = element_size_filename;
    else
        netgen::mparam.meshsizefilename = "";

    netgen::mparam.optsteps3d = optimization_steps;

    netgen::mparam.inverttrigs = invert_surface;
    netgen::mparam.inverttets = invert_volume;

    netgen::mparam.checkoverlap = check_overlaps;
    netgen::mparam.checkoverlappingboundary = check_overlapping_boundary;
}

void Initialize(int first_index, bool suppress_output) {
    index_offset = 1 - first_index;

    if (suppress_output) {
        netgen::testout = &nullOutputStream;
        netgen::mycout = &nullOutputStream;
        netgen::myerr = &nullOutputStream;
    }
    else {
        netgen::testout = &cout;
        netgen::mycout = &cout;
        netgen::myerr = &cerr;
    }
}

Mesh * NewMesh() {
    netgen::Mesh * mesh = new netgen::Mesh();
    mesh->AddFaceDescriptor(netgen::FaceDescriptor(1, 1, 0, 1));

    return (Mesh *)(void *)mesh;
}

void DeleteMesh(Mesh * mesh) {
    if (mesh != nullptr) {
        ((netgen::Mesh *)mesh)->DeleteMesh();

        delete (netgen::Mesh *)mesh;

        mesh = nullptr;
    }
}

int GetPointCount(Mesh * mesh) {
    return ((netgen::Mesh *)mesh)->GetNP();
}

int GetSurfaceCount(Mesh * mesh) {
    return ((netgen::Mesh *)mesh)->GetNSE();
}

int GetVolumeCount(Mesh * mesh) {
    return ((netgen::Mesh *)mesh)->GetNE();
}

void AddPoint(Mesh * mesh, double * point) {
    ((netgen::Mesh *)mesh)->AddPoint(netgen::Point3d(point[0], point[1], point[2]));
}

void AddSurfaceElement(Mesh * mesh, int * indices) {
    netgen::Element2d element(3);

    element.SetIndex(1);

    element.PNum(1) = indices[0] + index_offset;
    element.PNum(2) = indices[1] + index_offset;
    element.PNum(3) = indices[2] + index_offset;

    ((netgen::Mesh *)mesh)->AddSurfaceElement(element);
}

void AddVolumeElement(Mesh * mesh, int * indices) {
    netgen::Element element(4);

    element.SetIndex(1);

    element.PNum(1) = indices[0] + index_offset;
    element.PNum(2) = indices[1] + index_offset;
    element.PNum(3) = indices[2] + index_offset;
    element.PNum(4) = indices[3] + index_offset;

    ((netgen::Mesh *)mesh)->AddVolumeElement(element);
}

void GetPoint(Mesh * mesh, int index, double * point) {
    const netgen::Point3d & p = ((netgen::Mesh *)mesh)->Point(index + index_offset);

    point[0] = p.X();
    point[1] = p.Y();
    point[2] = p.Z();
}

void GetSurfaceElement(Mesh * mesh, int index, int * indices) {
    const netgen::Element2d & element = ((netgen::Mesh *)mesh)
        ->SurfaceElement(index + index_offset);

    indices[0] = element.PNum(1) - index_offset;
    indices[1] = element.PNum(2) - index_offset;
    indices[2] = element.PNum(3) - index_offset;
}

void GetVolumeElement(Mesh * mesh, int index, int * indices) {
    const netgen::Element & element = ((netgen::Mesh *)mesh)
        ->VolumeElement(index + index_offset);

    indices[0] = element.PNum(1) - index_offset;
    indices[1] = element.PNum(2) - index_offset;
    indices[2] = element.PNum(3) - index_offset;
    indices[3] = element.PNum(4) - index_offset;
}

void RestrictMeshSizeGlobal(Mesh * mesh, double size) {
    ((netgen::Mesh *)mesh)->SetGlobalH(size);
}

void RestrictMeshSizePoint(Mesh * mesh, double * point, double size) {
    ((netgen::Mesh *)mesh)->RestrictLocalH(netgen::Point3d(point[0],
        point[1], point[2]), size);
}

void RestrictMeshSizeBox(Mesh * mesh,
    double * min_point, double * max_point, double size) {
    for (double x = min_point[0]; x < max_point[0]; x += size) {
        for (double y = min_point[1]; y < max_point[1]; y += size) {
            for (double z = min_point[2]; z < max_point[2]; z += size)
                ((netgen::Mesh *)mesh)->RestrictLocalH(netgen::Point3d(x, y, z), size);
        }
    }
}

Status GenerateVolumeMesh(Mesh * mesh, MeshingParameters * parameters) {
    netgen::Mesh * m = (netgen::Mesh *)mesh;

    parameters->transferParameters();

    m->CalcLocalH(netgen::mparam.grading);

    int result = MeshVolume(netgen::mparam, *m);

    if (result != netgen::MESHING3_RESULT::MESHING3_OK)
        return Status::Failure;

    RemoveIllegalElements(*m);

    result = OptimizeVolume(netgen::mparam, *m);

    if (result != netgen::MESHING3_RESULT::MESHING3_OK)
        return Status::Failure;

    return Status::Success;
}

void UniformRefinement(Mesh * mesh) {
    netgen::Refinement refinement;

    refinement.Refine(*(netgen::Mesh *)mesh);
}

void GenerateSecondOrder(Mesh * mesh) {
    netgen::Refinement refinement;

    refinement.MakeSecondOrder(*(netgen::Mesh *)mesh);
}

STLGeometry * STLNewGeometry() {
    return (STLGeometry *)(void *)new netgen::STLGeometry;
}

void STLDeleteGeometry(STLGeometry * geometry) {
    if (geometry != nullptr) {
        delete (netgen::STLGeometry *)(void *)geometry;

        geometry = nullptr;
    }
}

void STLAddTriangle(STLGeometry * geometry,
    double * point1, double * point2, double * point3, double * normal) {
    netgen::Point<3> points[3];

    points[0] = netgen::Point<3>(point1[0], point1[1], point1[2]);
    points[1] = netgen::Point<3>(point2[0], point2[1], point2[2]);
    points[2] = netgen::Point<3>(point3[0], point3[1], point3[2]);

    netgen::Vec<3> n;

    if (normal != nullptr)
        n = netgen::Vec<3>(normal[0], normal[1], normal[2]);
    else
        n = netgen::Cross(points[0] - points[1], points[0] - points[2]);

    STLTriangleList.Append(netgen::STLReadTriangle(points, n));
}

Status STLInitializeGeometry(STLGeometry * geometry,
    Mesh * mesh, MeshingParameters * parameters) {
    netgen::STLGeometry * geo = (netgen::STLGeometry *)geometry;
    netgen::Mesh * m = (netgen::Mesh *)mesh;

    geo->InitSTLGeometry(STLTriangleList);
    STLTriangleList.SetSize(0);

    parameters->transferParameters();

    netgen::Vec3d offset = geo->GetBoundingBox().PMax() - geo->GetBoundingBox().PMin();
    offset *= 0.1;

    m->SetGlobalH(netgen::mparam.maxh);
    m->SetLocalH(geo->GetBoundingBox().PMin() - offset,
        geo->GetBoundingBox().PMax() + offset, netgen::mparam.grading);

    if (!netgen::mparam.meshsizefilename.empty())
        m->LoadLocalMeshSize(netgen::mparam.meshsizefilename);

    netgen::STLMeshing(*geo, *m);

    geo->edgesfound = 1;
    geo->surfacemeshed = 0;
    geo->surfaceoptimized = 0;
    geo->volumemeshed = 0;

    if (geo->GetStatus() == netgen::STLTopology::STL_GEOM_STATUS::STL_GOOD
        || geo->GetStatus() == netgen::STLTopology::STL_GEOM_STATUS::STL_WARNING)
        return Status::Success;

    return Status::Failure;
}

Status STLGenerateSurfaceMesh(STLGeometry * geometry,
    Mesh * mesh, MeshingParameters * parameters) {
    netgen::STLGeometry * geo = (netgen::STLGeometry *)geometry;
    netgen::Mesh * m = (netgen::Mesh *)mesh;

    parameters->transferParameters();

    int result = netgen::STLSurfaceMeshing(*geo, *m);

    if (result != netgen::MESHING3_RESULT::MESHING3_OK)
        return Status::Failure;

    geo->edgesfound = 1;
    geo->surfacemeshed = 1;
    geo->surfaceoptimized = 0;
    geo->volumemeshed = 0;

    netgen::STLSurfaceOptimization(*geo, *m, netgen::mparam);

    return Status::Success;
}

void STLUniformRefinement(STLGeometry * geometry, Mesh * mesh) {
    ((netgen::STLGeometry *)geometry)->GetRefinement()
        .Refine(*(netgen::Mesh *)mesh);
}

void STLGenerateSecondOrder(STLGeometry * geometry, Mesh * mesh) {
    ((netgen::STLGeometry *)geometry)->GetRefinement()
        .MakeSecondOrder(*(netgen::Mesh *)mesh);
}

NGLIB_NAMESPACE_END