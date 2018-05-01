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

#ifndef NGLIB_H
#define NGLIB_H

#ifndef NGLIB_EXPORTS
#define DLL_HEADER __declspec(dllimport)
#else
#define DLL_HEADER __declspec(dllexport)
#endif

#define NGLIB_NAMESPACE_BEGIN namespace nglib {
#define NGLIB_NAMESPACE_END }

// Maximum allowed number of nodes per volume element.
#define VOLUME_ELEMENT_MAX_POINTS 10

// Maximum allowed number of nodes per surface element.
#define SURFACE_ELEMENT_MAX_POINTS 8

NGLIB_NAMESPACE_BEGIN

// Status returned by Netgen functions.
enum Status {
    Success = 0,
    Failure
};

// Data type for Netgen mesh.
typedef void * Mesh;

// Data type for Netgen STL geometry.
typedef void * STLGeometry;

// Meshing parameters class.
class DLL_HEADER MeshingParameters {
public:
    bool local_element_size; // Use local element size modifiers.

    double min_element_size; // Minimum global element size allowed.
    double max_element_size; // Maximum global element size allowed.

    double grading; // Mesh grading (zero results a uniform mesh).

    double elements_per_edge; // Number of elements to generate per edge of the geometry.
    double elements_per_curve; // Elements to generate per curvature radius.

    bool second_order; // Generate second-order surface and volume elements.

    char * element_size_filename; // Optional external element size file.

    int optimization_steps; // Number of optimization steps to be used for mesh.

    bool invert_surface; // Invert all the surface triangle elements.
    bool invert_volume; // Invert all the surface triangle elements.

    bool check_overlaps; // Check for overlapping surfaces during surface meshing.
    bool check_overlapping_boundary; // Check for overlapping surface elements before volume meshing.

    // Default constructor for meshing parameters class.
    MeshingParameters();

    // Reset the meshing parameters to their defaults.
    void resetParameters();

    // Transfer local meshing parameters to internal meshing parameters.
    void transferParameters();
};

// Initialize Netgen library.
DLL_HEADER void Initialize(int first_index = 0, bool suppress_output = true);

// Create a new and empty Netgen mesh.
DLL_HEADER Mesh * NewMesh();

// Delete an existing Netgen mesh.
DLL_HEADER void DeleteMesh(Mesh * mesh);

// Returns the number of points inside a Netgen mesh.
DLL_HEADER int GetPointCount(Mesh * mesh);

// Returns the number of surface elements inside a Netgen mesh.
DLL_HEADER int GetSurfaceCount(Mesh * mesh);

// Returns the number of volume elements inside a Netgen mesh.
DLL_HEADER int GetVolumeCount(Mesh * mesh);

// Add a point to a Netgen mesh.
DLL_HEADER void AddPoint(Mesh * mesh, double * point);

// Add a surface element to a Netgen mesh.
DLL_HEADER void AddSurfaceElement(Mesh * mesh, int * indices);

// Add a volume element to a Netgen mesh.
DLL_HEADER void AddVolumeElement(Mesh * mesh, int * indices);

// Returns the point coordinates at a given index.
DLL_HEADER void GetPoint(Mesh * mesh, int index, double * point);

// Returns surface element at a given index.
DLL_HEADER void GetSurfaceElement(Mesh * mesh, int index, int * indices);

// Returns volume element at a given index.
DLL_HEADER void GetVolumeElement(Mesh * mesh, int index, int * indices);

// Apply a global restriction on mesh element size.
DLL_HEADER void RestrictMeshSizeGlobal(Mesh * mesh, double size);

// Locally restrict the mesh element size at the given point.
DLL_HEADER void RestrictMeshSizePoint(Mesh * mesh, double * point, double size);

// Locally restrict the mesh element size within a specified box.
DLL_HEADER void RestrictMeshSizeBox(Mesh * mesh,
    double * min_point, double * max_point, double size);

// Create a volume mesh given a surface Mesh.
DLL_HEADER Status GenerateVolumeMesh(Mesh * mesh, MeshingParameters * parameters);

// Uniform mesh refinement.
DLL_HEADER void UniformRefinement(Mesh * mesh);

// Generates second-order elements.
DLL_HEADER void GenerateSecondOrder(Mesh * mesh);

// Create a new and empty STL geometry.
DLL_HEADER STLGeometry * STLNewGeometry();

// Delete an existing STL geometry.
DLL_HEADER void STLDeleteGeometry(STLGeometry * geometry);

// Add a triangle to a STL geometry.
DLL_HEADER void STLAddTriangle(STLGeometry * geometry,
    double * point1, double * point2, double * point3, double * normal = nullptr);

// Call this function always after adding triangles to a STL geometry.
DLL_HEADER Status STLInitializeGeometry(STLGeometry * geometry,
    Mesh * mesh, MeshingParameters * parameters);

// Generates a surface mesh from STL geometry.
DLL_HEADER Status STLGenerateSurfaceMesh(STLGeometry * geometry,
    Mesh * mesh, MeshingParameters * parameters);

// Uniform mesh refinement with geometry adaptation.
DLL_HEADER void STLUniformRefinement(STLGeometry * geometry, Mesh * mesh);

// Generates second-order elements with geometry adaptation.
DLL_HEADER void STLGenerateSecondOrder(STLGeometry * geometry, Mesh * mesh);

NGLIB_NAMESPACE_END

#endif