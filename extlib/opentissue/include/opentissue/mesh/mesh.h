// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_MESH_H
#define OPENTISSUE_MESH_H

#include <opentissue/configuration.h>

#include <opentissue/mesh/default_point_container.h>
#include <opentissue/mesh/default_traits.h>
#include <opentissue/mesh/boundary_faces.h>
#include <opentissue/mesh/edges.h>
#include <opentissue/mesh/tetrahedral_mesh.h>
#include <opentissue/mesh/node.h>
#include <opentissue/mesh/tetrahedron.h>

#include <opentissue/math/basic_types.h>

namespace opentissue {
    namespace mesh {
        template <typename M = opentissue::math::BasicMathTypes<double, size_t>,
            typename N = mesh::DefaultNodeTraits<M>,
            typename T = mesh::DefaultTetrahedronTraits>
            class TetrahedralMesh : public opentissue::mesh::detail::TetrahedralMesh<M, N, T> {};
    }
}

#endif