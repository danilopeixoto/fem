// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_FEM_MESH_H
#define OPENTISSUE_FEM_MESH_H

#include <opentissue/configuration.h>

#include <opentissue/mesh/mesh.h>
#include <opentissue/fem/node_traits.h>
#include <opentissue/fem/tetrahedron_traits.h>

namespace opentissue {
    namespace fem {
        template<typename math_types>
        class Mesh : public opentissue::mesh::TetrahedralMesh<math_types,
            opentissue::fem::detail::NodeTraits<math_types>,
            opentissue::fem::detail::TetrahedronTraits<math_types>> {
        public:
            typedef typename math_types::real_type real_type;
            typedef typename math_types::value_traits value_traits;
            typedef typename math_types::vector_type vector_type;
            typedef typename math_types::matrix_type matrix_type;

            Mesh() {}

            Mesh(Mesh const & copy) : TetrahedralMesh(copy) {}

            Mesh(size_t size_nodes, size_t size_tetrahedra)
                : TetrahedralMesh(size_nodes, size_tetrahedra) {}
        };
    }
}

#endif