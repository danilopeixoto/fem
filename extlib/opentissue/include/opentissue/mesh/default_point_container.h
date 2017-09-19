// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_MESH_DEFAULT_POINT_CONTAINER_H
#define OPENTISSUE_MESH_DEFAULT_POINT_CONTAINER_H

#include <opentissue/configuration.h>

namespace opentissue {
    namespace mesh {
        template <typename M> struct default_point_container {
            typedef M tetrahedral_mesh_type;
            typedef typename tetrahedral_mesh_type::math_types math_types;
            typedef typename math_types::vector_type value_type;

            tetrahedral_mesh_type *m_mesh;

            default_point_container(tetrahedral_mesh_type *mesh) : m_mesh(mesh) {}

            value_type &operator[](unsigned int const &idx) {
                return m_mesh->node(idx)->m_model_coord;
            }

            value_type const &operator[](unsigned int const &idx) const {
                return m_mesh->node(idx)->m_model_coord;
            }

            void clear() {}
            size_t size() const { return m_mesh->size_nodes(); }
            void resize(size_t) {}
        };
    }
}

#endif