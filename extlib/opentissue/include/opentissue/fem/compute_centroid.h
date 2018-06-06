// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_FEM_COMPUTE_CENTROID_H
#define OPENTISSUE_FEM_COMPUTE_CENTROID_H

#include <opentissue/configuration.h>

namespace opentissue {
    namespace fem {
        namespace detail {
            template<typename tetrahedron_iterator>
            inline void compute_centroid(tetrahedron_iterator const &tetrahedron) {
                typedef typename tetrahedron_iterator::value_type::vector_type vector_type;

                vector_type const &n0 = tetrahedron->i()->m_world_coord;
                vector_type const &n1 = tetrahedron->j()->m_world_coord;
                vector_type const &n2 = tetrahedron->k()->m_world_coord;
                vector_type const &n3 = tetrahedron->m()->m_world_coord;

                tetrahedron->m_centroid = 0.25 * (n0 + n1 + n2 + n3);
            }
        }
    }
}

#endif