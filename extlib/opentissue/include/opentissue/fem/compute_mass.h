// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_FEM_COMPUTE_MASS_H
#define OPENTISSUE_FEM_COMPUTE_MASS_H

#include <opentissue/configuration.h>

namespace opentissue {
    namespace fem {
        namespace detail {
            template<typename fem_mesh>
            inline void compute_mass(fem_mesh &mesh) {
                typedef typename fem_mesh::real_type real_type;
                typedef typename fem_mesh::value_traits value_traits;
                typedef typename fem_mesh::node_iterator node_iterator;
                typedef typename fem_mesh::tetrahedron_iterator tetrahedron_iterator;

                node_iterator nbegin = mesh.node_begin();
                node_iterator nend = mesh.node_end();

                for (node_iterator n = nbegin; n != nend; n++)
                    n->m_mass = n->m_fixed ? value_traits::infinity() : 0;

                tetrahedron_iterator tbegin = mesh.tetrahedron_begin();
                tetrahedron_iterator tend = mesh.tetrahedron_end();

                for (tetrahedron_iterator t = tbegin; t != tend; t++) {
                    real_type amount = t->m_density * t->m_volume * 0.25;

                    t->i()->m_mass += amount;
                    t->j()->m_mass += amount;
                    t->k()->m_mass += amount;
                    t->m()->m_mass += amount;
                }
            }
        }
    }
}

#endif