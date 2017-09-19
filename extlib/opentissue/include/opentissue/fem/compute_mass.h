// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_FEM_COMPUTE_MASS_H
#define OPENTISSUE_FEM_COMPUTE_MASS_H

#include <opentissue/configuration.h>

#include <limits>

namespace opentissue {
    namespace fem {
        namespace detail {
            template <typename fem_mesh> inline void compute_mass(fem_mesh &mesh) {
                typedef typename fem_mesh::node_iterator node_iterator;
                typedef typename fem_mesh::tetrahedron_iterator tetrahedron_iterator;
                typedef typename fem_mesh::real_type real_type;

                node_iterator Nbegin = mesh.node_begin();
                node_iterator Nend = mesh.node_end();

                for (node_iterator N = Nbegin; N != Nend; ++N) {
                    if (N->m_fixed)
                        N->m_mass = std::numeric_limits<real_type>::max();
                    else
                        N->m_mass = 0;
                }

                tetrahedron_iterator Tbegin = mesh.tetrahedron_begin();
                tetrahedron_iterator Tend = mesh.tetrahedron_end();

                for (tetrahedron_iterator T = Tbegin; T != Tend; ++T) {
                    real_type amount = T->m_density * T->m_V * 0.25;
                    T->i()->m_mass += amount;
                    T->j()->m_mass += amount;
                    T->k()->m_mass += amount;
                    T->m()->m_mass += amount;
                }
            }
        }
    }
}

#endif