// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_FEM_UPDATE_POSITION_H
#define OPENTISSUE_FEM_UPDATE_POSITION_H

#include <opentissue/configuration.h>

namespace opentissue {
    namespace fem {
        namespace detail {
            template <typename fem_mesh, typename real_type>
            inline void update_position(fem_mesh &mesh, real_type const &dt) {
                typedef typename fem_mesh::node_iterator node_iterator;
                typedef typename fem_mesh::tetrahedron_iterator tetrahedron_iterator;

                node_iterator Nbegin = mesh.node_begin();
                node_iterator Nend = mesh.node_end();

                for (node_iterator N = Nbegin; N != Nend; ++N) {
                    if (N->m_fixed)
                        continue;

                    N->m_coord += dt * N->m_velocity;
                }
            }
        }
    }
}

#endif