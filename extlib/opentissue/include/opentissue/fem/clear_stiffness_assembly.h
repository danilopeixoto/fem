// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_FEM_CLEAR_STIFFNESS_ASSEMBLY_H
#define OPENTISSUE_FEM_CLEAR_STIFFNESS_ASSEMBLY_H

#include <opentissue/configuration.h>

namespace opentissue {
    namespace fem {
        namespace detail {
            template<typename fem_mesh>
            inline void clear_stiffness_assembly(fem_mesh &mesh) {
                typedef typename fem_mesh::node_iterator node_iterator;
                typedef typename fem_mesh::node_type::matrix_iterator matrix_iterator;

                node_iterator nbegin = mesh.node_begin();
                node_iterator nend = mesh.node_end();

                for (node_iterator n = nbegin; n != nend; n++) {
                    n->m_f0.clear();

                    for (matrix_iterator Kij = n->Kbegin(); Kij != n->Kend(); Kij++)
                        Kij->second.clear();
                }
            }
        }
    }
}

#endif