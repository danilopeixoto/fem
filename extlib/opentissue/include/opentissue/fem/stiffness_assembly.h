// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_FEM_STIFFNESS_ASSEMBLY_H
#define OPENTISSUE_FEM_STIFFNESS_ASSEMBLY_H

#include <opentissue/configuration.h>

#include <opentissue/math/math.h>

namespace opentissue {
    namespace fem {
        namespace detail {
            template<typename fem_mesh>
            inline void stiffness_assembly(fem_mesh &mesh) {
                typedef typename fem_mesh::vector_type vector_type;
                typedef typename fem_mesh::matrix_type matrix_type;
                typedef typename fem_mesh::node_iterator node_iterator;
                typedef typename fem_mesh::tetrahedron_iterator tetrahedron_iterator;

                tetrahedron_iterator tbegin = mesh.tetrahedron_begin();
                tetrahedron_iterator tend = mesh.tetrahedron_end();

                for (tetrahedron_iterator t = tbegin; t != tend; t++) {
                    matrix_type const &Re = t->m_Re;

                    for (unsigned int i = 0; i < 4; i++) {
                        node_iterator n_i = t->node(i);

                        vector_type f0;

                        for (unsigned int j = 0; j < 4; j++) {
                            node_iterator n_j = t->node(j);
                            vector_type const &x0_j = n_j->m_coord;
                            matrix_type const &Ke_ij = t->m_Ke[i][j];

                            f0 += Ke_ij * x0_j;

                            if (j >= i) {
                                matrix_type J_ij = Re * Ke_ij * math::trans(Re);

                                n_i->K((unsigned int)n_j->idx()) += J_ij;

                                if (j != i)
                                    n_j->K((unsigned int)n_i->idx()) += math::trans(J_ij);
                            }
                        }

                        n_i->m_f0 += Re * f0;
                    }
                }
            }
        }
    }
}

#endif