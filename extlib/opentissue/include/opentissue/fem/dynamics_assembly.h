// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_FEM_DYNAMICS_ASSEMBLY_H
#define OPENTISSUE_FEM_DYNAMICS_ASSEMBLY_H

#include <opentissue/configuration.h>

namespace opentissue {
    namespace fem {
        namespace detail {
            template <typename fem_mesh, typename real_type>
            inline void dynamics_assembly(fem_mesh &mesh, real_type const &mass_damping,
                real_type const &stiffness_damping, real_type const &dt) {
                typedef typename fem_mesh::vector_type vector_type;
                typedef typename fem_mesh::matrix_type matrix_type;
                typedef typename fem_mesh::node_iterator node_iterator;
                typedef typename fem_mesh::node_type::matrix_iterator matrix_iterator;

                node_iterator begin = mesh.node_begin();
                node_iterator end = mesh.node_end();

                for (node_iterator n_i = begin; n_i != end; ++n_i) {
                    unsigned int i = (unsigned int)n_i->idx();
                    vector_type &b_i = n_i->m_b;
                    real_type &m_i = n_i->m_mass;

                    b_i.clear();

                    matrix_iterator Kbegin = n_i->Kbegin();
                    matrix_iterator Kend = n_i->Kend();

                    for (matrix_iterator K = Kbegin; K != Kend; ++K) {
                        unsigned int j = K->first;
                        matrix_type &K_ij = K->second;
                        node_iterator n_j = mesh.node(j);
                        vector_type &x_j = n_j->m_coord;
                        matrix_type &A_ij = n_i->A(j);

                        A_ij = K_ij * (dt * dt);
                        b_i -= K_ij * x_j;

                        matrix_type c_ij = stiffness_damping * K_ij;

                        if (i == j) {
                            real_type tmp = mass_damping * m_i;

                            c_ij(0, 0) += tmp;
                            c_ij(1, 1) += tmp;
                            c_ij(2, 2) += tmp;

                            A_ij(0, 0) += m_i;
                            A_ij(1, 1) += m_i;
                            A_ij(2, 2) += m_i;
                        }

                        A_ij += dt * c_ij;
                    }

                    b_i -= n_i->m_f0;
                    b_i += n_i->m_f_external;
                    b_i *= dt;
                    b_i += n_i->m_velocity * m_i;
                }
            }
        }
    }
}

#endif