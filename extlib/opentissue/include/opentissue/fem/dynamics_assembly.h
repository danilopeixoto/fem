// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_FEM_DYNAMICS_ASSEMBLY_H
#define OPENTISSUE_FEM_DYNAMICS_ASSEMBLY_H

#include <opentissue/configuration.h>

#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>

namespace opentissue {
    namespace fem {
        namespace detail {
            template<typename fem_mesh, typename real_type>
            inline void dynamics_assembly(fem_mesh &mesh, real_type const &mass_damping,
                real_type const &stiffness_damping, real_type const &dt) {
                typedef typename fem_mesh::vector_type vector_type;
                typedef typename fem_mesh::matrix_type matrix_type;
                typedef typename fem_mesh::node_iterator node_iterator;
                typedef typename fem_mesh::node_type::matrix_iterator matrix_iterator;

                tbb::parallel_for(tbb::blocked_range<size_t>(0, mesh.size_nodes()),
                    [&](const tbb::blocked_range<size_t> &range) {
                    for (size_t i = range.begin(); i != range.end(); i++) {
                        node_iterator node = mesh.node(i);
                        unsigned int node_idx = (unsigned int)node->idx();

                        real_type const &m_i = node->m_mass;
                        vector_type &b_i = node->m_b;

                        b_i.clear();

                        matrix_iterator Kbegin = node->Kbegin();
                        matrix_iterator Kend = node->Kend();

                        for (matrix_iterator K = Kbegin; K != Kend; K++) {
                            unsigned int j = K->first;

                            matrix_type const &K_ij = K->second;
                            node_iterator n_j = mesh.node(j);
                            vector_type const &x_j = n_j->m_world_coord;

                            matrix_type &A_ij = node->A(j);

                            A_ij = K_ij * (dt * dt);
                            b_i -= K_ij * x_j;

                            matrix_type c_ij = stiffness_damping * K_ij;

                            if (node_idx == j) {
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

                        b_i -= node->m_f0;
                        b_i += node->m_f_external;
                        b_i *= dt;
                        b_i += node->m_velocity * m_i;
                    }
                });
            }
        }
    }
}

#endif