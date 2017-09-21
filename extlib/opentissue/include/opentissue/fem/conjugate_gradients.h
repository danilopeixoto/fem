// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_FEM_CONJUGATE_GRADIENTS_H
#define OPENTISSUE_FEM_CONJUGATE_GRADIENTS_H

#include <opentissue/configuration.h>

namespace opentissue {
    namespace fem {
        namespace detail {
            template <typename fem_mesh>
            inline void conjugate_gradients(fem_mesh &mesh, unsigned int max_iterations) {
                using std::fabs;

                typedef typename fem_mesh::real_type real_type;
                typedef typename fem_mesh::vector_type vector_type;
                typedef typename fem_mesh::matrix_type matrix_type;
                typedef typename fem_mesh::node_iterator node_iterator;
                typedef typename fem_mesh::node_type::matrix_iterator matrix_iterator;

                node_iterator begin = mesh.node_begin();
                node_iterator end = mesh.node_end();

                real_type epsilon = 1.0e-6;

                for (node_iterator n_i = begin; n_i != end; ++n_i) {
                    if (n_i->m_fixed)
                        continue;

                    n_i->m_residual = n_i->m_b;

                    matrix_iterator Abegin = n_i->Abegin();
                    matrix_iterator Aend = n_i->Aend();

                    for (matrix_iterator A = Abegin; A != Aend; ++A) {
                        unsigned int j = A->first;
                        matrix_type &A_ij = A->second;
                        node_iterator n_j = mesh.node(j);
                        vector_type &v_j = n_j->m_velocity;

                        n_i->m_residual -= A_ij * v_j;
                    }

                    n_i->m_prev = n_i->m_residual;
                }

                for (unsigned int iteration = 0; iteration < max_iterations; ++iteration) {
                    real_type d = 0;
                    real_type d2 = 0;

                    for (node_iterator n_i = begin; n_i != end; ++n_i) {
                        if (n_i->m_fixed)
                            continue;

                        n_i->m_update.clear();

                        matrix_iterator Abegin = n_i->Abegin();
                        matrix_iterator Aend = n_i->Aend();

                        for (matrix_iterator A = Abegin; A != Aend; ++A) {

                            unsigned int j = A->first;
                            node_iterator n_j = mesh.node(j);
                            matrix_type &A_ij = A->second;

                            n_i->m_update += A_ij * n_j->m_prev;
                        }

                        d += n_i->m_residual * n_i->m_residual;
                        d2 += n_i->m_prev * n_i->m_update;
                    }

                    if (fabs(d2) < epsilon)
                        d2 = epsilon;

                    real_type d3 = d / d2;
                    real_type d1 = 0;

                    for (node_iterator n_i = begin; n_i != end; ++n_i) {
                        if (n_i->m_fixed)
                            continue;

                        n_i->m_velocity += n_i->m_prev * d3;
                        n_i->m_residual -= n_i->m_update * d3;
                        d1 += n_i->m_residual * n_i->m_residual;
                    }

                    if (d1 < epsilon)
                        break;

                    if (fabs(d) < epsilon)
                        d = epsilon;

                    real_type d4 = d1 / d;

                    for (node_iterator n_i = begin; n_i != end; ++n_i) {
                        if (n_i->m_fixed)
                            continue;

                        n_i->m_prev = n_i->m_residual + n_i->m_prev * d4;
                    }
                }
            }
        }
    }
}

#endif