// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_FEM_CONJUGATE_GRADIENTS_H
#define OPENTISSUE_FEM_CONJUGATE_GRADIENTS_H

#include <opentissue/configuration.h>

#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include <tbb/blocked_range.h>

#include <cmath>
#include <array>

namespace opentissue {
    namespace fem {
        namespace detail {
            template<typename fem_mesh, typename real_type>
            inline void conjugate_gradients(fem_mesh &mesh, real_type const &dt,
                unsigned int max_iterations, real_type const &tolerance) {
                using std::fabs;

                typedef typename fem_mesh::value_traits value_traits;
                typedef typename fem_mesh::vector_type vector_type;
                typedef typename fem_mesh::matrix_type matrix_type;
                typedef typename fem_mesh::node_iterator node_iterator;
                typedef typename fem_mesh::node_type::matrix_iterator matrix_iterator;
                typedef std::array<real_type, 2> dual_real_type;

                node_iterator nbegin = mesh.node_begin();
                node_iterator nend = mesh.node_end();

                tbb::parallel_for(size_t(0), mesh.size_nodes(), [&](size_t i) {
                    node_iterator node = mesh.node(i);

                    if (node->m_fixed)
                        return;

                    node->m_residual = node->m_b;

                    matrix_iterator Abegin = node->Abegin();
                    matrix_iterator Aend = node->Aend();

                    for (matrix_iterator A = Abegin; A != Aend; A++) {
                        unsigned int j = A->first;

                        matrix_type &A_ij = A->second;
                        node_iterator n_j = mesh.node(j);
                        vector_type &v_j = n_j->m_velocity;

                        node->m_residual -= A_ij * v_j;
                    }

                    node->m_prev = node->m_residual;
                });

                for (unsigned int iteration = 0; iteration < max_iterations; iteration++) {
                    dual_real_type d;
                    d.fill(0.0);

                    d = tbb::parallel_reduce(tbb::blocked_range<size_t>(0, mesh.size_nodes()),
                        d, [&](const tbb::blocked_range<size_t> &range, dual_real_type s)
                        -> dual_real_type {
                        for (size_t i = range.begin(); i != range.end(); i++) {
                            node_iterator node = mesh.node(i);

                            if (node->m_fixed)
                                continue;

                            node->m_update.clear();

                            matrix_iterator Abegin = node->Abegin();
                            matrix_iterator Aend = node->Aend();

                            for (matrix_iterator A = Abegin; A != Aend; A++) {
                                unsigned int j = A->first;

                                node_iterator n_j = mesh.node(j);
                                matrix_type &A_ij = A->second;

                                node->m_update += A_ij * n_j->m_prev;
                            }

                            s[0] += node->m_residual * node->m_residual;
                            s[1] += node->m_prev * node->m_update;
                        }

                        return s;
                    }, [](dual_real_type const &a, dual_real_type const &b) -> dual_real_type {
                        dual_real_type s;

                        s[0] = a[0] + b[0];
                        s[1] = a[1] + b[1];

                        return s;
                    });

                    if (fabs(d[1]) < value_traits::epsilon())
                        d[1] = value_traits::epsilon();

                    real_type d2 = d[0] / d[1];
                    real_type d3 = 0;

                    for (node_iterator n = nbegin; n != nend; n++) {
                        if (n->m_fixed)
                            continue;

                        n->m_velocity += n->m_prev * d2;
                        n->m_residual -= n->m_update * d2;
                        d3 += n->m_residual * n->m_residual;
                    }

                    if (d3 < tolerance)
                        break;

                    if (fabs(d[0]) < value_traits::epsilon())
                        d[0] = value_traits::epsilon();

                    real_type d4 = d3 / d[0];

                    for (node_iterator n = nbegin; n != nend; n++) {
                        if (n->m_fixed)
                            continue;

                        n->m_prev = n->m_residual + n->m_prev * d4;
                    }
                }

                for (node_iterator n = nbegin; n != nend; n++) {
                    if (n->m_fixed)
                        continue;

                    n->m_world_coord += dt * n->m_velocity;
                }
            }
        }
    }
}

#endif