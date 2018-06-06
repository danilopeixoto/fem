// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_FEM_APPLY_PLASTICITY_FORCE_H
#define OPENTISSUE_FEM_APPLY_PLASTICITY_FORCE_H

#include <opentissue/configuration.h>

#include <opentissue/math/math.h>

#include <tbb/parallel_for.h>

#include <cmath>

namespace opentissue {
    namespace fem {
        namespace detail {
            template<typename fem_mesh>
            inline void apply_plasticity_force(fem_mesh &mesh) {
                typedef typename fem_mesh::real_type real_type;
                typedef typename fem_mesh::vector_type vector_type;
                typedef typename fem_mesh::node_iterator node_iterator;
                typedef typename fem_mesh::tetrahedron_iterator tetrahedron_iterator;

                tbb::parallel_for(size_t(0), mesh.size_tetrahedra(), [&](size_t i) {
                    tetrahedron_iterator tet = mesh.tetrahedron(i);

                    real_type e_total[6];
                    real_type e_elastic[6];

                    for (unsigned int j = 0; j < 6; j++)
                        e_elastic[j] = e_total[j] = 0;

                    for (unsigned int j = 0; j < 4; j++) {
                        node_iterator node = tet->node(j);

                        vector_type &x = node->m_world_coord;
                        vector_type &x0 = node->m_coord;

                        vector_type d = math::trans(tet->m_Re) * x - x0;

                        real_type bj = tet->m_B[j](0);
                        real_type cj = tet->m_B[j](1);
                        real_type dj = tet->m_B[j](2);

                        e_total[0] = bj * d(0);
                        e_total[1] = cj * d(1);
                        e_total[2] = dj * d(2);
                        e_total[3] = cj * d(0) + bj * d(1);
                        e_total[4] = dj * d(0) + bj * d(2);
                        e_total[5] = dj * d(1) + cj * d(2);
                    }

                    for (unsigned int j = 0; j < 6; j++)
                        e_elastic[j] = e_total[j] - tet->m_plastic[j];

                    real_type norm_elastic = 0;

                    for (unsigned int j = 0; j < 6; j++)
                        norm_elastic += e_elastic[j] * e_elastic[j];

                    norm_elastic = std::sqrt(norm_elastic);

                    if (norm_elastic > tet->m_yield) {
                        real_type amount = tet->m_creep * (1.0 - tet->m_yield / norm_elastic);

                        for (unsigned int j = 0; j < 6; j++)
                            tet->m_plastic[j] += amount * e_elastic[j];
                    }

                    real_type norm_plastic = 0;

                    for (unsigned int j = 0; j < 6; j++)
                        norm_plastic += tet->m_plastic[j] * tet->m_plastic[j];

                    norm_plastic = std::sqrt(norm_plastic);

                    if (norm_plastic > tet->m_max_yield) {
                        real_type scale = tet->m_max_yield / norm_plastic;

                        for (unsigned int j = 0; j < 6; j++)
                            tet->m_plastic[j] *= scale;
                    }

                    real_type const *plastic = tet->m_plastic;

                    for (unsigned int j = 0; j < 4; j++) {
                        real_type const &bj = tet->m_B[j](0);
                        real_type const &cj = tet->m_B[j](1);
                        real_type const &dj = tet->m_B[j](2);
                        real_type const &E0 = tet->m_E(0);
                        real_type const &E1 = tet->m_E(1);
                        real_type const &E2 = tet->m_E(2);

                        real_type bjE0 = bj * E0;
                        real_type bjE1 = bj * E1;
                        real_type bjE2 = bj * E2;
                        real_type cjE0 = cj * E0;
                        real_type cjE1 = cj * E1;
                        real_type cjE2 = cj * E2;
                        real_type djE0 = dj * E0;
                        real_type djE1 = dj * E1;
                        real_type djE2 = dj * E2;

                        vector_type f;

                        f(0) = bjE0 * plastic[0] + bjE1 * plastic[1] + bjE1 * plastic[2] +
                            cjE2 * plastic[3] + djE2 * plastic[4];
                        f(1) = cjE1 * plastic[0] + cjE0 * plastic[1] + cjE1 * plastic[2] +
                            bjE2 * plastic[3] + +djE2 * plastic[5];
                        f(2) = djE1 * plastic[0] + djE1 * plastic[1] + djE0 * plastic[2] +
                            bjE2 * plastic[4] + cjE2 * plastic[5];

                        f *= tet->m_volume0;
                        tet->node(j)->m_f_external += tet->m_Re * f;
                    }
                });
            }
        }
    }
}

#endif