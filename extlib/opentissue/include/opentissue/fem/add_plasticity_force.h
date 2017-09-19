// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_FEM_ADD_PLASTICITY_FORCE_H
#define OPENTISSUE_FEM_ADD_PLASTICITY_FORCE_H

#include <opentissue/configuration.h>

namespace opentissue {
    namespace fem {
        namespace detail {
            template <typename tetrahedron_iterator, typename real_type>
            inline void add_plasticity_force(tetrahedron_iterator begin,
                tetrahedron_iterator end,
                real_type const &dt) {
                using std::min;
                using std::sqrt;

                typedef typename tetrahedron_iterator::value_type::vector_type vector_type;

                for (tetrahedron_iterator T = begin; T != end; ++T) {
                    real_type e_total[6];
                    real_type e_elastic[6];

                    for (int i = 0; i < 6; ++i)
                        e_elastic[i] = e_total[i] = 0;

                    for (unsigned int j = 0; j < 4; ++j) {
                        vector_type &x_j = T->node(j)->m_coord;
                        vector_type &x0_j = T->node(j)->m_model_coord;

                        vector_type tmp = (trans(T->m_Re) * x_j) - x0_j;
                        real_type bj = T->m_B[j](0);
                        real_type cj = T->m_B[j](1);
                        real_type dj = T->m_B[j](2);

                        e_total[0] += bj * tmp(0);
                        e_total[1] += cj * tmp(1);
                        e_total[2] += dj * tmp(2);
                        e_total[3] += cj * tmp(0) + bj * tmp(1);
                        e_total[4] += dj * tmp(0) + bj * tmp(2);
                        e_total[5] += dj * tmp(1) + cj * tmp(2);
                    }

                    for (int i = 0; i < 6; ++i)
                        e_elastic[i] = e_total[i] - T->m_plastic[i];

                    real_type norm_elastic = 0;

                    for (int i = 0; i < 6; ++i)
                        norm_elastic += e_elastic[i] * e_elastic[i];

                    norm_elastic = sqrt(norm_elastic);

                    if (norm_elastic > T->m_yield) {
                        real_type amount = min(T->m_creep, 1.0 / dt)
                            * (1.0 - T->m_yield / norm_elastic);

                        for (int i = 0; i < 6; ++i)
                            T->m_plastic[i] += amount * e_elastic[i];
                    }

                    real_type norm_plastic = 0;

                    for (int i = 0; i < 6; ++i)
                        norm_plastic += T->m_plastic[i] * T->m_plastic[i];

                    norm_plastic = sqrt(norm_plastic);

                    if (norm_plastic > T->m_max_yield) {
                        real_type scale = T->m_max_yield / norm_plastic;

                        for (int i = 0; i < 6; ++i)
                            T->m_plastic[i] *= scale;
                    }

                    for (unsigned int j = 0; j < 4; ++j) {
                        real_type *plastic = T->m_plastic;
                        real_type bj = T->m_B[j](0);
                        real_type cj = T->m_B[j](1);
                        real_type dj = T->m_B[j](2);
                        real_type E0 = T->m_D(0);
                        real_type E1 = T->m_D(1);
                        real_type E2 = T->m_D(2);

                        vector_type f;

                        real_type bjE0 = bj * E0;
                        real_type bjE1 = bj * E1;
                        real_type bjE2 = bj * E2;
                        real_type cjE0 = cj * E0;
                        real_type cjE1 = cj * E1;
                        real_type cjE2 = cj * E2;
                        real_type djE0 = dj * E0;
                        real_type djE1 = dj * E1;
                        real_type djE2 = dj * E2;

                        f(0) = bjE0 * plastic[0] + bjE1 * plastic[1] + bjE1 * plastic[2] +
                            cjE2 * plastic[3] + djE2 * plastic[4];
                        f(1) = cjE1 * plastic[0] + cjE0 * plastic[1] + cjE1 * plastic[2] +
                            bjE2 * plastic[3] + +djE2 * plastic[5];
                        f(2) = djE1 * plastic[0] + djE1 * plastic[1] + djE0 * plastic[2] +
                            bjE2 * plastic[4] + cjE2 * plastic[5];

                        f *= T->m_V;
                        T->node(j)->m_f_external += T->m_Re * f;
                    }
                }
            }
        }
    }
}

#endif