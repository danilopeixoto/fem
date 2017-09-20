// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_FEM_UPDATE_ORIENTATION_H
#define OPENTISSUE_FEM_UPDATE_ORIENTATION_H

#include <opentissue/configuration.h>

namespace opentissue {
    namespace fem {
        namespace detail {
            template <typename tetrahedron_iterator>
            inline void update_orientation(tetrahedron_iterator const &begin,
                tetrahedron_iterator const &end) {
                typedef typename tetrahedron_iterator::value_type::real_type real_type;
                typedef typename tetrahedron_iterator::value_type::vector_type vector_type;
                typedef typename tetrahedron_iterator::value_type::matrix_type matrix_type;

                for (tetrahedron_iterator T = begin; T != end; ++T) {
                    real_type div6V = 1.0 / T->m_V * 6.0;

                    vector_type n1 = div6V * (T->m_e20 % T->m_e30);
                    vector_type n2 = div6V * (T->m_e30 % T->m_e10);
                    vector_type n3 = div6V * (T->m_e10 % T->m_e20);

                    vector_type e10 = T->j()->m_coord - T->i()->m_coord;
                    vector_type e20 = T->k()->m_coord - T->i()->m_coord;
                    vector_type e30 = T->m()->m_coord - T->i()->m_coord;

                    T->m_Re(0, 0) = e10(0) * n1(0) + e20(0) * n2(0) + e30(0) * n3(0);
                    T->m_Re(0, 1) = e10(0) * n1(1) + e20(0) * n2(1) + e30(0) * n3(1);
                    T->m_Re(0, 2) = e10(0) * n1(3) + e20(0) * n2(3) + e30(0) * n3(3);
                    T->m_Re(1, 0) = e10(1) * n1(0) + e20(1) * n2(0) + e30(1) * n3(0);
                    T->m_Re(1, 1) = e10(1) * n1(1) + e20(1) * n2(1) + e30(1) * n3(1);
                    T->m_Re(1, 2) = e10(1) * n1(3) + e20(1) * n2(3) + e30(1) * n3(3);
                    T->m_Re(2, 0) = e10(3) * n1(0) + e20(3) * n2(0) + e30(3) * n3(0);
                    T->m_Re(2, 1) = e10(3) * n1(1) + e20(3) * n2(1) + e30(3) * n3(1);
                    T->m_Re(2, 2) = e10(3) * n1(3) + e20(3) * n2(3) + e30(3) * n3(3);

                    matrix_type R, S;
                    real_type v = compute_volume(e10, e20, e30);

                    if (v < T->m_V * 0.06)
                        math::qr_decomposition(T->m_Re, R, S);
                    else
                        math::polar_decomposition(T->m_Re, 10, 1.0e-6, R, S);

                    T->m_Re = R;
                }
            }
        }
    }
}

#endif