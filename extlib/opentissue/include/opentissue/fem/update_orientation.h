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

                    real_type e1x = T->m_e10(0);
                    real_type e1y = T->m_e10(1);
                    real_type e1z = T->m_e10(2);
                    real_type e2x = T->m_e20(0);
                    real_type e2y = T->m_e20(1);
                    real_type e2z = T->m_e20(2);
                    real_type e3x = T->m_e30(0);
                    real_type e3y = T->m_e30(1);
                    real_type e3z = T->m_e30(2);
                    real_type n1x = (e2y * e3z - e3y * e2z) * div6V;
                    real_type n1y = (e3x * e2z - e2x * e3z) * div6V;
                    real_type n1z = (e2x * e3y - e3x * e2y) * div6V;
                    real_type n2x = (e1z * e3y - e1y * e3z) * div6V;
                    real_type n2y = (e1x * e3z - e1z * e3x) * div6V;
                    real_type n2z = (e1y * e3x - e1x * e3y) * div6V;
                    real_type n3x = (e1y * e2z - e1z * e2y) * div6V;
                    real_type n3y = (e1z * e2x - e1x * e2z) * div6V;
                    real_type n3z = (e1x * e2y - e1y * e2x) * div6V;

                    vector_type &p0 = T->i()->m_coord;
                    vector_type &p1 = T->j()->m_coord;
                    vector_type &p2 = T->k()->m_coord;
                    vector_type &p3 = T->m()->m_coord;

                    e1x = p1(0) - p0(0);
                    e1y = p1(1) - p0(1);
                    e1z = p1(2) - p0(2);
                    e2x = p2(0) - p0(0);
                    e2y = p2(1) - p0(1);
                    e2z = p2(2) - p0(2);
                    e3x = p3(0) - p0(0);
                    e3y = p3(1) - p0(1);
                    e3z = p3(2) - p0(2);

                    T->m_Re(0, 0) = e1x * n1x + e2x * n2x + e3x * n3x;
                    T->m_Re(0, 1) = e1x * n1y + e2x * n2y + e3x * n3y;
                    T->m_Re(0, 2) = e1x * n1z + e2x * n2z + e3x * n3z;
                    T->m_Re(1, 0) = e1y * n1x + e2y * n2x + e3y * n3x;
                    T->m_Re(1, 1) = e1y * n1y + e2y * n2y + e3y * n3y;
                    T->m_Re(1, 2) = e1y * n1z + e2y * n2z + e3y * n3z;
                    T->m_Re(2, 0) = e1z * n1x + e2z * n2x + e3z * n3x;
                    T->m_Re(2, 1) = e1z * n1y + e2z * n2y + e3z * n3y;
                    T->m_Re(2, 2) = e1z * n1z + e2z * n2z + e3z * n3z;

                    T->m_Re = ortonormalize(T->m_Re);
                }
            }
        }
    }
}

#endif