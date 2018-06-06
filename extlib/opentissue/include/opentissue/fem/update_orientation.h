// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_FEM_UPDATE_ORIENTATION_H
#define OPENTISSUE_FEM_UPDATE_ORIENTATION_H

#include <opentissue/configuration.h>

#include <opentissue/fem/compute_centroid.h>
#include <opentissue/fem/compute_volume.h>
#include <opentissue/math/math.h>

#include <tbb/parallel_for.h>

namespace opentissue {
    namespace fem {
        namespace detail {
            template<typename fem_mesh>
            inline void update_orientation(fem_mesh &mesh, unsigned int max_iterations) {
                typedef typename fem_mesh::real_type real_type;
                typedef typename fem_mesh::vector_type vector_type;
                typedef typename fem_mesh::matrix_type matrix_type;
                typedef typename fem_mesh::value_traits value_traits;
                typedef typename fem_mesh::tetrahedron_iterator tetrahedron_iterator;

                tbb::parallel_for(size_t(0), mesh.size_tetrahedra(), [&](size_t i) {
                    tetrahedron_iterator tet = mesh.tetrahedron(i);

                    vector_type const &n1 = tet->m_B[1];
                    vector_type const &n2 = tet->m_B[2];
                    vector_type const &n3 = tet->m_B[3];

                    vector_type &e10 = tet->m_e10;
                    vector_type &e20 = tet->m_e20;
                    vector_type &e30 = tet->m_e30;

                    e10 = tet->j()->m_world_coord - tet->i()->m_world_coord;
                    e20 = tet->k()->m_world_coord - tet->i()->m_world_coord;
                    e30 = tet->m()->m_world_coord - tet->i()->m_world_coord;

                    matrix_type &Re = tet->m_Re;

                    Re(0, 0) = e10(0) * n1(0) + e20(0) * n2(0) + e30(0) * n3(0);
                    Re(0, 1) = e10(0) * n1(1) + e20(0) * n2(1) + e30(0) * n3(1);
                    Re(0, 2) = e10(0) * n1(2) + e20(0) * n2(2) + e30(0) * n3(2);
                    Re(1, 0) = e10(1) * n1(0) + e20(1) * n2(0) + e30(1) * n3(0);
                    Re(1, 1) = e10(1) * n1(1) + e20(1) * n2(1) + e30(1) * n3(1);
                    Re(1, 2) = e10(1) * n1(2) + e20(1) * n2(2) + e30(1) * n3(2);
                    Re(2, 0) = e10(2) * n1(0) + e20(2) * n2(0) + e30(2) * n3(0);
                    Re(2, 1) = e10(2) * n1(1) + e20(2) * n2(1) + e30(2) * n3(1);
                    Re(2, 2) = e10(2) * n1(2) + e20(2) * n2(2) + e30(2) * n3(2);

                    tet->m_volume = compute_volume(e10, e20, e30);
                    compute_centroid(tet);

                    if (tet->m_volume < value_traits::epsilon()) {
                        Re(0, 0) = value_traits::one();
                        Re(0, 1) = value_traits::zero();
                        Re(0, 2) = value_traits::zero();
                        Re(1, 0) = value_traits::zero();
                        Re(1, 1) = value_traits::one();
                        Re(1, 2) = value_traits::zero();
                        Re(2, 0) = value_traits::zero();
                        Re(2, 1) = value_traits::zero();
                        Re(2, 2) = value_traits::one();
                    }
                    else if (tet->m_volume < 0.06 * tet->m_volume0)
                        math::orthonormalize(Re);
                    else {
                        matrix_type U, P;
                        math::polar_decomposition(Re, max_iterations, 1.0e-6, U, P);

                        Re = U;
                    }
                });
            }
        }
    }
}

#endif