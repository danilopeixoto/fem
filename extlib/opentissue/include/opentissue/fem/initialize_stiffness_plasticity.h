// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_FEM_INITIALIZE_STIFFNESS_PLASTICITY_H
#define OPENTISSUE_FEM_INITIALIZE_STIFFNESS_PLASTICITY_H

#include <opentissue/configuration.h>

#include <opentissue/fem/compute_volume.h>
#include <opentissue/fem/compute_ke.h>
#include <opentissue/fem/compute_b.h>
#include <opentissue/fem/compute_isotropic_elasticity.h>

namespace opentissue {
    namespace fem {
        namespace detail {
            template <typename fem_mesh, typename real_type>
            inline void initialize_stiffness_plasticity(fem_mesh &mesh,
                real_type const &density, real_type const &poisson,
                real_type const &young, real_type const &c_yield,
                real_type const &c_max_yield, real_type const &c_creep) {
                typedef typename fem_mesh::node_iterator node_iterator;
                typedef typename fem_mesh::tetrahedron_iterator tetrahedron_iterator;

                node_iterator Nbegin = mesh.node_begin();
                node_iterator Nend = mesh.node_end();

                for (node_iterator N = Nbegin; N != Nend; ++N)
                    N->m_coord = N->m_model_coord;

                tetrahedron_iterator Tbegin = mesh.tetrahedron_begin();
                tetrahedron_iterator Tend = mesh.tetrahedron_end();

                for (tetrahedron_iterator T = Tbegin; T != Tend; ++T) {
                    T->m_density = density;
                    T->m_poisson = poisson;
                    T->m_young = young;
                    T->m_yield = c_yield;
                    T->m_max_yield = c_max_yield;
                    T->m_creep = c_creep;

                    T->m_e10 = T->j()->m_model_coord - T->i()->m_model_coord;
                    T->m_e20 = T->k()->m_model_coord - T->i()->m_model_coord;
                    T->m_e30 = T->m()->m_model_coord - T->i()->m_model_coord;

                    T->m_V = compute_volume(T->m_e10, T->m_e20, T->m_e30);

                    assert(T->m_V > 0 || !"Element with negative volume found.");

                    compute_Ke(T->i()->m_model_coord, T->j()->m_model_coord,
                        T->k()->m_model_coord, T->m()->m_model_coord, T->m_young,
                        T->m_poisson, T->m_Ke);

                    for (int i = 0; i < 6; ++i)
                        T->m_plastic[i] = 0;

                    compute_b(T->m_e10, T->m_e20, T->m_e30, T->m_V, T->m_B);
                    compute_isotropic_elasticity(T->m_young, T->m_poisson, T->m_D);
                }
            }
        }
    }
}

#endif