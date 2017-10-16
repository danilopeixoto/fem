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
#include <opentissue/fem/compute_b.h>
#include <opentissue/fem/compute_isotropic_elasticity.h>
#include <opentissue/fem/compute_ke.h>

#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>

namespace opentissue {
    namespace fem {
        namespace detail {
            template<typename fem_mesh, typename real_type>
            inline void initialize_stiffness_plasticity(fem_mesh &mesh,
                real_type const &density, real_type const &poisson,
                real_type const &young, real_type const &c_yield,
                real_type const &c_max_yield, real_type const &c_creep) {
                typedef typename fem_mesh::node_iterator node_iterator;
                typedef typename fem_mesh::tetrahedron_iterator tetrahedron_iterator;

                node_iterator nbegin = mesh.node_begin();
                node_iterator nend = mesh.node_end();

                for (node_iterator n = nbegin; n != nend; n++)
                    n->m_world_coord = n->m_coord;

                tbb::parallel_for(tbb::blocked_range<size_t>(0, mesh.size_tetrahedra()),
                    [&](const tbb::blocked_range<size_t> &range) {
                    for (size_t i = range.begin(); i != range.end(); i++) {
                        tetrahedron_iterator tet = mesh.tetrahedron(i);

                        tet->m_density = density;
                        tet->m_poisson = poisson;
                        tet->m_young = young;
                        tet->m_yield = c_yield;
                        tet->m_max_yield = c_max_yield;
                        tet->m_creep = c_creep;

                        tet->m_e10 = tet->j()->m_coord - tet->i()->m_coord;
                        tet->m_e20 = tet->k()->m_coord - tet->i()->m_coord;
                        tet->m_e30 = tet->m()->m_coord - tet->i()->m_coord;

                        tet->m_volume = compute_volume(tet->m_e10, tet->m_e20, tet->m_e30);

                        compute_b(tet);
                        compute_isotropic_elasticity(tet);

                        compute_ke(tet);

                        for (unsigned int i = 0; i < 6; i++)
                            tet->m_plastic[i] = 0;
                    }
                });
            }
        }
    }
}

#endif