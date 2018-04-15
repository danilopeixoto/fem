// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_FEM_UPDATE_PARAMETERS_H
#define OPENTISSUE_FEM_UPDATE_PARAMETERS_H

#include <opentissue/configuration.h>

#include <opentissue/fem/compute_isotropic_elasticity.h>
#include <opentissue/fem/compute_ke.h>

#include <tbb/parallel_for.h>

namespace opentissue {
    namespace fem {
        template<typename fem_mesh, typename real_type>
        inline void update_parameters(fem_mesh &mesh, real_type const &density,
            real_type const &poisson, real_type const &young,
            real_type const &c_yield, real_type const &c_max_yield,
            real_type const &c_creep) {
            typedef typename fem_mesh::tetrahedron_iterator tetrahedron_iterator;

            tbb::parallel_for(size_t(0), mesh.size_tetrahedra(), [&](size_t i) {
                tetrahedron_iterator tet = mesh.tetrahedron(i);

                tet->m_density = density;
                tet->m_poisson = poisson;
                tet->m_young = young;
                tet->m_yield = c_yield;
                tet->m_max_yield = c_max_yield;
                tet->m_creep = c_creep;

                detail::compute_isotropic_elasticity(tet);

                detail::compute_ke(tet);

                for (unsigned int i = 0; i < 6; i++)
                    tet->m_plastic[i] = 0;
            });
        }
    }
}

#endif