// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_FEM_SIMULATE_H
#define OPENTISSUE_FEM_SIMULATE_H

#include <opentissue/configuration.h>

#include <opentissue/fem/clear_stiffness_assembly.h>
#include <opentissue/fem/update_orientation.h>
#include <opentissue/fem/stiffness_assembly.h>
#include <opentissue/fem/apply_plasticity_force.h>
#include <opentissue/fem/dynamics_assembly.h>
#include <opentissue/fem/conjugate_gradients.h>

#include <tbb/parallel_invoke.h>

namespace opentissue {
    namespace fem {
        template<typename fem_mesh, typename real_type>
        inline void simulate(fem_mesh &mesh, real_type const &mass_damping,
            real_type const &stiffness_damping, real_type const &time_step,
            unsigned int max_iterations) {
            tbb::parallel_invoke([&] { detail::clear_stiffness_assembly(mesh); },
                [&] { detail::update_orientation(mesh, max_iterations); });

            tbb::parallel_invoke([&] { detail::stiffness_assembly(mesh); },
                [&] { detail::apply_plasticity_force(mesh); });

            detail::dynamics_assembly(mesh, mass_damping, stiffness_damping, time_step);
            detail::conjugate_gradients(mesh, time_step, max_iterations, 1.0e-6);
        }
    }
}

#endif