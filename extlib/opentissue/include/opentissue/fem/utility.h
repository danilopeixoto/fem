// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_FEM_UTILITY_H
#define OPENTISSUE_FEM_UTILITY_H

#include <opentissue/configuration.h>

#include <opentissue/math/math.h>

namespace opentissue {
    namespace fem {
        template<typename fem_mesh, typename vector_type>
        inline void apply_external_force(fem_mesh &mesh, vector_type const & force) {
            typedef typename fem_mesh::node_iterator node_iterator;

            node_iterator nbegin = mesh.node_begin();
            node_iterator nend = mesh.node_end();

            for (node_iterator n = nbegin; n != nend; n++) {
                if (n->m_fixed)
                    continue;

                n->m_f_external += force;
            }
        }

        template<typename fem_mesh>
        inline void clear_external_forces(fem_mesh &mesh) {
            typedef typename fem_mesh::node_iterator node_iterator;

            node_iterator nbegin = mesh.node_begin();
            node_iterator nend = mesh.node_end();

            for (node_iterator n = nbegin; n != nend; n++) {
                if (n->m_fixed)
                    continue;

                n->m_f_external.clear();
            }
        }

        template<typename fem_mesh, typename vector_type>
        inline void apply_acceleration(fem_mesh &mesh, vector_type const & acceleration) {
            typedef typename fem_mesh::node_iterator node_iterator;

            node_iterator nbegin = mesh.node_begin();
            node_iterator nend = mesh.node_end();

            for (node_iterator n = nbegin; n != nend; n++) {
                if (n->m_fixed)
                    continue;

                n->m_f_external += n->m_mass * acceleration;
            }
        }

        template<typename fem_mesh, typename vector_type>
        inline void set_velocity(fem_mesh &mesh, vector_type const & velocity) {
            typedef typename fem_mesh::node_iterator node_iterator;

            node_iterator nbegin = mesh.node_begin();
            node_iterator nend = mesh.node_end();

            for (node_iterator n = nbegin; n != nend; n++) {
                if (n->m_fixed)
                    continue;

                n->m_velocity = velocity;
            }
        }

        template<typename fem_mesh, typename vector_type>
        inline void set_angular_velocity(fem_mesh &mesh,
            vector_type const & angular_velocity) {
            typedef typename fem_mesh::node_iterator node_iterator;

            vector_type mass_center;

            node_iterator nbegin = mesh.node_begin();
            node_iterator nend = mesh.node_end();

            for (node_iterator n = nbegin; n != nend; n++)
                mass_center += n->m_world_coord;

            if (mesh.size_nodes() != 0)
                mass_center /= (double)mesh.size_nodes();

            for (node_iterator n = nbegin; n != nend; n++) {
                if (n->m_fixed)
                    continue;

                n->m_velocity = math::cross(angular_velocity,
                    n->m_world_coord - mass_center);
            }
        }

        template<typename fem_mesh>
        inline void set_fixed(fem_mesh &mesh, bool fixed) {
            typedef fem_mesh::node_iterator node_iterator;

            node_iterator nbegin = mesh.node_begin();
            node_iterator nend = mesh.node_end();

            for (node_iterator n = nbegin; n != nend; n++)
                n->m_fixed = fixed;
        }
    }
}

#endif