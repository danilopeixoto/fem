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
        inline void apply_external_force(fem_mesh &mesh, vector_type const &force) {
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
        inline void apply_acceleration(fem_mesh &mesh, vector_type const &acceleration) {
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
        inline void set_velocity(fem_mesh &mesh, vector_type const &velocity) {
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
            vector_type const &angular_velocity) {
            typedef typename fem_mesh::node_iterator node_iterator;

            vector_type centroid;
            compute_centroid(mesh, centroid);

            node_iterator nbegin = mesh.node_begin();
            node_iterator nend = mesh.node_end();

            for (node_iterator n = nbegin; n != nend; n++) {
                if (n->m_fixed)
                    continue;

                n->m_velocity = math::cross(angular_velocity,
                    n->m_world_coord - centroid);
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

        template<typename fem_mesh>
        inline void reset_world_coordinate(fem_mesh &mesh) {
            typedef fem_mesh::node_iterator node_iterator;

            node_iterator nbegin = mesh.node_begin();
            node_iterator nend = mesh.node_end();

            for (node_iterator n = nbegin; n != nend; n++)
                n->m_world_coord = n->m_coord;
        }

        template<typename fem_mesh, typename vector_type>
        inline void compute_centroid(fem_mesh const &mesh, vector_type &centroid) {
            typedef typename fem_mesh::real_type real_type;
            typedef typename fem_mesh::const_tetrahedron_iterator const_tetrahedron_iterator;

            centroid(0) = 0;
            centroid(1) = 0;
            centroid(2) = 0;

            real_type volume = 0;

            const_tetrahedron_iterator tbegin = mesh.tetrahedron_begin();
            const_tetrahedron_iterator tend = mesh.tetrahedron_end();

            for (const_tetrahedron_iterator t = tbegin; t != tend; t++) {
                centroid += t->m_volume * t->m_centroid;
                volume += t->m_volume;
            }

            if (volume != 0)
                centroid /= volume;
        }
    }
}

#endif