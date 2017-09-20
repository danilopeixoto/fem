// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_MESH_NODE_H
#define OPENTISSUE_MESH_NODE_H

#include <opentissue/configuration.h>

#include <cassert>
#include <vector>

namespace opentissue {
    namespace mesh {
        template <typename tetrahedral_mesh_type_> class Node : public tetrahedral_mesh_type_::node_traits {
        public:
            typedef tetrahedral_mesh_type_ tetrahedral_mesh_type;
            typedef typename tetrahedral_mesh_type::node_type node_type;
            typedef typename tetrahedral_mesh_type::tetrahedron_type tetrahedron_type;
            typedef typename tetrahedral_mesh_type::index_type index_type;
            typedef std::vector<index_type> tetrahedra_index_container;
            typedef typename tetrahedra_index_container::iterator tetrahedra_idx_iterator;

        protected:
            index_type m_idx;
            tetrahedral_mesh_type *m_owner;
            tetrahedra_index_container m_tetrahedra;

        private:
            friend class core_access;

            void set_index(index_type idx) { m_idx = idx; }
            void set_owner(tetrahedral_mesh_type *owner) { m_owner = owner; }
            void tetrahedra_push_back(index_type idx) { m_tetrahedra.push_back(idx); }
            void tetrahedra_remove(index_type idx) { m_tetrahedra.remove(idx); }

        public:
            class tetrahedron_circulator {
            private:
                node_type *m_node;
                tetrahedra_idx_iterator m_it;

            public:
                tetrahedron_circulator() : m_node(0), m_it(0) {}

                tetrahedron_circulator(node_type *node, tetrahedra_idx_iterator pos)
                    : m_node(node), m_it(pos) {
                    assert(m_node || !"Node was null.");
                    assert(m_node->m_owner || !"Node owner was null.");
                }

                bool operator==(tetrahedron_circulator const &other) const {
                    return (m_node == other.m_node && m_it == other.m_it);
                }

                bool operator!=(tetrahedron_circulator const &other) const {
                    return !(*this == other);
                }

                tetrahedron_type &operator*() {
                    assert(m_node || !"Node was null.");
                    assert(m_node->m_owner || !"Node owner was null.");

                    return *(m_node->m_owner->tetrahedron(*m_it));
                }

                tetrahedron_type *operator->() {
                    assert(m_node || !"Node was null.");
                    assert(m_node->m_owner || !"Node owner was null.");

                    return &(*(m_node->m_owner->tetrahedron(*m_it)));
                }

                tetrahedron_circulator &operator++() {
                    assert(m_node || !"Node was null.");
                    assert(m_node->m_owner || !"Node owner was null.");

                    ++m_it;
                    return *this;
                }
            };

            tetrahedron_circulator begin() {
                return tetrahedron_circulator(this, m_tetrahedra.begin());
            }
            tetrahedron_circulator end() {
                return tetrahedron_circulator(this, m_tetrahedra.end());
            }

            size_t size_tetrahedra() const { return m_tetrahedra.size(); }

            bool isolated() const { return m_tetrahedra.empty(); }

        public:
            Node() : m_idx(tetrahedral_mesh_type::undefined()), m_owner(0), m_tetrahedra() {}

        public:
            index_type idx() const { return m_idx; }
            tetrahedral_mesh_type *owner() { return m_owner; }
            tetrahedral_mesh_type const *owner() const { return m_owner; }
        };
    }
}

#endif