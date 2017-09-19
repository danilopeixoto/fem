// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_MESH_EDGES_H
#define OPENTISSUE_MESH_EDGES_H

#include <opentissue/configuration.h>

#include <cassert>
#include <algorithm>
#include <vector>

namespace opentissue {
    namespace mesh {
        template <typename M, typename E> class Edge : public E {
        public:
            typedef M tetrahedral_mesh_type;
            typedef E edge_traits;
            typedef Edge<M, E> edge_type;
            typedef typename tetrahedral_mesh_type::index_type index_type;

        protected:
            index_type m_idxA;
            index_type m_idxB;

        public:
            Edge() : m_idxA(-1), m_idxB(-1) {}

            Edge(index_type const &indexA, index_type const &indexB)
                : m_idxA(indexA), m_idxB(indexB) {}

        public:
            index_type const &idxA() const { return m_idxA; }
            index_type const &idxB() const { return m_idxB; }

            bool operator==(edge_type const &edge) const {
                if (m_idxA == edge.idxA() && m_idxB == edge.idxB())
                    return true;

                if (m_idxB == edge.idxA() && m_idxA == edge.idxB())
                    return true;

                return false;
            }

            bool operator!=(edge_type const &edge) const { return !((*this) == edge); }
        };

        template <class M, typename E> class Edges {
        public:
            typedef M tetrahedral_mesh_type;
            typedef E edge_traits;
            typedef Edge<M, E> edge_type;
            typedef typename tetrahedral_mesh_type::index_type index_type;
            typedef std::vector<edge_type> edge_container;
            typedef typename edge_container::iterator edge_iterator;
            typedef typename edge_container::const_iterator const_edge_iterator;

        protected:
            typedef enum { white, grey, black } color_type;
            typedef std::vector<color_type> color_container;
            typedef typename tetrahedral_mesh_type::node_type node_type;
            typedef typename node_type::tetrahedron_circulator tetrahedron_type;
            typedef std::vector<node_type *> work_queue;

        protected:
            edge_container m_edges;

        public:
            edge_iterator begin() { return m_edges.begin(); }
            edge_iterator end() { return m_edges.end(); }
            const_edge_iterator begin() const { return m_edges.begin(); }
            const_edge_iterator end() const { return m_edges.end(); }

        protected:
            struct VisitEdge {
                void visit(index_type &idxA, index_type &idxB, work_queue &work,
                    color_container &colors, edge_container &edges,
                    tetrahedral_mesh_type &mesh) {
                    if (idxA == idxB)
                        return;

                    if (colors[idxB] == white) {
                        colors[idxB] = grey;
                        work.push_back(&(*(mesh.node(idxB))));
                    }

                    if (colors[idxB] != black)
                        edges.push_back(edge_type(idxA, idxB));
                }
            };

        public:
            Edges() {}

            Edges(tetrahedral_mesh_type &mesh) {
                index_type idxA, idxB;

                color_container colors(mesh.size_nodes());
                std::fill(colors.begin(), colors.end(), white);
                colors[0] = grey;

                node_type *node = &(*(mesh.node(0)));

                work_queue work;
                work.push_back(node);

                while (!work.empty()) {
                    node = work.back();
                    work.pop_back();

                    idxA = node->idx();

                    assert(colors[idxA] == grey || !"Non-grey node found.");

                    std::vector<index_type> neighbors;

                    for (tetrahedron_type T = node->begin(); T != node->end(); ++T) {
                        idxB = T->i()->idx();

                        if (idxB != idxA)
                            neighbors.push_back(idxB);

                        idxB = T->j()->idx();

                        if (idxB != idxA)
                            neighbors.push_back(idxB);

                        idxB = T->k()->idx();

                        if (idxB != idxA)
                            neighbors.push_back(idxB);

                        idxB = T->m()->idx();

                        if (idxB != idxA)
                            neighbors.push_back(idxB);
                    }

                    std::sort(neighbors.begin(), neighbors.end());
                    std::unique(neighbors.begin(), neighbors.end());

                    for (typename std::vector<index_type>::iterator n = neighbors.begin();
                        n != neighbors.end(); ++n)
                        VisitEdge().visit(idxA, *n, work, colors, m_edges, mesh);

                    colors[idxA] = black;
                }
            }
        };
    }
}

#endif