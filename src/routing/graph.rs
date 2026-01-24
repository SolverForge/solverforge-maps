//! Zero-erasure directed graph implementation.

use std::collections::HashMap;

/// Node index - zero-cost wrapper around usize.
#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash, Default)]
pub struct NodeIdx(pub usize);

impl NodeIdx {
    #[inline]
    pub fn new(index: usize) -> Self {
        Self(index)
    }

    #[inline]
    pub fn index(self) -> usize {
        self.0
    }
}

/// Edge index - zero-cost wrapper around usize.
#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub struct EdgeIdx(pub usize);

/// Directed graph with adjacency list representation.
///
/// Zero-erasure architecture: no Box, Arc, or dyn - all types concrete.
pub struct DiGraph<N, E> {
    nodes: Vec<N>,
    edges: Vec<E>,
    /// For each edge: (from_node, to_node)
    edge_endpoints: Vec<(NodeIdx, NodeIdx)>,
    /// Adjacency list: outgoing edges per node
    outgoing: Vec<Vec<EdgeIdx>>,
    /// Adjacency list: incoming edges per node (for retain_nodes)
    incoming: Vec<Vec<EdgeIdx>>,
}

impl<N, E> DiGraph<N, E> {
    pub fn new() -> Self {
        Self {
            nodes: Vec::new(),
            edges: Vec::new(),
            edge_endpoints: Vec::new(),
            outgoing: Vec::new(),
            incoming: Vec::new(),
        }
    }

    /// Add a node to the graph and return its index.
    pub fn add_node(&mut self, data: N) -> NodeIdx {
        let idx = NodeIdx(self.nodes.len());
        self.nodes.push(data);
        self.outgoing.push(Vec::new());
        self.incoming.push(Vec::new());
        idx
    }

    /// Add an edge from `from` to `to` and return its index.
    pub fn add_edge(&mut self, from: NodeIdx, to: NodeIdx, data: E) -> EdgeIdx {
        let idx = EdgeIdx(self.edges.len());
        self.edges.push(data);
        self.edge_endpoints.push((from, to));
        self.outgoing[from.0].push(idx);
        self.incoming[to.0].push(idx);
        idx
    }

    /// Get a reference to a node's data.
    #[inline]
    pub fn node_weight(&self, idx: NodeIdx) -> Option<&N> {
        self.nodes.get(idx.0)
    }

    /// Get a reference to an edge's data.
    #[inline]
    pub fn edge_weight(&self, idx: EdgeIdx) -> Option<&E> {
        self.edges.get(idx.0)
    }

    /// Get the endpoints (from, to) of an edge.
    #[inline]
    pub fn edge_endpoints(&self, idx: EdgeIdx) -> Option<(NodeIdx, NodeIdx)> {
        self.edge_endpoints.get(idx.0).copied()
    }

    /// Find an edge from `from` to `to` if one exists.
    pub fn find_edge(&self, from: NodeIdx, to: NodeIdx) -> Option<EdgeIdx> {
        self.outgoing
            .get(from.0)?
            .iter()
            .find(|&&e| {
                self.edge_endpoints
                    .get(e.0)
                    .map(|&(_, t)| t == to)
                    .unwrap_or(false)
            })
            .copied()
    }

    /// Number of nodes in the graph.
    #[inline]
    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }

    /// Number of edges in the graph.
    #[inline]
    pub fn edge_count(&self) -> usize {
        self.edges.len()
    }

    /// Iterator over all node indices.
    pub fn node_indices(&self) -> impl Iterator<Item = NodeIdx> + '_ {
        (0..self.nodes.len()).map(NodeIdx)
    }

    /// Iterator over all edge indices.
    pub fn edge_indices(&self) -> impl Iterator<Item = EdgeIdx> + '_ {
        (0..self.edges.len()).map(EdgeIdx)
    }

    /// Get outgoing edges from a node.
    #[inline]
    pub fn outgoing_edges(&self, node: NodeIdx) -> &[EdgeIdx] {
        self.outgoing
            .get(node.0)
            .map(|v| v.as_slice())
            .unwrap_or(&[])
    }

    /// Get incoming edges to a node.
    #[inline]
    pub fn incoming_edges(&self, node: NodeIdx) -> &[EdgeIdx] {
        self.incoming
            .get(node.0)
            .map(|v| v.as_slice())
            .unwrap_or(&[])
    }

    /// Retain only nodes for which the predicate returns true.
    ///
    /// This rebuilds the graph with only the retained nodes and edges
    /// between retained nodes.
    pub fn retain_nodes<F>(&mut self, mut f: F)
    where
        F: FnMut(NodeIdx, &N) -> bool,
    {
        // Determine which nodes to keep
        let keep: Vec<bool> = self
            .nodes
            .iter()
            .enumerate()
            .map(|(i, n)| f(NodeIdx(i), n))
            .collect();

        // Build mapping from old index to new index
        let mut old_to_new: HashMap<usize, usize> = HashMap::new();
        let mut new_idx = 0;
        for (old_idx, &kept) in keep.iter().enumerate() {
            if kept {
                old_to_new.insert(old_idx, new_idx);
                new_idx += 1;
            }
        }

        // Rebuild nodes
        let new_nodes: Vec<N> = self
            .nodes
            .drain(..)
            .enumerate()
            .filter(|(i, _)| keep[*i])
            .map(|(_, n)| n)
            .collect();

        // Rebuild edges - only keep edges where both endpoints are retained
        let mut new_edges: Vec<E> = Vec::new();
        let mut new_edge_endpoints: Vec<(NodeIdx, NodeIdx)> = Vec::new();

        let old_edges = std::mem::take(&mut self.edges);
        let old_endpoints = std::mem::take(&mut self.edge_endpoints);

        for (edge, (from, to)) in old_edges.into_iter().zip(old_endpoints.into_iter()) {
            if let (Some(&new_from), Some(&new_to)) =
                (old_to_new.get(&from.0), old_to_new.get(&to.0))
            {
                new_edges.push(edge);
                new_edge_endpoints.push((NodeIdx(new_from), NodeIdx(new_to)));
            }
        }

        // Rebuild adjacency lists
        let node_count = new_nodes.len();
        let mut new_outgoing: Vec<Vec<EdgeIdx>> = vec![Vec::new(); node_count];
        let mut new_incoming: Vec<Vec<EdgeIdx>> = vec![Vec::new(); node_count];

        for (edge_idx, &(from, to)) in new_edge_endpoints.iter().enumerate() {
            let e = EdgeIdx(edge_idx);
            new_outgoing[from.0].push(e);
            new_incoming[to.0].push(e);
        }

        self.nodes = new_nodes;
        self.edges = new_edges;
        self.edge_endpoints = new_edge_endpoints;
        self.outgoing = new_outgoing;
        self.incoming = new_incoming;
    }
}

impl<N, E> Default for DiGraph<N, E> {
    fn default() -> Self {
        Self::new()
    }
}
