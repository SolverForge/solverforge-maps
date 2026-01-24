//! Graph algorithms for routing: A*, Dijkstra, Kosaraju SCC.

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

use super::graph::{DiGraph, NodeIdx};

/// State for priority queue in A*/Dijkstra.
struct State {
    cost: f64,
    /// For A*: estimated total cost (g + h), for Dijkstra: just g
    estimated: f64,
    node: NodeIdx,
}

impl PartialEq for State {
    fn eq(&self, other: &Self) -> bool {
        self.estimated == other.estimated
    }
}

impl Eq for State {}

impl Ord for State {
    fn cmp(&self, other: &Self) -> Ordering {
        // Min-heap: reverse comparison
        other
            .estimated
            .partial_cmp(&self.estimated)
            .unwrap_or(Ordering::Equal)
    }
}

impl PartialOrd for State {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// A* pathfinding algorithm.
///
/// Finds the shortest path from `start` to a goal node (determined by `is_goal`).
/// Uses `edge_cost` to compute edge weights and `heuristic` for the A* estimate.
///
/// With a zero heuristic, this is equivalent to Dijkstra's algorithm.
///
/// Returns `Some((cost, path))` if a path is found, `None` otherwise.
pub fn astar<N, E, G, C, H>(
    graph: &DiGraph<N, E>,
    start: NodeIdx,
    is_goal: G,
    edge_cost: C,
    heuristic: H,
) -> Option<(f64, Vec<NodeIdx>)>
where
    G: Fn(NodeIdx) -> bool,
    C: Fn(&E) -> f64,
    H: Fn(NodeIdx) -> f64,
{
    let mut dist: HashMap<NodeIdx, f64> = HashMap::new();
    let mut came_from: HashMap<NodeIdx, NodeIdx> = HashMap::new();
    let mut heap = BinaryHeap::new();

    dist.insert(start, 0.0);
    heap.push(State {
        cost: 0.0,
        estimated: heuristic(start),
        node: start,
    });

    while let Some(State { cost, node, .. }) = heap.pop() {
        // Found goal
        if is_goal(node) {
            let mut path = vec![node];
            let mut current = node;
            while let Some(&prev) = came_from.get(&current) {
                path.push(prev);
                current = prev;
            }
            path.reverse();
            return Some((cost, path));
        }

        // Skip if we already found a better path
        if let Some(&d) = dist.get(&node) {
            if cost > d {
                continue;
            }
        }

        // Explore neighbors
        for &edge_idx in graph.outgoing_edges(node) {
            if let (Some(edge_data), Some((_, to))) =
                (graph.edge_weight(edge_idx), graph.edge_endpoints(edge_idx))
            {
                let next_cost = cost + edge_cost(edge_data);
                let is_better = dist.get(&to).map(|&d| next_cost < d).unwrap_or(true);

                if is_better {
                    dist.insert(to, next_cost);
                    came_from.insert(to, node);
                    heap.push(State {
                        cost: next_cost,
                        estimated: next_cost + heuristic(to),
                        node: to,
                    });
                }
            }
        }
    }

    None
}

/// Dijkstra's algorithm for single-source shortest paths.
///
/// Computes shortest paths from `start` to all reachable nodes.
/// Optionally stops at `goal` if provided.
///
/// Returns a map from node index to shortest distance.
pub fn dijkstra<N, E, C>(
    graph: &DiGraph<N, E>,
    start: NodeIdx,
    goal: Option<NodeIdx>,
    edge_cost: C,
) -> HashMap<NodeIdx, f64>
where
    C: Fn(&E) -> f64,
{
    let mut dist: HashMap<NodeIdx, f64> = HashMap::new();
    let mut heap = BinaryHeap::new();

    dist.insert(start, 0.0);
    heap.push(State {
        cost: 0.0,
        estimated: 0.0,
        node: start,
    });

    while let Some(State { cost, node, .. }) = heap.pop() {
        // Early exit if we reached the goal
        if let Some(g) = goal {
            if node == g {
                break;
            }
        }

        // Skip if we already found a better path
        if let Some(&d) = dist.get(&node) {
            if cost > d {
                continue;
            }
        }

        // Explore neighbors
        for &edge_idx in graph.outgoing_edges(node) {
            if let (Some(edge_data), Some((_, to))) =
                (graph.edge_weight(edge_idx), graph.edge_endpoints(edge_idx))
            {
                let next_cost = cost + edge_cost(edge_data);
                let is_better = dist.get(&to).map(|&d| next_cost < d).unwrap_or(true);

                if is_better {
                    dist.insert(to, next_cost);
                    heap.push(State {
                        cost: next_cost,
                        estimated: next_cost,
                        node: to,
                    });
                }
            }
        }
    }

    dist
}

/// Kosaraju's algorithm for finding strongly connected components.
///
/// Returns a vector of SCCs, where each SCC is a vector of node indices.
/// SCCs are returned in reverse topological order.
pub fn kosaraju_scc<N, E>(graph: &DiGraph<N, E>) -> Vec<Vec<NodeIdx>> {
    let n = graph.node_count();
    if n == 0 {
        return Vec::new();
    }

    // Phase 1: DFS on original graph to get finish order
    let mut visited = vec![false; n];
    let mut finish_order = Vec::with_capacity(n);

    for i in 0..n {
        if !visited[i] {
            dfs_forward(graph, NodeIdx(i), &mut visited, &mut finish_order);
        }
    }

    // Phase 2: DFS on reversed graph in reverse finish order
    let mut visited = vec![false; n];
    let mut sccs = Vec::new();

    for &node in finish_order.iter().rev() {
        if !visited[node.0] {
            let mut component = Vec::new();
            dfs_backward(graph, node, &mut visited, &mut component);
            sccs.push(component);
        }
    }

    sccs
}

/// DFS on the original graph, recording finish order.
fn dfs_forward<N, E>(
    graph: &DiGraph<N, E>,
    node: NodeIdx,
    visited: &mut [bool],
    finish_order: &mut Vec<NodeIdx>,
) {
    // Use explicit stack to avoid stack overflow on large graphs
    let mut stack = vec![(node, 0usize)]; // (node, edge_index)

    while let Some((current, edge_idx)) = stack.last_mut() {
        let current = *current;

        if !visited[current.0] {
            visited[current.0] = true;
        }

        let edges = graph.outgoing_edges(current);

        // Find next unvisited neighbor
        let mut found_next = false;
        while *edge_idx < edges.len() {
            let e = edges[*edge_idx];
            *edge_idx += 1;
            if let Some((_, to)) = graph.edge_endpoints(e) {
                if !visited[to.0] {
                    stack.push((to, 0));
                    found_next = true;
                    break;
                }
            }
        }

        if !found_next {
            finish_order.push(current);
            stack.pop();
        }
    }
}

/// DFS on the reversed graph (following incoming edges).
fn dfs_backward<N, E>(
    graph: &DiGraph<N, E>,
    node: NodeIdx,
    visited: &mut [bool],
    component: &mut Vec<NodeIdx>,
) {
    // Use explicit stack to avoid stack overflow on large graphs
    let mut stack = vec![(node, 0usize)];

    while let Some((current, edge_idx)) = stack.last_mut() {
        let current = *current;

        if !visited[current.0] {
            visited[current.0] = true;
            component.push(current);
        }

        let edges = graph.incoming_edges(current);

        // Find next unvisited neighbor (in reverse direction)
        let mut found_next = false;
        while *edge_idx < edges.len() {
            let e = edges[*edge_idx];
            *edge_idx += 1;
            if let Some((from, _)) = graph.edge_endpoints(e) {
                if !visited[from.0] {
                    stack.push((from, 0));
                    found_next = true;
                    break;
                }
            }
        }

        if !found_next {
            stack.pop();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_astar_simple_path() {
        let mut graph: DiGraph<(), f64> = DiGraph::new();
        let a = graph.add_node(());
        let b = graph.add_node(());
        let c = graph.add_node(());

        graph.add_edge(a, b, 1.0);
        graph.add_edge(b, c, 2.0);
        graph.add_edge(a, c, 5.0);

        let result = astar(&graph, a, |n| n == c, |&cost| cost, |_| 0.0);

        assert!(result.is_some());
        let (cost, path) = result.unwrap();
        assert_eq!(cost, 3.0);
        assert_eq!(path, vec![a, b, c]);
    }

    #[test]
    fn test_dijkstra_all_paths() {
        let mut graph: DiGraph<(), f64> = DiGraph::new();
        let a = graph.add_node(());
        let b = graph.add_node(());
        let c = graph.add_node(());

        graph.add_edge(a, b, 1.0);
        graph.add_edge(b, c, 2.0);
        graph.add_edge(a, c, 5.0);

        let costs = dijkstra(&graph, a, None, |&cost| cost);

        assert_eq!(costs.get(&a), Some(&0.0));
        assert_eq!(costs.get(&b), Some(&1.0));
        assert_eq!(costs.get(&c), Some(&3.0));
    }

    #[test]
    fn test_kosaraju_scc() {
        // Graph with two SCCs: {0, 1, 2} and {3}
        let mut graph: DiGraph<(), ()> = DiGraph::new();
        let n0 = graph.add_node(());
        let n1 = graph.add_node(());
        let n2 = graph.add_node(());
        let n3 = graph.add_node(());

        // Cycle: 0 -> 1 -> 2 -> 0
        graph.add_edge(n0, n1, ());
        graph.add_edge(n1, n2, ());
        graph.add_edge(n2, n0, ());

        // Connection to separate node
        graph.add_edge(n2, n3, ());

        let sccs = kosaraju_scc(&graph);

        assert_eq!(sccs.len(), 2);

        // One SCC should have 3 nodes, one should have 1
        let sizes: Vec<usize> = sccs.iter().map(|c| c.len()).collect();
        assert!(sizes.contains(&3));
        assert!(sizes.contains(&1));
    }
}
