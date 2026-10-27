//! Dynamic re-routing over a directed road graph.
//!
//! The route minimises travel time: each edge costs its length divided by the
//! lower of the posted limit and the speed the vehicle can hold. Closed edges
//! (road closures, construction zones, or a V2X congestion report) are skipped.
//! Closing or reopening an edge changes the next [`RoadGraph::route`] result, so
//! a replan after a closure needs no extra state.

use std::cmp::Reverse;
use std::collections::BinaryHeap;

/// One directed road segment.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Edge {
    /// Node the segment leaves from.
    pub from: usize,
    /// Node the segment arrives at.
    pub to: usize,
    /// Segment length, metres.
    pub length_m: f32,
    /// Posted limit, m/s.
    pub speed_limit_m_s: f32,
}

/// A planned route from one node to another.
#[derive(Clone, Debug, PartialEq)]
pub struct Route {
    /// Nodes visited, start first.
    pub nodes: Vec<usize>,
    /// Edge ids traversed, in order. One fewer than `nodes`.
    pub edges: Vec<usize>,
    /// Total length, metres.
    pub length_m: f32,
    /// Total travel time, seconds.
    pub time_s: f32,
}

/// Directed road graph with closable edges.
#[derive(Clone, Debug, Default)]
pub struct RoadGraph {
    node_count: usize,
    edges: Vec<Edge>,
    outgoing: Vec<Vec<usize>>,
    closed: Vec<bool>,
}

impl RoadGraph {
    /// Creates a graph with `node_count` nodes and no edges.
    #[must_use]
    pub fn new(node_count: usize) -> Self {
        Self {
            node_count,
            edges: Vec::new(),
            outgoing: vec![Vec::new(); node_count],
            closed: Vec::new(),
        }
    }

    /// Adds a directed edge and returns its id.
    ///
    /// Returns `None` if a node is out of range, or if the length or the speed
    /// limit is not a positive finite number.
    pub fn add_edge(
        &mut self,
        from: usize,
        to: usize,
        length_m: f32,
        speed_limit_m_s: f32,
    ) -> Option<usize> {
        if from >= self.node_count || to >= self.node_count {
            return None;
        }
        if !(length_m.is_finite() && length_m > 0.0) {
            return None;
        }
        if !(speed_limit_m_s.is_finite() && speed_limit_m_s > 0.0) {
            return None;
        }
        let id = self.edges.len();
        self.edges.push(Edge {
            from,
            to,
            length_m,
            speed_limit_m_s,
        });
        self.outgoing[from].push(id);
        self.closed.push(false);
        Some(id)
    }

    /// Closes an edge so routes avoid it. Returns `false` for an unknown id.
    pub fn close_edge(&mut self, id: usize) -> bool {
        self.set_closed(id, true)
    }

    /// Reopens a closed edge. Returns `false` for an unknown id.
    pub fn reopen_edge(&mut self, id: usize) -> bool {
        self.set_closed(id, false)
    }

    /// Returns `true` if the edge exists and is closed.
    #[must_use]
    pub fn is_closed(&self, id: usize) -> bool {
        self.closed.get(id).copied().unwrap_or(false)
    }

    fn set_closed(&mut self, id: usize, closed: bool) -> bool {
        match self.closed.get_mut(id) {
            Some(flag) => {
                *flag = closed;
                true
            }
            None => false,
        }
    }

    /// Finds the fastest open route from `start` to `goal`.
    ///
    /// Returns `None` if either node is out of range or no open route exists.
    /// A route from a node to itself is empty and takes zero time.
    #[must_use]
    pub fn route(&self, start: usize, goal: usize) -> Option<Route> {
        if start >= self.node_count || goal >= self.node_count {
            return None;
        }
        // Costs are integer microseconds so the heap has a total order.
        let mut cost = vec![u64::MAX; self.node_count];
        let mut via: Vec<Option<usize>> = vec![None; self.node_count];
        let mut heap = BinaryHeap::new();
        cost[start] = 0;
        heap.push(Reverse((0_u64, start)));

        while let Some(Reverse((spent, node))) = heap.pop() {
            if node == goal {
                break;
            }
            if spent > cost[node] {
                continue;
            }
            for &id in &self.outgoing[node] {
                if self.closed[id] {
                    continue;
                }
                let edge = self.edges[id];
                let next = spent.saturating_add(travel_us(&edge));
                if next < cost[edge.to] {
                    cost[edge.to] = next;
                    via[edge.to] = Some(id);
                    heap.push(Reverse((next, edge.to)));
                }
            }
        }

        if cost[goal] == u64::MAX {
            return None;
        }
        Some(self.trace(start, goal, &via, cost[goal]))
    }

    fn trace(&self, start: usize, goal: usize, via: &[Option<usize>], total_us: u64) -> Route {
        let mut nodes = vec![goal];
        let mut edges = Vec::new();
        let mut length_m = 0.0_f32;
        let mut node = goal;
        while node != start {
            let Some(id) = via[node] else { break };
            let edge = self.edges[id];
            edges.push(id);
            length_m += edge.length_m;
            node = edge.from;
            nodes.push(node);
        }
        nodes.reverse();
        edges.reverse();
        Route {
            nodes,
            edges,
            length_m,
            time_s: total_us as f32 / 1_000_000.0,
        }
    }
}

/// Travel time for one edge, in whole microseconds.
fn travel_us(edge: &Edge) -> u64 {
    let time_s = edge.length_m / edge.speed_limit_m_s;
    (time_s * 1_000_000.0).round() as u64
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Diamond: 0 -> 1 -> 3 is short but slow, 0 -> 2 -> 3 is longer but fast.
    fn diamond() -> (RoadGraph, [usize; 4]) {
        let mut g = RoadGraph::new(4);
        let a = g.add_edge(0, 1, 100.0, 5.0).unwrap();
        let b = g.add_edge(1, 3, 100.0, 5.0).unwrap();
        let c = g.add_edge(0, 2, 300.0, 20.0).unwrap();
        let d = g.add_edge(2, 3, 300.0, 20.0).unwrap();
        (g, [a, b, c, d])
    }

    #[test]
    fn picks_fastest_route_not_shortest() {
        let (g, [_, _, c, d]) = diamond();
        let r = g.route(0, 3).unwrap();
        assert_eq!(r.nodes, vec![0, 2, 3]);
        assert_eq!(r.edges, vec![c, d]);
        assert!((r.length_m - 600.0).abs() < 1e-3);
        assert!((r.time_s - 30.0).abs() < 1e-3);
    }

    #[test]
    fn closure_forces_replan_and_reopen_restores_it() {
        let (mut g, [_, _, c, _]) = diamond();
        assert!(g.close_edge(c));
        assert!(g.is_closed(c));
        let detour = g.route(0, 3).unwrap();
        assert_eq!(detour.nodes, vec![0, 1, 3]);
        assert!((detour.time_s - 40.0).abs() < 1e-3);

        assert!(g.reopen_edge(c));
        assert_eq!(g.route(0, 3).unwrap().nodes, vec![0, 2, 3]);
    }

    #[test]
    fn no_open_route_is_none() {
        let (mut g, [a, _, c, _]) = diamond();
        g.close_edge(a);
        g.close_edge(c);
        assert!(g.route(0, 3).is_none());
    }

    #[test]
    fn route_to_self_is_empty() {
        let (g, _) = diamond();
        let r = g.route(2, 2).unwrap();
        assert_eq!(r.nodes, vec![2]);
        assert!(r.edges.is_empty());
        assert_eq!(r.time_s, 0.0);
    }

    #[test]
    fn edges_are_directed() {
        let mut g = RoadGraph::new(2);
        g.add_edge(0, 1, 10.0, 5.0).unwrap();
        assert!(g.route(0, 1).is_some());
        assert!(g.route(1, 0).is_none());
    }

    #[test]
    fn invalid_input_is_rejected() {
        let mut g = RoadGraph::new(2);
        assert!(g.add_edge(0, 2, 10.0, 5.0).is_none());
        assert!(g.add_edge(0, 1, 0.0, 5.0).is_none());
        assert!(g.add_edge(0, 1, f32::NAN, 5.0).is_none());
        assert!(g.add_edge(0, 1, 10.0, -1.0).is_none());
        assert!(g.add_edge(0, 1, 10.0, f32::INFINITY).is_none());
        assert!(g.route(0, 5).is_none());
        assert!(!g.close_edge(0));
        assert!(!g.reopen_edge(0));
    }
}
