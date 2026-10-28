//! Detection: group LiDAR points into objects.
//!
//! Points closer than `link_m` to each other belong to the same object. The
//! groups are the connected components of that relation, found with union-find.
//! Each object is reported with its centroid and its axis-aligned extent.
//!
//! This is two-dimensional, on the ground plane, and does not classify the
//! objects it finds.

/// One detected object.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Cluster {
    /// Mean of the cluster's points, in metres.
    pub centroid: [f32; 2],
    /// Smallest corner of the extent, in metres.
    pub min: [f32; 2],
    /// Largest corner of the extent, in metres.
    pub max: [f32; 2],
    /// Number of points in the cluster.
    pub points: usize,
}

fn find(parent: &mut [usize], mut i: usize) -> usize {
    while parent[i] != i {
        parent[i] = parent[parent[i]];
        i = parent[i];
    }
    i
}

/// Groups `points` into clusters whose members are linked by distances of at
/// most `link_m`. Clusters are returned in the order of their first point.
///
/// This is quadratic in the number of points. A LiDAR sweep is a few hundred
/// points, so that is acceptable here.
#[must_use]
pub fn cluster(points: &[[f32; 2]], link_m: f32) -> Vec<Cluster> {
    let n = points.len();
    let mut parent: Vec<usize> = (0..n).collect();
    let link_sq = link_m * link_m;
    for i in 0..n {
        for j in (i + 1)..n {
            let dx = points[i][0] - points[j][0];
            let dy = points[i][1] - points[j][1];
            if dx * dx + dy * dy <= link_sq {
                let (a, b) = (find(&mut parent, i), find(&mut parent, j));
                if a != b {
                    parent[b] = a;
                }
            }
        }
    }

    // Root index to cluster slot, in order of first appearance.
    let mut slot = vec![usize::MAX; n];
    let mut sums: Vec<Accumulator> = Vec::new();
    for (i, p) in points.iter().enumerate() {
        let root = find(&mut parent, i);
        if slot[root] == usize::MAX {
            slot[root] = sums.len();
            sums.push(Accumulator::start(*p));
        }
        sums[slot[root]].add(*p);
    }
    sums.into_iter().map(Accumulator::finish).collect()
}

/// Running sum, extent and count for one cluster.
#[derive(Clone, Copy)]
struct Accumulator {
    sum: [f32; 2],
    min: [f32; 2],
    max: [f32; 2],
    count: usize,
}

impl Accumulator {
    fn start(p: [f32; 2]) -> Self {
        Self {
            sum: [0.0, 0.0],
            min: p,
            max: p,
            count: 0,
        }
    }

    fn add(&mut self, p: [f32; 2]) {
        self.sum = [self.sum[0] + p[0], self.sum[1] + p[1]];
        self.min = [self.min[0].min(p[0]), self.min[1].min(p[1])];
        self.max = [self.max[0].max(p[0]), self.max[1].max(p[1])];
        self.count += 1;
    }

    fn finish(self) -> Cluster {
        let n = self.count as f32;
        Cluster {
            centroid: [self.sum[0] / n, self.sum[1] / n],
            min: self.min,
            max: self.max,
            points: self.count,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn two_separated_groups_make_two_clusters() {
        let points = [
            [0.0, 0.0],
            [0.1, 0.0],
            [0.2, 0.1],
            [10.0, 10.0],
            [10.1, 10.2],
        ];
        let clusters = cluster(&points, 0.5);
        assert_eq!(clusters.len(), 2);
        assert_eq!(clusters[0].points, 3);
        assert_eq!(clusters[1].points, 2);
    }

    #[test]
    fn a_chain_of_points_linked_pairwise_is_one_cluster() {
        // Each neighbour is 0.4 m away, so the chain links even though its ends
        // are 1.6 m apart.
        let points: Vec<[f32; 2]> = (0..5).map(|i| [i as f32 * 0.4, 0.0]).collect();
        let clusters = cluster(&points, 0.5);
        assert_eq!(clusters.len(), 1);
        assert_eq!(clusters[0].points, 5);
    }

    #[test]
    fn the_extent_and_centroid_cover_the_points() {
        let points = [[1.0, 2.0], [3.0, 2.0], [2.0, 4.0]];
        let c = cluster(&points, 2.5)[0];
        assert_eq!(c.min, [1.0, 2.0]);
        assert_eq!(c.max, [3.0, 4.0]);
        assert!((c.centroid[0] - 2.0).abs() < 1e-6 && (c.centroid[1] - 8.0 / 3.0).abs() < 1e-6);
    }

    #[test]
    fn no_points_no_clusters() {
        assert!(cluster(&[], 1.0).is_empty());
    }
}
