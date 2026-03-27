use solverforge_maps::{BoundingBox, NetworkConfig, RoadNetwork};
use textplots::{Chart, Plot, Shape};

struct Location {
    name: &'static str,
    bbox: BoundingBox,
}

fn locations() -> Vec<Location> {
    vec![
        Location {
            name: "Philadelphia (City Hall area)",
            bbox: BoundingBox::new(39.946, -75.174, 39.962, -75.150),
        },
        Location {
            name: "Dragoncello (Poggio Rusco, IT)",
            bbox: BoundingBox::new(44.978, 11.095, 44.986, 11.108),
        },
        Location {
            name: "Clusone (Lombardy, IT)",
            bbox: BoundingBox::new(45.882, 9.940, 45.895, 9.960),
        },
    ]
}

fn plot_network(name: &str, network: &RoadNetwork, bbox: &BoundingBox) {
    let nodes: Vec<(f64, f64)> = network.nodes_iter().collect();
    let edges: Vec<(usize, usize, f64, f64)> = network.edges_iter().collect();

    if nodes.is_empty() {
        println!("\n{}: No data", name);
        return;
    }

    let mut segments: Vec<(f32, f32)> = Vec::new();
    let mut seen = std::collections::HashSet::new();
    for &(from, to, _, _) in &edges {
        let key = (from.min(to), from.max(to));
        if seen.insert(key) && from < nodes.len() && to < nodes.len() {
            let (lat1, lng1) = nodes[from];
            let (lat2, lng2) = nodes[to];
            segments.push((lng1 as f32, lat1 as f32));
            segments.push((lng2 as f32, lat2 as f32));
            segments.push((f32::NAN, f32::NAN));
        }
    }

    let intersections: Vec<(f32, f32)> = nodes
        .iter()
        .map(|(lat, lng)| (*lng as f32, *lat as f32))
        .collect();

    println!("\n{}", name);
    println!(
        "{} nodes, {} edges",
        network.node_count(),
        network.edge_count()
    );

    let x_min = bbox.min_lng as f32;
    let x_max = bbox.max_lng as f32;

    Chart::new(180, 60, x_min, x_max)
        .lineplot(&Shape::Lines(&segments))
        .lineplot(&Shape::Points(&intersections))
        .nice();
}

#[tokio::main]
async fn main() {
    let config = NetworkConfig::default();

    for loc in locations() {
        print!("\nFetching {}...", loc.name);
        match RoadNetwork::load_or_fetch(&loc.bbox, &config, None).await {
            Ok(network_ref) => {
                println!(" done");
                plot_network(loc.name, &network_ref, &loc.bbox);
            }
            Err(e) => {
                println!(" failed: {}", e);
            }
        }
    }
}
