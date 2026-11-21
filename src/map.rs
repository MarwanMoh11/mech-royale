#![allow(dead_code)]
use serde::Deserialize;
use std::collections::BinaryHeap;
use std::cmp::Ordering;
use std::fs;
use std::path::Path;

#[derive(Debug, Clone, Deserialize)]
struct RawWorld {
    bounds: RawBounds,
    terrain: RawTerrain,
    #[serde(default)]
    rules: RawRules,
    #[serde(default)]
    prisms: Vec<RawPrism>,
    #[serde(default)]
    trees: RawTrees,
}

#[derive(Debug, Clone, Deserialize)]
struct RawBounds {
    #[serde(rename = "minX")]
    min_x: f64,
    #[serde(rename = "maxX")]
    max_x: f64,
    #[serde(rename = "minY")]
    min_y: f64,
    #[serde(rename = "maxY")]
    max_y: f64,
}

#[derive(Debug, Clone, Deserialize)]
struct RawTerrain {
    #[serde(rename = "minX")]
    min_x: f64,
    #[serde(rename = "minY")]
    min_y: f64,
    step: f64,
    nx: usize,
    ny: usize,
    #[serde(default = "default_scale")]
    scale: f64,
}

fn default_scale() -> f64 { 0.01 }

#[derive(Debug, Clone, Default, Deserialize)]
struct RawRules {
    #[serde(rename = "canopyDensity", default = "default_canopy_density")]
    canopy_density: f64,
    #[serde(rename = "canopyBlockT", default = "default_canopy_block_t")]
    canopy_block_t: f64,
}

fn default_canopy_density() -> f64 { 0.35 }
fn default_canopy_block_t() -> f64 { 0.25 }

#[derive(Debug, Clone, Deserialize)]
struct RawPrism {
    #[serde(default)]
    kind: String,
    base: f64,
    top: f64,
    ring: Vec<f64>,
    #[serde(default)]
    holes: Vec<Vec<f64>>,
}

#[derive(Debug, Clone, Default, Deserialize)]
struct RawTrees {
    #[serde(default)]
    fields: Vec<String>,
    #[serde(default)]
    data: Vec<f64>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PrismKind {
    Building,
    Stadium,
    Bridge,
}

#[derive(Debug, Clone)]
pub struct Prism {
    pub id: usize,
    pub kind: PrismKind,
    pub base: f64,
    pub top: f64,
    pub min_x: f64,
    pub max_x: f64,
    pub min_y: f64,
    pub max_y: f64,
    pub outer: Vec<[f64; 2]>,
    pub holes: Vec<Vec<[f64; 2]>>,
    pub centroid: [f64; 2],
    pub area: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct Tree {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub height: f64,
    pub crown_r: f64,
    pub crown_base: f64,
    pub trunk_r: f64,
    pub crown_x: f64,
    pub crown_y: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct Rooftop {
    pub prism_id: usize,
    pub pos: [f64; 3], // safe interior point on top of roof
    pub height_above_ground: f64,
    pub area: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct RayHit {
    pub t: f64,
    pub pos: [f64; 3],
    pub prism_id: Option<usize>, // None = terrain
}

pub const NAV_STEP: f64 = 8.0;
const GRID_CELL: f64 = 16.0;

pub struct ArenaMap {
    pub min_x: f64,
    pub max_x: f64,
    pub min_y: f64,
    pub max_y: f64,
    // Terrain
    t_min_x: f64,
    t_min_y: f64,
    t_step: f64,
    t_nx: usize,
    t_ny: usize,
    t_scale: f64,
    terrain: Vec<i16>,
    // Prisms & spatial grid
    pub prisms: Vec<Prism>,
    grid_min_x: f64,
    grid_min_y: f64,
    grid_nx: usize,
    grid_ny: usize,
    prism_grid: Vec<Vec<u16>>,
    // Trees & spatial grid
    pub trees: Vec<Tree>,
    tree_grid: Vec<Vec<u16>>,
    pub canopy_density: f64,
    pub canopy_block_t: f64,
    // Tactical rooftops
    pub rooftops: Vec<Rooftop>,
    // Coarse 2D navigation grid (ground walkability, headroom for cube-growth bridge clearance, elevation, cover)
    pub nav_nx: usize,
    pub nav_ny: usize,
    pub nav_min_x: f64,
    pub nav_min_y: f64,
    pub nav_walkable: Vec<bool>,
    pub nav_headroom: Vec<f32>,
    pub nav_z: Vec<f64>,
    pub nav_cover: Vec<f32>,
}

impl ArenaMap {
    pub fn load(map_dir: &str) -> Result<Self, String> {
        let dir = Path::new(map_dir);
        let world_path = dir.join("v2").join("world.json");
        let terrain_path = dir.join("v2").join("terrain.i16");

        let world_str = fs::read_to_string(&world_path)
            .map_err(|e| format!("failed reading {}: {}", world_path.display(), e))?;
        let raw: RawWorld = serde_json::from_str(&world_str)
            .map_err(|e| format!("failed parsing world.json: {}", e))?;

        let raw_bytes = fs::read(&terrain_path)
            .map_err(|e| format!("failed reading {}: {}", terrain_path.display(), e))?;
        let expected_samples = raw.terrain.nx * raw.terrain.ny;
        if raw_bytes.len() < expected_samples * 2 {
            return Err(format!(
                "terrain.i16 too small: {} < {}",
                raw_bytes.len(),
                expected_samples * 2
            ));
        }
        let mut terrain = Vec::with_capacity(expected_samples);
        for chunk in raw_bytes[..expected_samples * 2].chunks_exact(2) {
            terrain.push(i16::from_le_bytes([chunk[0], chunk[1]]));
        }

        // Parse prisms
        let mut prisms = Vec::with_capacity(raw.prisms.len());
        let mut p_min_x = raw.bounds.min_x;
        let mut p_max_x = raw.bounds.max_x;
        let mut p_min_y = raw.bounds.min_y;
        let mut p_max_y = raw.bounds.max_y;

        for (id, rp) in raw.prisms.into_iter().enumerate() {
            let outer = flat_to_ring(&rp.ring);
            if outer.len() < 3 {
                continue;
            }
            let mut holes = Vec::with_capacity(rp.holes.len());
            for h in rp.holes {
                let hr = flat_to_ring(&h);
                if hr.len() >= 3 {
                    holes.push(hr);
                }
            }
            let mut min_x = f64::INFINITY;
            let mut max_x = f64::NEG_INFINITY;
            let mut min_y = f64::INFINITY;
            let mut max_y = f64::NEG_INFINITY;
            for &[x, y] in &outer {
                if x < min_x { min_x = x; }
                if x > max_x { max_x = x; }
                if y < min_y { min_y = y; }
                if y > max_y { max_y = y; }
            }
            p_min_x = p_min_x.min(min_x);
            p_max_x = p_max_x.max(max_x);
            p_min_y = p_min_y.min(min_y);
            p_max_y = p_max_y.max(max_y);

            let (area, centroid) = polygon_area_centroid(&outer);
            let kind = match rp.kind.as_str() {
                "bridge" => PrismKind::Bridge,
                "stadium" => PrismKind::Stadium,
                _ => PrismKind::Building,
            };
            prisms.push(Prism {
                id,
                kind,
                base: rp.base,
                top: rp.top,
                min_x,
                max_x,
                min_y,
                max_y,
                outer,
                holes,
                centroid,
                area,
            });
        }

        // Spatial grid for prisms and trees
        let grid_min_x = p_min_x - 32.0;
        let grid_min_y = p_min_y - 32.0;
        let grid_nx = (((p_max_x + 32.0) - grid_min_x) / GRID_CELL).ceil() as usize + 1;
        let grid_ny = (((p_max_y + 32.0) - grid_min_y) / GRID_CELL).ceil() as usize + 1;
        let mut prism_grid = vec![Vec::<u16>::new(); grid_nx * grid_ny];
        for (idx, p) in prisms.iter().enumerate() {
            let ix0 = (((p.min_x - 4.0) - grid_min_x) / GRID_CELL).floor().max(0.0) as usize;
            let ix1 = ((((p.max_x + 4.0) - grid_min_x) / GRID_CELL).floor().max(0.0) as usize).min(grid_nx - 1);
            let iy0 = (((p.min_y - 4.0) - grid_min_y) / GRID_CELL).floor().max(0.0) as usize;
            let iy1 = ((((p.max_y + 4.0) - grid_min_y) / GRID_CELL).floor().max(0.0) as usize).min(grid_ny - 1);
            for iy in iy0..=iy1 {
                for ix in ix0..=ix1 {
                    prism_grid[iy * grid_nx + ix].push(idx as u16);
                }
            }
        }

        // Parse trees
        let mut trees = Vec::new();
        let mut tree_grid = vec![Vec::<u16>::new(); grid_nx * grid_ny];
        let nf = raw.trees.fields.len();
        if nf > 0 && raw.trees.data.len() >= nf {
            let f_idx = |name: &str| raw.trees.fields.iter().position(|s| s == name);
            let ix_x = f_idx("x");
            let ix_y = f_idx("y");
            let ix_z = f_idx("z");
            let ix_h = f_idx("height");
            let ix_cr = f_idx("crownR");
            let ix_cb = f_idx("crownBase");
            let ix_tr = f_idx("trunkR");
            let ix_cx = f_idx("crownX");
            let ix_cy = f_idx("crownY");
            if let (Some(fx), Some(fy), Some(fz), Some(fh)) = (ix_x, ix_y, ix_z, ix_h) {
                let count = raw.trees.data.len() / nf;
                trees.reserve(count);
                for i in 0..count {
                    let row = &raw.trees.data[i * nf..(i + 1) * nf];
                    let x = row[fx];
                    let y = row[fy];
                    let z = row[fz];
                    let height = row[fh];
                    let crown_r = ix_cr.map(|k| row[k]).unwrap_or(3.0);
                    let crown_base = ix_cb.map(|k| row[k]).unwrap_or(z + height * 0.4);
                    let trunk_r = ix_tr.map(|k| row[k]).unwrap_or(0.5);
                    let crown_x = ix_cx.map(|k| row[k]).unwrap_or(x);
                    let crown_y = ix_cy.map(|k| row[k]).unwrap_or(y);
                    let tid = trees.len() as u16;
                    trees.push(Tree {
                        x, y, z, height, crown_r, crown_base, trunk_r, crown_x, crown_y,
                    });
                    let r = crown_r.max(trunk_r) + 1.0;
                    let gx0 = (((x.min(crown_x) - r) - grid_min_x) / GRID_CELL).floor().max(0.0) as usize;
                    let gx1 = ((((x.max(crown_x) + r) - grid_min_x) / GRID_CELL).floor().max(0.0) as usize).min(grid_nx - 1);
                    let gy0 = (((y.min(crown_y) - r) - grid_min_y) / GRID_CELL).floor().max(0.0) as usize;
                    let gy1 = ((((y.max(crown_y) + r) - grid_min_y) / GRID_CELL).floor().max(0.0) as usize).min(grid_ny - 1);
                    for gy in gy0..=gy1 {
                        for gx in gx0..=gx1 {
                            tree_grid[gy * grid_nx + gx].push(tid);
                        }
                    }
                }
            }
        }

        let mut map = ArenaMap {
            min_x: raw.bounds.min_x,
            max_x: raw.bounds.max_x,
            min_y: raw.bounds.min_y,
            max_y: raw.bounds.max_y,
            t_min_x: raw.terrain.min_x,
            t_min_y: raw.terrain.min_y,
            t_step: raw.terrain.step,
            t_nx: raw.terrain.nx,
            t_ny: raw.terrain.ny,
            t_scale: raw.terrain.scale,
            terrain,
            prisms,
            grid_min_x,
            grid_min_y,
            grid_nx,
            grid_ny,
            prism_grid,
            trees,
            tree_grid,
            canopy_density: raw.rules.canopy_density,
            canopy_block_t: raw.rules.canopy_block_t,
            rooftops: Vec::new(),
            nav_nx: 0,
            nav_ny: 0,
            nav_min_x: raw.bounds.min_x + 4.0,
            nav_min_y: raw.bounds.min_y + 4.0,
            nav_walkable: Vec::new(),
            nav_headroom: Vec::new(),
            nav_z: Vec::new(),
            nav_cover: Vec::new(),
        };

        map.build_rooftops_and_nav();
        Ok(map)
    }

    pub fn fallback_empty() -> Self {
        ArenaMap {
            min_x: -512.0,
            max_x: 555.0,
            min_y: -333.0,
            max_y: 388.0,
            t_min_x: -513.0,
            t_min_y: -334.0,
            t_step: 1.0,
            t_nx: 2,
            t_ny: 2,
            t_scale: 0.01,
            terrain: vec![0; 4],
            prisms: Vec::new(),
            grid_min_x: -550.0,
            grid_min_y: -370.0,
            grid_nx: 1,
            grid_ny: 1,
            prism_grid: vec![Vec::new()],
            trees: Vec::new(),
            tree_grid: vec![Vec::new()],
            canopy_density: 0.35,
            canopy_block_t: 0.25,
            rooftops: Vec::new(),
            nav_nx: 1,
            nav_ny: 1,
            nav_min_x: 0.0,
            nav_min_y: 0.0,
            nav_walkable: vec![true],
            nav_headroom: vec![999.0],
            nav_z: vec![0.0],
            nav_cover: vec![0.0],
        }
    }

    fn build_rooftops_and_nav(&mut self) {
        // 1. Identify climbable tactical rooftops with full cylinder clearance check!
        for (idx, p) in self.prisms.iter().enumerate() {
            if p.area < 45.0 {
                continue;
            }
            let mut pt = None;
            if self.point_in_prism_2d(idx, p.centroid[0], p.centroid[1])
                && !self.is_cylinder_blocked(p.centroid[0], p.centroid[1], p.top + 0.05, 2.7, 9.5, 0.5)
            {
                pt = Some(p.centroid);
            } else {
                'search: for gx in 1..=5 {
                    for gy in 1..=5 {
                        let x = p.min_x + (p.max_x - p.min_x) * (gx as f64 / 6.0);
                        let y = p.min_y + (p.max_y - p.min_y) * (gy as f64 / 6.0);
                        if self.point_in_prism_2d(idx, x, y)
                            && !self.is_cylinder_blocked(x, y, p.top + 0.05, 2.7, 9.5, 0.5)
                        {
                            pt = Some([x, y]);
                            break 'search;
                        }
                    }
                }
            }
            if let Some([rx, ry]) = pt {
                let tz = self.terrain_z(rx, ry);
                let h_ag = p.top - tz;
                if (6.0..=52.0).contains(&h_ag) {
                    self.rooftops.push(Rooftop {
                        prism_id: idx,
                        pos: [rx, ry, p.top],
                        height_above_ground: h_ag,
                        area: p.area,
                    });
                }
            }
        }

        // 2. Build 8m ground navigation grid with dynamic headroom (for cube-grown bridge clearance!)
        self.nav_min_x = self.min_x + 4.0;
        self.nav_min_y = self.min_y + 4.0;
        self.nav_nx = (((self.max_x - 4.0) - self.nav_min_x) / NAV_STEP).floor().max(1.0) as usize + 1;
        self.nav_ny = (((self.max_y - 4.0) - self.nav_min_y) / NAV_STEP).floor().max(1.0) as usize + 1;
        let n = self.nav_nx * self.nav_ny;
        self.nav_walkable = vec![false; n];
        self.nav_headroom = vec![999.0; n];
        self.nav_z = vec![0.0; n];
        self.nav_cover = vec![0.0; n];

        for iy in 0..self.nav_ny {
            let y = self.nav_min_y + iy as f64 * NAV_STEP;
            for ix in 0..self.nav_nx {
                let x = self.nav_min_x + ix as f64 * NAV_STEP;
                let tz = self.terrain_z(x, y);
                let idx = iy * self.nav_nx + ix;
                self.nav_z[idx] = tz;
                let slope_deg = self.terrain_slope_deg(x, y);
                if slope_deg > 48.0 {
                    self.nav_walkable[idx] = false;
                } else {
                    self.nav_walkable[idx] = !self.is_cylinder_blocked(x, y, tz, 2.7, 9.0, 1.2);
                    if self.nav_walkable[idx] {
                        // Compute minimum overhang headroom above ground (bridges 4..8!)
                        let mut min_hr = 999.0f64;
                        if let Some((gx, gy)) = self.grid_coords(x, y) {
                            for &pid in &self.prism_grid[gy * self.grid_nx + gx] {
                                let p = &self.prisms[pid as usize];
                                if p.base > tz + 1.2 && (self.point_in_prism_2d(pid as usize, x, y) || dist_sq_to_ring(x, y, &p.outer) < 12.0) {
                                    min_hr = min_hr.min(p.base - tz);
                                }
                            }
                        }
                        self.nav_headroom[idx] = min_hr as f32;
                    }
                }
            }
        }

        // Precompute cover score (fraction of 8 neighbors that are building walls!)
        for iy in 1..self.nav_ny.saturating_sub(1) {
            for ix in 1..self.nav_nx.saturating_sub(1) {
                let idx = iy * self.nav_nx + ix;
                if !self.nav_walkable[idx] {
                    continue;
                }
                let mut adj_wall = 0.0f32;
                for dy in -2i32..=2 {
                    for dx in -2i32..=2 {
                        if dx == 0 && dy == 0 { continue; }
                        let nx = ix as i32 + dx;
                        let ny = iy as i32 + dy;
                        if nx >= 0 && nx < self.nav_nx as i32 && ny >= 0 && ny < self.nav_ny as i32 {
                            if !self.nav_walkable[ny as usize * self.nav_nx + nx as usize] {
                                let d2 = (dx * dx + dy * dy) as f32;
                                adj_wall += 1.0 / d2;
                            }
                        }
                    }
                }
                self.nav_cover[idx] = adj_wall.min(4.0) * 0.25;
            }
        }
    }

    #[inline]
    pub fn terrain_z(&self, x: f64, y: f64) -> f64 {
        if self.t_nx < 2 || self.t_ny < 2 {
            return 0.0;
        }
        let fx = (x - self.t_min_x) / self.t_step;
        let fy = (y - self.t_min_y) / self.t_step;
        let i = (fx.floor() as isize).clamp(0, (self.t_nx - 2) as isize) as usize;
        let j = (fy.floor() as isize).clamp(0, (self.t_ny - 2) as isize) as usize;
        let tx = (fx - i as f64).clamp(0.0, 1.0);
        let ty = (fy - j as f64).clamp(0.0, 1.0);
        let z00 = self.terrain[j * self.t_nx + i] as f64 * self.t_scale;
        let z10 = self.terrain[j * self.t_nx + i + 1] as f64 * self.t_scale;
        let z01 = self.terrain[(j + 1) * self.t_nx + i] as f64 * self.t_scale;
        let z11 = self.terrain[(j + 1) * self.t_nx + i + 1] as f64 * self.t_scale;
        (z00 * (1.0 - tx) + z10 * tx) * (1.0 - ty) + (z01 * (1.0 - tx) + z11 * tx) * ty
    }

    pub fn terrain_slope_deg(&self, x: f64, y: f64) -> f64 {
        let h = 1.0;
        let dz_dx = (self.terrain_z(x + h, y) - self.terrain_z(x - h, y)) / (2.0 * h);
        let dz_dy = (self.terrain_z(x, y + h) - self.terrain_z(x, y - h)) / (2.0 * h);
        dz_dx.hypot(dz_dy).atan().to_degrees()
    }

    #[inline]
    fn grid_coords(&self, x: f64, y: f64) -> Option<(usize, usize)> {
        let gx = ((x - self.grid_min_x) / GRID_CELL).floor() as isize;
        let gy = ((y - self.grid_min_y) / GRID_CELL).floor() as isize;
        if gx >= 0 && gx < self.grid_nx as isize && gy >= 0 && gy < self.grid_ny as isize {
            Some((gx as usize, gy as usize))
        } else {
            None
        }
    }

    #[inline]
    pub fn point_in_prism_2d(&self, prism_idx: usize, x: f64, y: f64) -> bool {
        let p = &self.prisms[prism_idx];
        if x < p.min_x || x > p.max_x || y < p.min_y || y > p.max_y {
            return false;
        }
        if !point_in_ring(x, y, &p.outer) {
            return false;
        }
        for hole in &p.holes {
            if point_in_ring(x, y, hole) {
                return false;
            }
        }
        true
    }

    pub fn is_covered_above(&self, x: f64, y: f64, z_low: f64, headroom: f64) -> bool {
        if let Some((gx, gy)) = self.grid_coords(x, y) {
            for &pid in &self.prism_grid[gy * self.grid_nx + gx] {
                let p = &self.prisms[pid as usize];
                if p.top > z_low && p.base < z_low + headroom && self.point_in_prism_2d(pid as usize, x, y) {
                    return true;
                }
            }
        }
        false
    }

    /// Support floor height under (x, y) reachable from feet_z (stepping up by step_up).
    pub fn support_z(&self, x: f64, y: f64, feet_z: f64, step_up: f64, mech_h: f64) -> f64 {
        let mut best = self.terrain_z(x, y);
        if let Some((gx, gy)) = self.grid_coords(x, y) {
            for &pid in &self.prism_grid[gy * self.grid_nx + gx] {
                let p = &self.prisms[pid as usize];
                if p.top <= feet_z + step_up + 0.05 && p.top > best && self.point_in_prism_2d(pid as usize, x, y) {
                    if !self.is_covered_above(x, y, p.top + 0.1, mech_h - 0.1) {
                        best = p.top;
                    }
                }
            }
        }
        best
    }

    /// Highest prism top at (x, y) (or terrain if none).
    pub fn highest_surface_z(&self, x: f64, y: f64) -> (f64, Option<usize>) {
        let mut best = self.terrain_z(x, y);
        let mut best_p = None;
        if let Some((gx, gy)) = self.grid_coords(x, y) {
            for &pid in &self.prism_grid[gy * self.grid_nx + gx] {
                let p = &self.prisms[pid as usize];
                if p.top > best && self.point_in_prism_2d(pid as usize, x, y) {
                    best = p.top;
                    best_p = Some(pid as usize);
                }
            }
        }
        (best, best_p)
    }

    /// True if a vertical cylinder at (x, y, feet_z) of radius r and height h intersects any solid prism.
    pub fn is_cylinder_blocked(&self, x: f64, y: f64, feet_z: f64, r: f64, h: f64, step_up: f64) -> bool {
        if x < self.min_x + r || x > self.max_x - r || y < self.min_y + r || y > self.max_y - r {
            return true;
        }
        let gx0 = (((x - r) - self.grid_min_x) / GRID_CELL).floor().max(0.0) as usize;
        let gx1 = ((((x + r) - self.grid_min_x) / GRID_CELL).floor().max(0.0) as usize).min(self.grid_nx - 1);
        let gy0 = (((y - r) - self.grid_min_y) / GRID_CELL).floor().max(0.0) as usize;
        let gy1 = ((((y + r) - self.grid_min_y) / GRID_CELL).floor().max(0.0) as usize).min(self.grid_ny - 1);
        let r2 = r * r;
        for gy in gy0..=gy1 {
            for gx in gx0..=gx1 {
                for &pid in &self.prism_grid[gy * self.grid_nx + gx] {
                    let p = &self.prisms[pid as usize];
                    // Vertical overlap check: prism top must be above feet+step_up, base below head
                    if p.top <= feet_z + step_up || p.base >= feet_z + h {
                        continue;
                    }
                    if x + r < p.min_x || x - r > p.max_x || y + r < p.min_y || y - r > p.max_y {
                        continue;
                    }
                    if self.point_in_prism_2d(pid as usize, x, y) {
                        return true;
                    }
                    if dist_sq_to_ring(x, y, &p.outer) < r2 {
                        return true;
                    }
                    for hole in &p.holes {
                        if dist_sq_to_ring(x, y, hole) < r2 {
                            return true;
                        }
                    }
                }
            }
        }
        false
    }

    /// Wall push-out vector for smooth local steering around buildings.
    pub fn wall_repulsion(&self, x: f64, y: f64, feet_z: f64, check_r: f64, h: f64, step_up: f64) -> [f64; 2] {
        let mut rx = 0.0;
        let mut ry = 0.0;
        let gx0 = (((x - check_r) - self.grid_min_x) / GRID_CELL).floor().max(0.0) as usize;
        let gx1 = ((((x + check_r) - self.grid_min_x) / GRID_CELL).floor().max(0.0) as usize).min(self.grid_nx - 1);
        let gy0 = (((y - check_r) - self.grid_min_y) / GRID_CELL).floor().max(0.0) as usize;
        let gy1 = ((((y + check_r) - self.grid_min_y) / GRID_CELL).floor().max(0.0) as usize).min(self.grid_ny - 1);
        let r2 = check_r * check_r;
        let mut seen = [u16::MAX; 16];
        let mut seen_n = 0usize;

        for gy in gy0..=gy1 {
            for gx in gx0..=gx1 {
                for &pid in &self.prism_grid[gy * self.grid_nx + gx] {
                    if seen[..seen_n].contains(&pid) { continue; }
                    if seen_n < seen.len() { seen[seen_n] = pid; seen_n += 1; }
                    let p = &self.prisms[pid as usize];
                    if p.top <= feet_z + step_up || p.base >= feet_z + h {
                        continue;
                    }
                    if x + check_r < p.min_x || x - check_r > p.max_x || y + check_r < p.min_y || y - check_r > p.max_y {
                        continue;
                    }
                    let (cx, cy, d2) = closest_point_on_ring(x, y, &p.outer);
                    if d2 < r2 && d2 > 1e-8 {
                        let d = d2.sqrt();
                        let w = (check_r - d) / check_r;
                        rx += ((x - cx) / d) * w;
                        ry += ((y - cy) / d) * w;
                    }
                    for hole in &p.holes {
                        let (hx, hy, hd2) = closest_point_on_ring(x, y, hole);
                        if hd2 < r2 && hd2 > 1e-8 {
                            let d = hd2.sqrt();
                            let w = (check_r - d) / check_r;
                            rx += ((x - hx) / d) * w;
                            ry += ((y - hy) / d) * w;
                        }
                    }
                }
            }
        }
        [rx, ry]
    }

    /// Exact 3D raycast against all prisms and terrain.
    pub fn raycast_solid(&self, origin: [f64; 3], dir: [f64; 3], max_dist: f64) -> Option<RayHit> {
        if max_dist <= 0.0 || !max_dist.is_finite() {
            return None;
        }
        let mut best_t = max_dist;
        let mut best_prism: Option<usize> = None;
        let mut hit_any = false;

        // 1. Traverse 2D prism grid via DDA
        let end_x = origin[0] + dir[0] * max_dist;
        let end_y = origin[1] + dir[1] * max_dist;
        let fx0 = (origin[0] - self.grid_min_x) / GRID_CELL;
        let fy0 = (origin[1] - self.grid_min_y) / GRID_CELL;
        let fx1 = (end_x - self.grid_min_x) / GRID_CELL;
        let fy1 = (end_y - self.grid_min_y) / GRID_CELL;

        let mut ix = fx0.floor() as isize;
        let mut iy = fy0.floor() as isize;
        let end_ix = fx1.floor() as isize;
        let end_iy = fy1.floor() as isize;

        let step_x: isize = if dir[0] > 1e-9 { 1 } else if dir[0] < -1e-9 { -1 } else { 0 };
        let step_y: isize = if dir[1] > 1e-9 { 1 } else if dir[1] < -1e-9 { -1 } else { 0 };

        let t_delta_x = if step_x != 0 { (GRID_CELL / dir[0]).abs() } else { f64::INFINITY };
        let t_delta_y = if step_y != 0 { (GRID_CELL / dir[1]).abs() } else { f64::INFINITY };

        let mut t_max_x = if step_x > 0 {
            (((ix + 1) as f64 * GRID_CELL + self.grid_min_x) - origin[0]) / dir[0]
        } else if step_x < 0 {
            ((ix as f64 * GRID_CELL + self.grid_min_x) - origin[0]) / dir[0]
        } else {
            f64::INFINITY
        };
        let mut t_max_y = if step_y > 0 {
            (((iy + 1) as f64 * GRID_CELL + self.grid_min_y) - origin[1]) / dir[1]
        } else if step_y < 0 {
            ((iy as f64 * GRID_CELL + self.grid_min_y) - origin[1]) / dir[1]
        } else {
            f64::INFINITY
        };

        let mut tested = [0u64; 5]; // bitset for up to 320 prisms! Zero heap allocs!
        let max_steps = ((end_ix - ix).abs() + (end_iy - iy).abs() + 2).min(200);

        for _ in 0..max_steps {
            if ix >= 0 && ix < self.grid_nx as isize && iy >= 0 && iy < self.grid_ny as isize {
                let cell = &self.prism_grid[iy as usize * self.grid_nx + ix as usize];
                for &pid in cell {
                    let p_idx = pid as usize;
                    let word = p_idx >> 6;
                    let bit = 1u64 << (p_idx & 63);
                    if word < tested.len() {
                        if (tested[word] & bit) != 0 {
                            continue;
                        }
                        tested[word] |= bit;
                    }
                    if let Some(t_hit) = self.ray_vs_prism(p_idx, origin, dir, best_t) {
                        if t_hit < best_t {
                            best_t = t_hit;
                            best_prism = Some(p_idx);
                            hit_any = true;
                        }
                    }
                }
            }
            if ix == end_ix && iy == end_iy {
                break;
            }
            let t_next = t_max_x.min(t_max_y);
            if t_next > best_t {
                break;
            }
            if t_max_x < t_max_y {
                ix += step_x;
                t_max_x += t_delta_x;
            } else {
                iy += step_y;
                t_max_y += t_delta_y;
            }
        }

        // 2. Check terrain intersection up to best_t (1.8m steps for berm precision)
        let step: f64 = 1.8;
        let mut prev_t: f64 = 0.0;
        let mut t = step.min(best_t);
        while prev_t < best_t {
            let x = origin[0] + dir[0] * t;
            let y = origin[1] + dir[1] * t;
            let z = origin[2] + dir[2] * t;
            let tz = self.terrain_z(x, y);
            if z <= tz {
                // Binary search refine between prev_t and t
                let mut lo = prev_t;
                let mut hi = t;
                for _ in 0..6 {
                    let mid = 0.5 * (lo + hi);
                    let mx = origin[0] + dir[0] * mid;
                    let my = origin[1] + dir[1] * mid;
                    let mz = origin[2] + dir[2] * mid;
                    if mz <= self.terrain_z(mx, my) {
                        hi = mid;
                    } else {
                        lo = mid;
                    }
                }
                if hi < best_t {
                    best_t = hi;
                    best_prism = None;
                    hit_any = true;
                }
                break;
            }
            if t >= best_t {
                break;
            }
            prev_t = t;
            t = (t + step).min(best_t);
        }

        if hit_any {
            Some(RayHit {
                t: best_t,
                pos: [
                    origin[0] + dir[0] * best_t,
                    origin[1] + dir[1] * best_t,
                    origin[2] + dir[2] * best_t,
                ],
                prism_id: best_prism,
            })
        } else {
            None
        }
    }

    fn ray_vs_prism(&self, p_idx: usize, o: [f64; 3], d: [f64; 3], max_t: f64) -> Option<f64> {
        let p = &self.prisms[p_idx];
        // Fast 3D AABB slab test
        let (mut tmin, mut tmax) = (0.0f64, max_t);
        for axis in 0..3 {
            let (bmin, bmax) = match axis {
                0 => (p.min_x, p.max_x),
                1 => (p.min_y, p.max_y),
                _ => (p.base, p.top),
            };
            if d[axis].abs() < 1e-12 {
                if o[axis] < bmin || o[axis] > bmax {
                    return None;
                }
            } else {
                let inv = 1.0 / d[axis];
                let mut t1 = (bmin - o[axis]) * inv;
                let mut t2 = (bmax - o[axis]) * inv;
                if t1 > t2 {
                    std::mem::swap(&mut t1, &mut t2);
                }
                tmin = tmin.max(t1);
                tmax = tmax.min(t2);
                if tmin > tmax {
                    return None;
                }
            }
        }

        // Check if origin itself is inside the prism
        if o[2] >= p.base && o[2] <= p.top && self.point_in_prism_2d(p_idx, o[0], o[1]) {
            return Some(0.0);
        }

        let mut best = f64::INFINITY;

        // Top & bottom caps
        if d[2].abs() > 1e-12 {
            for cap_z in [p.top, p.base] {
                let t = (cap_z - o[2]) / d[2];
                if t >= 0.0 && t < best && t <= max_t {
                    let hx = o[0] + d[0] * t;
                    let hy = o[1] + d[1] * t;
                    if self.point_in_prism_2d(p_idx, hx, hy) {
                        best = t;
                    }
                }
            }
        }

        // Side walls (outer ring + holes)
        best = best.min(ray_vs_ring_walls(o, d, p.base, p.top, &p.outer, best.min(max_t)));
        for hole in &p.holes {
            best = best.min(ray_vs_ring_walls(o, d, p.base, p.top, hole, best.min(max_t)));
        }

        if best <= max_t {
            Some(best)
        } else {
            None
        }
    }

    /// True if direct shot line from `a` to `b` is unobstructed by buildings, bridges, or terrain.
    #[inline]
    pub fn has_shot_los(&self, a: [f64; 3], b: [f64; 3]) -> bool {
        let dx = b[0] - a[0];
        let dy = b[1] - a[1];
        let dz = b[2] - a[2];
        let dist = (dx * dx + dy * dy + dz * dz).sqrt();
        if dist < 0.25 {
            return true;
        }
        let inv = 1.0 / dist;
        let dir = [dx * inv, dy * inv, dz * inv];
        self.raycast_solid(a, dir, dist - 0.15).is_none()
    }

    /// Checks whether trees (trunks or dense foliage) block vision along segment a -> b.
    pub fn trees_block_vision(&self, a: [f64; 3], b: [f64; 3]) -> bool {
        let dx = b[0] - a[0];
        let dy = b[1] - a[1];
        let dz = b[2] - a[2];
        let dist = (dx * dx + dy * dy + dz * dz).sqrt();
        if dist < 0.5 {
            return false;
        }
        let inv = 1.0 / dist;
        let dir = [dx * inv, dy * inv, dz * inv];
        let max_canopy_len = -(self.canopy_block_t.max(1e-4)).ln() / self.canopy_density.max(1e-4);

        let min_x = a[0].min(b[0]) - 6.0;
        let max_x = a[0].max(b[0]) + 6.0;
        let min_y = a[1].min(b[1]) - 6.0;
        let max_y = a[1].max(b[1]) + 6.0;
        let gx0 = ((min_x - self.grid_min_x) / GRID_CELL).floor().max(0.0) as usize;
        let gx1 = (((max_x - self.grid_min_x) / GRID_CELL).floor().max(0.0) as usize).min(self.grid_nx - 1);
        let gy0 = ((min_y - self.grid_min_y) / GRID_CELL).floor().max(0.0) as usize;
        let gy1 = (((max_y - self.grid_min_y) / GRID_CELL).floor().max(0.0) as usize).min(self.grid_ny - 1);

        let mut canopy_len = 0.0;
        // Traverse only cells near line (skip far cells in AABB)
        let nx_perp = -dir[1];
        let ny_perp = dir[0];
        let hlen = (dir[0] * dir[0] + dir[1] * dir[1]).sqrt();

        for gy in gy0..=gy1 {
            let cy = self.grid_min_y + (gy as f64 + 0.5) * GRID_CELL;
            for gx in gx0..=gx1 {
                let cx = self.grid_min_x + (gx as f64 + 0.5) * GRID_CELL;
                if hlen > 1e-6 {
                    let perp = ((cx - a[0]) * nx_perp + (cy - a[1]) * ny_perp).abs() / hlen;
                    if perp > GRID_CELL * 1.15 {
                        continue;
                    }
                }
                for &tid in &self.tree_grid[gy * self.grid_nx + gx] {
                    let tr = &self.trees[tid as usize];
                    // Trunk cylinder check
                    if let Some((t0, t1)) = ray_cylinder_z(a, dir, dist, tr.x, tr.y, tr.trunk_r, tr.z, tr.crown_base) {
                        if t1 > t0 {
                            return true;
                        }
                    }
                    // Crown sphere/cylinder approximation
                    let crown_mid_z = 0.5 * (tr.crown_base + (tr.z + tr.height));
                    let crown_rz = 0.5 * ((tr.z + tr.height) - tr.crown_base).max(1.0);
                    if let Some((t0, t1)) = ray_ellipsoid(a, dir, dist, tr.crown_x, tr.crown_y, crown_mid_z, tr.crown_r, crown_rz) {
                        canopy_len += (t1 - t0).max(0.0);
                        if canopy_len >= max_canopy_len {
                            return true;
                        }
                    }
                }
            }
        }
        false
    }

    /// Check if a parabolic mortar trajectory clears solid obstacles up to 95% of flight time.
    pub fn mortar_arc_clear(&self, origin: [f64; 3], vel: [f64; 3], gravity: f64, flight_t: f64) -> bool {
        let steps = 14;
        let check_t = (flight_t * 0.95).max(0.15);
        let mut prev = origin;
        for i in 1..=steps {
            let t = check_t * (i as f64 / steps as f64);
            let cur = [
                origin[0] + vel[0] * t,
                origin[1] + vel[1] * t,
                origin[2] + vel[2] * t - 0.5 * gravity * t * t,
            ];
            if !self.has_shot_los(prev, cur) {
                return false;
            }
            prev = cur;
        }
        true
    }

    /// Fast 2D A* on the 8m navigation grid with dynamic mech radius/height (cube-safe bridges!) & closed skip!
    pub fn next_waypoint(&self, from: [f64; 2], to: [f64; 2], feet_z: f64) -> [f64; 2] {
        self.next_waypoint_sized(from, to, feet_z, 2.7, 9.2, 1.2)
    }

    pub fn next_waypoint_sized(
        &self,
        from: [f64; 2],
        to: [f64; 2],
        feet_z: f64,
        mech_r: f64,
        mech_h: f64,
        step_up: f64,
    ) -> [f64; 2] {
        let d = (to[0] - from[0]).hypot(to[1] - from[1]);
        if d < 5.0 {
            return to;
        }
        // Check straight segment using ground tz (fixes downhill wall-blindness!)
        let sample_n = (d / 4.0).ceil().clamp(2.0, 24.0) as usize;
        let mut clear = true;
        let from_tz = self.terrain_z(from[0], from[1]);
        let on_roof = feet_z > from_tz + 3.0;
        for i in 1..sample_n {
            let frac = i as f64 / sample_n as f64;
            let sx = from[0] + (to[0] - from[0]) * frac;
            let sy = from[1] + (to[1] - from[1]) * frac;
            let tz = self.terrain_z(sx, sy);
            let chk_z = if on_roof { self.support_z(sx, sy, feet_z, step_up, mech_h) } else { tz };
            if self.terrain_slope_deg(sx, sy) > 48.0
                || self.is_cylinder_blocked(sx, sy, chk_z, mech_r + 0.2, mech_h, step_up)
            {
                clear = false;
                break;
            }
        }
        if clear {
            return to;
        }

        if self.nav_nx < 2 || self.nav_ny < 2 {
            return to;
        }

        let start = self.nearest_walkable_cell(from, mech_h as f32);
        let goal = self.nearest_walkable_cell(to, mech_h as f32);
        if start == goal {
            return to;
        }

        let n = self.nav_nx * self.nav_ny;
        let mut g_score = vec![f32::INFINITY; n];
        let mut came_from = vec![u32::MAX; n];
        let mut heap = BinaryHeap::with_capacity(256);

        let s_idx = start.1 * self.nav_nx + start.0;
        let g_idx = goal.1 * self.nav_nx + goal.0;
        g_score[s_idx] = 0.0;
        heap.push(AStarNode {
            f: octile(start, goal),
            idx: s_idx as u32,
        });

        let dirs: [(isize, isize, f32); 8] = [
            (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142),
        ];

        let mut expanded = 0usize;
        let mut best_closest = s_idx;
        let mut best_h = octile(start, goal);
        let need_hr = mech_h as f32 + 0.15;

        while let Some(cur) = heap.pop() {
            let ci = cur.idx as usize;
            let cx = ci % self.nav_nx;
            let cy = ci / self.nav_nx;
            let cg = g_score[ci];
            let ch = octile((cx, cy), goal);
            // Skip stale heap duplicates (critical for Ohio Stadium concave detours!)
            if cur.f > cg + ch + 1e-3 {
                continue;
            }
            if ci == g_idx {
                best_closest = g_idx;
                break;
            }
            expanded += 1;
            if expanded > 3200 {
                break;
            }
            if ch < best_h {
                best_h = ch;
                best_closest = ci;
            }
            for &(dx, dy, cost) in &dirs {
                let nx = cx as isize + dx;
                let ny = cy as isize + dy;
                if nx < 0 || nx >= self.nav_nx as isize || ny < 0 || ny >= self.nav_ny as isize {
                    continue;
                }
                let ux = nx as usize;
                let uy = ny as usize;
                let ni = uy * self.nav_nx + ux;
                if !self.nav_walkable[ni] || self.nav_headroom[ni] < need_hr {
                    continue;
                }
                if dx != 0 && dy != 0 {
                    let i1 = cy * self.nav_nx + ux;
                    let i2 = uy * self.nav_nx + cx;
                    if !self.nav_walkable[i1] || self.nav_headroom[i1] < need_hr
                        || !self.nav_walkable[i2] || self.nav_headroom[i2] < need_hr
                    {
                        continue;
                    }
                }
                let ng = cg + cost;
                if ng < g_score[ni] {
                    g_score[ni] = ng;
                    came_from[ni] = ci as u32;
                    heap.push(AStarNode {
                        f: ng + octile((ux, uy), goal),
                        idx: ni as u32,
                    });
                }
            }
        }

        if best_closest == s_idx {
            return to;
        }
        let mut path = Vec::with_capacity(64);
        let mut cur = best_closest;
        while cur != s_idx && came_from[cur] != u32::MAX {
            path.push(cur);
            cur = came_from[cur] as usize;
            if path.len() > 256 { break; }
        }
        path.reverse();

        let mut chosen = path[0];
        for &cell_idx in path.iter().take(6) {
            let wp = self.cell_to_world(cell_idx);
            let mid = [0.5 * (from[0] + wp[0]), 0.5 * (from[1] + wp[1])];
            let tz = self.terrain_z(mid[0], mid[1]);
            if !self.is_cylinder_blocked(mid[0], mid[1], tz, mech_r + 0.2, mech_h, step_up) {
                chosen = cell_idx;
            } else {
                break;
            }
        }
        self.cell_to_world(chosen)
    }

    fn cell_to_world(&self, idx: usize) -> [f64; 2] {
        let ix = idx % self.nav_nx;
        let iy = idx / self.nav_nx;
        [
            self.nav_min_x + ix as f64 * NAV_STEP,
            self.nav_min_y + iy as f64 * NAV_STEP,
        ]
    }

    fn nearest_walkable_cell(&self, p: [f64; 2], need_hr: f32) -> (usize, usize) {
        let ix = (((p[0] - self.nav_min_x) / NAV_STEP).round() as isize)
            .clamp(0, (self.nav_nx - 1) as isize) as usize;
        let iy = (((p[1] - self.nav_min_y) / NAV_STEP).round() as isize)
            .clamp(0, (self.nav_ny - 1) as isize) as usize;
        let ok = |ux: usize, uy: usize| {
            let idx = uy * self.nav_nx + ux;
            self.nav_walkable[idx] && self.nav_headroom[idx] >= need_hr
        };
        if ok(ix, iy) {
            return (ix, iy);
        }
        for rad in 1..=8isize {
            for dy in -rad..=rad {
                for dx in -rad..=rad {
                    if dx.abs() != rad && dy.abs() != rad { continue; }
                    let nx = ix as isize + dx;
                    let ny = iy as isize + dy;
                    if nx >= 0 && nx < self.nav_nx as isize && ny >= 0 && ny < self.nav_ny as isize {
                        let ux = nx as usize;
                        let uy = ny as usize;
                        if ok(ux, uy) {
                            return (ux, uy);
                        }
                    }
                }
            }
        }
        (ix, iy)
    }

    pub fn cover_at(&self, p: [f64; 2]) -> f32 {
        if self.nav_nx < 2 || self.nav_ny < 2 {
            return 0.0;
        }
        let ix = (((p[0] - self.nav_min_x) / NAV_STEP).round() as isize)
            .clamp(0, (self.nav_nx - 1) as isize) as usize;
        let iy = (((p[1] - self.nav_min_y) / NAV_STEP).round() as isize)
            .clamp(0, (self.nav_ny - 1) as isize) as usize;
        self.nav_cover[iy * self.nav_nx + ix]
    }
}

#[derive(Copy, Clone)]
struct AStarNode {
    f: f32,
    idx: u32,
}

impl Eq for AStarNode {}
impl PartialEq for AStarNode {
    fn eq(&self, other: &Self) -> bool { self.idx == other.idx }
}
impl Ord for AStarNode {
    fn cmp(&self, other: &Self) -> Ordering {
        other.f.partial_cmp(&self.f).unwrap_or(Ordering::Equal)
    }
}
impl PartialOrd for AStarNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> { Some(self.cmp(other)) }
}

#[inline]
fn octile(a: (usize, usize), b: (usize, usize)) -> f32 {
    let dx = (a.0 as isize - b.0 as isize).abs() as f32;
    let dy = (a.1 as isize - b.1 as isize).abs() as f32;
    let (mn, mx) = if dx < dy { (dx, dy) } else { (dy, dx) };
    mx + 0.4142 * mn
}

fn flat_to_ring(flat: &[f64]) -> Vec<[f64; 2]> {
    let n = flat.len() / 2;
    let mut ring = Vec::with_capacity(n);
    for i in 0..n {
        ring.push([flat[2 * i], flat[2 * i + 1]]);
    }
    // Drop closing duplicate vertex if present
    if ring.len() > 2 {
        let first = ring[0];
        let last = ring[ring.len() - 1];
        if (first[0] - last[0]).abs() < 1e-6 && (first[1] - last[1]).abs() < 1e-6 {
            ring.pop();
        }
    }
    ring
}

fn polygon_area_centroid(ring: &[[f64; 2]]) -> (f64, [f64; 2]) {
    let n = ring.len();
    if n < 3 {
        return (0.0, [0.0, 0.0]);
    }
    let mut a2 = 0.0;
    let mut cx = 0.0;
    let mut cy = 0.0;
    for i in 0..n {
        let p0 = ring[i];
        let p1 = ring[(i + 1) % n];
        let cross = p0[0] * p1[1] - p1[0] * p0[1];
        a2 += cross;
        cx += (p0[0] + p1[0]) * cross;
        cy += (p0[1] + p1[1]) * cross;
    }
    if a2.abs() < 1e-6 {
        (0.0, ring[0])
    } else {
        (a2.abs() * 0.5, [cx / (3.0 * a2), cy / (3.0 * a2)])
    }
}

#[inline]
pub fn point_in_ring(x: f64, y: f64, ring: &[[f64; 2]]) -> bool {
    let n = ring.len();
    if n < 3 {
        return false;
    }
    let mut inside = false;
    let mut j = n - 1;
    for i in 0..n {
        let xi = ring[i][0];
        let yi = ring[i][1];
        let xj = ring[j][0];
        let yj = ring[j][1];
        let intersect = ((yi > y) != (yj > y))
            && (x < (xj - xi) * (y - yi) / (yj - yi) + xi);
        if intersect {
            inside = !inside;
        }
        j = i;
    }
    inside
}

#[inline]
fn closest_point_on_ring(x: f64, y: f64, ring: &[[f64; 2]]) -> (f64, f64, f64) {
    let n = ring.len();
    let mut best_d2 = f64::INFINITY;
    let mut best_x = x;
    let mut best_y = y;
    for i in 0..n {
        let a = ring[i];
        let b = ring[(i + 1) % n];
        let abx = b[0] - a[0];
        let aby = b[1] - a[1];
        let denom = abx * abx + aby * aby;
        let t = if denom > 1e-12 {
            (((x - a[0]) * abx + (y - a[1]) * aby) / denom).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let px = a[0] + abx * t;
        let py = a[1] + aby * t;
        let d2 = (x - px) * (x - px) + (y - py) * (y - py);
        if d2 < best_d2 {
            best_d2 = d2;
            best_x = px;
            best_y = py;
        }
    }
    (best_x, best_y, best_d2)
}

#[inline]
fn dist_sq_to_ring(x: f64, y: f64, ring: &[[f64; 2]]) -> f64 {
    closest_point_on_ring(x, y, ring).2
}

#[inline]
fn ray_vs_ring_walls(o: [f64; 3], d: [f64; 3], base: f64, top: f64, ring: &[[f64; 2]], max_t: f64) -> f64 {
    let mut best = f64::INFINITY;
    let n = ring.len();
    for i in 0..n {
        let p0 = ring[i];
        let p1 = ring[(i + 1) % n];
        let sx = p1[0] - p0[0];
        let sy = p1[1] - p0[1];
        let denom = d[0] * sy - d[1] * sx;
        if denom.abs() < 1e-12 {
            continue;
        }
        let qx = p0[0] - o[0];
        let qy = p0[1] - o[1];
        let t = (qx * sy - qy * sx) / denom;
        if t < 0.0 || t >= best || t > max_t {
            continue;
        }
        let u = (qx * d[1] - qy * d[0]) / denom;
        if u < 0.0 || u > 1.0 {
            continue;
        }
        let hz = o[2] + d[2] * t;
        if hz >= base && hz <= top {
            best = t;
        }
    }
    best
}

fn ray_cylinder_z(
    o: [f64; 3],
    d: [f64; 3],
    max_t: f64,
    cx: f64,
    cy: f64,
    r: f64,
    z_min: f64,
    z_max: f64,
) -> Option<(f64, f64)> {
    let ox = o[0] - cx;
    let oy = o[1] - cy;
    let a = d[0] * d[0] + d[1] * d[1];
    let b = 2.0 * (ox * d[0] + oy * d[1]);
    let c = ox * ox + oy * oy - r * r;
    let (mut t0, mut t1) = if a < 1e-12 {
        if c <= 0.0 { (0.0, max_t) } else { return None; }
    } else {
        let disc = b * b - 4.0 * a * c;
        if disc < 0.0 { return None; }
        let s = disc.sqrt();
        ((-b - s) / (2.0 * a), (-b + s) / (2.0 * a))
    };
    t0 = t0.max(0.0);
    t1 = t1.min(max_t);
    if t0 >= t1 { return None; }
    if d[2].abs() < 1e-12 {
        if o[2] < z_min || o[2] > z_max { return None; }
    } else {
        let mut tz0 = (z_min - o[2]) / d[2];
        let mut tz1 = (z_max - o[2]) / d[2];
        if tz0 > tz1 { std::mem::swap(&mut tz0, &mut tz1); }
        t0 = t0.max(tz0);
        t1 = t1.min(tz1);
        if t0 >= t1 { return None; }
    }
    Some((t0, t1))
}

fn ray_ellipsoid(
    o: [f64; 3],
    d: [f64; 3],
    max_t: f64,
    cx: f64,
    cy: f64,
    cz: f64,
    rxy: f64,
    rz: f64,
) -> Option<(f64, f64)> {
    let inv_xy = 1.0 / rxy.max(0.1);
    let inv_z = 1.0 / rz.max(0.1);
    let ox = (o[0] - cx) * inv_xy;
    let oy = (o[1] - cy) * inv_xy;
    let oz = (o[2] - cz) * inv_z;
    let dx = d[0] * inv_xy;
    let dy = d[1] * inv_xy;
    let dz = d[2] * inv_z;
    let a = dx * dx + dy * dy + dz * dz;
    if a < 1e-12 { return None; }
    let b = 2.0 * (ox * dx + oy * dy + oz * dz);
    let c = ox * ox + oy * oy + oz * oz - 1.0;
    let disc = b * b - 4.0 * a * c;
    if disc < 0.0 { return None; }
    let s = disc.sqrt();
    let t0 = ((-b - s) / (2.0 * a)).max(0.0);
    let t1 = ((-b + s) / (2.0 * a)).min(max_t);
    if t0 < t1 { Some((t0, t1)) } else { None }
}
