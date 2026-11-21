#![allow(dead_code)]
use crate::map::ArenaMap;
use crate::protocol::*;
use std::collections::HashMap;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MatchMode {
    Unknown,
    Practice1v15,
    Ffa,
}

#[derive(Debug, Clone)]
pub struct EnemyTrack {
    pub id: i64,
    pub pos: [f64; 3],
    pub vel: [f64; 3],
    pub accel: [f64; 3],
    pub aim: [f64; 2],
    pub hp: f64,
    pub max_hp: f64,
    pub cubes: i64,
    pub weapons: Vec<String>,
    pub charging: bool,
    pub charge_start_t: Option<f64>,
    pub last_seen_t: f64,
    pub visible_now: bool,
    pub damaged_by_us: f64,
    pub dmg_dealt_to_us: f64,
    pub last_hit_us_t: f64,
    pub prev_hp: f64,
}

impl EnemyTrack {
    pub fn has_weapon(&self, w: &str) -> bool {
        self.weapons.iter().any(|x| x == w)
    }
    pub fn scale(&self, size_per_cube: f64) -> f64 {
        1.0 + size_per_cube * (self.cubes.max(0) as f64)
    }
    pub fn radius(&self, base_r: f64, size_per_cube: f64) -> f64 {
        base_r * self.scale(size_per_cube)
    }
    pub fn height(&self, base_h: f64, size_per_cube: f64) -> f64 {
        base_h * self.scale(size_per_cube)
    }
    pub fn center(&self, base_h: f64, size_per_cube: f64) -> [f64; 3] {
        [self.pos[0], self.pos[1], self.pos[2] + 0.5 * self.height(base_h, size_per_cube)]
    }
    pub fn muzzle(&self, base_muzzle: f64, size_per_cube: f64) -> [f64; 3] {
        [self.pos[0], self.pos[1], self.pos[2] + base_muzzle * self.scale(size_per_cube)]
    }
    pub fn charge_elapsed(&self, now: f64) -> f64 {
        if self.charging {
            self.charge_start_t.map(|s| (now - s).max(0.0)).unwrap_or(0.0)
        } else {
            0.0
        }
    }
}

#[derive(Debug, Clone)]
pub struct SoundContact {
    pub kind: String,
    pub est_pos: [f64; 2],
    pub bearing: f64,
    pub dist_est: f64,
    pub t: f64,
}

#[derive(Debug, Clone)]
pub struct KnownPickup {
    pub id: i64,
    pub kind: String,
    pub pos: [f64; 3],
    pub last_seen_t: f64,
}

#[derive(Debug, Clone)]
pub struct KnownCrate {
    pub id: i64,
    pub pos: [f64; 3],
    pub hp: f64,
    pub last_seen_t: f64,
}

#[derive(Debug, Clone)]
pub struct KnownCube {
    pub id: i64,
    pub pos: [f64; 3],
    pub last_seen_t: f64,
}

pub struct WorldState {
    pub config: Config,
    pub my_slot: i64,
    pub our_kills: i64,
    pub initial_alive: i64,
    pub enemies: HashMap<i64, EnemyTrack>,
    pub npcs: HashMap<i64, ActorObs>,
    pub sounds: Vec<SoundContact>,
    pub pickups: HashMap<i64, KnownPickup>,
    pub crates: HashMap<i64, KnownCrate>,
    pub cubes: HashMap<i64, KnownCube>,
    // Combat / survival state
    pub last_combat_t: f64,
    pub last_damaged_t: f64, // non-zone attacker only!
    pub last_damaged_bearing: Option<f64>,
    pub last_damaged_by: Option<i64>,
    pub last_damaged_rear: bool,
    pub laser_shorted_until: f64,
    pub last_enemy_los_t: f64,
    pub my_laser_charge_start_t: Option<f64>,
    // Mode detection (Practice 1v15 vs Final FFA)
    pub ffa_evidence: f64,
    pub practice_evidence: f64,
    pub mode: MatchMode,
    // Movement / stuck tracking
    pub prev_pos: [f64; 3],
    pub prev_t: f64,
    pub last_cmd_move: [f64; 2],
    pub stuck_time: f64,
    pub stuck_escape_until: f64,
    pub stuck_escape_dir: [f64; 2],
    pub zone_boost_active: bool,
    // Rooftop climb commitment & tactical goal hysteresis
    pub climb_goal: Option<([f64; 3], f64)>,
    pub climb_cooldown_until: f64,
    pub last_tactical_goal: Option<[f64; 3]>,
    // Strafe & awareness scan
    pub strafe_sign: f64,
    pub next_strafe_flip_t: f64,
    pub scan_yaw: f64,
    pub rng_state: u64,
}

impl WorldState {
    pub fn new(hello: &Hello) -> Self {
        let seed = (hello.seed as u64)
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        WorldState {
            config: hello.config.clone(),
            my_slot: hello.slot,
            our_kills: 0,
            initial_alive: 16,
            enemies: HashMap::new(),
            npcs: HashMap::new(),
            sounds: Vec::new(),
            pickups: HashMap::new(),
            crates: HashMap::new(),
            cubes: HashMap::new(),
            last_combat_t: -100.0,
            last_damaged_t: -100.0,
            last_damaged_bearing: None,
            last_damaged_by: None,
            last_damaged_rear: false,
            laser_shorted_until: -10.0,
            last_enemy_los_t: -100.0,
            my_laser_charge_start_t: None,
            ffa_evidence: 0.0,
            practice_evidence: 0.0,
            mode: MatchMode::Unknown,
            prev_pos: [0.0; 3],
            prev_t: -1.0,
            last_cmd_move: [0.0, 0.0],
            stuck_time: 0.0,
            stuck_escape_until: -10.0,
            stuck_escape_dir: [1.0, 0.0],
            zone_boost_active: false,
            climb_goal: None,
            climb_cooldown_until: -10.0,
            last_tactical_goal: None,
            strafe_sign: 1.0,
            next_strafe_flip_t: 1.5,
            scan_yaw: 0.0,
            rng_state: if seed == 0 { 0x9E3779B97F4A7C15 } else { seed },
        }
    }

    pub fn next_rand(&mut self) -> f64 {
        let mut x = self.rng_state;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.rng_state = x;
        (x >> 11) as f64 * (1.0 / ((1u64 << 53) as f64))
    }

    pub fn update(&mut self, obs: &Observation, map: &ArenaMap) {
        let t = obs.t;
        let dt = if self.prev_t >= 0.0 { (t - self.prev_t).clamp(0.05, 1.0) } else { 0.1 };
        if self.prev_t < 0.0 {
            self.initial_alive = obs.alive.max(1);
            self.prev_pos = obs.me.pos;
            self.scan_yaw = obs.me.aim[0];
        }

        // Track our own laser charging start time (robust even if server caps reported charge at full!)
        let cur_laser_chg = obs
            .me
            .weapons
            .iter()
            .flatten()
            .find(|w| w.kind == "laser")
            .and_then(|w| w.charge)
            .unwrap_or(0.0);
        if cur_laser_chg > 0.0 {
            if self.my_laser_charge_start_t.is_none() {
                self.my_laser_charge_start_t = Some(t - cur_laser_chg);
            }
        } else {
            self.my_laser_charge_start_t = None;
        }

        // 1. Process events
        let mut hit_by_us_ids: Vec<(i64, f64)> = Vec::new();
        for ev in &obs.events {
            match ev.kind.as_str() {
                "damaged" => {
                    self.last_combat_t = t;
                    // Only update directional attacker state for non-zone hits!
                    if let Some(b) = ev.bearing {
                        self.last_damaged_t = t;
                        self.last_damaged_bearing = Some(b);
                        self.last_damaged_rear = ev.rear.unwrap_or(false);
                        if let Some(by) = ev.by {
                            self.last_damaged_by = Some(by);
                            if let Some(tr) = self.enemies.get_mut(&by) {
                                tr.dmg_dealt_to_us += ev.amount.unwrap_or(0.0);
                                tr.last_hit_us_t = t;
                            }
                        }
                    }
                }
                "hit" => {
                    self.last_combat_t = t;
                    if let Some(tid) = ev.target {
                        hit_by_us_ids.push((tid, ev.amount.unwrap_or(0.0)));
                    }
                }
                "effect" => {
                    if ev.effect.as_deref() == Some("short") {
                        let short_s = self.config.weapon_f("lightning", "short_s", 2.0);
                        self.laser_shorted_until = t + short_s;
                    }
                }
                "kill" => {
                    self.our_kills += 1;
                    if let Some(tid) = ev.target {
                        self.enemies.remove(&tid);
                    }
                }
                _ => {}
            }
        }
        if !obs.me.in_zone {
            self.last_combat_t = t;
        }

        // 2. Refresh NPCs first (needed to avoid NPC-damage false positives in mode detection!)
        self.npcs.clear();
        for a in &obs.mechs {
            if a.id != obs.me.id && a.is_npc() {
                self.npcs.insert(a.id, a.clone());
            }
        }

        // Update tracked enemy mechs
        for tr in self.enemies.values_mut() {
            tr.visible_now = false;
        }

        let base_h = self.config.mech_f("height", 9.0);
        let spc = self.config.cubes.size_per_cube;

        for a in &obs.mechs {
            if a.id == obs.me.id || !a.is_mech() {
                continue;
            }
            let hp = a.hp.unwrap_or(1400.0);
            let max_hp = a.max_hp.unwrap_or(1400.0);
            let cubes = a.cubes.unwrap_or(0);
            let near_any_npc = self
                .npcs
                .values()
                .any(|n| dist2d([a.pos[0], a.pos[1]], [n.pos[0], n.pos[1]]) < 95.0);

            if let Some(tr) = self.enemies.get_mut(&a.id) {
                let gap = (t - tr.last_seen_t).max(0.05);
                if gap <= 0.35 {
                    let dv = [
                        (a.vel[0] - tr.vel[0]) / gap,
                        (a.vel[1] - tr.vel[1]) / gap,
                        (a.vel[2] - tr.vel[2]) / gap,
                    ];
                    tr.accel = [
                        0.5 * tr.accel[0] + 0.5 * dv[0].clamp(-60.0, 60.0),
                        0.5 * tr.accel[1] + 0.5 * dv[1].clamp(-60.0, 60.0),
                        0.5 * tr.accel[2] + 0.5 * dv[2].clamp(-60.0, 60.0),
                    ];
                } else {
                    tr.accel = [0.0; 3];
                }
                let hp_drop = tr.hp - hp;
                let us_dmg: f64 = hit_by_us_ids
                    .iter()
                    .filter(|(id, _)| *id == a.id)
                    .map(|(_, amt)| *amt)
                    .sum();
                let in_z = dist2d([a.pos[0], a.pos[1]], obs.zone.center) < obs.zone.radius - 5.0;
                // Only count as FFA damage if >= 40 dmg (above single 25-dmg NPC bullet) and no NPC nearby!
                if hp_drop > us_dmg + 40.0 && in_z && !near_any_npc {
                    self.ffa_evidence += 2.2;
                }
                tr.prev_hp = tr.hp;
                tr.pos = a.pos;
                tr.vel = a.vel;
                tr.aim = a.aim;
                tr.hp = hp;
                tr.max_hp = max_hp;
                tr.cubes = cubes;
                tr.weapons = a.weapons.clone();
                if a.charging {
                    if !tr.charging {
                        tr.charge_start_t = Some(t);
                    }
                } else {
                    tr.charge_start_t = None;
                }
                tr.charging = a.charging;
                tr.last_seen_t = t;
                tr.visible_now = true;
                tr.damaged_by_us += us_dmg;
            } else {
                let us_dmg: f64 = hit_by_us_ids
                    .iter()
                    .filter(|(id, _)| *id == a.id)
                    .map(|(_, amt)| *amt)
                    .sum();
                let in_z = dist2d([a.pos[0], a.pos[1]], obs.zone.center) < obs.zone.radius - 8.0;
                if hp < max_hp - 180.0 && us_dmg == 0.0 && in_z && !near_any_npc && t > 5.0 {
                    self.ffa_evidence += 1.5;
                }
                self.enemies.insert(
                    a.id,
                    EnemyTrack {
                        id: a.id,
                        pos: a.pos,
                        vel: a.vel,
                        accel: [0.0; 3],
                        aim: a.aim,
                        hp,
                        max_hp,
                        cubes,
                        weapons: a.weapons.clone(),
                        charging: a.charging,
                        charge_start_t: if a.charging { Some(t) } else { None },
                        last_seen_t: t,
                        visible_now: true,
                        damaged_by_us: us_dmg,
                        dmg_dealt_to_us: 0.0,
                        last_hit_us_t: -100.0,
                        prev_hp: hp,
                    },
                );
            }
        }

        self.enemies.retain(|_, tr| t - tr.last_seen_t <= 12.0);

        // 3. Robust Mode Detection (Practice 1v15 vs Final 16-mech FFA, zero false positives!)
        let non_self_deaths = (self.initial_alive - obs.alive) - self.our_kills;
        if (non_self_deaths >= 2 && obs.zone.radius > 160.0) || (non_self_deaths >= 1 && t < 95.0) {
            self.ffa_evidence += 4.5 * (non_self_deaths as f64);
        }

        // Only count an enemy beam as FFA evidence if it points at ANOTHER known mech (not crates/NPCs/near-miss on us!)
        for b in &obs.beams {
            if b.by == obs.me.id || b.path.len() < 2 {
                continue;
            }
            let p0 = b.path[0];
            let p1 = b.path[1];
            let beam_yaw = (p1[1] - p0[1]).atan2(p1[0] - p0[0]);
            let to_us_yaw = (obs.me.pos[1] - p0[1]).atan2(obs.me.pos[0] - p0[0]);
            if angle_diff(beam_yaw, to_us_yaw).abs() > 0.35 {
                let aimed_at_other_mech = self.enemies.values().any(|other| {
                    if other.id == b.by || !other.visible_now {
                        return false;
                    }
                    let y_other = (other.pos[1] - p0[1]).atan2(other.pos[0] - p0[0]);
                    angle_diff(beam_yaw, y_other).abs() < 0.22
                });
                if aimed_at_other_mech {
                    self.ffa_evidence += 2.0;
                }
            }
        }

        // Practice 1v15 evidence: 2+ visible enemies ignoring each other while both aiming at us
        let vis_ids: Vec<i64> = self
            .enemies
            .values()
            .filter(|e| e.visible_now)
            .map(|e| e.id)
            .collect();
        if vis_ids.len() >= 2 && non_self_deaths <= 0 {
            let mut both_aiming_us = 0;
            for &id in &vis_ids {
                if let Some(e) = self.enemies.get(&id) {
                    let yaw_to_us = (obs.me.pos[1] - e.pos[1]).atan2(obs.me.pos[0] - e.pos[0]);
                    if angle_diff(e.aim[0], yaw_to_us).abs() < 0.35 {
                        both_aiming_us += 1;
                    }
                }
            }
            if both_aiming_us >= 2 {
                let e0 = &self.enemies[&vis_ids[0]];
                let e1 = &self.enemies[&vis_ids[1]];
                if dist3d(e0.pos, e1.pos) < 110.0
                    && map.has_shot_los(e0.center(base_h, spc), e1.center(base_h, spc))
                {
                    self.practice_evidence += 0.35;
                }
            }
        }
        if t > 40.0 && non_self_deaths <= 0 {
            self.practice_evidence += 0.1;
        }

        self.mode = if self.ffa_evidence >= 4.0 {
            MatchMode::Ffa
        } else if self.practice_evidence >= 2.0 {
            MatchMode::Practice1v15
        } else if t > 25.0 && non_self_deaths <= 0 {
            MatchMode::Practice1v15
        } else {
            MatchMode::Unknown
        };

        // 4. Update sounds
        self.sounds.retain(|s| t - s.t <= 2.5);
        for s in &obs.sounds {
            if s.kind == "death" || s.kind == "drop" {
                continue;
            }
            let d_est = match s.dist.as_str() {
                "near" => 35.0,
                "mid" => 105.0,
                _ => 210.0,
            };
            let est_pos = [
                obs.me.pos[0] + s.bearing.cos() * d_est,
                obs.me.pos[1] + s.bearing.sin() * d_est,
            ];
            self.sounds.push(SoundContact {
                kind: s.kind.clone(),
                est_pos,
                bearing: s.bearing,
                dist_est: d_est,
                t: t - s.age,
            });
        }
        if self.sounds.len() > 24 {
            let drain_n = self.sounds.len() - 24;
            self.sounds.drain(0..drain_n);
        }

        // 5. Remember seen pickups, crates, cubes (prune vanished ones inside proximity OR 100-deg FOV!)
        let should_be_visible = |p: [f64; 3], z_off: f64| -> bool {
            let d = dist2d([obs.me.pos[0], obs.me.pos[1]], [p[0], p[1]]);
            let pt = [p[0], p[1], p[2] + z_off];
            if d < 36.0 {
                return map.has_shot_los(obs.me.eye_pos(), pt);
            }
            if d < 125.0 {
                let ang = angle_diff((p[1] - obs.me.pos[1]).atan2(p[0] - obs.me.pos[0]), obs.me.aim[0]).abs();
                if ang < 50.0f64.to_radians()
                    && map.has_shot_los(obs.me.eye_pos(), pt)
                    && !map.trees_block_vision(obs.me.eye_pos(), pt)
                {
                    return true;
                }
            }
            false
        };

        let vis_pickup_ids: Vec<i64> = obs.pickups.iter().map(|p| p.id).collect();
        for p in &obs.pickups {
            self.pickups.insert(
                p.id,
                KnownPickup { id: p.id, kind: p.kind.clone(), pos: p.pos, last_seen_t: t },
            );
        }
        self.pickups.retain(|id, p| {
            if vis_pickup_ids.contains(id) { return true; }
            if t - p.last_seen_t > 20.0 { return false; }
            !should_be_visible(p.pos, 0.8)
        });

        let vis_crate_ids: Vec<i64> = obs.crates.iter().map(|c| c.id).collect();
        for c in &obs.crates {
            self.crates.insert(
                c.id,
                KnownCrate { id: c.id, pos: c.pos, hp: c.hp.unwrap_or(300.0), last_seen_t: t },
            );
        }
        self.crates.retain(|id, c| {
            if vis_crate_ids.contains(id) { return true; }
            if t - c.last_seen_t > 18.0 { return false; }
            !should_be_visible(c.pos, 1.2)
        });

        let vis_cube_ids: Vec<i64> = obs.cubes.iter().map(|c| c.id).collect();
        for c in &obs.cubes {
            self.cubes.insert(
                c.id,
                KnownCube { id: c.id, pos: c.pos, last_seen_t: t },
            );
        }
        self.cubes.retain(|id, c| {
            if vis_cube_ids.contains(id) { return true; }
            if t - c.last_seen_t > 15.0 { return false; }
            !should_be_visible(c.pos, 0.8)
        });

        // 6. Stuck detection (works off-ground too, e.g. overhangs/steep slopes; biases escape toward zone!)
        let cmd_len = self.last_cmd_move[0].hypot(self.last_cmd_move[1]);
        let moved = dist2d([obs.me.pos[0], obs.me.pos[1]], [self.prev_pos[0], self.prev_pos[1]]);
        let intentional_vertical_climb = self.climb_goal.is_some() && obs.me.vel[2] > 3.0;
        if cmd_len > 0.35 && moved < 0.25 * (dt / 0.1) && !intentional_vertical_climb {
            self.stuck_time += dt;
        } else {
            self.stuck_time = (self.stuck_time - 2.0 * dt).max(0.0);
        }
        if self.stuck_time > 0.65 && t > self.stuck_escape_until {
            // Test 8 candidate directions; prefer unblocked directions biased toward next_zone!
            let to_zone_yaw = (obs.zone.next_center[1] - obs.me.pos[1])
                .atan2(obs.zone.next_center[0] - obs.me.pos[0]);
            let base_ang = self.last_cmd_move[1].atan2(self.last_cmd_move[0]);
            let offsets = [1.3, -1.3, 2.0, -2.0, 2.6, -2.6, 0.7, -0.7];
            let mut best_dir = [to_zone_yaw.cos(), to_zone_yaw.sin()];
            let mut best_sc = f64::NEG_INFINITY;
            for &off in &offsets {
                let ang = base_ang + off;
                let tx = obs.me.pos[0] + ang.cos() * 7.0;
                let ty = obs.me.pos[1] + ang.sin() * 7.0;
                let tz = map.terrain_z(tx, ty);
                if map.is_cylinder_blocked(tx, ty, tz, obs.me.radius() + 0.3, obs.me.height(), obs.me.step_up()) {
                    continue;
                }
                let align_zone = angle_diff(ang, to_zone_yaw).cos();
                let sc = align_zone * 2.0 + self.next_rand() * 0.4;
                if sc > best_sc {
                    best_sc = sc;
                    best_dir = [ang.cos(), ang.sin()];
                }
            }
            self.stuck_escape_dir = best_dir;
            self.stuck_escape_until = t + 0.85;
            self.stuck_time = 0.0;
        }

        // Strafe rhythm flip
        if t >= self.next_strafe_flip_t {
            self.strafe_sign = -self.strafe_sign;
            self.next_strafe_flip_t = t + 1.2 + self.next_rand() * 1.1;
        }

        self.prev_pos = obs.me.pos;
        self.prev_t = t;
    }

    pub fn is_ffa(&self) -> bool {
        self.mode == MatchMode::Ffa
    }

    pub fn time_until_regen(&self, obs: &Observation) -> f64 {
        let elapsed = obs.t - self.last_combat_t;
        (obs.me.regen_delay() - elapsed).max(0.0)
    }

    pub fn is_laser_shorted(&self, t: f64) -> bool {
        t < self.laser_shorted_until
    }
}
