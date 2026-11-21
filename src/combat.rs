use crate::map::ArenaMap;
use crate::protocol::*;
use crate::world_state::{EnemyTrack, WorldState};

#[allow(dead_code)]
#[derive(Debug, Clone)]
pub struct CombatDecision {
    pub aim: Option<[f64; 2]>,
    pub fire: [bool; 2],
    pub reload: bool,
    pub target_enemy_id: Option<i64>,
    pub target_has_los: bool,
    pub charging_laser: bool,
    pub laser_ready_soon: bool,
    pub want_peek: bool,
}

#[derive(Debug, Clone, Copy)]
pub struct DodgeOutput {
    pub vec: [f64; 2],
    pub urgency: f64,
    pub want_boost: bool,
    pub want_jump: bool,
}

pub fn find_weapon_slot(me: &SelfObs, kind: &str) -> Option<usize> {
    me.weapons.iter().position(|w| {
        w.as_ref().map(|x| x.kind.as_str() == kind).unwrap_or(false)
    })
}

pub fn cube_damage_mult(cubes: i64, cfg: &Config) -> f64 {
    let capped = (cubes.max(0) as f64).min(cfg.cubes.stat_cap);
    1.0 + cfg.cubes.damage_per_cube * capped
}

/// Predicts actual slewed torso aim after 1 sim tick (0.05s) given the commanded aim we are sending!
#[inline]
fn slew_one_tick(cur_aim: [f64; 2], cmd_aim: [f64; 2], turn_rate: f64, pitch_rate: f64, min_p: f64, max_p: f64) -> [f64; 2] {
    let dy = angle_diff(cmd_aim[0], cur_aim[0]).clamp(-turn_rate * 0.05, turn_rate * 0.05);
    let dp = (cmd_aim[1].clamp(min_p, max_p) - cur_aim[1]).clamp(-pitch_rate * 0.05, pitch_rate * 0.05);
    [wrap_angle(cur_aim[0] + dy), (cur_aim[1] + dp).clamp(min_p, max_p)]
}

/// Checks if a given actual aim `[yaw, pitch]` hits target cylinder `(target_r, target_h)` at `dist`.
#[inline]
fn aim_hits_cylinder(actual_aim: [f64; 2], desired_aim: [f64; 2], dist: f64, target_r: f64, target_h: f64) -> bool {
    let rem_yaw = angle_diff(desired_aim[0], actual_aim[0]).abs();
    let rem_pitch = (desired_aim[1] - actual_aim[1]).abs();
    let yaw_tol = (target_r / dist.max(1.0)).atan().clamp(0.012, 0.25);
    let pitch_tol = (target_h * 0.45 / dist.max(1.0)).atan().clamp(0.015, 0.25);
    rem_yaw <= yaw_tol && rem_pitch <= pitch_tol
}

/// Multi-height LOS check against an enemy mech (fixes head-glitching blindspots on roofs/hills!).
/// Returns Some(best_visible_point) checking chest, upper torso, then lower torso.
pub fn best_los_point(map: &ArenaMap, from: [f64; 3], e_pos: [f64; 3], e_h: f64) -> Option<[f64; 3]> {
    for frac in [0.50, 0.78, 0.26] {
        let pt = [e_pos[0], e_pos[1], e_pos[2] + e_h * frac];
        if map.has_shot_los(from, pt) {
            return Some(pt);
        }
    }
    None
}

/// Predicts straight projectile intercept aim INCLUDING 1-tick (0.05s) action latency for both shooter & target!
pub fn lead_aim_straight(
    muzzle: [f64; 3],
    my_vel: [f64; 3],
    target_pos: [f64; 3],
    target_vel: [f64; 3],
    proj_speed: f64,
    shot_offset: f64,
) -> ([f64; 2], [f64; 3], f64) {
    // Advance both positions by 0.05s (tick 1 firing instant)
    let m0 = [
        muzzle[0] + my_vel[0] * 0.05,
        muzzle[1] + my_vel[1] * 0.05,
        muzzle[2] + my_vel[2] * 0.05,
    ];
    let t0 = [
        target_pos[0] + target_vel[0] * 0.05,
        target_pos[1] + target_vel[1] * 0.05,
        target_pos[2] + target_vel[2] * 0.05,
    ];
    let mut pred = t0;
    let mut flight_t = 0.0;
    for _ in 0..3 {
        let d = dist3d(m0, pred);
        flight_t = ((d - shot_offset).max(0.0)) / proj_speed.max(1.0);
        let lead_t = flight_t.min(0.85);
        pred = [
            t0[0] + target_vel[0] * lead_t,
            t0[1] + target_vel[1] * lead_t,
            t0[2] + target_vel[2] * lead_t * 0.5,
        ];
    }
    let dx = pred[0] - m0[0];
    let dy = pred[1] - m0[1];
    let dz = pred[2] - m0[2];
    let horiz = dx.hypot(dy).max(1e-6);
    ([dy.atan2(dx), dz.atan2(horiz)], pred, flight_t)
}

/// Solves mortar launch pitch accounting for shot-origin offset (radius + 0.5) & aiming at feet+1.0m for splash!
pub fn solve_mortar_arc(
    map: &ArenaMap,
    me: &SelfObs,
    target_feet: [f64; 3],
    target_vel: [f64; 3],
    speed: f64,
    gravity: f64,
    prefer_high: bool,
) -> Option<([f64; 2], f64)> {
    let tube_z = me.pos[2] + me.mortar_height();
    let tube = [me.pos[0], me.pos[1], tube_z];
    let min_p = me.min_pitch();
    let max_p = me.max_pitch();
    let off = me.radius() + 0.5;
    let aim_target_base = [target_feet[0], target_feet[1], target_feet[2] + 1.0];

    let arcs = if prefer_high { [true, false] } else { [false, true] };
    for high_arc in arcs {
        let mut pred = aim_target_base;
        let mut launch_o = tube;
        let mut solved: Option<(f64, f64, f64)> = None;

        for _ in 0..4 {
            let dx = pred[0] - launch_o[0];
            let dy = pred[1] - launch_o[1];
            let r = dx.hypot(dy);
            if r < 11.0 {
                solved = None;
                break;
            }
            let dz = pred[2] - launch_o[2];
            let v2 = speed * speed;
            let disc = v2 * v2 - gravity * (gravity * r * r + 2.0 * dz * v2);
            if disc < 0.0 {
                solved = None;
                break;
            }
            let s = disc.sqrt();
            let pitch = if high_arc {
                ((v2 + s) / (gravity * r)).atan()
            } else {
                ((v2 - s) / (gravity * r)).atan()
            };
            if pitch < min_p || pitch > max_p {
                solved = None;
                break;
            }
            let vh = speed * pitch.cos();
            if vh < 1.0 {
                solved = None;
                break;
            }
            let t_flight = r / vh;
            if t_flight > 6.6 {
                solved = None;
                break;
            }
            let yaw = dy.atan2(dx);
            let dir = aim_dir(yaw, pitch);
            launch_o = [
                tube[0] + dir[0] * off,
                tube[1] + dir[1] * off,
                tube[2] + dir[2] * off,
            ];
            let lead_t = t_flight.min(2.0);
            pred = [
                aim_target_base[0] + target_vel[0] * lead_t * 0.72,
                aim_target_base[1] + target_vel[1] * lead_t * 0.72,
                aim_target_base[2],
            ];
            solved = Some((yaw, pitch, t_flight));
        }

        if let Some((yaw, pitch, t_flight)) = solved {
            let dir = aim_dir(yaw, pitch);
            let launch_origin = [
                tube[0] + dir[0] * off,
                tube[1] + dir[1] * off,
                tube[2] + dir[2] * off,
            ];
            if !map.has_shot_los(tube, launch_origin) {
                continue;
            }
            let vel = [dir[0] * speed, dir[1] * speed, dir[2] * speed];
            if map.mortar_arc_clear(launch_origin, vel, gravity, t_flight) {
                return Some(([yaw, pitch], t_flight));
            }
        }
    }
    None
}

/// Compute active evasion vector from incoming projectiles, mortar shells, and charging enemy lasers!
/// Uses consistent lateral dodge direction & ground boost (NEVER jumps into hitscan lasers!).
pub fn compute_dodge(obs: &Observation, state: &WorldState, map: &ArenaMap) -> DodgeOutput {
    let me = &obs.me;
    let my_r = me.radius();
    let my_c = me.center_pos();
    let base_muzzle = state.config.mech_f("muzzle_height", 7.0);
    let spc = state.config.cubes.size_per_cube;
    let hp_frac = me.hp / me.max_hp.max(1.0);

    let mut dx = 0.0;
    let mut dy = 0.0;
    let mut urgency = 0.0f64;
    let mut want_boost = false;

    let mortar_g = state.config.weapon_f("mortar", "gravity", 22.0);
    let mortar_r = state.config.weapon_f("mortar", "splash_radius", 7.0) + 3.5;

    // 1. Incoming projectiles
    for p in &obs.projectiles {
        if p.kind == "mortar" {
            let mut best_d = f64::INFINITY;
            let mut imp = p.pos;
            let mut imp_t = 0.0;
            for step in 1..=25 {
                let st = step as f64 * 0.1;
                let px = p.pos[0] + p.vel[0] * st;
                let py = p.pos[1] + p.vel[1] * st;
                let pz = p.pos[2] + p.vel[2] * st - 0.5 * mortar_g * st * st;
                let gz = map.terrain_z(px, py);
                let d_xy = (px - me.pos[0]).hypot(py - me.pos[1]);
                if pz <= me.pos[2] + me.height() && pz >= me.pos[2] - 4.0 && d_xy < best_d {
                    best_d = d_xy;
                    imp = [px, py, pz];
                    imp_t = st;
                }
                if pz <= gz {
                    if d_xy < best_d {
                        best_d = d_xy;
                        imp = [px, py, gz];
                        imp_t = st;
                    }
                    break;
                }
            }
            if best_d < mortar_r + my_r && imp_t < 2.2 {
                let ex = me.pos[0] - imp[0];
                let ey = me.pos[1] - imp[1];
                let elen = ex.hypot(ey);
                let (ux, uy) = if elen > 0.1 { (ex / elen, ey / elen) } else { (state.strafe_sign, 0.0) };
                let w = ((mortar_r + my_r - best_d) / mortar_r).clamp(0.4, 1.6) * 3.2;
                dx += ux * w;
                dy += uy * w;
                urgency = urgency.max(2.8);
                if imp_t < 1.1 && me.energy > 10.0 {
                    want_boost = true;
                }
            }
        } else {
            let speed = (p.vel[0] * p.vel[0] + p.vel[1] * p.vel[1] + p.vel[2] * p.vel[2]).sqrt();
            if speed < 5.0 { continue; }
            let rx = my_c[0] - p.pos[0];
            let ry = my_c[1] - p.pos[1];
            let rz = my_c[2] - p.pos[2];
            let t_ca = (rx * p.vel[0] + ry * p.vel[1] + rz * p.vel[2]) / (speed * speed);
            if t_ca < 0.0 || t_ca > 0.60 { continue; }
            let cx = p.pos[0] + p.vel[0] * t_ca;
            let cy = p.pos[1] + p.vel[1] * t_ca;
            let cz = p.pos[2] + p.vel[2] * t_ca;
            let miss_xy = (my_c[0] - cx).hypot(my_c[1] - cy);
            let miss_z = (my_c[2] - cz).abs();
            if miss_xy < my_r + 2.4 && miss_z < me.height() * 0.65 {
                let vh = p.vel[0].hypot(p.vel[1]).max(1e-6);
                let px_dir = -p.vel[1] / vh;
                let py_dir = p.vel[0] / vh;
                // Consistent lateral dodge (avoid per-bullet stuttering!)
                let side_dot = (my_c[0] - cx) * px_dir + (my_c[1] - cy) * py_dir;
                let sign = if side_dot.abs() > my_r * 0.65 { side_dot.signum() } else { state.strafe_sign };
                let w = (1.1 - t_ca).clamp(0.3, 1.1);
                dx += px_dir * sign * w;
                dy += py_dir * sign * w;
                urgency = urgency.max(0.95);
                if hp_frac < 0.40 && t_ca < 0.30 && miss_xy < my_r + 0.8 && me.energy > 25.0 {
                    want_boost = true;
                }
            }
        }
    }

    // 2. Charging Enemy Lasers (start dodging at 0.22s to beat 2x laser_charge 0.42s upgrades! NEVER jump!)
    for e in state.enemies.values() {
        if !e.visible_now || !e.charging { continue; }
        let em = e.muzzle(base_muzzle, spc);
        let d = dist3d(em, my_c);
        if d > 285.0 { continue; }
        let edir = aim_dir(e.aim[0], e.aim[1]);
        let vx = my_c[0] - em[0];
        let vy = my_c[1] - em[1];
        let vz = my_c[2] - em[2];
        let proj = vx * edir[0] + vy * edir[1] + vz * edir[2];
        if proj <= 0.0 || proj > d + 10.0 { continue; }
        let px = em[0] + edir[0] * proj;
        let py = em[1] + edir[1] * proj;
        let pz = em[2] + edir[2] * proj;
        let miss = dist3d(my_c, [px, py, pz]);
        let chg_t = e.charge_elapsed(obs.t);
        if miss < my_r + 6.5 && map.has_shot_los(em, my_c) {
            let perp_x = -edir[1];
            let perp_y = edir[0];
            let plen = perp_x.hypot(perp_y).max(1e-6);
            let ux = perp_x / plen;
            let uy = perp_y / plen;
            let side = (my_c[0] - px) * ux + (my_c[1] - py) * uy;
            let sign = if side.abs() > 1.0 { side.signum() } else { state.strafe_sign };
            let w = if chg_t >= 0.28 { 3.5 } else { 1.6 };
            dx += ux * sign * w;
            dy += uy * sign * w;
            urgency = urgency.max(if chg_t >= 0.28 { 3.0 } else { 1.5 });
            if chg_t >= 0.30 && miss < my_r + 5.0 && me.energy > 10.0 {
                want_boost = true;
            }
        }
    }

    let len = dx.hypot(dy);
    let vec = if len > 1e-6 { [dx / len, dy / len] } else { [0.0, 0.0] };
    DodgeOutput { vec, urgency, want_boost, want_jump: false }
}

/// Decides target, aim, firing triggers [slot0, slot1], and reload with zero weapon aim conflicts!
pub fn decide_combat(
    obs: &Observation,
    state: &mut WorldState,
    map: &ArenaMap,
    hold_fire_for_regen: bool,
    farm_crate_id: Option<i64>,
    farm_npc_id: Option<i64>,
) -> CombatDecision {
    let me = &obs.me;
    let my_muzzle = me.muzzle_pos();
    let my_r = me.radius();
    let base_r = state.config.mech_f("radius", 2.5);
    let base_h = state.config.mech_f("height", 9.0);
    let spc = state.config.cubes.size_per_cube;

    let gun_slot = find_weapon_slot(me, "gun");
    let laser_slot = find_weapon_slot(me, "laser");
    let mortar_slot = find_weapon_slot(me, "mortar");
    let light_slot = find_weapon_slot(me, "lightning");

    let have_laser = laser_slot.and_then(|s| me.weapons.get(s)).and_then(|w| w.as_ref());
    let have_gun = gun_slot.and_then(|s| me.weapons.get(s)).and_then(|w| w.as_ref());
    let have_mortar = mortar_slot.and_then(|s| me.weapons.get(s)).and_then(|w| w.as_ref());
    let have_light = light_slot.and_then(|s| me.weapons.get(s)).and_then(|w| w.as_ref());

    let laser_charge = have_laser.and_then(|w| w.charge).unwrap_or(0.0);
    let laser_elapsed = state
        .my_laser_charge_start_t
        .map(|s| (obs.t - s).max(laser_charge))
        .unwrap_or(laser_charge);
    let laser_full = have_laser.map(|w| w.stat("charge", 0.75)).unwrap_or(0.75);
    let laser_min = have_laser.map(|w| w.stat("min_charge", 0.15)).unwrap_or(0.15);
    let laser_hold = have_laser.map(|w| w.stat("hold_s", 2.0)).unwrap_or(2.0);
    let laser_cd = have_laser.map(|w| w.cooldown).unwrap_or(99.0);
    let laser_range = have_laser.map(|w| w.stat("range", 280.0)).unwrap_or(280.0);
    let laser_dmg_base = have_laser.map(|w| w.stat("damage", 315.0)).unwrap_or(315.0);
    let laser_min_frac = have_laser.map(|w| w.stat("min_frac", 0.35)).unwrap_or(0.35);
    let laser_cookoff = have_laser.map(|w| w.stat("cookoff", 0.6)).unwrap_or(0.6);

    let light_range = have_light.map(|w| w.stat("range", 45.0)).unwrap_or(45.0);
    let light_cone = have_light.map(|w| w.stat("cone_deg", 30.0)).unwrap_or(30.0).to_radians();
    let light_energy = have_light.map(|w| w.stat("energy", 3.0)).unwrap_or(3.0);

    let gun_speed = have_gun.map(|w| w.stat("speed", 252.0)).unwrap_or(252.0);
    let gun_range = have_gun.map(|w| w.stat("range", 230.0)).unwrap_or(230.0);
    let gun_ready = have_gun
        .map(|g| g.ammo.unwrap_or(0.0) > 0.0 && g.reload.unwrap_or(0.0) <= 0.0)
        .unwrap_or(false);

    let mortar_speed = have_mortar.map(|w| w.stat("speed", 72.0)).unwrap_or(72.0);
    let mortar_grav = have_mortar.map(|w| w.stat("gravity", 22.0)).unwrap_or(22.0);
    let mortar_cd = have_mortar.map(|w| w.cooldown).unwrap_or(99.0);

    let dmg_mult = cube_damage_mult(me.cubes, &state.config);

    // 1. Score all tracked enemies with multi-height LOS!
    let mut best_enemy: Option<(&EnemyTrack, f64, Option<[f64; 3]>)> = None;
    let mut any_lightning_threat_close = false;
    let mut any_enemy_within_70 = false;
    let mut any_enemy_has_los = false;

    for e in state.enemies.values() {
        let age = obs.t - e.last_seen_t;
        if age > 5.0 { continue; }
        let eh = e.height(base_h, spc);
        let ec = e.center(base_h, spc);
        let d = dist3d(my_muzzle, ec);
        if d < 70.0 { any_enemy_within_70 = true; }
        if e.visible_now && e.has_weapon("lightning") && d < 50.0 {
            any_lightning_threat_close = true;
        }
        if d > 285.0 { continue; }

        let los_pt = if e.visible_now {
            best_los_point(map, my_muzzle, e.pos, eh)
        } else {
            None
        };
        if los_pt.is_some() { any_enemy_has_los = true; }

        // In true FFA early/mid game, avoid poking full-HP far enemies who aren't fighting us
        if state.is_ffa() && obs.alive > 4 && !e.charging && e.last_hit_us_t < obs.t - 4.0 && d > 115.0 && e.hp > e.max_hp * 0.72 {
            continue;
        }

        let mut score = 0.0;
        if los_pt.is_some() { score += 130.0; } else if e.visible_now { score += 40.0; } else { score += 12.0 - age * 2.5; }
        score += (260.0 - d).max(0.0) * 0.55;
        let hp_frac = (e.hp / e.max_hp.max(1.0)).clamp(0.0, 1.0);
        score += (1.0 - hp_frac) * 165.0;
        if e.hp <= 340.0 { score += 90.0; }
        if e.last_hit_us_t > obs.t - 3.0 { score += 65.0; }
        if e.charging { score += 60.0; }
        if have_laser.is_some() && e.has_weapon("mortar") { score += 55.0; }
        if have_light.is_some() && e.has_weapon("laser") && d < light_range + 8.0 { score += 80.0; }
        if have_mortar.is_some() && e.has_weapon("lightning") && d < 90.0 { score += 45.0; }

        if best_enemy.map(|(_, bs, _)| score > bs).unwrap_or(true) {
            best_enemy = Some((e, score, los_pt));
        }
    }

    if any_enemy_has_los {
        state.last_enemy_los_t = obs.t;
    }

    let mut fire = [false, false];
    let mut reload = false;
    let mut cmd_aim: Option<[f64; 2]> = None;
    let mut target_enemy_id = None;
    let mut target_has_los = false;
    let mut want_peek = false;

    // Safe proactive gun reload ONLY when out of LOS for >= 1.0s (no 1-tick flicker reloads!)
    if let Some(g) = have_gun {
        let ammo = g.ammo.unwrap_or(12.0);
        let rel = g.reload.unwrap_or(0.0);
        if rel <= 0.0 && ammo <= 3.0 && !any_enemy_has_los && (obs.t - state.last_enemy_los_t) >= 1.0 {
            reload = true;
        }
    }

    // Check if an unseen mech shot us recently (strict rear-armor override over crates/NPCs!)
    let under_unseen_fire = (obs.t - state.last_damaged_t) < 2.0
        && state.last_damaged_bearing.is_some()
        && state
            .last_damaged_by
            .map(|id| id < 100 && !state.enemies.get(&id).map(|e| e.visible_now).unwrap_or(false))
            .unwrap_or(false);

    // 2. Combat against selected enemy mech
    if let Some((e, _, los_pt)) = best_enemy {
        target_enemy_id = Some(e.id);
        let has_los = los_pt.is_some();
        target_has_los = has_los;
        let aim_pt = los_pt.unwrap_or_else(|| e.center(base_h, spc));
        let er = e.radius(base_r, spc);
        let eh = e.height(base_h, spc);
        let d = dist3d(my_muzzle, aim_pt);

        let allow_shoot = !hold_fire_for_regen || has_los || e.hp <= 360.0 || d < 48.0;

        // Direct 1-tick hitscan aim (accounting for BOTH our velocity and enemy velocity!)
        let my_next_muz = [
            my_muzzle[0] + me.vel[0] * 0.05,
            my_muzzle[1] + me.vel[1] * 0.05,
            my_muzzle[2] + me.vel[2] * 0.05,
        ];
        let lead_hitscan = [
            aim_pt[0] + e.vel[0] * 0.05,
            aim_pt[1] + e.vel[1] * 0.05,
            aim_pt[2] + e.vel[2] * 0.05,
        ];
        let l_dx = lead_hitscan[0] - my_next_muz[0];
        let l_dy = lead_hitscan[1] - my_next_muz[1];
        let l_dz = lead_hitscan[2] - my_next_muz[2];
        let hitscan_aim = [l_dy.atan2(l_dx), l_dz.atan2(l_dx.hypot(l_dy).max(1e-6))];

        // Gun lead aim (accounting for 1-tick latency + flight time)
        let (gun_aim, gun_pred, _) = lead_aim_straight(my_muzzle, me.vel, aim_pt, e.vel, gun_speed, my_r + 0.5);
        let gun_pred_los = has_los && map.has_shot_los(my_next_muz, gun_pred);
        let desired_gun_aim = if gun_pred_los { gun_aim } else { hitscan_aim };

        // Step A: Decide PRIMARY weapon that owns `cmd_aim`
        let mut mortar_solution: Option<[f64; 2]> = None;
        if have_mortar.is_some() && allow_shoot && mortar_cd <= 0.05 && d >= 14.0 && d <= 230.0 {
            let prefer_high = !has_los;
            if let Some((m_aim, _ft)) = solve_mortar_arc(map, me, e.pos, e.vel, mortar_speed, mortar_grav, prefer_high) {
                mortar_solution = Some(m_aim);
            }
        }

        // Don't start laser charge if: no LOS (fixes memory_s wall-charging!), shorted, lightning close, OR close-range <44m while holding gun/lightning!
        let close_range_speed_trap = d < 44.0 && (gun_ready || (have_light.is_some() && me.energy >= light_energy)) && e.hp > 420.0;
        let laser_can_start = laser_slot.is_some()
            && allow_shoot
            && has_los
            && laser_cd <= 0.05
            && !state.is_laser_shorted(obs.t)
            && d <= laser_range
            && !any_lightning_threat_close
            && !close_range_speed_trap;

        let mortar_owns_aim = mortar_solution.is_some()
            && laser_charge <= 0.0
            && (!has_los
                || (e.has_weapon("lightning") && d < 78.0)
                || (have_laser.is_none() || laser_cd > 0.35));

        if laser_charge > 0.0 {
            cmd_aim = Some(hitscan_aim);
        } else if mortar_owns_aim {
            cmd_aim = mortar_solution;
        } else if laser_can_start {
            cmd_aim = Some(hitscan_aim);
        } else if gun_ready && has_los && d <= gun_range {
            cmd_aim = Some(desired_gun_aim);
        } else {
            cmd_aim = Some(hitscan_aim);
        }

        // Compute the EXACT slewed torso aim we will actually have on tick 1!
        let chosen_aim = cmd_aim.unwrap_or(hitscan_aim);
        let next_actual_aim = slew_one_tick(
            me.aim, chosen_aim, me.turn_rate(), me.pitch_rate(), me.min_pitch(), me.max_pitch(),
        );

        // Verify actual firing ray clears immediate walls (no barrel poke into cover!)
        let shot_dir = aim_dir(next_actual_aim[0], next_actual_aim[1]);
        let shot_o = [
            my_next_muz[0] + shot_dir[0] * (my_r + 0.5),
            my_next_muz[1] + shot_dir[1] * (my_r + 0.5),
            my_next_muz[2] + shot_dir[2] * (my_r + 0.5),
        ];
        let barrel_clear = map.has_shot_los(my_next_muz, shot_o);

        // Step B: Trigger each equipped weapon ONLY if `next_actual_aim` satisfies THAT weapon's alignment!
        // B1) LASER
        if let Some(ls) = laser_slot {
            if laser_charge > 0.0 {
                let aligned = aim_hits_cylinder(next_actual_aim, hitscan_aim, d, er * 0.88, eh * 0.85);
                let frac = (laser_charge / laser_full.max(0.1)).clamp(0.0, 1.0);
                let cook_mult = if e.has_weapon("mortar") { 1.0 + laser_cookoff } else { 1.0 };
                let est_dmg = laser_dmg_base * (laser_min_frac + (1.0 - laser_min_frac) * frac) * dmg_mult * cook_mult;
                let armor_margin = if e.cubes > 0 { 1.24 } else { 1.04 };

                let full_ready = laser_charge >= laser_full - 0.06;
                let early_kill = laser_charge >= laser_min && est_dmg >= e.hp * armor_margin;
                let about_to_autofire = laser_elapsed >= laser_full + laser_hold - 0.15;

                if laser_charge >= laser_full - 0.15 {
                    want_peek = true;
                }

                if has_los && barrel_clear && aligned && (full_ready || early_kill || about_to_autofire) {
                    fire[ls] = false; // RELEASE -> HITSCAN BEAM FIRES!
                } else {
                    fire[ls] = true; // KEEP HOLDING CHARGE!
                }
            } else if laser_can_start {
                fire[ls] = true; // START CHARGING!
            }
        }

        // B2) MORTAR (tight 1.5-deg yaw / 1-deg pitch gate using actual slewed aim!)
        if let Some(ms) = mortar_slot {
            if mortar_owns_aim {
                if let Some(m_aim) = mortar_solution {
                    let dy = angle_diff(m_aim[0], next_actual_aim[0]).abs();
                    let dp = (m_aim[1] - next_actual_aim[1]).abs();
                    if dy < 0.028 && dp < 0.018 {
                        fire[ms] = true;
                    }
                }
            }
        }

        // B3) LIGHTNING (fire down to 3 energy! Instant 240 DPS + shorts enemy laser!)
        if let Some(ts) = light_slot {
            if allow_shoot && has_los && d <= light_range - 0.5 && !mortar_owns_aim && me.energy >= light_energy {
                let yaw_err = angle_diff(hitscan_aim[0], next_actual_aim[0]).abs();
                if yaw_err <= light_cone * 0.45 {
                    fire[ts] = true;
                }
            }
        }

        // B4) GUN (check tick-1 OR tick-2 slewed aim since each action spans 2 sim ticks; add 0.3m bullet radius!)
        if let Some(gs) = gun_slot {
            if allow_shoot && gun_ready && has_los && barrel_clear && d <= gun_range && !mortar_owns_aim {
                let tick2_aim = slew_one_tick(
                    next_actual_aim, chosen_aim, me.turn_rate(), me.pitch_rate(), me.min_pitch(), me.max_pitch(),
                );
                let hit1 = aim_hits_cylinder(next_actual_aim, desired_gun_aim, d, (er + 0.4) * 1.15, eh * 1.0);
                let hit2 = aim_hits_cylinder(tick2_aim, desired_gun_aim, d, (er + 0.4) * 1.15, eh * 1.0);
                let hit_dir = aim_hits_cylinder(next_actual_aim, hitscan_aim, d, (er + 0.4) * 1.15, eh * 1.0);
                if hit1 || hit2 || (d < 85.0 && hit_dir) {
                    fire[gs] = true;
                }
            }
        }
    }

    // 3. Unseen rear-attacker snap (overrides farming immediately!)
    if under_unseen_fire && !target_has_los {
        if let Some(b) = state.last_damaged_bearing {
            cmd_aim = Some([b, 0.0]);
        }
    }

    // 4a. ALWAYS defend against an aggroed NPC shooting us (even if low HP or an enemy is nearby!)
    let npc_aggro_active = farm_npc_id
        .and_then(|nid| state.npcs.get(&nid))
        .map(|n| dist2d([me.pos[0], me.pos[1]], [n.pos[0], n.pos[1]]) < 48.0 || state.last_damaged_by == Some(n.id))
        .unwrap_or(false);

    // 4b. Crate / NPC farming when safe (OR self-defense against aggroed NPC when no mech has LOS!)
    let in_enemy_fight = (target_enemy_id.is_some() && (target_has_los || fire[0] || fire[1] || laser_charge > 0.0))
        || under_unseen_fire
        || (any_enemy_within_70 && !npc_aggro_active);

    if !in_enemy_fight && (!hold_fire_for_regen || npc_aggro_active) {
        // Farm NPC drone (2 cubes!) or aggroed NPC
        if let Some(nid) = farm_npc_id {
            if let Some(npc) = state.npcs.get(&nid) {
                let nc = [npc.pos[0], npc.pos[1], npc.pos[2] + 1.75];
                let d = dist3d(my_muzzle, nc);
                if map.has_shot_los(my_muzzle, nc) && d < 220.0 {
                    let (naim, _, _) = lead_aim_straight(my_muzzle, me.vel, nc, npc.vel, gun_speed, my_r + 0.5);
                    cmd_aim = Some(naim);
                    let nxt = slew_one_tick(me.aim, naim, me.turn_rate(), me.pitch_rate(), me.min_pitch(), me.max_pitch());
                    if let Some(gs) = gun_slot.filter(|_| gun_ready && d < gun_range) {
                        if aim_hits_cylinder(nxt, naim, d, 1.8, 3.5) { fire[gs] = true; }
                    } else if let Some(ts) = light_slot.filter(|_| d < light_range - 1.0 && me.energy > 30.0) {
                        if angle_diff(naim[0], nxt[0]).abs() < light_cone * 0.45 { fire[ts] = true; }
                    } else if let Some(ls) = laser_slot {
                        if laser_charge > 0.0 {
                            let aligned = aim_hits_cylinder(nxt, naim, d, 1.8, 3.5);
                            fire[ls] = !(aligned && laser_charge >= laser_full - 0.06);
                        } else if laser_cd <= 0.05 && d < laser_range {
                            fire[ls] = true;
                        }
                    }
                }
            }
        }
        // Farm crate (1 cube + 50% weapon drop!)
        if cmd_aim.is_none() {
            if let Some(cid) = farm_crate_id {
                if let Some(cr) = state.crates.get(&cid) {
                    let cc = [cr.pos[0], cr.pos[1], cr.pos[2] + 1.2];
                    let d = dist3d(my_muzzle, cc);
                    if map.has_shot_los(my_muzzle, cc) && d < 140.0 {
                        let c_dx = cc[0] - my_muzzle[0];
                        let c_dy = cc[1] - my_muzzle[1];
                        let c_dz = cc[2] - my_muzzle[2];
                        let caim = [c_dy.atan2(c_dx), c_dz.atan2(c_dx.hypot(c_dy).max(1e-6))];
                        cmd_aim = Some(caim);
                        let nxt = slew_one_tick(me.aim, caim, me.turn_rate(), me.pitch_rate(), me.min_pitch(), me.max_pitch());
                        if let Some(gs) = gun_slot.filter(|_| gun_ready && d < gun_range) {
                            if aim_hits_cylinder(nxt, caim, d, 1.4, 2.4) { fire[gs] = true; }
                        } else if let Some(ts) = light_slot.filter(|_| d < light_range - 1.0 && me.energy > 35.0) {
                            if angle_diff(caim[0], nxt[0]).abs() < light_cone * 0.45 { fire[ts] = true; }
                        } else if let Some(ls) = laser_slot {
                            if laser_charge > 0.0 {
                                let aligned = aim_hits_cylinder(nxt, caim, d, 1.4, 2.4);
                                fire[ls] = !(aligned && laser_charge >= laser_full - 0.06);
                            } else if laser_cd <= 0.05 {
                                fire[ls] = true;
                            }
                        }
                    }
                }
            }
        }
    }

    // 5. Fallback rear-armor / sound check when idle
    if cmd_aim.is_none() {
        if obs.t - state.last_damaged_t < 2.5 {
            if let Some(b) = state.last_damaged_bearing {
                cmd_aim = Some([b, 0.0]);
            }
        } else if let Some(s) = state.sounds.iter().rev().find(|s| obs.t - s.t < 1.5 && s.dist_est < 120.0) {
            cmd_aim = Some([s.bearing, 0.0]);
        }
    }

    CombatDecision {
        aim: cmd_aim,
        fire,
        reload,
        target_enemy_id,
        target_has_los,
        charging_laser: laser_charge > 0.0 || laser_slot.map(|ls| fire[ls]).unwrap_or(false),
        laser_ready_soon: laser_slot.is_some() && (laser_charge >= laser_full - 0.15 || laser_cd <= 0.25),
        want_peek,
    }
}
