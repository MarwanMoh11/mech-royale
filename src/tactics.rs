use crate::combat::{best_los_point, compute_dodge, decide_combat, find_weapon_slot};
use crate::map::ArenaMap;
use crate::protocol::*;
use crate::world_state::WorldState;

/// Score upgrade choices dynamically based on loadout, cubes, HP, and Practice vs FFA mode.
pub fn pick_upgrade(offer: &Offer, obs: &Observation, state: &WorldState) -> usize {
    let me = &obs.me;
    let have_laser = find_weapon_slot(me, "laser").is_some();
    let have_gun = find_weapon_slot(me, "gun").is_some();
    let have_mortar = find_weapon_slot(me, "mortar").is_some();
    let have_light = find_weapon_slot(me, "lightning").is_some();
    let is_ffa = state.is_ffa();

    let mut best_idx = 0usize;
    let mut best_score = f64::NEG_INFINITY;

    for (i, ch) in offer.choices.iter().enumerate() {
        let mut s = match ch.id.as_str() {
            "nanites" => 116.0,
            "armor" => 109.0,
            "plating" => if is_ffa { 113.0 } else { 102.0 },
            "laser_damage" => if have_laser { 105.0 } else { 40.0 },
            "laser_charge" => if have_laser { 103.0 } else { 40.0 },
            "servos" => 97.0,
            "jets" => 88.0,
            "reactor" => 78.0,
            "capacitor" => 72.0,
            "gun_rate" => if have_gun { 89.0 } else { 25.0 },
            "gun_damage" => if have_gun { 88.0 } else { 25.0 },
            "gun_velocity" => if have_gun { 70.0 } else { 20.0 },
            "gun_mag" => if have_gun { 68.0 } else { 20.0 },
            "lightning_efficiency" => if have_light { 86.0 } else { 30.0 },
            "lightning_range" => if have_light { 85.0 } else { 30.0 },
            "lightning_chain" => if have_light { if is_ffa { 55.0 } else { 82.0 } } else { 20.0 },
            "mortar_reload" => if have_mortar { 83.0 } else { 30.0 },
            "mortar_splash" => if have_mortar { 78.0 } else { 30.0 },
            "mortar_velocity" => if have_mortar { 74.0 } else { 30.0 },
            "sensors" => if is_ffa { 72.0 } else { 62.0 },
            "gyros" => 66.0,
            "laser_pierce" => if have_laser && !is_ffa { 62.0 } else { 45.0 },
            _ => 50.0,
        };
        if ch.id == "nanites" && me.hp < me.max_hp * 0.75 {
            s += 10.0;
        }
        if s > best_score {
            best_score = s;
            best_idx = i;
        }
    }
    best_idx
}

fn weapon_rank(kind: &str, is_ffa: bool) -> f64 {
    match kind {
        "laser" => 100.0,
        "lightning" => if is_ffa { 76.0 } else { 84.0 },
        "mortar" => if is_ffa { 74.0 } else { 68.0 },
        "gun" => 65.0,
        _ => 10.0,
    }
}

/// Decides if we should grab/swap a nearby weapon pickup right now.
pub fn decide_weapon_pickup(obs: &Observation, state: &WorldState) -> (Option<i64>, Option<usize>, Option<[f64; 3]>) {
    let me = &obs.me;
    let reach = state.config.loot.pickup_radius;
    let is_ffa = state.is_ffa();

    let s0 = me.weapons.first().and_then(|w| w.as_ref());
    let s1 = me.weapons.get(1).and_then(|w| w.as_ref());
    let already_has = |k: &str| {
        s0.map(|w| w.kind == k).unwrap_or(false) || s1.map(|w| w.kind == k).unwrap_or(false)
    };
    let laser_charging_slot = |slot: usize| -> bool {
        me.weapons
            .get(slot)
            .and_then(|w| w.as_ref())
            .map(|w| w.kind == "laser" && w.charge.unwrap_or(0.0) > 0.0)
            .unwrap_or(false)
    };

    let evaluate_kind = |kind: &str| -> Option<(f64, Option<usize>)> {
        if already_has(kind) {
            return None;
        }
        if s0.is_none() || s1.is_none() {
            if kind == "gun" && (s0.is_some() || s1.is_some()) {
                return None;
            }
            return Some((weapon_rank(kind, is_ffa) + 50.0, None));
        }
        let w0 = s0.unwrap();
        let w1 = s1.unwrap();

        // Always upgrade to Laser if we don't hold it yet (keep Gun or Lightning alongside Laser!)
        if kind == "laser" {
            let drop_slot = if w1.kind == "mortar" { 1 }
                else if w0.kind == "mortar" { 0 }
                else if w1.kind != "gun" { 1 }
                else { 0 };
            return Some((140.0, Some(drop_slot)));
        }
        // If holding Gun + Mortar and find Lightning, upgrade Mortar -> Lightning (avoids +60% enemy laser cook-off!)
        if kind == "lightning" && already_has("mortar") {
            let m_slot = if w0.kind == "mortar" { 0 } else { 1 };
            return Some((92.0, Some(m_slot)));
        }
        // If holding Laser + Gun and find Lightning with >= 1 energy upgrade -> Laser + Lightning god loadout!
        if kind == "lightning" && already_has("laser") && already_has("gun") {
            let energy_ups = me.upgrades.iter().filter(|u| u.as_str() == "jets" || u.as_str() == "reactor" || u.as_str() == "capacitor").count();
            if energy_ups >= 1 {
                let gun_slot = if w0.kind == "gun" { 0 } else { 1 };
                return Some((86.0, Some(gun_slot)));
            }
        }
        None
    };

    // 1. Immediate pickup within reach (4m horiz & 4m vert)
    let mut best_take: Option<(f64, i64, Option<usize>)> = None;
    for p in &obs.pickups {
        let dh = dist2d([me.pos[0], me.pos[1]], [p.pos[0], p.pos[1]]);
        let dz = (me.pos[2] - p.pos[2]).abs();
        if dh < reach - 0.15 && dz < 3.85 {
            if let Some((val, drop_slot)) = evaluate_kind(&p.kind) {
                if let Some(ds) = drop_slot {
                    if laser_charging_slot(ds) { continue; }
                }
                if best_take.map(|(bv, _, _)| val > bv).unwrap_or(true) {
                    best_take = Some((val, p.id, drop_slot));
                }
            }
        }
    }
    if let Some((_, pid, ds)) = best_take {
        return (Some(pid), ds, None);
    }

    // 2. Best desired pickup to walk toward
    let mut best_goal: Option<(f64, [f64; 3])> = None;
    for p in state.pickups.values() {
        if let Some((val, _)) = evaluate_kind(&p.kind) {
            let d = dist2d([me.pos[0], me.pos[1]], [p.pos[0], p.pos[1]]);
            if d > 95.0 { continue; }
            if dist2d([p.pos[0], p.pos[1]], obs.zone.center) > obs.zone.radius - 5.0 { continue; }
            let dz = p.pos[2] - me.pos[2];
            if dz > 28.0 && me.energy < 55.0 { continue; }
            let sc = val - d * 1.1 - dz.max(0.0) * 1.5;
            if best_goal.map(|(bs, _)| sc > bs).unwrap_or(true) {
                best_goal = Some((sc, p.pos));
            }
        }
    }

    (None, None, best_goal.map(|(_, pos)| pos))
}

/// Computes future zone circle at time `t_eval` WITHOUT double-shrinking!
/// Remember: `obs.zone.center`/`radius` is ALREADY the live interpolated circle at `obs.t`!
fn zone_at(z: &ZoneObs, now: f64, t_eval: f64) -> ([f64; 2], f64) {
    if t_eval <= z.shrink_start || z.shrink_end <= z.shrink_start {
        (z.center, z.radius)
    } else if t_eval >= z.shrink_end {
        (z.next_center, z.next_radius)
    } else {
        let base_t = now.max(z.shrink_start);
        let f = ((t_eval - base_t) / (z.shrink_end - base_t).max(0.05)).clamp(0.0, 1.0);
        (
            [
                z.center[0] + (z.next_center[0] - z.center[0]) * f,
                z.center[1] + (z.next_center[1] - z.center[1]) * f,
            ],
            z.radius + (z.next_radius - z.radius) * f,
        )
    }
}

struct EnemySummary {
    id: i64,
    pos: [f64; 3],
    muzzle: [f64; 3],
    center: [f64; 3],
    dist: f64,
    has_los: bool,
    hp: f64,
    charging: bool,
    has_lightning: bool,
}

/// Core per-tick decision function combining combat, dodging, 3D cover/rooftops, loot, and zone tactics.
pub fn decide_action(obs: &Observation, state: &mut WorldState, map: &ArenaMap) -> Action {
    let me = &obs.me;
    let t = obs.t;
    let is_ffa = state.is_ffa();
    let base_h = state.config.mech_f("height", 9.0);
    let base_muz = state.config.mech_f("muzzle_height", 7.0);
    let spc = state.config.cubes.size_per_cube;

    let mut act = Action {
        tick: obs.tick,
        ..Default::default()
    };

    // 1. Upgrade offer
    if let Some(offer) = &obs.offer {
        if !offer.choices.is_empty() {
            act.pick = Some(pick_upgrade(offer, obs, state));
        }
    }

    // 2. Weapon pickup / swap
    let (take_id, drop_slot, desired_pickup_pos) = decide_weapon_pickup(obs, state);
    act.take = take_id;
    act.drop = drop_slot;

    // 3. Analyze immediate threats & exposure using muzzle/upper-torso multi-height LOS!
    let my_muzzle = me.muzzle_pos();
    let my_upper = [me.pos[0], me.pos[1], me.pos[2] + me.height() * 0.78];
    let mut visible_enemies: Vec<EnemySummary> = Vec::new();
    let mut enemies_with_los = 0usize;
    let mut nearest_enemy_d = f64::INFINITY;
    let mut any_enemy_charging = false;
    let mut enemy_lightning_within_52 = false;

    for e in state.enemies.values() {
        if t - e.last_seen_t > 5.0 { continue; }
        let eh = e.height(base_h, spc);
        let ec = e.center(base_h, spc);
        let em = e.muzzle(base_muz, spc);
        let d = dist3d(my_muzzle, ec);
        if d < nearest_enemy_d { nearest_enemy_d = d; }
        let los = e.visible_now
            && (best_los_point(map, my_muzzle, e.pos, eh).is_some() || map.has_shot_los(em, my_upper));
        if los { enemies_with_los += 1; }
        if e.visible_now && e.charging { any_enemy_charging = true; }
        let has_lt = e.has_weapon("lightning");
        if has_lt && d < 52.0 { enemy_lightning_within_52 = true; }
        visible_enemies.push(EnemySummary {
            id: e.id, pos: e.pos, muzzle: em, center: ec, dist: d,
            has_los: los, hp: e.hp, charging: e.charging, has_lightning: has_lt,
        });
    }

    // Inject synthetic unseen threat when damaged from outside FOV (non-zone mech!)
    if (t - state.last_damaged_t) < 2.0 {
        if let (Some(by), Some(bear)) = (state.last_damaged_by, state.last_damaged_bearing) {
            if by < 100 && !visible_enemies.iter().any(|e| e.id == by) {
                let gx = me.pos[0] + bear.cos() * 75.0;
                let gy = me.pos[1] + bear.sin() * 75.0;
                let gz = map.terrain_z(gx, gy);
                visible_enemies.push(EnemySummary {
                    id: by, pos: [gx, gy, gz], muzzle: [gx, gy, gz + 7.0], center: [gx, gy, gz + 4.5],
                    dist: 75.0, has_los: true, hp: 1200.0, charging: false, has_lightning: false,
                });
                enemies_with_los += 1;
                nearest_enemy_d = nearest_enemy_d.min(75.0);
            }
        }
    }

    let hp_frac = me.hp / me.max_hp.max(1.0);
    // FIX P1 #8: NEVER gate hold_fire_for_regen on `time_to_regen < 3.5` (that broke regen on ticks 0..1.5s!)
    let low_hp_thresh = if is_ffa { 0.70 } else { 0.54 };
    let hold_fire_for_regen = me.in_zone && hp_frac < low_hp_thresh && enemies_with_los == 0;

    // 4. Select best farm crate (preferring LOS!) & NPC drone (always handle aggroed NPCs!)
    let mut farm_crate_id = None;
    let mut farm_crate_pos = None;
    let mut farm_npc_id = None;
    let mut farm_npc_pos = None;

    let npc_hit_us = state.last_damaged_by.map(|id| id >= 100 && id < 1000).unwrap_or(false)
        && (t - state.last_damaged_t) < 2.5;
    {
        let mut best_aggro_d = if npc_hit_us { 95.0 } else { 52.0 };
        for n in state.npcs.values() {
            let d = dist2d([me.pos[0], me.pos[1]], [n.pos[0], n.pos[1]]);
            if d < best_aggro_d && map.has_shot_los(my_muzzle, [n.pos[0], n.pos[1], n.pos[2] + 1.75]) {
                best_aggro_d = d;
                farm_npc_id = Some(n.id);
                farm_npc_pos = Some(n.pos);
            }
        }
    }

    if me.in_zone && enemies_with_los == 0 && nearest_enemy_d > 70.0 {
        let crate_max_d = if me.cubes < 6 || me.weapons.get(1).and_then(|w| w.as_ref()).is_none() {
            115.0
        } else {
            75.0
        };
        let mut best_c_score = f64::NEG_INFINITY;
        for c in state.crates.values() {
            let d = dist2d([me.pos[0], me.pos[1]], [c.pos[0], c.pos[1]]);
            if d >= crate_max_d { continue; }
            if dist2d([c.pos[0], c.pos[1]], obs.zone.center) >= obs.zone.radius - 10.0 { continue; }
            let los_bonus = if map.has_shot_los(my_muzzle, [c.pos[0], c.pos[1], c.pos[2] + 1.2]) { 40.0 } else { 0.0 };
            let sc = los_bonus - d;
            if sc > best_c_score {
                best_c_score = sc;
                farm_crate_id = Some(c.id);
                farm_crate_pos = Some(c.pos);
            }
        }
        if farm_npc_id.is_none() && hp_frac > 0.50 {
            let mut best_nd = if me.cubes < 10 { 115.0 } else { 85.0 };
            for n in state.npcs.values() {
                let d = dist2d([me.pos[0], me.pos[1]], [n.pos[0], n.pos[1]]);
                if d < best_nd && dist2d([n.pos[0], n.pos[1]], obs.zone.center) < obs.zone.radius - 10.0 {
                    best_nd = d;
                    farm_npc_id = Some(n.id);
                    farm_npc_pos = Some(n.pos);
                }
            }
        }
    }

    // 5. Combat decision
    let combat = decide_combat(obs, state, map, hold_fire_for_regen, farm_crate_id, farm_npc_id);
    act.fire = Some(combat.fire);
    if combat.reload {
        act.reload = Some(true);
    }

    // 6. Active Dodge
    let dodge = compute_dodge(obs, state, map);

    // 7. Zone urgency calculation (capped safe_margin so Phase 5/6 still uses tactical combat!)
    let dist_to_cur_center = dist2d([me.pos[0], me.pos[1]], obs.zone.center);
    let dist_to_next_center = dist2d([me.pos[0], me.pos[1]], obs.zone.next_center);
    let margin_cur = obs.zone.radius - dist_to_cur_center;
    let margin_next = obs.zone.next_radius - dist_to_next_center;
    let time_to_shrink_end = (obs.zone.shrink_end - t).max(0.0);
    let travel_time_next = (-margin_next).max(0.0) / me.walk_speed().max(5.0);
    let safe_margin = (obs.zone.radius * 0.42).min(if obs.zone.dps >= 100.0 { 18.0 } else { 14.0 });
    let zone_urgent = !me.in_zone
        || (obs.zone.radius > 10.0 && margin_cur < safe_margin)
        || (obs.zone.next_radius > 5.0
            && margin_next < -4.0
            && (t >= obs.zone.shrink_start - 10.0 || travel_time_next + 11.0 >= time_to_shrink_end));

    // Zone boost hysteresis (eliminates energy thrashing at 12.0!)
    if me.energy > 22.0 {
        state.zone_boost_active = true;
    } else if me.energy < 8.0 {
        state.zone_boost_active = false;
    }

    // 8. Macro & Tactical Goal Selection
    let goal_3d: [f64; 3];
    let mut want_boost = dodge.want_boost;
    let mut want_jump = dodge.want_jump;
    let mut stealth_walk = false;

    // Best nearby cube
    let mut best_cube_pos: Option<[f64; 3]> = None;
    {
        let mut best_d = if enemies_with_los == 0 { 115.0 } else if enemies_with_los == 1 && hp_frac > 0.65 { 26.0 } else { 12.0 };
        for cb in state.cubes.values() {
            let d = dist2d([me.pos[0], me.pos[1]], [cb.pos[0], cb.pos[1]]);
            let dz = (cb.pos[2] - me.pos[2]).abs();
            if d < best_d && dz < 16.0 && dist2d([cb.pos[0], cb.pos[1]], obs.zone.center) < obs.zone.radius - 4.0 {
                best_d = d;
                best_cube_pos = Some(cb.pos);
            }
        }
    }

    // Check supply drops (including finishing capture when we're already holding it!)
    let mut best_drop_pos: Option<[f64; 3]> = None;
    let mut finishing_drop_pos: Option<[f64; 3]> = None;
    for dr in &obs.drops {
        let d = dist2d([me.pos[0], me.pos[1]], [dr.pos[0], dr.pos[1]]);
        if dr.holder == Some(me.id) && dr.progress > 1.3 && enemies_with_los <= 1 && hp_frac > 0.30 {
            finishing_drop_pos = Some(dr.pos);
        }
        let eta = d / me.walk_speed().max(8.0);
        if dr.lands_in <= eta + 12.0 && d < 180.0 && enemies_with_los <= 1 {
            if !is_ffa || hp_frac < 0.75 || dr.holder.is_none() || dr.holder == Some(me.id) {
                best_drop_pos = Some(dr.pos);
            }
        }
    }

    // Active committed rooftop climb check
    let mut committed_climb: Option<[f64; 3]> = None;
    if let Some((cg, start_t)) = state.climb_goal {
        let reached = me.pos[2] >= cg[2] - 0.6 && dist2d([me.pos[0], me.pos[1]], [cg[0], cg[1]]) < 24.0;
        let timed_out = t - start_t > 5.5;
        let out_of_fuel = me.energy < 6.0;
        let overhang = map.is_covered_above(me.pos[0], me.pos[1], me.pos[2] + me.height() - 0.5, 3.5);
        let too_hot = enemies_with_los >= 2 && hp_frac < 0.45;
        if reached || timed_out || out_of_fuel || overhang || zone_urgent || too_hot {
            state.climb_goal = None;
            if !reached {
                state.climb_cooldown_until = t + 4.0;
            }
        } else {
            committed_climb = Some(cg);
        }
    }

    let missing_second_weapon = me.weapons.get(1).and_then(|w| w.as_ref()).is_none();
    let safe_for_quick_cube = hp_frac > 0.65 && !any_enemy_charging && !enemy_lightning_within_52;

    if zone_urgent {
        goal_3d = find_safe_zone_target(obs, map);
        if (!me.in_zone || margin_cur < 10.0 || obs.zone.dps >= 100.0) && state.zone_boost_active {
            want_boost = true;
        }
    } else if let Some(dp) = finishing_drop_pos {
        // Don't step out of a supply drop ring at 3/4 seconds!
        goal_3d = dp;
    } else if let Some(cg) = committed_climb {
        goal_3d = cg;
    } else if missing_second_weapon
        && hp_frac > 0.52
        && enemies_with_los <= 1
        && !any_enemy_charging
        && !enemy_lightning_within_52
        && desired_pickup_pos.map(|p| dist2d([me.pos[0], me.pos[1]], [p[0], p[1]]) < 55.0 && (p[2] - me.pos[2]).abs() < 6.0).unwrap_or(false)
    {
        goal_3d = desired_pickup_pos.unwrap();
        if me.energy > 30.0 { want_boost = true; }
    } else if safe_for_quick_cube
        && best_cube_pos.map(|p| dist2d([me.pos[0], me.pos[1]], [p[0], p[1]]) < 22.0 && enemies_with_los <= 1).unwrap_or(false)
    {
        goal_3d = best_cube_pos.unwrap();
        if me.energy > 35.0 { want_boost = true; }
    } else if !visible_enemies.is_empty() && nearest_enemy_d < 195.0 {
        let need_heal = hp_frac < low_hp_thresh || enemies_with_los >= 2;
        let want_cover_only = need_heal
            || (combat.charging_laser && !combat.want_peek && enemies_with_los >= 1)
            || (is_ffa && obs.alive > 4 && hp_frac < 0.85 && visible_enemies.iter().all(|e| e.hp > 700.0));

        goal_3d = pick_tactical_position(
            obs,
            state,
            map,
            &visible_enemies,
            combat.target_enemy_id,
            want_cover_only,
            combat.want_peek,
        );
        state.last_tactical_goal = Some(goal_3d);

        // Boost to cover if exposed to multiple enemies, low HP, or escaping lightning range!
        let escaping_lightning = enemy_lightning_within_52 && find_weapon_slot(me, "lightning").is_none();
        if (enemies_with_los >= 2 || (need_heal && enemies_with_los >= 1) || escaping_lightning) && me.energy > 16.0 {
            want_boost = true;
        }
    } else if let Some(cp) = best_cube_pos {
        goal_3d = cp;
        if dist2d([me.pos[0], me.pos[1]], [cp[0], cp[1]]) > 12.0 && me.energy > 40.0 {
            want_boost = true;
        }
    } else if let Some(pp) = desired_pickup_pos {
        goal_3d = pp;
        if me.energy > 40.0 { want_boost = true; }
    } else if let Some(dp) = best_drop_pos {
        goal_3d = dp;
        if dist2d([me.pos[0], me.pos[1]], [dp[0], dp[1]]) > 22.0 && me.energy > 45.0 {
            want_boost = true;
        }
    } else if let Some(cp) = farm_crate_pos {
        let d = dist2d([me.pos[0], me.pos[1]], [cp[0], cp[1]]);
        goal_3d = if d > 12.0 { cp } else { me.pos };
    } else if let Some(np) = farm_npc_pos {
        let d = dist2d([me.pos[0], me.pos[1]], [np[0], np[1]]);
        if d > 58.0 {
            goal_3d = np;
        } else if d < 38.0 {
            let ang = (me.pos[1] - np[1]).atan2(me.pos[0] - np[0]);
            let tx = me.pos[0] + ang.cos() * 20.0;
            let ty = me.pos[1] + ang.sin() * 20.0;
            goal_3d = [tx, ty, map.terrain_z(tx, ty)];
        } else {
            let ang = (me.pos[1] - np[1]).atan2(me.pos[0] - np[0]) + state.strafe_sign * 1.4;
            let tx = me.pos[0] + ang.cos() * 14.0;
            let ty = me.pos[1] + ang.sin() * 14.0;
            goal_3d = [tx, ty, map.terrain_z(tx, ty)];
        }
    } else {
        goal_3d = pick_macro_hold_spot(obs, state, map);
        if is_ffa && dist2d([me.pos[0], me.pos[1]], [goal_3d[0], goal_3d[1]]) < 14.0 {
            stealth_walk = true;
        }
    }

    // 9. Pathfinding (cube-sized A*!) + Tight Wall Sliding + Dodge Blending + TRUE-Rooftop Climb Controller!
    let mut move_vec = [0.0, 0.0];
    let dist_goal_2d = dist2d([me.pos[0], me.pos[1]], [goal_3d[0], goal_3d[1]]);
    let dz_goal = goal_3d[2] - me.pos[2];
    let goal_is_elevated_roof = goal_3d[2] > map.terrain_z(goal_3d[0], goal_3d[1]) + 1.5;

    if t < state.stuck_escape_until {
        move_vec = state.stuck_escape_dir;
        let overhang = map.is_covered_above(me.pos[0], me.pos[1], me.pos[2] + me.height() - 0.5, 4.0);
        if me.grounded && !overhang && me.energy > 30.0 {
            want_jump = true;
        }
    } else if dist_goal_2d > 2.0 {
        // Use cube-sized A* whenever > 6m away (except final 12m vertical climb onto an actual roof!)
        let use_astar = dist_goal_2d > 6.0 && !(goal_is_elevated_roof && dist_goal_2d < 14.0);
        let wp = if use_astar {
            map.next_waypoint_sized(
                [me.pos[0], me.pos[1]],
                [goal_3d[0], goal_3d[1]],
                me.pos[2],
                me.radius(),
                me.height(),
                me.step_up(),
            )
        } else {
            [goal_3d[0], goal_3d[1]]
        };
        let dx = wp[0] - me.pos[0];
        let dy = wp[1] - me.pos[1];
        let d = dx.hypot(dy).max(1e-6);
        move_vec = [dx / d, dy / d];

        // Tight wall repulsion (+1.1m) so we hug cover cleanly without snagging!
        if !(goal_is_elevated_roof && dist_goal_2d < 12.0) {
            let rep = map.wall_repulsion(me.pos[0], me.pos[1], me.pos[2], me.radius() + 1.1, me.height(), me.step_up());
            move_vec[0] += rep[0] * 0.7;
            move_vec[1] += rep[1] * 0.7;
        }
    }

    // Blend active dodge vector!
    if dodge.urgency > 0.1 {
        let dw = dodge.urgency.clamp(0.5, 3.0);
        move_vec[0] += dodge.vec[0] * dw;
        move_vec[1] += dodge.vec[1] * dw;
    }

    let mlen = move_vec[0].hypot(move_vec[1]);
    if mlen > 1e-6 {
        let scale = if stealth_walk && dodge.urgency < 0.2 {
            (4.3 / me.walk_speed().max(5.0)).min(0.4)
        } else if dist_goal_2d < 4.0 && dodge.urgency < 0.2 {
            (dist_goal_2d / 4.0).clamp(0.2, 1.0)
        } else {
            1.0
        };
        move_vec = [(move_vec[0] / mlen) * scale, (move_vec[1] / mlen) * scale];
    }

    // FIX P0 #1: ONLY trigger rooftop jetpack climb when goal is an ACTUAL elevated roof (NOT gentle hills!)
    // AND we are horizontally close to its ledge (< 14m, or already mid-climb)!
    let wall_ahead_for_roof = goal_is_elevated_roof && dist_goal_2d < 16.0;
    let can_start_climb = t >= state.climb_cooldown_until
        && wall_ahead_for_roof
        && (enemies_with_los == 0 || (enemies_with_los == 1 && hp_frac > 0.65 && dz_goal <= 20.0));

    if dz_goal > me.step_up() + 0.5 && (state.climb_goal.is_some() || can_start_climb) {
        let energy_needed = (dz_goal / 9.2) * me.jet_drain();
        let overhang = map.is_covered_above(me.pos[0], me.pos[1], me.pos[2] + me.height() - 0.5, 4.0);
        if !overhang && (me.energy > energy_needed + 10.0 || (state.climb_goal.is_some() && me.energy > 6.0)) {
            if state.climb_goal.is_none() {
                state.climb_goal = Some((goal_3d, t));
            }
            if me.grounded {
                want_jump = true;
                want_boost = false;
            } else {
                let pulse_off = dodge.urgency < 0.5 && enemies_with_los == 0 && me.vel[2] >= 8.7 && (obs.tick % 6 == 0) && me.energy < 70.0;
                want_boost = !pulse_off;
            }
            if me.pos[2] < goal_3d[2] - 0.5 && dist_goal_2d < 10.0 {
                move_vec[0] *= 0.30;
                move_vec[1] *= 0.30;
            }
        }
    } else if !me.grounded && state.climb_goal.is_some() && dz_goal > 0.2 && me.energy > 6.0 {
        want_boost = true;
    }

    // Don't waste energy jetting in air when dropping off a roof!
    if !me.grounded && state.climb_goal.is_none() && (!goal_is_elevated_roof || dz_goal <= 0.3) {
        want_boost = false;
    }

    // 10. Aim fallback with periodic full 360-degree rear check (eliminates 108-deg blindspot!)
    let final_aim = if let Some(a) = combat.aim {
        a
    } else {
        let move_yaw = if mlen > 0.1 { move_vec[1].atan2(move_vec[0]) } else { state.scan_yaw };
        let cycle = t % 3.2;
        let offset = if cycle > 2.65 {
            std::f64::consts::PI // Quick rear-view check every 3.2s!
        } else {
            (t * 1.6).sin() * 1.15
        };
        state.scan_yaw = wrap_angle(move_yaw + offset);
        [state.scan_yaw, -0.04]
    };

    state.last_cmd_move = move_vec;
    act.move_dir = Some(move_vec);
    act.boost = Some(want_boost);
    act.jump = Some(want_jump);
    act.aim = Some(final_aim);
    act
}

#[inline]
fn in_stadium_bowl(x: f64, y: f64, z: f64) -> bool {
    x > -375.0 && x < -180.0 && y > 10.0 && y < 255.0 && z < 0.0
}

/// Finds nearest valid walkable ground OR safe rooftop spot strictly inside `next_zone` (works even when nr = 0!).
fn find_safe_zone_target(obs: &Observation, map: &ArenaMap) -> [f64; 3] {
    let me = &obs.me;
    let nc = obs.zone.next_center;
    let nr = obs.zone.next_radius;

    let dx = nc[0] - me.pos[0];
    let dy = nc[1] - me.pos[1];
    let d = dx.hypot(dy).max(1e-6);
    let target_r = (nr * 0.50).min((nr - 8.0).max(0.0)).max(0.0);
    let desired = if d > target_r {
        [nc[0] - (dx / d) * target_r, nc[1] - (dy / d) * target_r]
    } else {
        nc
    };

    let check_ground = |x: f64, y: f64| -> Option<[f64; 3]> {
        if x < map.min_x + 8.0 || x > map.max_x - 8.0 || y < map.min_y + 8.0 || y > map.max_y - 8.0 {
            return None;
        }
        let tz = map.terrain_z(x, y);
        if map.terrain_slope_deg(x, y) < 46.0
            && !map.is_cylinder_blocked(x, y, tz, me.radius() + 0.4, me.height(), me.step_up())
        {
            return Some([x, y, tz]);
        }
        None
    };

    if let Some(p) = check_ground(desired[0], desired[1]) {
        return p;
    }
    if let Some(p) = check_ground(nc[0], nc[1]) {
        return p;
    }

    // Spiral outward up to max(nr, 42m) so Phase 6 (nr = 0) still escapes solid buildings!
    let max_r = nr.max(42.0);
    for ring in 1..=7 {
        let r = (ring as f64 * 6.0).min(max_r);
        for k in 0..12 {
            let ang = (k as f64 / 12.0) * std::f64::consts::TAU;
            let x = nc[0] + ang.cos() * r;
            let y = nc[1] + ang.sin() * r;
            if let Some(p) = check_ground(x, y) {
                return p;
            }
        }
    }

    // Fallback: closest climbable rooftop near next_center!
    if let Some(roof) = map
        .rooftops
        .iter()
        .min_by(|a, b| dist2d([a.pos[0], a.pos[1]], nc).partial_cmp(&dist2d([b.pos[0], b.pos[1]], nc)).unwrap_or(std::cmp::Ordering::Equal))
    {
        if dist2d([roof.pos[0], roof.pos[1]], nc) < nr + 30.0 {
            return roof.pos;
        }
    }

    [nc[0], nc[1], map.terrain_z(nc[0], nc[1])]
}

/// Evaluates local cover, 1v1 isolation, anti-lightning spacing, and top 6 climbable rooftops.
fn pick_tactical_position(
    obs: &Observation,
    state: &WorldState,
    map: &ArenaMap,
    enemies: &[EnemySummary],
    primary_id: Option<i64>,
    want_cover_only: bool,
    want_peek: bool,
) -> [f64; 3] {
    let me = &obs.me;
    let my_h = me.height();
    let my_muz_h = me.muzzle_height();
    let my_r = me.radius();
    let my_tz = map.terrain_z(me.pos[0], me.pos[1]);
    let on_rooftop = me.pos[2] > my_tz + 5.0;
    let (z_future_c, z_future_r) = zone_at(&obs.zone, obs.t, obs.t + 14.0);
    let have_light = find_weapon_slot(me, "lightning").is_some();
    let have_laser = find_weapon_slot(me, "laser").is_some();
    let zone_in_stadium = in_stadium_bowl(obs.zone.next_center[0], obs.zone.next_center[1], -5.0);

    let preferred_dist = if have_laser {
        92.0
    } else if have_light {
        34.0
    } else {
        62.0
    };

    let mut candidates: Vec<[f64; 3]> = Vec::with_capacity(52);
    candidates.push(me.pos);

    // 3 concentric rings (10m, 22m, 40m) checking BOTH cylinder clearance AND terrain slope!
    for &(rad, count) in &[(10.0, 10usize), (22.0, 12usize), (40.0, 10usize)] {
        for k in 0..count {
            let ang = (k as f64 / count as f64) * std::f64::consts::TAU;
            let cx = me.pos[0] + ang.cos() * rad;
            let cy = me.pos[1] + ang.sin() * rad;
            if cx < map.min_x + 8.0 || cx > map.max_x - 8.0 || cy < map.min_y + 8.0 || cy > map.max_y - 8.0 {
                continue;
            }
            let sz = map.support_z(cx, cy, me.pos[2], me.step_up(), my_h);
            let is_ground = (sz - map.terrain_z(cx, cy)).abs() < 0.5;
            if is_ground && map.terrain_slope_deg(cx, cy) > 46.0 {
                continue;
            }
            if !map.is_cylinder_blocked(cx, cy, sz, my_r + 0.3, my_h, me.step_up()) {
                candidates.push([cx, cy, sz]);
            }
        }
    }

    // Building-shadow cover candidates up to 95m away when seeking hard cover!
    if want_cover_only || enemies.len() >= 2 {
        if let Some(nearest_e) = enemies.iter().min_by(|a, b| a.dist.partial_cmp(&b.dist).unwrap_or(std::cmp::Ordering::Equal)) {
            let mut nearby_prisms: Vec<(f64, usize)> = Vec::new();
            for (pid, p) in map.prisms.iter().enumerate() {
                if p.area < 40.0 || p.top - p.base < 9.0 { continue; }
                let d = dist2d([me.pos[0], me.pos[1]], p.centroid);
                if d < 105.0 { nearby_prisms.push((d, pid)); }
            }
            nearby_prisms.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
            for &(_, pid) in nearby_prisms.iter().take(5) {
                let p = &map.prisms[pid];
                let dx = p.centroid[0] - nearest_e.center[0];
                let dy = p.centroid[1] - nearest_e.center[1];
                let len = dx.hypot(dy).max(1e-6);
                let half_ext = ((p.max_x - p.min_x).max(p.max_y - p.min_y) * 0.52 + 5.0).min(38.0);
                let cx = (p.centroid[0] + (dx / len) * half_ext).clamp(map.min_x + 8.0, map.max_x - 8.0);
                let cy = (p.centroid[1] + (dy / len) * half_ext).clamp(map.min_y + 8.0, map.max_y - 8.0);
                let tz = map.terrain_z(cx, cy);
                if map.terrain_slope_deg(cx, cy) < 46.0
                    && !map.is_cylinder_blocked(cx, cy, tz, my_r + 0.4, my_h, me.step_up())
                {
                    candidates.push([cx, cy, tz]);
                }
            }
        }
    }

    // Top 6 nearest climbable rooftops ONLY when climb cooldown expired & not taking heavy fire (latency-capped!)
    let cur_los_count = enemies.iter().filter(|e| e.has_los).count();
    if me.energy > 42.0 && obs.t >= state.climb_cooldown_until && cur_los_count == 0 {
        let max_climb = ((me.energy - 14.0) / me.jet_drain().max(5.0)) * 9.0;
        let mut roof_cands: Vec<(f64, [f64; 3])> = Vec::new();
        for roof in &map.rooftops {
            let d = dist2d([me.pos[0], me.pos[1]], [roof.pos[0], roof.pos[1]]);
            let dz = roof.pos[2] - me.pos[2];
            if d < 50.0 && dz > 2.5 && dz < max_climb {
                if !map.is_cylinder_blocked(roof.pos[0], roof.pos[1], roof.pos[2] + 0.05, my_r + 0.2, my_h, 0.5) {
                    roof_cands.push((d, roof.pos));
                }
            }
        }
        roof_cands.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        for &(_, rp) in roof_cands.iter().take(6) {
            candidates.push(rp);
        }
    }

    // Filter active nearby threats for raycast evaluation (cap at <= 165m for strict < 2ms p99!)
    let active_enemies: Vec<&EnemySummary> = enemies.iter().filter(|e| e.dist <= 165.0 || Some(e.id) == primary_id).collect();
    let primary_enemy = primary_id.and_then(|pid| enemies.iter().find(|e| e.id == pid));

    let mut best_pos = me.pos;
    let mut best_score = f64::NEG_INFINITY;

    for &cand in &candidates {
        let cand_muz = [cand[0], cand[1], cand[2] + my_muz_h];
        let cand_upper = [cand[0], cand[1], cand[2] + my_h * 0.78];
        let mut sc = 0.0;

        // Zone safety
        let dc_cur = dist2d([cand[0], cand[1]], obs.zone.center);
        let dc_fut = dist2d([cand[0], cand[1]], z_future_c);
        let z_pad = (obs.zone.radius * 0.18).min(6.0);
        if dc_cur > obs.zone.radius - z_pad {
            sc -= 2000.0 + (dc_cur - obs.zone.radius) * 25.0;
        }
        if z_future_r > 6.0 && dc_fut > z_future_r - 8.0 {
            sc -= (dc_fut - (z_future_r - 8.0)) * (2.5 + obs.zone.dps * 0.04);
        }

        if !zone_in_stadium && in_stadium_bowl(cand[0], cand[1], cand[2]) {
            sc -= 220.0;
        }

        // Exact LOS exposure (enemy muzzle -> our upper torso) & shooting LOS (our muzzle -> primary center)!
        let mut exposed = 0usize;
        let mut primary_los = false;
        let mut primary_dist = 100.0;
        let mut min_enemy_d = f64::INFINITY;
        let mut charging_los = 0usize;
        let mut enemies_on_my_height = 0usize;

        for e in &active_enemies {
            let d = dist3d(cand_muz, e.center);
            if d < min_enemy_d { min_enemy_d = d; }
            if on_rooftop && (e.pos[2] - me.pos[2]).abs() < 4.0 && d < 35.0 {
                enemies_on_my_height += 1;
            }
            // Anti-lightning spacing (CRITICAL P0 #4 fix!): NEVER linger inside 50m of an enemy lightning carrier!
            if e.has_lightning && !have_light && d < 52.0 {
                sc -= (52.0 - d) * 11.0;
            }
            let enemy_sees_us = map.has_shot_los(e.muzzle, cand_upper);
            if enemy_sees_us {
                exposed += 1;
                if e.charging { charging_los += 1; }
            }
            if Some(e.id) == primary_id {
                primary_los = map.has_shot_los(cand_muz, e.center);
                primary_dist = d;
            }
        }

        if want_cover_only {
            if exposed == 0 {
                sc += 245.0;
            } else {
                sc -= (exposed as f64) * 135.0;
            }
            sc -= (charging_los as f64) * 120.0;
            sc += min_enemy_d.min(95.0) * 0.7;
        } else {
            // 1v1 sightline isolation!
            if exposed == 1 && primary_los {
                sc += if want_peek { 195.0 } else { 145.0 };
            } else if exposed == 0 {
                sc += if want_peek { 40.0 } else { 105.0 };
            } else {
                sc -= ((exposed - 1) as f64) * 170.0;
            }
            sc -= (charging_los as f64) * 80.0;

            let range_err = (primary_dist - preferred_dist).abs();
            sc -= range_err * 0.45;
            if !have_light && min_enemy_d < 26.0 {
                sc -= (26.0 - min_enemy_d) * 6.0;
            }
        }

        // High ground & rooftop retention
        let height_adv = cand[2] - map.terrain_z(cand[0], cand[1]);
        sc += height_adv.clamp(0.0, 38.0) * 2.0;
        if on_rooftop && cand[2] < me.pos[2] - 4.0 && enemies_on_my_height < 2 {
            sc -= 75.0;
        }

        sc += map.cover_at([cand[0], cand[1]]) as f64 * 35.0;

        let move_d = dist2d([cand[0], cand[1]], [me.pos[0], me.pos[1]]);
        let dz = (cand[2] - me.pos[2]).max(0.0);
        sc -= move_d * 0.30 + dz * 0.85;

        // Hysteresis toward previous tactical goal
        if let Some(prev_g) = state.last_tactical_goal {
            if dist2d([cand[0], cand[1]], [prev_g[0], prev_g[1]]) < 10.0 {
                sc += 18.0;
            }
        }

        // Tangential strafe momentum bonus during 1v1 duels (makes us WAY harder to hit with gun/mortar!)
        if !want_cover_only && move_d > 5.0 && move_d < 26.0 {
            if let Some(pe) = primary_enemy {
                let to_e_yaw = (pe.pos[1] - me.pos[1]).atan2(pe.pos[0] - me.pos[0]);
                let tan_yaw = to_e_yaw + state.strafe_sign * std::f64::consts::FRAC_PI_2;
                let move_yaw = (cand[1] - me.pos[1]).atan2(cand[0] - me.pos[0]);
                sc += angle_diff(move_yaw, tan_yaw).cos() * 18.0;
            }
        }

        if sc > best_score {
            best_score = sc;
            best_pos = cand;
        }
    }

    best_pos
}

/// Picks a strong macro hold spot (rooftop or high-cover building edge inside `next_zone`).
fn pick_macro_hold_spot(obs: &Observation, _state: &WorldState, map: &ArenaMap) -> [f64; 3] {
    let me = &obs.me;
    let nc = obs.zone.next_center;
    let nr = obs.zone.next_radius;

    let mut best_spot = find_safe_zone_target(obs, map);
    let mut best_score = f64::NEG_INFINITY;

    for roof in &map.rooftops {
        let d_nc = dist2d([roof.pos[0], roof.pos[1]], nc);
        if d_nc > (nr - 12.0).max(8.0) {
            continue;
        }
        let d_me = dist2d([roof.pos[0], roof.pos[1]], [me.pos[0], me.pos[1]]);
        let sc = roof.height_above_ground.min(35.0) * 2.2
            + roof.area.sqrt().min(30.0) * 1.2
            - d_nc * 0.45
            - d_me * 0.30;
        if sc > best_score {
            best_score = sc;
            best_spot = roof.pos;
        }
    }

    if best_score == f64::NEG_INFINITY {
        let r_sample = (nr * 0.6).min(60.0);
        for k in 0..8 {
            let ang = (k as f64 / 8.0) * std::f64::consts::TAU;
            let x = nc[0] + ang.cos() * r_sample;
            let y = nc[1] + ang.sin() * r_sample;
            let tz = map.terrain_z(x, y);
            if map.terrain_slope_deg(x, y) > 46.0
                || map.is_cylinder_blocked(x, y, tz, me.radius() + 0.5, me.height(), me.step_up())
            {
                continue;
            }
            let d_me = dist2d([x, y], [me.pos[0], me.pos[1]]);
            let cov = map.cover_at([x, y]) as f64;
            let sc = cov * 45.0 - d_me * 0.2;
            if sc > best_score {
                best_score = sc;
                best_spot = [x, y, tz];
            }
        }
    }

    best_spot
}
