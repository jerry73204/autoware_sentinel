use vstd::prelude::*;
use crate::ghost_types::*;

verus! {

// Gear constants matching production code
pub const DRIVE: u8 = 2;
pub const REVERSE: u8 = 20;
pub const PARK: u8 = 22;

/// A gear value is valid if it is one of the three defined gears.
pub open spec fn valid_gear(g: u8) -> bool {
    g == DRIVE || g == REVERSE || g == PARK
}

/// Ghost model of ShiftDecider.
pub struct GhostShiftDecider {
    pub park_on_goal: bool,
    pub prev_command: u8,
}

/// Ghost model of the decide function.
///
/// Autoware states: WAITING_FOR_ROUTE=2, DRIVING=5, ARRIVED_GOAL=6.
pub open spec fn ghost_decide(
    sd: GhostShiftDecider,
    autoware_state: u8,
    velocity_zone: VelocityZone,
    current_gear: u8,
    park_on_goal: bool,
) -> u8 {
    if autoware_state == 5 {
        // DRIVING
        match velocity_zone {
            VelocityZone::Forward => DRIVE,
            VelocityZone::Reverse => REVERSE,
            VelocityZone::DeadZone => sd.prev_command,
        }
    } else if (autoware_state == 6 || autoware_state == 2) && park_on_goal {
        PARK
    } else {
        current_gear
    }
}

/// The decide function always produces a valid gear when inputs are valid.
proof fn decide_produces_valid_gear(
    sd: GhostShiftDecider,
    autoware_state: u8,
    velocity_zone: VelocityZone,
    current_gear: u8,
)
    requires
        valid_gear(sd.prev_command),
        valid_gear(current_gear),
        autoware_state == 2 || autoware_state == 3 || autoware_state == 5 || autoware_state == 6,
    ensures
        valid_gear(ghost_decide(sd, autoware_state, velocity_zone, current_gear, sd.park_on_goal)),
{
    // Verus discharges this automatically from the spec definition.
}

/// No direct DRIVE→REVERSE transition through the dead zone.
///
/// If the vehicle was in DRIVE and enters the dead zone, the output is still DRIVE
/// (the previous command is held).
proof fn no_direct_drive_reverse_transition()
    ensures ({
        let sd = GhostShiftDecider { park_on_goal: true, prev_command: DRIVE };
        ghost_decide(sd, 5, VelocityZone::DeadZone, DRIVE, true) == DRIVE
    }),
{
    // Follows directly from spec: DeadZone → prev_command = DRIVE.
}

} // verus!
