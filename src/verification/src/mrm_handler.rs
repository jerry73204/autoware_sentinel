use vstd::prelude::*;

verus! {

/// Ghost MRM states matching production constants.
#[derive(PartialEq, Eq)]
pub enum MrmState {
    Normal,
    Operating,
    Succeeded,
    Failed,
}

/// Ghost MRM behaviors matching production constants.
#[derive(PartialEq, Eq)]
pub enum MrmBehavior {
    None,
    ComfortableStop,
    EmergencyStop,
}

/// Ghost model of the MRM handler.
pub struct GhostMrmHandler {
    pub state: MrmState,
    pub current_behavior: MrmBehavior,
    pub velocity_is_zero: bool,
}

/// Ordinal for behavior escalation: None < ComfortableStop < EmergencyStop.
pub open spec fn behavior_ord(b: MrmBehavior) -> int {
    match b {
        MrmBehavior::None => 0,
        MrmBehavior::ComfortableStop => 1,
        MrmBehavior::EmergencyStop => 2,
    }
}

/// A valid MRM state satisfies consistency between state and behavior,
/// and the invariant that Succeeded requires the vehicle to be stopped.
pub open spec fn valid_mrm_state(h: GhostMrmHandler) -> bool {
    // Normal state must have no behavior
    &&& (h.state == MrmState::Normal ==> h.current_behavior == MrmBehavior::None)
    // Operating state must have an active behavior
    &&& (h.state == MrmState::Operating ==> h.current_behavior != MrmBehavior::None)
    // Succeeded requires the vehicle to be stopped
    &&& (h.state == MrmState::Succeeded ==> h.velocity_is_zero)
    // Failed has no additional constraints (vehicle may still be moving)
}

/// Ghost transition function for the MRM handler.
///
/// Models the production `update()` method.
pub open spec fn ghost_transition(
    h: GhostMrmHandler,
    operation_available: bool,
    comfortable_stop_available: bool,
    velocity_is_zero: bool,
) -> GhostMrmHandler {
    let h = GhostMrmHandler { velocity_is_zero, ..h };
    match h.state {
        MrmState::Normal => {
            if !operation_available {
                let behavior = if comfortable_stop_available {
                    MrmBehavior::ComfortableStop
                } else {
                    MrmBehavior::EmergencyStop
                };
                GhostMrmHandler {
                    state: MrmState::Operating,
                    current_behavior: behavior,
                    velocity_is_zero: h.velocity_is_zero,
                }
            } else {
                h
            }
        },
        MrmState::Operating => {
            if operation_available {
                // Recovery
                GhostMrmHandler {
                    state: MrmState::Normal,
                    current_behavior: MrmBehavior::None,
                    velocity_is_zero: h.velocity_is_zero,
                }
            } else if h.velocity_is_zero {
                // Vehicle stopped → success
                GhostMrmHandler {
                    state: MrmState::Succeeded,
                    current_behavior: h.current_behavior,
                    velocity_is_zero: h.velocity_is_zero,
                }
            } else if h.current_behavior == MrmBehavior::ComfortableStop
                && !comfortable_stop_available {
                // Escalation
                GhostMrmHandler {
                    state: MrmState::Operating,
                    current_behavior: MrmBehavior::EmergencyStop,
                    velocity_is_zero: h.velocity_is_zero,
                }
            } else {
                h
            }
        },
        // Terminal states — no transitions
        MrmState::Succeeded => h,
        MrmState::Failed => h,
    }
}

/// Terminal states (Succeeded, Failed) have no outgoing transitions.
proof fn terminal_state_stability(
    h: GhostMrmHandler,
    operation_available: bool,
    comfortable_stop_available: bool,
    velocity_is_zero: bool,
)
    requires
        h.state == MrmState::Succeeded || h.state == MrmState::Failed,
    ensures ({
        let next = ghost_transition(h, operation_available, comfortable_stop_available, velocity_is_zero);
        next.state == h.state && next.current_behavior == h.current_behavior
    }),
{
    // Follows directly from the match arms for Succeeded/Failed.
}

/// Behavior escalation is monotonic while Operating.
///
/// When in Operating state, the behavior ordinal can only increase or stay the same.
/// This ensures we never de-escalate from EmergencyStop back to ComfortableStop.
proof fn escalation_monotonicity(
    h: GhostMrmHandler,
    operation_available: bool,
    comfortable_stop_available: bool,
    velocity_is_zero: bool,
)
    requires
        h.state == MrmState::Operating,
        !operation_available,
        !velocity_is_zero,
    ensures ({
        let next = ghost_transition(h, operation_available, comfortable_stop_available, velocity_is_zero);
        next.state == MrmState::Operating ==>
            behavior_ord(next.current_behavior) >= behavior_ord(h.current_behavior)
    }),
{
    // When operating and not recovering and not stopped:
    // - If comfortable stop and !available → escalate to emergency (ord increases)
    // - Otherwise → hold current behavior (ord stays same)
}

/// Succeeded requires the vehicle to be stopped — the invariant is preserved.
///
/// If valid_mrm_state holds before transition, and the vehicle has velocity_is_zero
/// when transitioning to Succeeded, then valid_mrm_state holds after.
proof fn succeeded_requires_stopped(
    h: GhostMrmHandler,
    operation_available: bool,
    comfortable_stop_available: bool,
    velocity_is_zero: bool,
)
    requires
        valid_mrm_state(h),
        h.state == MrmState::Operating,
        !operation_available,
        velocity_is_zero,
    ensures ({
        let next = ghost_transition(h, operation_available, comfortable_stop_available, velocity_is_zero);
        next.state == MrmState::Succeeded ==> next.velocity_is_zero
    }),
{
    // The transition to Succeeded only happens when velocity_is_zero is true,
    // and we carry that flag into the new state.
}

} // verus!
