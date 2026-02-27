use vstd::prelude::*;

verus! {

/// Velocity zone classification used by ShiftDecider and EmergencyStop proofs.
///
/// Velocities are represented as scaled integers (millimeters/second) to avoid
/// floating-point reasoning in Verus.
pub enum VelocityZone {
    Forward,
    Reverse,
    DeadZone,
}

/// Threshold for dead-zone classification: 0.01 m/s = 10 mm/s.
pub const DEAD_ZONE_THRESHOLD: int = 10;

/// Classify a scaled velocity (mm/s) into a velocity zone.
pub open spec fn velocity_zone(v_scaled: int) -> VelocityZone {
    if v_scaled > DEAD_ZONE_THRESHOLD {
        VelocityZone::Forward
    } else if v_scaled < -DEAD_ZONE_THRESHOLD {
        VelocityZone::Reverse
    } else {
        VelocityZone::DeadZone
    }
}

} // verus!
