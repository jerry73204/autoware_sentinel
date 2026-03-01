//! Test fixtures for Autoware Sentinel integration tests

mod zenohd_router;
mod sentinel;

pub use zenohd_router::*;
pub use sentinel::*;

// Re-export commonly used items for convenience
pub use crate::process::ManagedProcess;
pub use crate::ros2::{
    Ros2Process, is_autoware_msgs_available, is_rmw_zenoh_available, is_ros2_available,
    require_ros2_autoware,
};
