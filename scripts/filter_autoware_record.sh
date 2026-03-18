#!/usr/bin/env bash
# Filter a play_launch record.json to remove sentinel-replaced nodes plus
# nodes whose health-checking would block autonomous mode engagement.
#
# Removes:
#   node[]:      mrm_handler,
#                diagnostic_graph_aggregator (aggregator_node + converter_node)
#                  — publishes /system/operation_mode/availability based on
#                    diagnostic heartbeats; blocks engagement when sentinel
#                    replaces vehicle_cmd_gate (heartbeat goes missing),
#                hazard_status_converter — converts aggregator output to
#                  HazardStatus (depends on aggregator)
#   container[]: mrm_emergency_stop_operator_container,
#                mrm_comfortable_stop_operator_container
#   load_node[]: autoware_vehicle_cmd_gate, autoware_shift_decider,
#                autoware_operation_mode_transition_manager,
#                autoware_control_validator,
#                operation_mode (autoware_default_adapi_universe)
#                autoware_state (autoware_default_adapi_universe)
#                diagnostics (autoware_default_adapi_universe) — ADAPI
#                  diagnostics adaptor that reads aggregator output
#                interface (autoware_default_adapi) — change_to_autonomous
#                  service conflicts with sentinel's own service
#
# Usage: filter_autoware_record.sh <input.json> <output.json>

set -eo pipefail

if [[ $# -ne 2 ]]; then
  echo "Usage: $0 <input.json> <output.json>" >&2
  exit 1
fi

INPUT="$1"
OUTPUT="$2"

if [[ ! -f "$INPUT" ]]; then
  echo "Error: input file not found: $INPUT" >&2
  exit 1
fi

mkdir -p "$(dirname "$OUTPUT")"

jq '
  .node |= [.[] | select(
    ((.package // "") != "autoware_diagnostic_graph_aggregator")
    and ((.name // "") != "hazard_status_converter")
    and (.name == null or ((.name | split("/") | last) != "mrm_handler"))
  )]
  | .container |= [.[] | select(
    .name == null or (
      (.name | split("/") | last) != "mrm_emergency_stop_operator_container"
      and (.name | split("/") | last) != "mrm_comfortable_stop_operator_container"
    )
  )]
  | .load_node |= [.[] | select(
    .package != "autoware_vehicle_cmd_gate"
    and .package != "autoware_shift_decider"
    and .package != "autoware_operation_mode_transition_manager"
    and .package != "autoware_control_validator"
    and ((.node_name != "operation_mode") or (.package != "autoware_default_adapi_universe"))
    and ((.node_name != "autoware_state") or (.package != "autoware_default_adapi_universe"))
    and ((.node_name != "diagnostics") or (.package != "autoware_default_adapi_universe"))
    and ((.node_name != "interface") or (.package != "autoware_default_adapi"))
  )]
' "$INPUT" > "$OUTPUT"
