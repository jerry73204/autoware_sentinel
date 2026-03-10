#!/usr/bin/env bash
# Filter a play_launch record.json to remove the 7 nodes replaced by the sentinel,
# plus the ADAPI operation_mode adaptor (whose change_to_autonomous service conflicts
# with the sentinel's own service).
#
# Removes:
#   node[]:      mrm_handler
#   container[]: mrm_emergency_stop_operator_container,
#                mrm_comfortable_stop_operator_container
#   load_node[]: autoware_vehicle_cmd_gate, autoware_shift_decider,
#                autoware_operation_mode_transition_manager,
#                autoware_control_validator,
#                operation_mode (autoware_default_adapi_universe)
#                autoware_state (autoware_default_adapi_universe)
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
    .name == null or ((.name | split("/") | last) != "mrm_handler")
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
  )]
' "$INPUT" > "$OUTPUT"
