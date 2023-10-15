#!/bin/bash

# Get the node publishing to /cmd_vel
node=$(rostopic info /cmd_vel 2>&1 | grep "Publishers:" -A 1 | tail -n1 | awk -F' ' '{print $2}' | cut -d'(' -f1)
# Check if we found any node
if [ ! -z "$node" ]; then
  rosnode kill "$node"
  echo "Killed node: $node"
else
  echo "No node publishing to /cmd_vel found."
fi
