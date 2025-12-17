#!/bin/bash
# this file goes in /usr/local/bin/ Wait until Tailscale assigns an IP (i.e., the network is ready)
echo "Waiting for Tailscale connection..."

until /usr/bin/tailscale ip -4 | grep -q '^100\.'; do
    sleep 1
done

echo "Tailscale is ready."
exit 0

