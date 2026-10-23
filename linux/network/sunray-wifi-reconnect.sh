#!/bin/sh
# Run as a oneshot service. Never disconnect or replace an active connection.
set -eu
export LC_ALL=C
iface=${WIFI_INTERFACE:-wlan0}
: "${WIFI_CONNECTION_UUID:?Set WIFI_CONNECTION_UUID to the saved WLAN profile UUID}"
radio=$(nmcli -t -f WIFI general) || exit 0
[ "$radio" = enabled ] || exit 0
state=$(nmcli -g GENERAL.STATE device show "$iface" 2>/dev/null) || exit 0
case "$state" in
  30\ *|120\ *) ;; # disconnected or failed; leave connecting/connected/unmanaged alone
  *) exit 0 ;;
esac
# An explicit activation also retries profiles blocked by a previous no-secrets
# failure. Use the existing saved password; never log or request credentials.
printf 'WLAN recovery: reconnecting %s using saved profile\n' "$iface"
if nmcli --wait 35 connection up uuid "$WIFI_CONNECTION_UUID" ifname "$iface"; then
  printf 'WLAN recovery: connected\n'
else
  printf 'WLAN recovery: attempt failed; timer will retry\n'
fi
