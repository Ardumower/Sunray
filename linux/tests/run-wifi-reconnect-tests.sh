#!/bin/sh
set -eu
cd "$(dirname "$0")/../.."
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT HUP INT TERM
cat > "$work/nmcli" <<'MOCK'
#!/bin/sh
case "$*" in
 '-t -f WIFI general') echo "${TEST_RADIO:-enabled}" ;;
 '-g GENERAL.STATE device show wlan0')
   [ "${TEST_MISSING:-0}" = 0 ] || exit 10
   echo "$TEST_STATE" ;;
 '--wait 35 connection up uuid test-profile ifname wlan0')
   echo attempt >> "$TEST_ATTEMPTS"
   exit "${TEST_FAIL:-0}" ;;
 *) echo "Unexpected nmcli arguments: $*" >&2; exit 99 ;;
esac
MOCK
chmod +x "$work/nmcli"
export PATH="$work:$PATH" WIFI_CONNECTION_UUID=test-profile TEST_ATTEMPTS="$work/attempts"
for state in '10 (unmanaged)' '20 (unavailable)' '40 (prepare)' '50 (config)' '60 (need-auth)' '70 (ip-config)' '100 (connected)' '110 (deactivating)'; do
 TEST_STATE="$state" sh linux/network/sunray-wifi-reconnect.sh
 [ ! -e "$TEST_ATTEMPTS" ]
done
TEST_RADIO=disabled TEST_STATE='30 (disconnected)' sh linux/network/sunray-wifi-reconnect.sh
TEST_MISSING=1 TEST_STATE='30 (disconnected)' sh linux/network/sunray-wifi-reconnect.sh
[ ! -e "$TEST_ATTEMPTS" ]
TEST_STATE='30 (disconnected)' sh linux/network/sunray-wifi-reconnect.sh
TEST_FAIL=4 TEST_STATE='120 (failed)' sh linux/network/sunray-wifi-reconnect.sh
TEST_STATE='30 (disconnected)' sh linux/network/sunray-wifi-reconnect.sh
[ "$(wc -l < "$TEST_ATTEMPTS" | tr -d ' ')" = 3 ]
echo 'PASS WLAN recovery: disconnected/failed retried; connected, activating, disabled, unmanaged and missing devices untouched; retry after failed attempt'
