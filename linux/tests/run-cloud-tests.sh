#!/bin/sh
set -eu
cd "$(dirname "$0")/../.."
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT HUP INT TERM
cxx=${CXX:-g++}
"$cxx" -std=c++14 -pthread linux/tests/cloud-worker-test.cpp -o "$work/worker"
"$work/worker"
"$cxx" -std=c++14 -pthread linux/tests/wifi-status-test.cpp -o "$work/wifi-status"
mkdir "$work/bin"
cat > "$work/bin/wpa_cli" <<'EOF'
#!/bin/sh
case "$WIFI_TEST_STATE" in
  STALL) exec sleep 10 ;;
  FAIL) echo wpa_state=COMPLETED; exit 1 ;;
  *) printf 'bssid=test\nwpa_state=%s\n' "$WIFI_TEST_STATE" ;;
esac
EOF
chmod +x "$work/bin/wpa_cli"
PATH="$work/bin:$PATH" "$work/wifi-status"
"$cxx" -std=c++14 -pthread -ffunction-sections -fdata-sections -Wl,--gc-sections -I linux/src \
  linux/tests/cloud-tls-stall-test.cpp linux/src/BridgeSecureClient.cpp \
  linux/src/Stream.cpp linux/src/WString.cpp linux/src/Print.cpp linux/src/IPAddress.cpp \
  -lssl -lcrypto -o "$work/tls"
"$work/tls"
