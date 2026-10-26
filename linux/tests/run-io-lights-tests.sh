#!/bin/sh
set -eu
cd "$(dirname "$0")/../.."
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT HUP INT TERM
"${CXX:-c++}" -std=c++11 -Wall -Wextra -Werror linux/tests/io-lights-test.cpp -o "$work/test"
"$work/test"
if [ -n "${IOEXT_PROJECT:-}" ]; then
 cmp sunray/IoLightsProtocol.h "$IOEXT_PROJECT/IoLightsProtocol.h"
 cp sunray/IoLightsPolicy.h sunray/IoLightsProtocol.h "$IOEXT_PROJECT/LedScenes.h" "$IOEXT_PROJECT/LedLayout.h" "$work/"
 "${CXX:-c++}" -std=c++11 -Wall -Wextra -Werror -I"$work" linux/tests/io-lights-integration-test.cpp -o "$work/integration"
 "$work/integration"
fi
