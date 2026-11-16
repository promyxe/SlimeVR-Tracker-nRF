#!/bin/bash
# Run the tracker firmware on a simulated nRF52 (BabbleSim) with emulated ICM-45686/QMC6309.
#
#   tools/bsim/sim.sh setup                       # once: docker image slime-bsim:ready
#   tools/bsim/sim.sh build                       # firmware for nrf52_bsim -> $OUT/build
#   tools/bsim/sim.sh run <rec.csv> <out.csv> <s> # feed a recording, log output quaternions
#
# Recordings: tools/replay/sim.py (raw-collection format). Compare: tools/bsim/compare.py.
# Env: WORKSPACE (zephyr-workspace, default ../zephyr-workspace), OUT (default /tmp/slime-bsim),
#      SLIME_SIM_MAG=0 (keep mag off), SLIME_SIM_MAGCAL=0 (skip the automatic "mag cal").
set -e
APP=$(cd "$(dirname "$0")/../.." && pwd)
WORKSPACE=$(cd "${WORKSPACE:-$APP/../zephyr-workspace}" && pwd)
OUT=${OUT:-/tmp/slime-bsim}
mkdir -p "$OUT"
DOCKER=(docker run --rm --platform linux/amd64 -v "$APP":/app -v "$WORKSPACE":/ws -v "$OUT":/out)

case "$1" in
setup)
	docker build --platform linux/amd64 -t slime-bsim "$APP/tools/bsim"
	docker rm -f slime-bsim-setup >/dev/null 2>&1 || true
	docker run --platform linux/amd64 --name slime-bsim-setup -v "$WORKSPACE":/ws slime-bsim bash -c '
		pip install -q -r /ws/zephyr/scripts/requirements-base.txt
		mkdir -p /opt/bsim && cp -r /ws/tools/bsim/. /opt/bsim/
		make -C /opt/bsim everything -j8 > /opt/bsim/build.log'
	docker commit slime-bsim-setup slime-bsim:ready >/dev/null
	docker rm slime-bsim-setup >/dev/null
	;;
build)
	# The radio model only accepts BLE packet formats; ESB differs, so downgrade those checks
	# to warnings in a patched copy mounted over nrf_hw_models.
	"${DOCKER[@]}" slime-bsim:ready bash -c '
		rm -rf /out/hw && cp -r /ws/modules/bsim_hw_models/nrf_hw_models /out/hw
		python3 - <<PY
p = "/out/hw/src/HW_models/NHW_RADIO_utils.c"
s = open(p).read()
cut = s.index("static void nrfra_check_802154_conf")
open(p, "w").write(s[:cut].replace("bs_trace_error_line_time(", "bs_trace_warning_line_time(") + s[cut:])
PY'
	"${DOCKER[@]}" -v "$OUT/hw":/ws/modules/bsim_hw_models/nrf_hw_models slime-bsim:ready bash -c '
		cd /ws && west build -p always -b nrf52_bsim -d /out/build /app > /out/build.log 2>&1 || { tail -20 /out/build.log; exit 1; }'
	echo "built $OUT/build/app/zephyr/zephyr.exe"
	;;
run)
	[ $# -eq 4 ] || { echo "usage: $0 run <rec.csv> <out.csv> <seconds>"; exit 2; }
	REC=$(cd "$(dirname "$2")" && pwd)/$(basename "$2")
	DST=$(cd "$(dirname "$3")" && pwd)
	"${DOCKER[@]}" -v "$(dirname "$REC")":/rec -v "$DST":/dst \
		-e SLIME_SIM_MAG="${SLIME_SIM_MAG:-1}" -e SLIME_SIM_MAGCAL="${SLIME_SIM_MAGCAL:-1}" \
		-e SLIME_SIM_CSV="/rec/$(basename "$REC")" -e SLIME_SIM_OUT="/dst/$(basename "$3")" \
		slime-bsim:ready bash -c "
		cd /opt/bsim/bin  # the phy loads its channel models relative to bin/
		./bs_2G4_phy_v1 -s=slime -D=1 -sim_length=${4}e6 > /out/phy.log 2>&1 &
		/out/build/app/zephyr/zephyr.exe -s=slime -d=0 > /out/fw.log 2>&1
		wait"
	echo "firmware log: $OUT/fw.log"
	;;
*)
	sed -n 2,10p "$0"
	exit 2
	;;
esac
