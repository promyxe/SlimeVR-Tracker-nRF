#!/bin/bash
# Run the tracker firmware on a simulated nRF52 (BabbleSim) with emulated ICM-45686/QMC6309.
#
#   tools/bsim/sim.sh setup                       # once: docker image slime-bsim:ready
#   tools/bsim/sim.sh build                       # firmware for nrf52_bsim -> $OUT/build
#   tools/bsim/sim.sh run <rec.csv> <out.csv> <s> # feed a recording, log output quaternions
#   tools/bsim/sim.sh build-rx                    # receiver firmware -> $OUT/rx (RECEIVER repo)
#   tools/bsim/sim.sh fullstack <s> <rec.csv>...  # receiver + one tracker per recording -> $OUT/fs
#
# Recordings: tools/replay/sim.py (raw-collection format). Compare: tools/bsim/compare.py,
# full stack: tools/bsim/fullstack.py (link/latency/heading), tools/bsim/phy_timing.py (radio timing).
# Env: WORKSPACE (zephyr-workspace, default ../zephyr-workspace), OUT (default /tmp/slime-bsim),
#      RECEIVER (default ../SlimeVR-Tracker-nRF-Receiver),
#      SLIME_SIM_MAG=0 (keep mag off), SLIME_SIM_MAGCAL=0 (skip the automatic "mag cal"),
#      fullstack: STAGGER_US (boot offset between trackers), WLAN (bs_device_2G4_WLAN_actmod args,
#      e.g. "-channel=6 -ConfigSet=5 -power=10"), PHY_ARGS (e.g. -defmodem=BLE_simple, needed for
#      interference to have an effect).
set -e
APP=$(cd "$(dirname "$0")/../.." && pwd)
WORKSPACE=$(cd "${WORKSPACE:-$APP/../zephyr-workspace}" && pwd)
OUT=${OUT:-/tmp/slime-bsim}
mkdir -p "$OUT"
DOCKER=(docker run --rm --platform linux/amd64 -v "$APP":/app -v "$WORKSPACE":/ws -v "$OUT":/out)

# Patched nrf_hw_models copy ($OUT/hw), mounted over the module for builds:
# - the radio model only accepts BLE packet formats; ESB differs, so downgrade those checks to warnings
# - RX listens on every RXADDRESSES pipe and sets RXMATCH (ESB data/ACKs use pipes 1-7)
patch_hw() {
	"${DOCKER[@]}" slime-bsim:ready bash -c '
		rm -rf /out/hw && cp -r /ws/modules/bsim_hw_models/nrf_hw_models /out/hw
		python3 - <<PY
d = "/out/hw/src/HW_models/"
s = open(d + "NHW_RADIO_utils.c").read()
cut = s.index("static void nrfra_check_802154_conf")
s = s[:cut].replace("bs_trace_error_line_time(", "bs_trace_warning_line_time(") + s[cut:]
old = "  rx_req->n_addr = 1;\n\n  rx_req->pream_and_addr_duration"
new = """  rx_req->n_addr = 0;
  for (uint i = 0; i < 8; i++) {
    if ((NRF_RADIO_regs.RXADDRESSES >> i) & 1) {
      rx_addresses[rx_req->n_addr++] = nhwra_get_address(i);
    }
  }
  if (rx_req->n_addr == 0) {
    rx_req->n_addr = 1;
  }

  rx_req->pream_and_addr_duration"""
assert s.count(old) == 1
open(d + "NHW_RADIO_utils.c", "w").write(s.replace(old, new))
s = open(d + "NHW_RADIO.c").read()
old = "    NRF_RADIO_regs.RXMATCH = 0; //The only we support so far\n"
new = """    NRF_RADIO_regs.RXMATCH = 0;
    for (uint i = 0; i < 8; i++) {
      if (((NRF_RADIO_regs.RXADDRESSES >> i) & 1)
          && nhwra_get_address(i) == rx_status.rx_resp.phy_address) {
        NRF_RADIO_regs.RXMATCH = i;
        break;
      }
    }
"""
assert s.count(old) == 1
open(d + "NHW_RADIO.c", "w").write(s.replace(old, new))
PY'
}

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
	patch_hw
	"${DOCKER[@]}" -v "$OUT/hw":/ws/modules/bsim_hw_models/nrf_hw_models slime-bsim:ready bash -c '
		cd /ws && west build -p always -b nrf52_bsim -d /out/build /app > /out/build.log 2>&1 || { tail -20 /out/build.log; exit 1; }'
	echo "built $OUT/build/app/zephyr/zephyr.exe"
	;;
build-rx)
	RX=$(cd "${RECEIVER:-$APP/../SlimeVR-Tracker-nRF-Receiver}" && pwd)
	patch_hw
	"${DOCKER[@]}" -v "$RX":/rxapp -v "$OUT/hw":/ws/modules/bsim_hw_models/nrf_hw_models slime-bsim:ready bash -c '
		cd /ws && west build -p always -b nrf52_bsim -d /out/rx /rxapp > /out/rx.log 2>&1 || { tail -20 /out/rx.log; exit 1; }'
	echo "built $OUT/rx/app/zephyr/zephyr.exe"
	;;
fullstack)
	[ $# -ge 3 ] || { echo "usage: $0 fullstack <seconds> <rec.csv>..."; exit 2; }
	SECS=$2; shift 2
	mkdir -p "$OUT/fs"; rm -rf "$OUT"/fs/*
	MOUNTS=(); RECS=(); i=1
	for r in "$@"; do
		MOUNTS+=(-v "$(cd "$(dirname "$r")" && pwd)/$(basename "$r")":/rec/$i.csv -v "$(cd "$(dirname "$r")" && pwd)/$(basename "${r%.csv}").meta.txt":/rec/$i.meta.txt)
		RECS+=(/rec/$i.csv); i=$((i+1))
	done
	"${DOCKER[@]}" "${MOUNTS[@]}" -e STAGGER_US="${STAGGER_US:-0}" -e WLAN="${WLAN:-}" -e PHY_ARGS="${PHY_ARGS:-}" \
		slime-bsim:ready bash -c '
		cd /opt/bsim/bin; SECS=$1; shift; N=$(( $# + 1 )); FS=/out/fs
		if [ -n "$WLAN" ]; then N=$((N + 1)); ./bs_device_2G4_WLAN_actmod -s=fs -d=$((N - 1)) -rs=7 $WLAN > $FS/wlan.log 2>&1 & fi
		./bs_2G4_phy_v1 -s=fs -D=$N -sim_length=${SECS}e6 -dump $PHY_ARGS > $FS/phy.log 2>&1 &
		SLIME_SIM_OUT=$FS/rx_hid.csv /out/rx/app/zephyr/zephyr.exe -s=fs -d=0 -rs=100 > $FS/rx.log 2>&1 &
		i=1
		for rec in "$@"; do
			OFF=$(( (i - 1) * STAGGER_US )); echo $OFF > $FS/trk$i.offset
			SLIME_SIM_MAGCAL=0 SLIME_SIM_CSV=$rec SLIME_SIM_OUT=$FS/trk$i.csv /out/build/app/zephyr/zephyr.exe \
				-s=fs -d=$i -rs=$((200 + i)) -start_offset=$OFF -delay_init > $FS/trk$i.log 2>&1 &
			i=$((i + 1))
		done
		wait
		cp -r /opt/bsim/results/fs $FS/dump' _ "$SECS" "${RECS[@]}"
	echo "output: $OUT/fs (rx_hid.csv, trkN.csv/.log, dump/); evaluate:"
	echo "  tools/bsim/fullstack.py $OUT/fs/rx_hid.csv --trk 1:$OUT/fs/trk1.csv[:truth.npy] ..."
	echo "  tools/bsim/phy_timing.py $OUT/fs/dump $OUT/fs/rx.log"
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
	sed -n 2,19p "$0"
	exit 2
	;;
esac
