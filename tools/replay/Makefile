VQF ?= ../../vqf-c/src
SRC = vqf_replay.c $(VQF)/vqf.c ../../src/sensor/calibration/mag_offset_track.c

vqf_replay: $(SRC) ../../src/sensor/calibration/mag_offset_track.h
	$(CC) -O2 -std=c11 -Wall -Wextra -I$(VQF) $(SRC) -lm -o $@

check: vqf_replay
	python3 synth_check.py ./vqf_replay

sweep: vqf_replay
	python3 synth_check.py ./vqf_replay --sweep

clean:
	rm -f vqf_replay

.PHONY: check sweep clean
