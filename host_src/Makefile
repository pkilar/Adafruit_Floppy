PYTHON3 = python3
.DEFAULT_GOAL := all

# File fixtures are checked without the optional Greaseweazle dependency.
.PHONY: check-files
check-files: test_mfm_files test_mfm_bpb
	./test_mfm_files
	./test_mfm_bpb

test_mfm_files: test_mfm_files.c ../src/mfm_impl.h
	$(CC) -iquote ../src -Wall -Werror -ggdb3 -Og -o $@ $<

test_mfm_bpb: test_mfm_bpb.c ../src/mfm_bpb.h
	$(CC) -iquote ../src -Wall -Werror -ggdb3 -Og -o $@ $<

.PHONY: check-fm
check-fm: test_fm_decode
	./test_fm_decode

test_fm_decode: test_fm_decode.c ../src/mfm_impl.h
	$(CC) -iquote ../src -Wall -Werror -ggdb3 -Og -o $@ $<

.PHONY: check-atari-track
check-atari-track: test_atari_track.c ../src/atari_track.h ../src/mfm_impl.h ../src/atari_disk.h
	$(CC) -iquote ../src -Wall -Werror -ggdb3 -Og -o /tmp/test_atari_track $<
	/tmp/test_atari_track

.PHONY: check-atari-gw
check-atari-gw: atari_gw_dump.c check_atari_gw.py ../src/atari_track.h ../src/mfm_impl.h ../src/atari_disk.h
	$(CC) -iquote ../src -Wall -Werror -ggdb3 -Og -o /tmp/atari_gw_dump atari_gw_dump.c
	$(PYTHON3) check_atari_gw.py /tmp/atari_gw_dump

.PHONY: check-catalog
check-catalog:
	$(CXX) -std=c++11 -Wall -Wextra -Werror -I../src image_catalog.cpp ../src/Adafruit_FloppyImage.cpp -o /tmp/floppsy-image-catalog
	$(PYTHON3) test_image_catalog.py /tmp/floppsy-image-catalog

.PHONY: check-atr
check-atr:
	$(CXX) -std=c++11 -Wall -Wextra -Werror -I../src test_atari_atr.cpp ../src/Adafruit_AtariATR.cpp -o /tmp/floppsy-test-atari-atr
	/tmp/floppsy-test-atari-atr

.PHONY: check-atari-prot
check-atari-prot: test_atari_prot.c ../src/atari_prot_track.h ../src/atari_track.h ../src/mfm_impl.h ../src/atari_disk.h ../src/atari_atx.h
	$(CC) -iquote ../src -Wall -Werror -ggdb3 -Og -o /tmp/test_atari_prot $<
	/tmp/test_atari_prot

.PHONY: check-atx
check-atx:
	$(CXX) -std=c++11 -Wall -Wextra -Werror -I../src test_atari_atx.cpp ../src/Adafruit_AtariATX.cpp -o /tmp/test_atari_atx
	/tmp/test_atari_atx

.PHONY: check-sio
check-sio:
	$(CXX) -std=c++11 -Wall -Wextra -Werror -I../src test_atari_sio.cpp ../src/Adafruit_AtariSIO.cpp -o /tmp/test_atari_sio
	/tmp/test_atari_sio

.PHONY: all
all: check checkfm

.PHONY: check-layout
check-layout:
	$(CC) -iquote ../src -Wall -Werror -ggdb3 -Og -o /tmp/floppy-test-layout test_track_layout.c
	/tmp/floppy-test-layout

.PHONY: check
check: main check_flux.py
	./main
	$(PYTHON3) check_flux.py flux0 > decode0
	$(PYTHON3) check_flux.py flux1 > decode1
	$(PYTHON3) check_flux.py flux2 > decode2

.PHONY: checkfm
checkfm: main_fm check_flux.py
	./main_fm
	$(PYTHON3) check_flux.py --fm fluxfm > decodefm

main: main.c ../src/mfm_impl.h Makefile test_flux.h
	gcc -iquote ../src -Wall -Werror -ggdb3 -Og -o $@ $<

main_fm: main_fm.c ../src/mfm_impl.h Makefile
	gcc -iquote ../src -Wall -Werror -ggdb3 -Og -o $@ $<

test_flux.h: make_flux.py greaseweazle/scripts/greaseweazle/version.py
	$(PYTHON3) $< $@

greaseweazle/scripts/greaseweazle/version.py:
	$(MAKE) -C greaseweazle
