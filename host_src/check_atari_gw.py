#!/usr/bin/env python3
"""Cross-check src/atari_track.h against Greaseweazle's IBM codec.

  1. C encoder -> Greaseweazle decoder (SD/ED/DD on head 0 and DD on head 1
     as "dd1", cylinder 7, 300 RPM, 24 MHz)
  2. Greaseweazle encoder -> C decoder (same tracks)

Usage: check_atari_gw.py ./atari_gw_dump
"""
import pathlib
import random
import subprocess
import sys
import tempfile

sys.path.insert(0, str(pathlib.Path(__file__).parent / "greaseweazle/src"))
from greaseweazle.codec.codec import get_diskdef
from greaseweazle.codec.ibm.ibm import IBMTrack_FixedDef
from greaseweazle.flux import Flux

SAMPLE_FREQ = 24_000_000
CYL = 7
# name -> (sectors, sector bytes, physical order of sector IDs, head)
FORMATS = {
    "sd": (18, 128, list(range(1, 18, 2)) + list(range(2, 19, 2)), 0),
    "ed": (26, 128, list(range(1, 26, 2)) + list(range(2, 27, 2)), 0),
    "dd": (18, 256, list(range(18, 0, -1)), 0),
    "dd1": (18, 256, list(range(18, 0, -1)), 1),
}

failures = 0


def report(name, ok, detail=""):
    global failures
    if not ok:
        failures += 1
    print(f"{'PASS' if ok else 'FAIL'}: {name}" + (f" ({detail})" if detail else ""))
    return ok


def track_def(name):
    """Greaseweazle track definition for a density."""
    secs, bps, _, head = FORMATS[name]
    if name == "sd":
        return get_diskdef("atari.90").mk_track(CYL, head)
    d = IBMTrack_FixedDef("ibm.mfm")
    for k, v in [("secs", secs), ("bps", bps), ("rate", 260), ("rpm", 300),
                 ("iam", "no")]:
        d.add_param(k, str(v))
    d.finalise()
    return d.mk_track(CYL, head)


def complement(b):
    return bytes(x ^ 0xFF for x in b)


def check_enc_to_gw(tmp):
    for name, (secs, bps, order, head) in FORMATS.items():
        label = f"C encode -> GW decode [{name}]"
        base = tmp / f"enc_{name}"
        pulses = base.with_suffix(".pulses").read_bytes()
        sio = base.with_suffix(".sio").read_bytes()
        _, cyl, _, rev_ticks, npulses, _ = map(
            int, base.with_suffix(".meta").read_text().split())
        flux = Flux([float(rev_ticks)], [float(p) for p in pulses],
                    SAMPLE_FREQ, index_cued=True)
        track = track_def(name)
        track.decode_flux(flux, None)
        raw = sorted(track.raw.sectors, key=lambda s: s.start)
        report(label + " all sectors good CRC",
               len(raw) == secs and all(s.crc == 0 for s in raw)
               and track.nr_missing() == 0,
               f"{sum(s.crc == 0 for s in raw)}/{secs} found, "
               f"{track.nr_missing()} missing")
        ids_ok = all((s.idam.c, s.idam.h, s.idam.n) == (cyl, head, (bps // 128).bit_length() - 1)
                     for s in raw)
        report(label + " C/H/N ids", ids_ok)
        report(label + " DAM mark FB", all(s.dam.mark == 0xFB for s in raw))
        report(label + " physical sector order",
               [s.idam.r for s in raw] == order,
               "got " + ",".join(str(s.idam.r) for s in raw))
        by_id = {s.idam.r: s.dam.data for s in raw}
        data_ok = all(
            by_id.get(i + 1) == complement(sio[i * bps:(i + 1) * bps])
            for i in range(secs))
        report(label + " data is ones complement of SIO data", data_ok)


def check_gw_to_dec(tmp, exe):
    rnd = random.Random(12345)
    expected = {}
    for name, (secs, bps, _, head) in FORMATS.items():
        track = track_def(name)
        gw_data = bytes(rnd.randrange(256) for _ in range(secs * bps))
        track.set_img_track(gw_data)
        # set_img_track assigns data by sector ID order, as get_img_track returns
        assert bytes(track.get_img_track()) == gw_data
        master = track.master_track()
        bits = master.bits
        scale = SAMPLE_FREQ * master.time_per_rev / len(bits)
        pulses = bytearray()
        last = 0
        for i in bits.search(1):
            t = round(i * scale)
            pulses.append(t - last)  # raises if > 255
            last = t
        base = tmp / f"gw_{name}"
        base.with_suffix(".pulses").write_bytes(bytes(pulses))
        density = {"sd": 0, "ed": 1, "dd": 2, "dd1": 2}[name]
        base.with_suffix(".meta").write_text(f"{density} {CYL} {head}\n")
        expected[name] = gw_data
    subprocess.run([exe, "decode", str(tmp)], check=True)
    for name, (secs, bps, _, _head) in FORMATS.items():
        label = f"GW encode -> C decode [{name}]"
        out = (tmp / f"gw_{name}.out").read_bytes()
        validity, data = out[:secs], out[secs:]
        report(label + " all sectors decoded", all(validity) and len(validity) == secs,
               f"{sum(1 for v in validity if v)}/{secs}")
        want = complement(expected[name])
        report(label + " data is ones complement of GW data", data == want)


def main():
    exe = sys.argv[1] if len(sys.argv) > 1 else "./atari_gw_dump"
    with tempfile.TemporaryDirectory(prefix="atari_gw_") as d:
        tmp = pathlib.Path(d)
        subprocess.run([exe, "encode", str(tmp)], check=True)
        check_enc_to_gw(tmp)
        check_gw_to_dec(tmp, exe)
    print(f"{failures} failure(s)")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
