#!/usr/bin/env python3
"""Test harness for bbstream_cli.

Takes a real log, wraps it into packets the way the firmware does (arbitrary cut points are *harder* than the
firmware's, which cuts at frame ends), optionally damages the stream, feeds it to bbstream_cli in odd chunk
sizes (through a pipe, or a pty to exercise the termios code) and compares the decoded rows with the output of
the unmodified reference decoder (blackbox_decode).

Run: python3 test_bbstream.py [path/to/LOG.BFL]
"""
import os
import pty
import random
import subprocess
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from blackbox_packets import encode_packet, PAYLOAD_MAX, FLAG_HEADER, FLAG_FIRST, FLAG_LAST  # noqa: E402

CLI = os.environ.get("BBSTREAM_CLI", os.path.join(HERE, "obj", "bbstream_cli"))
REF = os.path.join(HERE, "obj", "blackbox_decode")
DEFAULT_LOG = os.path.join(HERE, "..", "..", "..", "logs-mockup", "LOG00043.BFL")


def HEADER_BYTES(data):
    pos = 0
    while data[pos:pos + 2] == b"H ":
        pos = data.index(b"\n", pos) + 1
    return pos


def slow_names(data):
    """names of the S-frame fields (they are stale after a lost S frame, so damaged streams cannot be exact there)"""
    for line in data[:HEADER_BYTES(data)].split(b"\n"):
        if line.startswith(b"H Field S name:"):
            return set(line.split(b":", 1)[1].decode().split(","))
    return set()


def read_csv(text_lines):
    names = [c.strip() for c in text_lines[0].split(",")]
    rows = [[int(c) for c in l.split(",")] for l in text_lines[1:] if l.strip()]
    return names, rows


def reference(path):
    out = subprocess.run([REF, "--stdout", path], capture_output=True, text=True, check=True).stdout
    return read_csv(out.splitlines())


def packetize(data, rng, max_len=PAYLOAD_MAX, header_every=0):
    """Cut at random places, like a flush at arbitrary points. Returns the list of packets.

    header_every > 0: like the firmware's blackbox_serial_header_interval, put a copy of the header into the
    header channel (own seq) after every that many data packets."""
    header = data[:HEADER_BYTES(data)]
    pkts, seq, hseq, i, count = [], 0, 0, 0, 0
    while i < len(data):
        n = rng.choice([max_len, max_len, rng.randint(1, max_len)])
        pkts.append(encode_packet(seq, data[i:i + n]))
        seq = (seq + 1) & 0xFF
        i += n
        count += 1
        if header_every and count % header_every == 0 and i > len(header):
            for k in range(0, len(header), max_len):
                flags = FLAG_HEADER | (FLAG_FIRST if k == 0 else 0) | (FLAG_LAST if k + max_len >= len(header) else 0)
                pkts.append(encode_packet(hseq, header[k:k + max_len], flags))
                hseq = (hseq + 1) & 0xFF
    return pkts


def header_packets(pkts, header_bytes):
    """number of leading packets that carry the header"""
    total = 0
    for i, p in enumerate(pkts):
        total += len(p) - 6
        if total >= header_bytes + 1:
            return i + 1
    return len(pkts)


def damage(pkts, rng, p_drop=0.0, p_flip=0.0, p_junk=0.0, keep_first=0):
    """keep_first: leave the first packets alone (the header: without it the log cannot be decoded at all)."""
    out = list(pkts[:keep_first])
    for p in pkts[keep_first:]:
        if rng.random() < p_drop:
            continue
        if rng.random() < p_flip:
            b = bytearray(p)
            b[rng.randrange(len(b))] ^= 1 << rng.randrange(8)
            p = bytes(b)
        if rng.random() < p_junk:
            out.append(bytes(rng.randrange(256) for _ in range(rng.randint(1, 20))))
        out.append(p)
    return b"".join(out)


def flip(packet):
    return packet[:-1] + bytes([packet[-1] ^ 0xFF])


def run_cli(stream, rng, use_pty=False, args=()):
    """stream: bytes, or a list of byte strings that are fed with a pause of 0.7 s in between"""
    csv_path = tempfile.mktemp(suffix=".csv")
    chunks = []
    for part in (stream if isinstance(stream, list) else [stream]):
        if chunks:
            chunks.append(None)  # pause
        i = 0
        while i < len(part):
            n = rng.choice([100, 1000, 4096] if use_pty else [1, 3, 16, 100, 1000, 4096])
            chunks.append(part[i:i + n])
            i += n
    if use_pty:
        master, slave = pty.openpty()
        import termios
        attr = termios.tcgetattr(slave)
        attr[3] &= ~(termios.ECHO | termios.ICANON)
        termios.tcsetattr(slave, termios.TCSANOW, attr)
        proc = subprocess.Popen([CLI, "-d", os.ttyname(slave), "-b", "115200", "-o", csv_path, "-p", "0"],
                                stderr=subprocess.PIPE, text=True)
        def feed():
            time.sleep(0.3)  # the CLI flushes the input when it opens the port
            for c in chunks:
                if c is None:
                    time.sleep(0.7)
                    continue
                os.write(master, c)
                time.sleep(len(c) / 800e3)  # a serial port cannot be stopped: stay below the decoder's speed
            time.sleep(1.0)
            proc.send_signal(2)  # SIGINT: the reader stops at the end of a tty
        t = threading.Thread(target=feed)
        t.start()
        err = proc.stderr.read()
        t.join()
        proc.wait()
        os.close(master)
        os.close(slave)
    else:
        proc = subprocess.Popen([CLI, "-f", "-", "-o", csv_path, "-p", "0", *args], stdin=subprocess.PIPE,
                                stderr=subprocess.PIPE, text=False)
        def feed():
            for c in chunks:
                if c is None:
                    proc.stdin.flush()
                    time.sleep(0.7)
                    continue
                try:
                    proc.stdin.write(c)
                except BrokenPipeError:
                    break
            proc.stdin.close()
        t = threading.Thread(target=feed)
        t.start()
        err = proc.stderr.read().decode()
        t.join()
        proc.wait()
    assert "Sanitizer" not in err and "runtime error" not in err, err
    with open(csv_path) as f:
        names, rows = read_csv(f.read().splitlines())
    os.unlink(csv_path)
    return names, rows, err.strip().splitlines()[-1] if err.strip() else ""


def compare(ref, got, allow_missing, ignore=()):
    """Every decoded row must equal the reference row with the same iteration. Returns (matched, missing)."""
    rnames, rrows = ref
    gnames, grows = got
    assert gnames == rnames, f"column names differ:\n{gnames[:5]}\n{rnames[:5]}"
    by_it = {r[0]: r for r in rrows}
    bad = 0
    seen = set()
    prev = -1
    for r in grows:
        assert r[0] > prev, f"iterations not increasing at {r[0]}"
        prev = r[0]
        seen.add(r[0])
        ref_row = by_it.get(r[0])
        if ref_row is None or any(a != b for a, b, nm in zip(ref_row, r, rnames) if nm not in ignore):
            bad += 1
    missing = [it for it in by_it if it not in seen]
    assert bad == 0, f"{bad} decoded rows differ from the reference"
    assert allow_missing or len(missing) <= 3, f"{len(missing)} rows missing"
    return len(grows), len(missing)


def stress(log, n):
    """Many random damage patterns; no decoded row may ever differ from the reference."""
    data = open(log, "rb").read()
    ref = reference(log)
    by_it = {r[0]: r for r in ref[1]}
    slow = slow_names(data)
    total = bad_total = 0
    kinds = [dict(p_flip=0.02), dict(p_drop=0.05), dict(p_drop=0.05, p_flip=0.05, p_junk=0.05), dict(p_junk=0.2)]
    for seed in range(n):
        rng = random.Random(100 + seed)
        repeats = seed % 2 == 1  # with repeats the header may be damaged as well
        pk = packetize(data, rng, header_every=40 if repeats else 0)
        stream = damage(pk, rng, keep_first=0 if repeats else header_packets(pk, HEADER_BYTES(data)), **kinds[seed % 4])
        names, rows, status = run_cli(stream, rng)
        if names == [""]:
            names = ref[0]
        bad = [r for r in rows if by_it.get(r[0]) is None
               or any(a != b for a, b, nm in zip(by_it[r[0]], r, ref[0]) if nm not in slow)]
        total += len(rows)
        bad_total += len(bad)
        if bad:
            print(f"seed {seed} {kinds[seed % 4]}: {len(bad)} wrong rows | {status}")
    print(f"stress: {n} runs, {total} rows decoded, {bad_total} wrong")
    assert bad_total == 0


def main():
    if len(sys.argv) > 2 and sys.argv[1] == "--stress":
        return stress(sys.argv[3] if len(sys.argv) > 3 else DEFAULT_LOG, int(sys.argv[2]))
    log = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_LOG
    data = open(log, "rb").read()
    ref = reference(log)
    print(f"reference: {len(ref[1])} rows, {len(ref[0])} columns")
    rng = random.Random(1234)
    slow = slow_names(data)

    for name, kwargs, use_pty, allow_missing in [
        ("clean pipe", {}, False, False),
        ("clean pty", {}, True, False),
        ("flipped bits 1%", dict(p_flip=0.01), False, True),
        ("dropped packets 2%", dict(p_drop=0.02), False, True),
        ("dropped+flipped+junk", dict(p_drop=0.03, p_flip=0.03, p_junk=0.03), False, True),
    ]:
        pk = packetize(data, rng)
        stream = damage(pk, rng, keep_first=header_packets(pk, HEADER_BYTES(data)), **kwargs)
        names, rows, status = run_cli(stream, rng, use_pty)
        matched, missing = compare(ref, (names, rows), allow_missing, () if not allow_missing else slow)
        print(f"{name:24s} ok: {matched} rows exact, {missing} of reference missing | {status}")
        if not allow_missing:
            assert matched >= len(ref[1]) - 3
        else:
            assert matched > 0.3 * len(ref[1]), "too little survived"

    # with the header repeated, a damaged header, a late start and a damaged repeat are all survivable
    for name, build in [
        ("clean + header repeats", lambda pk: b"".join(pk)),
        ("damaged header + repeats", lambda pk: b"".join(pk[:3] + [flip(pk[3])] + pk[4:])),
        ("late start + repeats", lambda pk: b"".join(pk)[1000:]),
        # the initial header is damaged and 1% of the repeat packets are lost, so about half of the copies are unusable
        ("damaged hdr, repeats -1%", lambda pk: b"".join(
            [p for i, p in enumerate(pk[:3] + [flip(pk[3])] + pk[4:]) if not (p[3] & 0x80 and rng.random() < 0.01)])),
    ]:
        pk = packetize(data, rng, header_every=40)
        names, rows, status = run_cli(build(pk), rng)
        matched, missing = compare(ref, (names, rows), True, slow)
        print(f"{name:24s} ok: {matched} rows exact, {missing} of reference missing | {status}")
        assert matched > 0.3 * len(ref[1]), "too little survived"
        assert "used" in status

    # a second log whose own header is lost: its repeats carry the same header, so only the pause tells them apart
    pk_a = packetize(data, rng, header_every=40)
    pk_b = packetize(data, rng, header_every=40)
    b_stream = b"".join(pk_b[header_packets(pk_b, HEADER_BYTES(data)):])
    names, rows, status = run_cli([b"".join(pk_a), b_stream], rng, args=("-g", "300"))
    cuts = [i for i in range(1, len(rows)) if rows[i][0] <= rows[i - 1][0]]
    assert len(cuts) == 1, f"expected two logs, iterations restart {len(cuts)} times"
    for part in (rows[:cuts[0]], rows[cuts[0]:]):
        compare(ref, (names, part), True)
    n_b = len(rows) - cuts[0]
    print(f"{'2nd log, header lost':24s} ok: {cuts[0]} + {n_b} rows exact | {status}")
    assert cuts[0] >= len(ref[1]) - 3 and n_b > 0.8 * len(ref[1])
    assert "logs 2" in status

    # without repeats a damaged header or a late start makes the log undecodable, but must not crash anything
    pk = packetize(data, rng)
    names, rows, status = run_cli(b"".join(pk[:3] + [flip(pk[3])] + pk[4:]), rng)
    print(f"{'damaged header':24s} ok: {len(rows)} rows (expected 0) | {status}")
    assert len(rows) == 0 or names == [""]

    # joining the stream late, in the middle of a packet
    pk = packetize(data, rng)
    names, rows, status = run_cli(b"".join(pk)[1000:], rng)
    print(f"{'late start':24s} ok: {len(rows)} rows (expected 0) | {status}")
    print("all good")


if __name__ == "__main__":
    main()
