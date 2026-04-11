# Bad-frame Skip & Threshold — Design

Date: 2026-04-11
Component: `mcap2vid export`
Test fixture: `/tmp/3fb1af532be00cbb738322ca81b46165_09_22_32_third_view.mcap`
Test topic: `/third_view/camera/image_raw/compressed`

## Problem

Some MCAP recordings contain occasional malformed compressed-image messages — e.g. a 474,192-byte JPEG with no SOI marker but a valid trailing `FF D9`, ~1 in 8912 frames. The upstream recorder has been cleared: the bad bytes come straight from the GStreamer appsink buffer and are faithfully stored in the MCAP. `mcap2vid export` currently aborts the whole run on the first decode error (`decode_frames_parallel` uses `?` on the first failing frame), turning a single corrupt frame into a failed 8911-frame export.

## Goal

Skip bad frames and continue, with:

- timeline preserved (output MP4 has the same frame count and duration as the input range)
- clear per-frame and summary logging to stderr
- a safety threshold so catastrophically broken files still fail loudly
- no change to the existing success path, the FFmpeg pipe, or the FTSS timestamp atom

## Non-goals

- Repairing bad JPEGs (e.g. synthesising a missing SOI)
- Any change to `list`, `verify`, or `metadata` subcommands
- Fixing the upstream recorder — that investigation lives in the user's Python recorder, not this repo
- Per-frame bad-frame reports written to a file (stderr is sufficient)

## Architecture

The change is localized to two files:

### `src/decoder.rs`

Introduce `DecodeOutcome`:

```rust
pub enum DecodeOutcome {
    Ok(DecodedFrame),
    Bad(BadFrame),
}

pub struct BadFrame {
    pub sequence: u64,
    pub timestamp: f64,
    pub size_bytes: usize,
    pub reason: String, // ≤ 120 chars, derived from decoder error
}
```

`decode_frames_parallel` changes signature to `fn(&[VideoFrame]) -> Vec<DecodeOutcome>` — it never short-circuits on a decode error. Real I/O errors (e.g. a broken MCAP chunk) still bubble up from upstream code; those are not "bad frames".

The single-frame `decode_frame(&VideoFrame) -> Result<DecodedFrame>` signature is **unchanged**. It's still called from `main.rs` lines 201 and 240 during width/height sniffing of the first frame. If that specific first frame happens to be bad, the export still aborts early with a clear error — sniffing can't guess dimensions from a corrupt payload. Implementation note: consider iterating past leading bad frames during sniff (try frames 0..N until one decodes) so a single bad first frame doesn't kill the run; decided at implementation time based on how cleanly it fits into `extract_first_frame`.

Each worker runs inside `std::panic::catch_unwind` so a panic from the `image` crate on hostile input becomes `BadFrame { reason: "panic: …" }` instead of killing the entire rayon pool and the export with it.

`size_bytes` is the compressed/raw payload length (`data.len()` in the matching branch of `FrameData`).

Outcomes are sorted by `sequence` before returning, the same way decoded frames are sorted today, so ordering is preserved across good and bad outcomes.

### `src/main.rs` — export loop

A new helper owns the "repeat previous" state machine:

```rust
struct BadFrameState {
    last_good: Option<DecodedFrame>,
    bad_count: usize,
    total_expected: usize,
    max_abs: usize,
    max_pct: f64,
}
```

For each outcome in each batch:

1. `Ok(frame)` → write to encoder, push timestamp, cache clone as `last_good`, `pb.inc(1)`.
2. `Bad(b)` with `last_good = Some(prev)` → log per-frame line, clone `prev`, set its `timestamp` and `sequence` to the bad frame's values, write to encoder, push the bad frame's timestamp, `pb.inc(1)`, `bad_count += 1`, then check threshold.
3. `Bad(b)` with `last_good = None` (bad frame appears before any good one) → log per-frame line with `action=drop`, do **not** write anything, do **not** push timestamp, `pb.inc(1)`, `bad_count += 1`, then check threshold.

Threshold check: if `bad_count > max_abs && (bad_count as f64) / (total_expected as f64) > max_pct / 100.0`, return an `anyhow::bail!` with the summary text; the existing `encoder.finish()` + error-propagation path kills the FFmpeg subprocess and ensures no partial MP4 is finalized.

At the end of a successful export the summary line is printed unconditionally — even if zero bad frames — so runs are self-documenting.

## Detection scope

`BadFrame` is produced for any `decode_frame` failure. Today that covers:

- `image::load_from_memory*` returning `ImageError` (the user's no-SOI JPEG case)
- `decode_raw` returning `anyhow!("Insufficient image data: …")`
- `decode_raw` returning `anyhow!("RGB conversion error: …")`
- `decode_raw` returning `anyhow!("Unsupported image encoding: …")`

The "unsupported encoding" case is lumped in with bad frames deliberately: if the whole topic has the wrong encoding, the threshold will trip on the second frame, which produces a clearer error than an opaque first-frame abort. A one-off frame with a weird encoding is rare but survivable.

## Logging

All output goes to **stderr** (`eprintln!` or the `Log` helper), never stdout — `-o -` streams the MP4 to stdout and must remain clean.

Per-frame log line (emitted as each bad frame is encountered):

```
[bad-frame] seq=12345 ts=1775035863.543123 size=474192 reason="jpeg decode: invalid marker" action=repeat-previous
```

Fields:

- `seq` — `VideoFrame.sequence`
- `ts` — ROS2 timestamp, 6 decimals
- `size` — bytes in the original payload
- `reason` — decoder error `.to_string()`, truncated to 120 chars, newlines stripped
- `action` — `repeat-previous` or `drop`

End-of-run summary (always printed for an export, including zero-bad runs):

```
[bad-frame] summary: 1 bad / 8912 total (0.011%), threshold=10 abs / 1.00%
```

Abort message on threshold exceeded (error path, causes non-zero exit):

```
aborting: bad-frame threshold exceeded — 47 bad / 4000 scanned (1.18%) > 1.00% after 10 free
```

## Threshold (option C)

Two CLI flags on `Commands::Export`:

```rust
/// Maximum bad frames always tolerated regardless of percentage
#[arg(long, default_value = "10")]
max_bad_frames: usize,

/// Maximum bad frames as percent of total (beyond --max-bad-frames)
#[arg(long, default_value = "1.0")]
max_bad_frames_pct: f64,
```

Semantics:

- The first `max_bad_frames` bad frames never trip the threshold (protects short exports from 1 corrupt blob).
- Beyond that, `bad_count / total_expected > max_bad_frames_pct / 100.0` aborts the export.
- `total_expected` is known up-front: `scan.count` for local, `frames.len()` for the S3 path.
- Set both to zero to enforce "no bad frames allowed" (used by the threshold integration test).

Defaults rationale: the fixture case is ~1 in 8912 (≈0.011%) and should sail through. A truly broken file (>1% bad) probably shouldn't silently produce a video.

## CLI

Only `Commands::Export` gains flags:

```
--max-bad-frames <N>          default 10
--max-bad-frames-pct <P>      default 1.0
```

No new subcommand. `list`, `verify`, and `metadata` are untouched.

## Testing

### Unit tests (`src/decoder.rs`)

- Valid 2×2 `rgb8` raw frame → `DecodeOutcome::Ok`
- Truncated `rgb8` raw frame → `DecodeOutcome::Bad` with `reason` containing `"Insufficient image data"`
- Garbage JPEG bytes (compressed path) → `DecodeOutcome::Bad` with `reason` containing `"jpeg"` or the `image` crate error string
- Outcomes are sorted by `sequence` across a mix of good and bad frames

### Panic-safety test

A frame whose decode path is forced to panic (test-only hook or a known-panicking input for the `image` crate version in `Cargo.lock`) must produce `DecodeOutcome::Bad { reason: "panic: …" }` and must not poison the rayon pool. Implementation step will confirm which approach is reliable for the pinned `image` version.

### Integration test (manual, documented in this spec)

Fixture: `/tmp/3fb1af532be00cbb738322ca81b46165_09_22_32_third_view.mcap`
Topic: `/third_view/camera/image_raw/compressed`

Run:

```
cargo run --release -- export \
  -i /tmp/3fb1af532be00cbb738322ca81b46165_09_22_32_third_view.mcap \
  -t /third_view/camera/image_raw/compressed \
  -o /tmp/out.mp4
```

Expected:

- Exit code 0
- stderr contains at least one `[bad-frame] seq=… action=repeat-previous` line
- stderr contains the `[bad-frame] summary:` line
- `mcap2vid verify -i /tmp/out.mp4` reports the full expected frame count
- `ffprobe -v error -count_frames -select_streams v:0 -show_entries stream=nb_read_frames /tmp/out.mp4` matches the MCAP topic frame count

### Threshold test

Same command with `--max-bad-frames 0 --max-bad-frames-pct 0.0`:

- Exit code non-zero
- stderr contains the `aborting: bad-frame threshold exceeded` message
- No usable MP4 is produced (or any partial file is clearly incomplete)

## Edge cases

- **Bad frame at the very start** — no `last_good` to repeat, logged with `action=drop`, counted toward threshold, no timestamp pushed; the FTSS array stays consistent with the frames actually in the MP4.
- **Consecutive bad frames** — each one repeats the same `last_good`; the viewer sees a multi-frame freeze; each counts separately toward the threshold.
- **All frames bad** — threshold trips (unless user set defaults to zero); export aborts with a useful message rather than producing an empty MP4.
- **First frame bad during width/height sniffing** — covered in the architecture section; either aborts early with a clear message or (implementation choice) iterates past leading bad frames to find one it can decode.
- **`-o -` stdout mode** — per-frame and summary logs already go to stderr, so streaming output is unaffected. FTSS embedding is already skipped in stdout mode; that stays.
- **S3 path** — `FrameSource::S3` and `FrameSource::Local` both feed the same batch loop, so the new logic covers both without duplication.

## Files touched

- `src/decoder.rs` — add `DecodeOutcome` / `BadFrame`, refactor `decode_frames_parallel`, add `catch_unwind`, unit tests
- `src/cli.rs` — two new flags on `Commands::Export`
- `src/main.rs` — plumb the flags, introduce `BadFrameState`, rewire the two batch loops (`FrameSource::S3` and `FrameSource::Local`), summary print
- No changes to `mcap_reader`, `mcap_index`, `s3_reader`, `encoder`, `timestamp`, `ros2_msgs`
