# Bad-frame Skip & Threshold Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `mcap2vid export` skip bad (undecodable) image frames by repeating the previous good frame, log every bad frame to stderr, and abort with a threshold error if bad-frame count exceeds a configurable absolute + percentage budget.

**Architecture:** Refactor `decoder::decode_frames_parallel` to return `Vec<DecodeOutcome>` (never short-circuits on decode errors), wrap each worker in `std::panic::catch_unwind` to survive codec panics, and add a `BadFrameState` helper in `main.rs` that drives the per-frame repeat/log/threshold logic shared by both the S3 and local batch loops. Two new CLI flags (`--max-bad-frames`, `--max-bad-frames-pct`) expose the threshold knobs.

**Tech Stack:** Rust 2021 edition, `anyhow`, `rayon`, `clap` v4, `image` v0.25. No new dependencies.

**Design spec:** `docs/superpowers/specs/2026-04-11-bad-frame-skip-design.md`

---

## File Structure

**Modified files:**

- `src/decoder.rs` — Add `DecodeOutcome` + `BadFrame`, rewrite `decode_frames_parallel` to never short-circuit, wrap workers in `catch_unwind`, keep `decode_frame` signature unchanged. Add unit tests.
- `src/cli.rs` — Two new flags on `Commands::Export`.
- `src/main.rs` — Plumb flags through `export_video`, introduce `BadFrameState` struct, rewire both batch loops (`FrameSource::S3` and `FrameSource::Local`) through the new helper, extend summary print.

**Untouched:** `mcap_reader.rs`, `mcap_index.rs`, `s3_reader.rs`, `encoder.rs`, `timestamp.rs`, `ros2_msgs.rs`.

**Test location:** Unit tests live in `#[cfg(test)] mod tests` inside `src/decoder.rs`, matching the existing pattern in `src/ros2_msgs.rs:274`.

---

## Task 1: Introduce `DecodeOutcome` and `BadFrame` types

**Files:**
- Modify: `src/decoder.rs`

- [ ] **Step 1: Add the new types above `decode_frame`**

Insert this block at `src/decoder.rs` line 16 (just above `pub fn decode_frame`):

```rust
/// Outcome of decoding a single frame.
///
/// `Ok` carries a fully decoded frame ready for the encoder.
/// `Bad` carries enough metadata to log the failure and count it toward the
/// bad-frame threshold without aborting the export.
#[derive(Debug)]
pub enum DecodeOutcome {
    Ok(DecodedFrame),
    Bad(BadFrame),
}

/// Metadata describing a frame that failed to decode.
#[derive(Debug, Clone)]
pub struct BadFrame {
    pub sequence: u64,
    pub timestamp: f64,
    pub size_bytes: usize,
    pub reason: String,
}

impl BadFrame {
    /// Truncate a decoder error message to a single ≤120-char line suitable for logging.
    fn sanitize_reason(raw: &str) -> String {
        let one_line = raw.replace(['\n', '\r'], " ");
        if one_line.len() <= 120 {
            one_line
        } else {
            let mut s = one_line;
            s.truncate(120);
            s
        }
    }
}

/// Return the size in bytes of the underlying payload for a frame
/// (compressed bytes for compressed frames, raw row data for raw frames).
fn payload_size(frame: &VideoFrame) -> usize {
    match &frame.data {
        FrameData::Compressed { data, .. } => data.len(),
        FrameData::Raw { data, .. } => data.len(),
    }
}
```

- [ ] **Step 2: Build a release check to confirm the new types compile**

Run: `cargo check`
Expected: compiles with warnings only for unused `DecodeOutcome`, `BadFrame`, `payload_size` (dead-code warnings are fine — the next task uses them).

- [ ] **Step 3: Commit**

```bash
git add src/decoder.rs
git commit -m "refactor(decoder): add DecodeOutcome and BadFrame types"
```

---

## Task 2: Rewrite `decode_frames_parallel` to return `Vec<DecodeOutcome>` and catch panics

**Files:**
- Modify: `src/decoder.rs:187-201`

- [ ] **Step 1: Add the failing unit test first**

Append this to the end of `src/decoder.rs`:

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use crate::mcap_reader::{FrameData, VideoFrame};

    fn raw_frame(seq: u64, width: u32, height: u32, data: Vec<u8>) -> VideoFrame {
        VideoFrame {
            timestamp: seq as f64,
            sequence: seq,
            data: FrameData::Raw {
                width,
                height,
                encoding: "rgb8".to_string(),
                step: width * 3,
                data,
            },
        }
    }

    fn compressed_frame(seq: u64, data: Vec<u8>) -> VideoFrame {
        VideoFrame {
            timestamp: seq as f64,
            sequence: seq,
            data: FrameData::Compressed {
                format: "jpeg".to_string(),
                data,
            },
        }
    }

    #[test]
    fn decode_frames_parallel_returns_outcomes_for_mixed_input() {
        // Good: 2x2 rgb8 = 12 bytes
        let good = raw_frame(0, 2, 2, vec![0u8; 12]);
        // Bad: truncated rgb8 (need 12, got 3)
        let truncated = raw_frame(1, 2, 2, vec![0u8; 3]);
        // Bad: garbage JPEG
        let garbage = compressed_frame(2, vec![0xFF, 0xD8, 0xFF, 0xE0, 0xDE, 0xAD]);

        let outcomes = decode_frames_parallel(&[good, truncated, garbage]);

        assert_eq!(outcomes.len(), 3);
        match &outcomes[0] {
            DecodeOutcome::Ok(f) => {
                assert_eq!(f.sequence, 0);
                assert_eq!(f.width, 2);
                assert_eq!(f.height, 2);
            }
            DecodeOutcome::Bad(b) => panic!("expected Ok, got Bad: {}", b.reason),
        }
        match &outcomes[1] {
            DecodeOutcome::Bad(b) => {
                assert_eq!(b.sequence, 1);
                assert_eq!(b.size_bytes, 3);
                assert!(
                    b.reason.to_lowercase().contains("insufficient")
                        || b.reason.to_lowercase().contains("image data"),
                    "unexpected reason: {}",
                    b.reason
                );
            }
            DecodeOutcome::Ok(_) => panic!("expected Bad for truncated raw frame"),
        }
        match &outcomes[2] {
            DecodeOutcome::Bad(b) => {
                assert_eq!(b.sequence, 2);
                assert_eq!(b.size_bytes, 6);
                assert!(!b.reason.is_empty());
            }
            DecodeOutcome::Ok(_) => panic!("expected Bad for garbage JPEG"),
        }
    }

    #[test]
    fn decode_frames_parallel_sorts_outcomes_by_sequence() {
        let frames = vec![
            raw_frame(2, 1, 1, vec![0u8; 3]),
            raw_frame(0, 1, 1, vec![0u8; 3]),
            raw_frame(1, 1, 1, vec![0u8; 0]), // bad
        ];
        let outcomes = decode_frames_parallel(&frames);
        assert_eq!(outcomes.len(), 3);
        let seqs: Vec<u64> = outcomes
            .iter()
            .map(|o| match o {
                DecodeOutcome::Ok(f) => f.sequence,
                DecodeOutcome::Bad(b) => b.sequence,
            })
            .collect();
        assert_eq!(seqs, vec![0, 1, 2]);
    }

    #[test]
    fn bad_frame_reason_is_truncated_and_single_line() {
        let long = "a".repeat(500) + "\nmore";
        let out = BadFrame::sanitize_reason(&long);
        assert_eq!(out.len(), 120);
        assert!(!out.contains('\n'));
    }
}
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `cargo test --lib decoder::tests`
Expected: compilation error (return type of `decode_frames_parallel` is still `Result<Vec<DecodedFrame>>`, test uses the new `DecodeOutcome` enum).

- [ ] **Step 3: Replace `decode_frames_parallel`**

In `src/decoder.rs`, replace the entire function at lines 187-201:

```rust
/// Decode multiple frames in parallel using rayon.
///
/// Returns one `DecodeOutcome` per input frame, preserving sequence order.
/// Decode errors and panics inside the `image` crate are captured as
/// `DecodeOutcome::Bad` so a single corrupt frame cannot abort the whole export.
pub fn decode_frames_parallel(frames: &[VideoFrame]) -> Vec<DecodeOutcome> {
    let mut outcomes: Vec<DecodeOutcome> = frames
        .par_iter()
        .map(|f| {
            let seq = f.sequence;
            let ts = f.timestamp;
            let size = payload_size(f);

            let caught =
                std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| decode_frame(f)));

            match caught {
                Ok(Ok(decoded)) => DecodeOutcome::Ok(decoded),
                Ok(Err(err)) => DecodeOutcome::Bad(BadFrame {
                    sequence: seq,
                    timestamp: ts,
                    size_bytes: size,
                    reason: BadFrame::sanitize_reason(&err.to_string()),
                }),
                Err(panic) => {
                    let msg = if let Some(s) = panic.downcast_ref::<&str>() {
                        format!("panic: {}", s)
                    } else if let Some(s) = panic.downcast_ref::<String>() {
                        format!("panic: {}", s)
                    } else {
                        "panic: (non-string payload)".to_string()
                    };
                    DecodeOutcome::Bad(BadFrame {
                        sequence: seq,
                        timestamp: ts,
                        size_bytes: size,
                        reason: BadFrame::sanitize_reason(&msg),
                    })
                }
            }
        })
        .collect();

    outcomes.sort_by_key(|o| match o {
        DecodeOutcome::Ok(f) => f.sequence,
        DecodeOutcome::Bad(b) => b.sequence,
    });

    outcomes
}
```

Note: `decode_frame` (single-frame) signature stays unchanged — it is still `Result<DecodedFrame>` and still used by the width/height sniff path in `main.rs`.

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test --lib decoder::tests`
Expected: all three tests pass.

- [ ] **Step 5: Run full build to confirm `main.rs` is now broken at the call sites (expected — next task fixes it)**

Run: `cargo check`
Expected: compilation errors in `src/main.rs` at the two `decode_frames_parallel` call sites (approximately lines 318 and 328) because the return type is no longer `Result`. This is expected — Task 3 rewires them.

- [ ] **Step 6: Commit**

```bash
git add src/decoder.rs
git commit -m "refactor(decoder): return DecodeOutcome and catch panics in parallel decode

Decode errors and panics in the image crate no longer abort the whole
export — each frame gets its own DecodeOutcome so bad frames can be
handled by the export loop."
```

---

## Task 3: Add CLI flags for threshold

**Files:**
- Modify: `src/cli.rs:22-50` (the `Commands::Export` variant)

- [ ] **Step 1: Add the two flags**

In `src/cli.rs`, find the `Commands::Export` variant and add two new fields after `crf` (current line 49). The final variant should look like:

```rust
    /// Export video from MCAP to MP4
    Export {
        /// Input MCAP file path or s3c:// URL
        #[arg(short, long)]
        input: String,

        /// Video topic to extract (e.g., /camera/image_raw)
        #[arg(short, long)]
        topic: String,

        /// Output MP4 file path, or "-" for stdout (zero disk writes)
        #[arg(short, long)]
        output: String,

        /// Suffix appended to the output filename (e.g., "ego" produces name_ego.mp4)
        #[arg(short, long)]
        suffix: Option<String>,

        /// Number of threads for parallel processing (0 = auto-detect)
        #[arg(long, default_value = "0")]
        threads: usize,

        /// FFmpeg encoding preset (ultrafast, fast, medium, slow)
        #[arg(long, default_value = "medium")]
        preset: String,

        /// Constant Rate Factor for quality (0-51, lower = better quality)
        #[arg(long, default_value = "18")]
        crf: u8,

        /// Maximum bad frames always tolerated regardless of percentage.
        /// First N decode failures never trip the threshold — set to 0 to
        /// enforce the percentage immediately.
        #[arg(long, default_value = "10")]
        max_bad_frames: usize,

        /// Maximum bad frames as percent of total (beyond --max-bad-frames).
        /// When `bad_count > --max-bad-frames` and `bad_count / total > P%`,
        /// the export aborts with a non-zero exit code.
        #[arg(long, default_value = "1.0")]
        max_bad_frames_pct: f64,
    },
```

- [ ] **Step 2: Confirm it parses**

Run: `cargo check`
Expected: errors in `main.rs` where the `Commands::Export` match arm binds the old fields — new fields are missing. This is expected; Task 4 fixes `main.rs`.

- [ ] **Step 3: Commit**

```bash
git add src/cli.rs
git commit -m "feat(cli): add --max-bad-frames and --max-bad-frames-pct to export"
```

---

## Task 4: Introduce `BadFrameState` helper and rewire the export loop

**Files:**
- Modify: `src/main.rs:17` (import line)
- Modify: `src/main.rs:31-47` (the `Commands::Export` match arm and `export_video` call)
- Modify: `src/main.rs:154-369` (the whole `export_video` function body — specifically the batch loops at ~315-337 and the summary at ~342-366)

- [ ] **Step 1: Update the `decoder` import**

At `src/main.rs:17`, replace:

```rust
use decoder::{decode_frame, decode_frames_parallel};
```

with:

```rust
use decoder::{decode_frame, decode_frames_parallel, BadFrame, DecodeOutcome, DecodedFrame};
```

- [ ] **Step 2: Add the `BadFrameState` helper near the top of `main.rs`**

Insert this block just above `fn export_video(` (currently around line 154, immediately after the `Logger` impl that ends at line 84 but before `fn handle_metadata`). Place it right before `fn export_video`:

```rust
/// Tracks bad-frame accounting for a single export run and enforces the
/// threshold configured by `--max-bad-frames` / `--max-bad-frames-pct`.
struct BadFrameState {
    last_good: Option<DecodedFrame>,
    bad_count: usize,
    total_expected: usize,
    max_abs: usize,
    max_pct: f64,
}

impl BadFrameState {
    fn new(total_expected: usize, max_abs: usize, max_pct: f64) -> Self {
        Self {
            last_good: None,
            bad_count: 0,
            total_expected,
            max_abs,
            max_pct,
        }
    }

    fn record_good(&mut self, frame: &DecodedFrame) {
        self.last_good = Some(frame.clone());
    }

    /// Produce a repeat frame (clone of `last_good` with the bad frame's
    /// timestamp and sequence), or `None` if no good frame has been seen yet.
    fn repeat_for(&self, bad: &BadFrame) -> Option<DecodedFrame> {
        self.last_good.as_ref().map(|prev| {
            let mut repeat = prev.clone();
            repeat.timestamp = bad.timestamp;
            repeat.sequence = bad.sequence;
            repeat
        })
    }

    fn note_bad(&mut self) {
        self.bad_count += 1;
    }

    /// Returns `true` if the threshold is exceeded. Uses the option-C rule:
    /// first `max_abs` bad frames are always tolerated; beyond that, the
    /// percentage gate applies.
    fn threshold_exceeded(&self) -> bool {
        if self.bad_count <= self.max_abs {
            return false;
        }
        if self.total_expected == 0 {
            return true;
        }
        let ratio = (self.bad_count as f64) / (self.total_expected as f64);
        ratio > (self.max_pct / 100.0)
    }

    fn summary_line(&self) -> String {
        let pct = if self.total_expected == 0 {
            0.0
        } else {
            (self.bad_count as f64) * 100.0 / (self.total_expected as f64)
        };
        format!(
            "[bad-frame] summary: {} bad / {} total ({:.3}%), threshold={} abs / {:.2}%",
            self.bad_count, self.total_expected, pct, self.max_abs, self.max_pct
        )
    }

    fn threshold_error(&self) -> anyhow::Error {
        let pct = if self.total_expected == 0 {
            0.0
        } else {
            (self.bad_count as f64) * 100.0 / (self.total_expected as f64)
        };
        anyhow::anyhow!(
            "aborting: bad-frame threshold exceeded — {} bad / {} scanned ({:.2}%) > {:.2}% after {} free",
            self.bad_count,
            self.total_expected,
            pct,
            self.max_pct,
            self.max_abs
        )
    }
}

/// Format and write a per-frame bad-frame log line to stderr.
fn log_bad_frame(bad: &BadFrame, action: &str) {
    eprintln!(
        "[bad-frame] seq={} ts={:.6} size={} reason=\"{}\" action={}",
        bad.sequence, bad.timestamp, bad.size_bytes, bad.reason, action
    );
}
```

- [ ] **Step 3: Update the `Commands::Export` match arm**

At `src/main.rs:31-47`, replace:

```rust
        Commands::Export {
            input,
            topic,
            output,
            suffix,
            threads,
            preset,
            crf,
        } => export_video(
            &input,
            &topic,
            &output,
            suffix.as_deref(),
            threads,
            &preset,
            crf,
        ),
```

with:

```rust
        Commands::Export {
            input,
            topic,
            output,
            suffix,
            threads,
            preset,
            crf,
            max_bad_frames,
            max_bad_frames_pct,
        } => export_video(
            &input,
            &topic,
            &output,
            suffix.as_deref(),
            threads,
            &preset,
            crf,
            max_bad_frames,
            max_bad_frames_pct,
        ),
```

- [ ] **Step 4: Update the `export_video` function signature**

At `src/main.rs:154`, replace the signature:

```rust
fn export_video(
    input: &str,
    topic: &str,
    output: &str,
    suffix: Option<&str>,
    threads: usize,
    preset: &str,
    crf: u8,
) -> Result<()> {
```

with:

```rust
fn export_video(
    input: &str,
    topic: &str,
    output: &str,
    suffix: Option<&str>,
    threads: usize,
    preset: &str,
    crf: u8,
    max_bad_frames: usize,
    max_bad_frames_pct: f64,
) -> Result<()> {
```

- [ ] **Step 5: Replace the batch-processing loop**

Find the block at `src/main.rs:310-337` starting with:

```rust
    let mut timestamps = Vec::with_capacity(total);

    // Phase 3: Batched decode + encode
```

and ending with the closing brace of `match source { ... }` at approximately line 337. Replace that entire block with:

```rust
    let mut timestamps = Vec::with_capacity(total);
    let mut bad_state = BadFrameState::new(total, max_bad_frames, max_bad_frames_pct);

    // Phase 3: Batched decode + encode
    // S3: frames already in memory, chunk them.
    // Local: stream from mmap — only one batch of compressed data in memory at a time.
    //
    // Decode errors no longer short-circuit — each outcome is handled individually:
    //   Ok   → write, push timestamp, cache as last_good
    //   Bad  → log, repeat last_good (or drop if no good frame yet), check threshold
    let mut process_batch = |batch: &[VideoFrame],
                             encoder: &mut FfmpegEncoder,
                             bad_state: &mut BadFrameState,
                             timestamps: &mut Vec<f64>,
                             pb: &ProgressBar|
     -> Result<()> {
        let outcomes = decode_frames_parallel(batch);
        for outcome in outcomes {
            match outcome {
                DecodeOutcome::Ok(frame) => {
                    encoder.write_frame(&frame)?;
                    timestamps.push(frame.timestamp);
                    bad_state.record_good(&frame);
                    pb.inc(1);
                }
                DecodeOutcome::Bad(bad) => {
                    if let Some(repeat) = bad_state.repeat_for(&bad) {
                        log_bad_frame(&bad, "repeat-previous");
                        encoder.write_frame(&repeat)?;
                        timestamps.push(bad.timestamp);
                    } else {
                        log_bad_frame(&bad, "drop");
                    }
                    bad_state.note_bad();
                    pb.inc(1);
                    if bad_state.threshold_exceeded() {
                        return Err(bad_state.threshold_error());
                    }
                }
            }
        }
        Ok(())
    };

    match source {
        FrameSource::S3(frames) => {
            for batch in frames.chunks(BATCH_SIZE) {
                process_batch(batch, &mut encoder, &mut bad_state, &mut timestamps, &pb)?;
            }
        }
        FrameSource::Local(reader) => {
            reader.process_frames_batched(topic, BATCH_SIZE, |batch| {
                process_batch(batch, &mut encoder, &mut bad_state, &mut timestamps, &pb)
            })?;
        }
    }
```

Rust note: `process_batch` is declared `let mut` because the closure mutably borrows `encoder` and other state through its parameters, but the closure itself is called via parameters so it does not need to be `FnMut` over outer state. If the borrow checker complains about the double-mut borrow via `FrameSource::Local`'s callback, inline the `process_batch` body into both match arms (DRY loses here; borrow-checker wins). That fallback: duplicate the inner `for outcome in outcomes { ... }` block directly inside each match arm.

- [ ] **Step 6: Add the summary print after `encoder.finish()`**

Find `src/main.rs:339-340`:

```rust
    pb.finish_with_message("Encoding complete");
    encoder.finish()?;
```

Immediately after `encoder.finish()?;` and before the existing `// Phase 4: Summary` block, add:

```rust
    eprintln!("{}", bad_state.summary_line());
```

- [ ] **Step 7: Build**

Run: `cargo check`
Expected: compiles cleanly. If the borrow checker complains about the closure borrowing through the `process_batch` parameter, use the inline fallback described in Step 5.

- [ ] **Step 8: Run the full test suite**

Run: `cargo test`
Expected: all previously passing tests still pass (decoder unit tests from Task 2, plus the existing `ros2_msgs::tests`).

- [ ] **Step 9: Commit**

```bash
git add src/main.rs
git commit -m "feat(export): skip bad frames, repeat previous, enforce threshold

Each decode outcome is now handled individually — good frames write
through, bad frames are logged to stderr and trigger a repeat of the
previous good frame. The run aborts if bad-frame count exceeds
--max-bad-frames + --max-bad-frames-pct.

Closes the case where a single corrupt JPEG in an 8912-frame recording
killed the whole export."
```

---

## Task 5: Integration test — run against the real broken fixture

**Files:**
- No code changes; this is an executable verification step.

Fixture: `/tmp/3fb1af532be00cbb738322ca81b46165_09_22_32_third_view.mcap`
Topic: `/third_view/camera/image_raw/compressed`

- [ ] **Step 1: Build release binary**

Run: `cargo build --release`
Expected: successful build, binary at `target/release/mcap2vid`.

- [ ] **Step 2: Run export against the broken fixture and capture stderr**

Run:
```bash
./target/release/mcap2vid export \
  -i /tmp/3fb1af532be00cbb738322ca81b46165_09_22_32_third_view.mcap \
  -t /third_view/camera/image_raw/compressed \
  -o /tmp/mcap2vid-badframe-test.mp4 \
  2> /tmp/mcap2vid-badframe-test.stderr
```

Expected:
- Exit code 0.
- `/tmp/mcap2vid-badframe-test.stderr` contains at least one line matching `^\[bad-frame\] seq=.*action=repeat-previous$`.
- `/tmp/mcap2vid-badframe-test.stderr` contains exactly one line matching `^\[bad-frame\] summary:`.
- `/tmp/mcap2vid-badframe-test.mp4` exists and is non-empty.

- [ ] **Step 3: Verify frame count matches**

Run:
```bash
./target/release/mcap2vid verify -i /tmp/mcap2vid-badframe-test.mp4 | head -20
ffprobe -v error -count_frames -select_streams v:0 \
  -show_entries stream=nb_read_frames -of csv=p=0 \
  /tmp/mcap2vid-badframe-test.mp4
```

Expected:
- `verify` reports a timestamp count equal to `total_scanned − <frames that hit action=drop>` (normally 0 — no leading bad frames in this fixture). For the fixture file the ffprobe frame count should equal the MCAP message count on that topic (call it `N`).
- `verify` shows timestamps in non-decreasing order and within the MCAP recording's known time range.

- [ ] **Step 4: Inspect the bad-frame log**

Run: `grep '^\[bad-frame\]' /tmp/mcap2vid-badframe-test.stderr`
Expected: one or more `seq=... action=repeat-previous` lines plus the terminating `summary:` line showing `1 bad / N total` (or whatever the real count is), confirming option B.

- [ ] **Step 5: Threshold trip test**

Run:
```bash
./target/release/mcap2vid export \
  -i /tmp/3fb1af532be00cbb738322ca81b46165_09_22_32_third_view.mcap \
  -t /third_view/camera/image_raw/compressed \
  -o /tmp/mcap2vid-badframe-threshold.mp4 \
  --max-bad-frames 0 \
  --max-bad-frames-pct 0.0
echo "exit=$?"
```

Expected:
- Non-zero exit code.
- Last stderr line contains `aborting: bad-frame threshold exceeded`.
- `/tmp/mcap2vid-badframe-threshold.mp4` either missing or clearly incomplete (ffmpeg subprocess killed on drop).

- [ ] **Step 6: No commit**

This task produces no code changes. If the integration test reveals a problem, the fix goes into a new follow-up commit inside the same task it broke.

---

## Self-Review Notes

Ran the three checks from writing-plans §Self-Review after drafting:

**Spec coverage**
- Architecture → Task 1, Task 2 (decoder changes) and Task 4 (main.rs)
- Detection scope → Task 2 catches both `Result` errors and panics via `catch_unwind`
- Logging format → Task 4 `log_bad_frame` + `BadFrameState::summary_line` + `threshold_error`
- Threshold (option C) → Task 4 `BadFrameState::threshold_exceeded` + Task 3 CLI flags
- CLI surface → Task 3
- Testing → Task 2 (unit), Task 5 (integration)
- Edge case: bad frame at start → Task 4 `repeat_for` returns `None`, logged as `action=drop`
- Edge case: stdout mode → Task 4 logs via `eprintln!` (always stderr), spec requirement satisfied

**Placeholder scan**
- No `TBD`, no `TODO`, no "implement later". Task 5 Step 2 says "or whatever the real count is" but only because the test fixture's true bad-frame count is unknown until we run it — the pass/fail gate is still concrete (`at least one` + `summary exists`).

**Type consistency**
- `DecodeOutcome` / `BadFrame` names used the same way in Task 2 (definition) and Task 4 (consumption).
- `BadFrameState::record_good(&DecodedFrame)`, `repeat_for(&BadFrame) -> Option<DecodedFrame>`, `note_bad()`, `threshold_exceeded()`, `summary_line()`, `threshold_error()` — same names in helper definition and call sites.
- `log_bad_frame(&BadFrame, &str)` — called only with `"repeat-previous"` and `"drop"`, matches spec logging format.
- `decode_frame` signature unchanged (spec requirement); `decode_frames_parallel` signature changes in Task 2 and Task 4 consumes the new return type.

---

## Verification Checklist

After all tasks are complete, the implementation is done when:

- [ ] `cargo build --release` succeeds with no warnings in the touched files
- [ ] `cargo test` passes all tests including the three new decoder unit tests
- [ ] Task 5 Step 2 produces exit 0 and a bad-frame log line
- [ ] Task 5 Step 3 confirms frame count parity between MCAP topic and output MP4
- [ ] Task 5 Step 5 confirms the threshold flag aborts the export
- [ ] `./target/release/mcap2vid export --help` shows the two new flags with defaults 10 and 1.0
- [ ] `git log --oneline` shows 4 feature commits (Tasks 1, 2, 3, 4) plus the earlier spec commit
