# V0.8 Stage 3 protocol: robust-navigation reference health

Preregistered 2026-11-06 before the Stage 3 implementation was executed. This
protocol follows `results/v08_stage2/blocked_followup.md`; it is **not** another
CUSUM threshold sweep and does not promote a detector.

## Question

Can an opt-in, shared batch/learned-stream whole-DR-reset-interval guard-reference
health state stop contaminated GNSS references from propagating into the guard,
while retaining fixed lateral/vertical NHC and documenting fallback reason codes?

The mechanism is a navigation robustness change. It is not asserted to close the
mandatory DETECTION gate, and its detection numbers are descriptive diagnostics,
not a new candidate selection.

## Frozen mechanism semantics

- New config flag `guard_ref_health` defaults to **False**. False preserves all
  historical batch/stream behavior exactly.
- A `GuardReferenceHealth` interval opens at start and at every scheduled
  `dr_reset_period` boundary. Within the interval it records each received GNSS
  epoch as accepted/fresh/finite/non-forced/non-alarm/non-recovery. Any rejected,
  forced, nonfinite, stale, missing, alarm or recovery epoch makes the interval
  permanently unhealthy; a later healthy fix never erases that history.
- When unhealthy, ACI and hybrid bias-monitor reference updates are held, learned
  forward aiding is dropped (hybrid and learned_vel fall back to fixed NHC;
  learned_nhc keeps its lateral/vertical constraints), and DR is not reseeded.
- At a scheduled boundary the interval is **always** replaced by a new assessment
  interval, whether or not a reseed happened. This bounded-interval rule avoids a
  permanent post-outage deadlock. Reseed still additionally requires no alarm,
  `g < 0.25 h`, and an accepted fix younger than 1.5 s. DR is never reseeded while
  alarmed.
- `guard_ref_max_age` defaults to 1.5 s, matching the historical freshness check.

## Experiments (all validation-only selection; no tuning)

1. **Mechanism parity** with the loaded legacy checkpoint
   `results/motionnet.pt`: clean 40 s seed 500 (Stage 1 hybrid config) and a 50 s
   seed-500 run with a 10–14 s outage. Compare flag-off history, flag-on clean
   no-op behavior, and batch/stream final position/full-covariance equality.
2. **Validation descriptive campaign**, seeds 500–503: Stage 2 frozen scenario
   definitions (600 s, spoof onset 300 s, rates 0.2/0.5/1.0 m/s, plus clean),
   Stage 2 base config and its frozen diagnostic `(k,h)=(3,50)`. Run
   `guard_ref_health` False and True. This is not a detector promotion; the
   frozen stage-2 candidate remains unpromoted.
3. **Held-out diagnostic**, seeds 900–909, executed only after the Stage 3 code
   and this protocol are frozen. Same scenarios and two flag values. These
   outcomes are diagnostic development evidence and are excluded from any
   implementation/threshold choice.

No attack, noise, correlation, checkpoint or scenario definition is changed.

## Metrics

Per run: alarm time, post-onset delay (null otherwise), final horizontal error,
number of DR reseeds, skipped reset boundaries, guard-reference fallback epochs,
first fallback time, interval count and cumulative reason codes. Summaries keep
clean exposure, clean alarms, detected/missed/pre-onset alarms per rate, and
healthy/outage coverage separate. The parity phase reports maximum absolute
batch/stream position and covariance differences.

## Predeclared Stage 3 assessment

Stage 3 mechanism evidence is acceptable only if:

- the full pytest suite is green;
- flag-off loaded-checkpoint batch/stream parity is unchanged;
- flag-on clean all-healthy runs are a no-op relative to flag-off;
- an outage produces observable skipped reseeds/fallback epochs with explicit
  reason codes and exact loaded-checkpoint batch/stream parity;
- results and commands are preserved under `results/v08_stage3/`.

Failure of any item is reported as a failure. **DETECTION remains a separate
mandatory V0.8 gate**; Stage 3 results will not be used to declare it passed.
