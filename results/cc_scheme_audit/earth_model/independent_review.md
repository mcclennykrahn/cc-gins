# Independent review: scheme_earth_model

PASS for the implementation, measured evidence and final report docs/scheme_earth_model_audit.md. No blocking findings.

The inverse at ccgins/sim.py:166–183 correctly solves the existing mechanization: R_next = Exp(-w dt) R Exp(gyro dt), and a = R accel + g - 2 w×v. CLI wiring at ccgins/cli.py:136–137 and filter wiring at ccgins/pipeline.py:185 use the same configured Earth model resolved and validated at pipeline.py:93–103. The audit diagnostic at scripts/audit_scheme.py:74–89 receives that explicit model rather than silently using fixed gravity.

Independent checks used held-out seed118001, 60s propagation, gravity9.793 and9.81, with and without Earth rotation. Largest accumulated position closure error was3.83e-11m. Default signed traces on900/118001/118002 were exactly unchanged. Targeted regression of Earth, simulator and CLI tests passed:23 tests in5.28s.

Configured-Earth seed900 and118002 horizontal RMSE increases after the fix. This is a correction of mismatched sensor synthesis, not an accuracy improvement. Before/after comparison changes true IMU angular rate and specific force while retaining trajectory, random sensor draws and GNSS. Old pure-INS diagnostics ignored configured Earth and are superseded, not directly comparable as matched-physics sensor budgets.

Remaining limitations: fixed local ENU omits geodetic transport; truth-relative warm starts and three paired scenarios do not establish causal acquisition or operational error tails. DETECTION remains FAIL and confirmed_v08=false. Stream configuration still duplicates Earth construction for valid configurations; this patch does not change stream behavior.

Evidence: /workspace/ccgins-v08/results/cc_scheme_audit/earth_model/independent_check.json.

Final report review checked bug/fix citations, initial-state and Q/update source lines, before/after segment values and explicit sensor-input-change qualifications. No blocking findings. Full-suite regression remains the parent delivery responsibility.
