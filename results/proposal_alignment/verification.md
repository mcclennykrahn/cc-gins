# Session verification

- Full final substantive implementation: `OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python -m pytest -q` → **118 passed in 42.27s**. Existing tests were unchanged; six focused TCN/input/checkpoint/CLI tests added.
- Historical checkpoints loaded and forwarded: `results/motionnet.pt`, `results/kitti/motionnet_kitti.pt`, `results/i2nav/motionnet_foldA.pt`, `results/i2nav/motionnet_foldB.pt`.
- Paired experiment command in report.md executed successfully after the guard-input fix; all test/validation cases retained. Metrics unchanged by separate guard inference. No hyperparameter retuning.
- Full primary CLI: `OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python -m ccgins.cli run --config configs/proposal_primary.yaml --architecture proposal_tcn --out results/proposal_alignment/cli_primary` completed; output is synthetic simulation IMU/GNSS, seed 900. Reports carry explicit source annotations. The small saved checkpoints are smoke models, not replacements for historical published models.
- Training interface checked with `.venv/bin/python scripts/train_motionnet.py --help`; full 20-trajectory retraining was not run. The measured mini training uses the same training API and both architectures.
- `git diff --check` clean before commits. `.dsh-prompts/` excluded from staging.

Remaining validation: reference-conditioned training versus filter-conditioned inference domain gap; larger paired simulation and held-out real drive/fold retraining; statistically meaningful consistency/integrity validation. The TCN performed worse than legacy in this smoke. No complete-project percentage or closed-loop guarantee is inferred.
