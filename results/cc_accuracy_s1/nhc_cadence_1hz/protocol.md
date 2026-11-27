# Candidate nhc_cadence_1hz

New bounded hypothesis: decimate fixed NHC from 10 Hz to 1 Hz (pseudo_every=100
at dt=.01), retaining sigma=.1 m/s on both channels. Repeated constraints can
count correlated lateral slip/vertical motion too strongly. No parameters will
be retuned after seed exposure. Development: 900–902; fresh held-out: 29201–29205.
Same example_sim.yaml inputs and initialization per baseline/candidate pair;
180 seconds, scoring from 30 seconds, outages 60–90 and 120–180 seconds.
Acceptance requires lower development and held-out mean horizontal RMSE, with
per-seed regressions and coverage disclosed. A negative result remains unpromoted.

Architecture review: right-invariant SE_2(3) navigation with additive IMU biases,
first-order covariance propagation and Joseph updates. NHC limits lateral and
vertical drift but leaves forward speed weakly observed in GNSS outages. Existing
bias random walks and nominal-speed vibration noise are calibration opportunities,
but changing them together would confound attribution. Outage fallback and guard
health are already reviewed Stage3 mechanisms. Smoothing would require new state
history/backward covariance support and introduce offline latency; deferred.
Cadence tuning uses existing batch/stream scheduling and changes no estimator code.
The closed vertical-NHC candidate is not reused or reopened. Detection stays FAIL;
confirmed_v08=false. Simulation evidence cannot establish real-data accuracy.
