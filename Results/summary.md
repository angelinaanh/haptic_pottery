# Benchmark summary

Data from `VirtualPottery --bench`, summarised by `Evaluation/plot_comparison.py`.

| | Baseline | Method 2 |
|---|---|---|
| Physics rate | 1000 Hz (in servo loop) | 250 Hz (own thread) |
| Peak push force, 200x180 grid [N] | 0.591 | 0.429 |
| Peak push force, 800x360 grid [N] | 0.622 | 0.423 |
| Change between grids | 5.0 % | 1.3 % |
| Largest step before filter, groove 400x360 [mN] | 32.0 | 6.2 |
| Largest step sent, groove 400x360 [mN] | 5.4 | 6.2 |
| Delay added by the force filter [ms] | 23 | 0 (no filter) |
| Physics step, groove 200x180 [us] | 159 | 42 |
| Physics step, groove 400x360 [us] | 349 | 115 |
| Physics step, groove 800x360 [us] | 662 | 206 |
| Physics step, groove 1600x720 [us] | 1515 | 715 |
| 1 kHz loop work, 1600x720: mean / max [us] | 1516.4 / 2789 | 0.5 / 158 |
| Servo rate, 1600x720 [Hz] | 640 | 1000 |
