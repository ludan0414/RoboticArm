# P1 settling tolerance for TCP carry line

Revision base: 07f7ad6. In loading_096, carry_p1 succeeded, but carry_p2 failed before planning with `CARRY_TCP_LINE: start is off taught TCP line`. The recorded start joints differed from the P1 target by about 0.018 rad on several axes. The prior 2 cm ideal-line gate did not account for that normal arrival tolerance.

The executor now accepts a start within 0.03 rad per joint of taught P1 and plans its Cartesian line from the measured current state. A mid-line resume still must satisfy the 2 cm line gate. The failure message reports measured progress and distance. No collision checks were reintroduced. Focused offline tests: 3 passed. Package build passed. No robot motion was commanded.
