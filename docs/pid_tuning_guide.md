# PID Tuning Guide

## Recommended Sequence
1. Zero all I and D gains.
2. Increase P until oscillation, then back off ~30%.
3. Add D slowly to dampen oscillation.
4. Add I only if there is steady-state error.

## Parameters
Edit `src/config/parameters.h` and reflash.
