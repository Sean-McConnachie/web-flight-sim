# Validation

This document will hold the flight test targets that prove the model is correct.
Each target comes from the reference data in `docs/aircraft-me262.md`. A flight
test in `test/flight/` measures one target and compares the result against the
tolerance. The current result column stays at "not measured" until the test
runs. Update the column when a test lands, and record the date and the model
version in the log section.

## Target table

| Test | Target | Tolerance | Confidence | Current result |
| --- | --- | --- | --- | --- |
| Maximum level speed at 6000 m | 870 km/h | 5 percent | firm | not measured |
| Maximum level speed at sea level | 827 km/h | 5 percent | firm | not measured |
| Stall speed clean at 6400 kg | 175 km/h | 5 percent | firm | not measured |
| Rate of climb at sea level | 20 m/s | 10 percent | firm | not measured |
| Service ceiling | 11450 m | 5 percent | firm | not measured |
| Takeoff run at 7130 kg | 1100 m | 10 percent | firm | not measured |
| Mach tuck onset | 0.83 | 0.02 | firm | not measured |
| Mach limit | 0.86 | 0.02 | firm | not measured |
| Idle to full power spool time | 8 to 10 s | 1 s | firm | not measured |
| Static thrust per engine at sea level | 8.8 kN | 3 percent | firm | not measured |
| Load factor limit | +7 g and -3 g | none | estimated | not measured |
| Trimmed level flight holds altitude | drift below 5 m in 60 s | none | derived | not measured |
| Free fall matches g0 | 9.80665 m/s2 | 0.1 percent | firm | not measured |
| Standard atmosphere density at 6000 m | ISA table value | 0.5 percent | firm | not measured |

## How to run a flight test

## Test method for each target

## Tolerance policy

## Handling a target that the model cannot meet

## Result log

## Sources
