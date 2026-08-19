# RPM Sensor sim

## What it does

* **Primary sensor** (pin 9) models the primary clutch sheave reluctor, 36 teeth.
* **Rear sensor** (pin 10) models the rear/wheel output sensor, 44 teeth.

Instead of spinning real hardware, the sketch toggles the two output pins at a frequency calculated from a target RPM and tooth count, so your controller under test "sees" realistic gear-tooth pulse trains on its input pins.

It also injects **faults** on either channel so you can verify your controller handles bad sensor data gracefully:

|Fault|Code|Effect|
|-|-|-|
|`NONE`|0|Clean, steady pulse train|
|`DROPOUT`|1|\~15% of toggle edges are randomly skipped (simulates a broken wire or missed teeth)|
|`NOISE`|2|Normal pulses plus rare random glitch edges injected (simulates EMI)|
|`SLIP`|3|Not an edge fault the rear RPM is simply set lower than the primary RPM in the test case, simulating belt slip|

Each **test case** bundles a primary RPM, rear RPM, and fault type into a named scenario

## How to select a test case

Near the top of the file:

```cpp
const uint8_t TEST_SELECTED = 0; // which test to run
```

This is the **index into the `tests[]` array**, containing several modifiable preset test cases, after changing the sketch can be reuploaded

