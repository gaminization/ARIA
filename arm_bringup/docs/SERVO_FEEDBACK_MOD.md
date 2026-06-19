# Servo Feedback Modification Guide

How to tap the internal potentiometer wiper on MG995 and SG90 servos for position feedback.

---

## Why This Modification?

Standard hobby servos (MG995, SG90) don't have a position output pin. They use an internal potentiometer to track angle, but this signal stays inside the servo's control circuit. By tapping the potentiometer wiper, we can read the actual servo angle via the ESP32's ADC — enabling:

- **Teach mode**: Physically move the arm, and the system records positions
- **Stall detection**: Know when a servo is blocked and can't reach its target
- **Real → Sim sync**: The simulation mirrors the real arm's actual position

---

## Tools Needed

| Tool | Use |
|------|-----|
| Small Phillips screwdriver | Open servo case |
| 30 AWG wire (silicone insulated) | Signal wire from pot |
| Soldering iron (fine tip) | Solder to pot center pin |
| Solder (0.5mm rosin core) | Attach wire |
| Heat shrink tubing (1mm) | Insulate solder joint |
| Multimeter | Verify connection |
| Hot glue gun | Strain relief |

---

## MG995 Procedure

### Step 1: Open the Case

1. Remove the 4 bottom screws (long Phillips).
2. Carefully lift the bottom plate.
3. You'll see the PCB with the motor driver IC and the potentiometer.

### Step 2: Locate the Potentiometer

The pot is a small blue or black rotary component connected to the output shaft via a gear. It has **3 pins**:

```
 ┌─────────┐
 │   POT   │
 │         │
 ├─┬───┬───┤
 │ │   │   │
GND WIPER VCC
 │   │     │
 └───┘     └── These go to the servo PCB
```

The **center pin** is the wiper — this is what we need.

### Step 3: Solder the Wire

1. Tin the end of a 15cm piece of 30 AWG wire.
2. Apply a tiny amount of solder to the pot's center (wiper) pin.
3. Quickly solder the wire to the center pin.
4. Apply heat shrink over the joint.
5. **Do NOT** touch the outer two pins — those are GND and VCC.

### Step 4: Route the Wire

1. Route the wire alongside the existing servo cable.
2. Use a small notch in the case edge or drill a 1mm hole.
3. Apply hot glue at the exit point for strain relief.
4. Reassemble the servo case (carefully route wire so it doesn't pinch).

### Step 5: Connect to ESP32

1. Connect the tapped wiper wire to the designated ADC pin (see wiring table).
2. The pot already gets GND and VCC from the servo's internal circuit.
3. **No additional resistors or voltage dividers needed** — the pot output is 0–5V on MG995, but since the ESP32 ADC is 0–3.3V, you should use a simple voltage divider:

```
Pot Wiper ──── 10K ────┬──── ESP32 ADC Pin
                       │
                      20K
                       │
                      GND

Output voltage: 0–3.3V (from 0–5V pot range)
```

> **Note**: For SG90 servos running at 3.3V logic, the pot output is already within the ESP32's ADC range. No divider needed.

---

## SG90 Procedure

The SG90 is smaller, so the modification requires more care.

### Step 1: Open the Case

1. Remove the 4 tiny bottom screws.
2. The pot is directly coupled to the output shaft.

### Step 2: Locate the Pot

Same 3-pin layout as MG995, but much smaller. The center pin is the wiper.

### Step 3: Solder

1. Use a very fine soldering tip.
2. Pre-tin both the wire and the center pin.
3. Touch-solder quickly (< 2 seconds) to avoid melting the pot housing.
4. Apply 1mm heat shrink.

### Step 4: Route and Reassemble

1. Route wire alongside existing cable.
2. The SG90 case has very tight tolerances — ensure wire doesn't interfere with the gear train.
3. Hot glue strain relief at exit.

### Step 5: Connect to ESP32

SG90 pots typically output 0–3.3V, so connect directly to the ADC pin:

```
Pot Wiper ──── directly ──── ESP32 ADC Pin
```

---

## Verification

After modification, verify the connection:

### Multimeter Test

1. Power the servo (don't connect signal wire).
2. Measure voltage between pot wiper wire and servo GND.
3. Manually rotate the servo horn.
4. Voltage should sweep smoothly from ~0.1V to ~3.0V (SG90) or ~0.1V to ~4.8V (MG995).

### ESP32 ADC Test

Upload a simple test sketch:

```cpp
void setup() {
    Serial.begin(115200);
    analogReadResolution(12);
}

void loop() {
    int raw = analogRead(34);  // Your ADC pin
    float voltage = raw * 3.3 / 4095.0;
    float angle = raw * 180.0 / 4095.0;
    Serial.printf("ADC: %d  V: %.2f  Angle: %.1f°\n", raw, voltage, angle);
    delay(100);
}
```

Manually rotate the servo and verify:
- ADC value sweeps from ~200 to ~3800
- Angle estimate roughly matches visual position
- No dead spots or jumps (would indicate a loose connection)

---

## Calibration

After verifying all 6 servos, run the ARIA calibration wizard:

```bash
aria hardware wizard
```

This will automatically:
1. Command each servo to 0° and 180°
2. Record ADC endpoints
3. Compute linear mapping
4. Save to `arm_control/config/servo_calibration.yaml`

---

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| ADC always reads 0 | Wire not connected | Check solder joint |
| ADC always reads 4095 | Wire on VCC pin, not wiper | Resolder to center pin |
| ADC jumps erratically | Loose solder joint | Reflow solder, add flux |
| ADC doesn't track full range | Wrong voltage divider | Adjust resistor values |
| Servo doesn't work after mod | Wire pinching gears | Re-route wire, check gears spin freely |
| Servo makes grinding noise | Wire caught in gear train | Open case, re-route wire |

---

## Important Notes

- This modification **does not void** the servo's functionality — you are only reading a signal, not modifying the control loop.
- If you prefer not to open servos, use **Option C (Command Tracking)** in the firmware configuration. You'll lose teach mode and stall detection, but everything else works.
- For production use, consider **Option B (External Potentiometers)** mounted on the joint axes for maximum reliability.
