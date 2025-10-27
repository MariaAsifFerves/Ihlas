# Water Filter Management System - Complete Code Overview

## System Purpose
This is an embedded system for a 5-stage water filter management unit. It tracks filter life, monitors water flow, detects errors, and controls solenoid valves and pumps.

## Hardware Components

### Inputs (GPIO Ports)
- **Buttons**: 3 buttons (BUTTON1, BUTTON2, BUTTON3) on GPIOD
- **Pressure Sensors**: 
  - LOW_PRESSURE_SENS (ABS - Alçak Basınç Switch) on GPIOB.PIN_4
  - HIGH_PRESSURE_SENS (YBS - Yüksek Basınç Switch) on GPIOB.PIN_5
- **Flow Meter**: External interrupt on GPIOC.PIN_7 (triggers `Flow_Pulse()`)
- **Water Leak Sensor**: ADC channel 4 (GPIOD.PIN_3)

### Outputs (GPIO Ports)
- **Pump**: GPIOA.PIN_1
- **Solenoid Valve**: GPIOA.PIN_2
- **LCD Backlight**: GPIOA.PIN_3
- **Buzzer**: GPIOD.PIN_1
- **LED**: GPIOD.PIN_2
- **Display**: VK1621 LCD controller on GPIOC

### Memory Storage (Flash)
- Filter life values stored in flash memory:
  - `First_Filter_Add`: 0x7800
  - `Second_Filter_Add`: 0x7850
  - `Third_Filter_Add`: 0x7900
  - `Fourth_Filter_Add`: 0x7950
  - `Fifth_Filter_Add`: 0x7A00
  - `GALON_Add`: 0x7A50 (total water)
  - `E5_Add`: 0x7B00 (E5 feature enable)
  - `Cflow_Add`: 0x7B50 (flow calibration)

---

## Display System (7 Screens)

The system has 7 display modes (screen_mode 0-6):

### Screen Mode 0: Total GALON
- Shows: Total water passed through system in gallons
- Format: 4 digits

### Screen Mode 1: Filter 1 Life
- Shows: Remaining life for first filter (in gallons)
- Format: F-1 + 4 digits
- Default: 1500 gallons

### Screen Mode 2: Filter 2 Life  
- Shows: Remaining life for second filter
- Format: F-2 + 4 digits
- Default: 3000 gallons

### Screen Mode 3: Filter 3 Life
- Shows: Remaining life for third filter
- Format: F-3 + 4 digits
- Default: 3000 gallons

### Screen Mode 4: Filter 4 Life
- Shows: Remaining life for fourth filter
- Format: F-4 + 4 digits
- Default: 6000 gallons

### Screen Mode 5: Filter 5 Life
- Shows: Remaining life for fifth filter  
- Format: F-5 + 4 digits
- Default: 3000 gallons

### Screen Mode 6: Flow Rate
- Shows: Current water flow rate in LPM (Liters Per Minute)
- Format: 4 digits with dot separator
- Updates every ~5 seconds during water filling

---

## Button System

### BUTTON 1 - Screen Navigation
- **Single Press**: Navigate to next screen (screen_mode++)
- **Cycles**: 0 → 1 → 2 → 3 → 4 → 5 → 6 → 0
- **In Configuration**: Moves to next digit for editing

### BUTTON 2 - Menu/Config Mode
- **Hold 3 seconds alone**: Enters `Set_Mode` (Quick Reset mode)
  - Resets filter life to predetermined values (1500, 3000, 4500, 6000, 0)
  - Values cycle: 1500 → 3000 → 4500 → 6000 → 0
  
- **Hold 3 seconds with BUTTON3**: Enters `set_mode2_digit` (Manual Entry mode)
  - Allows digit-by-digit manual entry of filter life values
  - Press Button3 to increment current digit (0-9)
  - Press Button1 to move to next digit position
  
- **Press to Exit**: When in any configuration mode, press Button2 to exit and save

### BUTTON 3 - Value Increment / System Power
- **In `set_mode2_digit` mode**: Increments current digit (0→1→2→...→9→0)
- **Hold 3 seconds**: Toggles system ON/OFF (when not in config mode)

---

## Error System

### Error Type 0 (E0): No Water Supply
- **Trigger**: Low pressure switch (ABS) active for 5 seconds continuously
- **Condition**: `low_pressure_stable_on == true`
- **Action**: Error=0, pump/valve OFF, bell/buzzer ON, screen_mode reset to 0

### Error Type 1 (E1): Filter Life Expired
- **Trigger**: Any filter life reaches 0
- **Condition**: `First_FLife==0 || Second_FLife==0 || ... || Fifth_FLife==0`
- **Action**: Error=1, bell/buzzer ON

### Error Type 2 (E2): Water Leak Detected
- **Trigger**: ADC reading > 300mV for 5+ seconds
- **Condition**: `Water_Leak==1` (ADC voltage high = water detected at leak sensor)
- **Action**: Pump/valve OFF immediately, Error=2, bell/buzzer ON

### Error Type 3 (E3): Filter Replacement Warning
- **Trigger**: Any filter life drops below 50 gallons
- **Condition**: `First_FLife<50 || ... || Fifth_FLife<50`
- **Action**: Error=3, buzzer warning (less urgent than E1)

### Error Type 4 (E4): Normal Operation
- **Status**: No errors, system running normally

### Error Type 5 (E5): Solenoid Detection Failed
- **Trigger**: Water flow detected but flowmeter not counting pulses
- **Condition**: After pump starts, no flowmeter pulses detected
- **Purpose**: Detects stuck solenoid valve or blocked system
- **Action**: Error=5, pump/valve OFF, buzzer alert

### Error Type 6 (E6): Solenoid Still Frozen After 3 Hours
- **Trigger**: E5 error persists for 3 hours (110000 seconds)
- **Condition**: `E6_work_cnt > 110000`
- **Action**: Upgrades to Error=6, different display code

### Error Type 7 (E7): Mechanic Valve Issue
- **Trigger**: Mechanical valve problem after E6 error
- **Condition**: Special mechanic valve pulse detection
- **Action**: Final error state, maintenance required

---

## Water Flow Detection

### Flow Meter Integration
- **Hardware**: External interrupt on falling edge (GPIOC.PIN_7)
- **Function**: `Flow_Pulse()` increments `pulse` counter
- **Calibration**: `Const_Flow_Value = 2548` pulses per gallon

### Flow Calculation
Every 2548 pulses:
- Increment `GALON++` (total water counter)
- Decrement all filter life values by 1 gallon
- Save to flash memory via `Flash_Write2()`
- Calculate flow rate: `FLOW = 2000 * water_filling_pulse / Const_Flow_Value`

### Water Filling States
```c
water_filling = true;  // Water is flowing
screen_mode = 6;       // Show flow rate screen
screen_rolling = true;  // Show animated filling screen
```

---

## Pressure Switch Logic (YBS & ABS)

### HIGH_PRESSURE_SENS (YBS - Yüksek Basınç)
- **Purpose**: Detects full tank, stops pump
- **Logic**: 5-second stability timer
  - `high_pressure_stable_on`: YBS active for 5+ seconds = TANK FULL
  - `high_pressure_stable_off`: YBS inactive for 5+ seconds = Tank empty enough

### LOW_PRESSURE_SENS (ABS - Alçak Basınç)
- **Purpose**: Detects no water supply
- **Logic**: 5-second stability timer
  - `low_pressure_stable_on`: ABS active for 5+ seconds = NO WATER (E0 error)
  - `low_pressure_stable_off`: ABS inactive for 5+ seconds = Water supply OK

### Pump Control Logic
```c
// Start pump when:
- Both switches OFF for 5 seconds AND
- No errors (E0, E2) AND
- No water leak AND
- System unlocked

// Stop pump when:
- HIGH pressure ON for 5 seconds (tank full) OR
- LOW pressure ON for 5 seconds (no water - E0 error)
```

---

## Solenoid Valve Diagnostics (E5 Feature)

When `E5_active == true`, the system monitors solenoid valve operation:

1. **Pump starts**: `selenoid_check_time = 1`
2. **Flow check**: After ~200 seconds, check if `flowmeter_stability_pulse > 0`
3. **No pulses detected**: Error=5 (solenoid stuck closed or system blocked)
4. **Pulses detected**: Normal operation, clear check

### E5 Lock Progression
```c
E5_lock = true;  // After error detected
E6_work_cnt++;   // Start 3-hour timer
// After 3 hours:
E6_lock = true; Error = 6; E5_lock = false;
// After mechanic valve issue:
E7_lock = true; Error = 7; E6_lock = false;
```

---

## Configuration Modes

### Set_Mode (Quick Reset)
**Activation**: Press Button2 for 3 seconds (while not in config)

**Function**: Quickly reset filter life to standard values
- Increment Button3: Cycle through 1500 → 3000 → 4500 → 6000 → 0
- Press Button1: Move to next digit (not active in Set_Mode)
- Press Button2: Exit and save

### set_mode2_digit (Manual Entry)
**Activation**: Press Button2 + Button3 together for 3 seconds

**Function**: Manually set exact filter life values
- Press Button3: Increment current digit (0→1→2→...→9→0)
- Press Button1: Move to next digit position (ones → tens → hundreds → thousands)
- Press Button2: Exit and save

---

## Flash Memory Operations

### Flash_Write2() Function
Saves all system data to flash memory (persists after power off):
```c
FMC_Unlock();
FMC_ErasePage(0x7800);
FMC_ProgramWord(First_Filter_Add, First_FLife);
FMC_ProgramWord(Second_Filter_Add, Second_FLife);
// ... (all filters, GALON, E5, Cflow)
FMC_Lock();
```

### Auto-Save Trigger
Filter life decrements and flash is updated every 2548 flowmeter pulses (every gallon).

---

## Timer System (TMR4)

Runs at 1 second intervals (`tick == 150`):
- Button debounce timers
- Screen auto-scroll timer
- Error visibility timers
- Buzzer control
- Water leak timing
- Pressure switch stability timers
- E6/E7 diagnostic timers

---

## Power Management

### System ON/OFF
- **Activation**: Button3 held for 3 seconds
- **OFF State**: LCD off, all outputs off, enters sleep mode
- **ON State**: LCD on, system active, buzzer beep

### Auto-Power Management
System automatically turns off features when not needed to conserve power.

---

## Key Variables Explained

| Variable | Purpose |
|----------|---------|
| `SYSTEM_ON` | Global power state |
| `screen_mode` | Current display (0-6) |
| `Error` | Current error code (0-7) |
| `water_filling` | Water actively flowing |
| `Set_Mode` | Quick reset mode active |
| `set_mode2_digit` | Manual entry mode active |
| `filter_save` | Flag to trigger flash write |
| `onetime_*` | One-shot flags for events |
| `*_lock` | Error state locks (prevent clearing) |

---

## State Machine Flow

### Normal Operation
```
Power ON → Check Flash → Load Filter Life → Check Errors
         ↓
    Display Screen 0-6 → Monitor Flow → Decrement Filters
         ↓
    Every Gallon → Save Flash
         ↓
    Filter < 50 → Show Warning (E3)
         ↓
    Filter = 0 → Show Error (E1)
```

### Water Filling Operation
```
Pump Start → Water Flowing → Flow Meter Pulses
         ↓
    Show Screen 6 (Flow Rate)
         ↓
    2548 pulses → Decrement Filters → Save Flash
         ↓
    Tank Full (YBS ON) → Pump Stop → Return to Screen 0
```

### Error Handling
```
Error Detected → Display Error Code → Buzzer Alert
         ↓
    Enter Safe State (Pump OFF, Valve CLOSED)
         ↓
    Wait for User Action or Error Clear
```

---

## Code Structure

1. **Lines 1-151**: Header, GPIO config, delay functions
2. **Lines 152-427**: `main()` initialization
3. **Lines 428-1178**: Main loop (state machine)
4. **Lines 1179-1341**: Display functions
5. **Lines 1342-1352**: Flash operations
6. **Lines 1353-1546**: Timer ISR (TMR4)
7. **Lines 1547-2297**: LCD segment display driver
8. **Lines 2088-2180**: Flow meter interrupt handler
9. **Lines 2181-2297**: ADC/water leak detection

This is a sophisticated multi-stage filter management system with comprehensive error handling and user interface!
