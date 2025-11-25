# LoRa Configuration - UART Conflict Resolution

## ⚠️ CRITICAL ISSUE: UART Conflict Detected

**Your Specification**: LoRa on USART1
**Existing Configuration**: BLE (RN4871) on USART1

Both peripherals cannot share the same UART!

---

## Current Hardware Assignments

| Peripheral | UART | Baud Rate | Status |
|------------|------|-----------|--------|
| **BLE (RN4871)** | USART1 | 115200 | ✅ Currently in code |
| **LoRa Module** | **USART1** | 9600 | ⚠️ **CONFLICT!** |
| **GPS Module** | USART3 | 9600 | ✅ OK |

---

## Resolution Options

### **Option 1: Keep BLE on USART1, Move LoRa to USART2** (Recommended)

**Hardware**: Wire LoRa TX/RX to USART2 pins instead of USART1

**Code Changes**: Minor (just change huart2 references in lora_module.c)

**Advantages**:
- Minimal code changes
- BLE functionality preserved (all existing BLE code works)
- Clean separation of peripherals

**Configuration**:
```
USART1 @ 115200 → BLE (RN4871)
USART2 @ 9600   → LoRa (E32/E22)
USART3 @ 9600   → GPS (u-blox)
```

---

### **Option 2: Move BLE to USART2, LoRa on USART1** (As you specified)

**Hardware**: Wire BLE TX/RX to USART2 pins, LoRa stays on USART1

**Code Changes**: Moderate (update BLE module to use huart2)

**Advantages**:
- Matches your stated hardware configuration
- LoRa on USART1 as specified

**Configuration**:
```
USART1 @ 9600   → LoRa (E32/E22)
USART2 @ 115200 → BLE (RN4871)
USART3 @ 9600   → GPS (u-blox)
```

**Required Code Updates**:
1. Change `ble_module.c` to use `&huart2` instead of `&huart1`
2. Update `ble_module.h` documentation
3. Change `lora_module.c` to use `&huart1` (from current `&huart2`)
4. Update UART callback routing in `main.c`

---

### **Option 3: Disable BLE, LoRa on USART1**

**Hardware**: Only LoRa and GPS active

**Code Changes**: Minimal (remove BLE init, update LoRa to USART1)

**Advantages**:
- Simplest if you don't need BLE
- LoRa on USART1 as specified

**Configuration**:
```
USART1 @ 9600   → LoRa (E32/E22)
USART3 @ 9600   → GPS (u-blox)
```

---

## GPIO Configuration Status

✅ **Already Correctly Configured in main.h:**

```c
#define M1_Pin GPIO_PIN_12
#define M1_GPIO_Port GPIOB

#define M0_Pin GPIO_PIN_13
#define M0_GPIO_Port GPIOB

#define AUX_Pin GPIO_PIN_14
#define AUX_GPIO_Port GPIOB
```

These match your specifications exactly!

---

## Questions to Clarify

Before I update the code, please confirm:

1. **Which option do you prefer?** (Option 1, 2, or 3)

2. **Is BLE actually connected to USART1?**
   - If not, maybe BLE is already on USART2 and the conflict doesn't exist

3. **What is your actual hardware wiring?**
   - BLE: USART?
   - LoRa: USART?
   - GPS: USART3 (confirmed)

---

## Current LoRa Module Code Status

**File**: `Src/lora_module.c`
**Current UART**: `&huart2` (USART2)
**Your Specification**: USART1

**File**: `Src/main.c` (callbacks I just added)
**Current**: Assumes USART2 for LoRa
**Your Specification**: USART1

---

## Recommended Action

**I recommend Option 1** (Keep BLE on USART1, move LoRa to USART2) because:

1. Minimal code changes
2. Preserves all existing BLE functionality
3. USART2 is available and suitable for LoRa
4. Cleaner separation of high-speed (BLE 115200) and low-speed (LoRa 9600) UARTs

**However**, if your hardware is already wired with LoRa on USART1, I can implement any of the three options.

**Please tell me**:
- Which option matches your actual hardware?
- Or confirm if you want me to proceed with Option 1 (requires rewiring LoRa to USART2)

I'll update all the code once you clarify! 🔧
