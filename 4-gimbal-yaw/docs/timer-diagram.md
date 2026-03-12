# STM32 Timer Clock Diagram

```
System Clock (SYSCLK)
72 MHz
│
├─ AHB Bus (HCLK)  ÷1 = 72 MHz
│   │
│   ├─ APB2 (PCLK2)  ÷1 = 72 MHz
│   │   └─ TIM1 clock = 72 MHz  ← advanced timer on APB2
│   │
│   └─ APB1 (PCLK1)  ÷2 = 36 MHz
│       └─ TIM2/TIM4 clock = 36 MHz × 2* = 72 MHz  ← general timers on APB1
│                                                       (*HAL doubles when APB≠AHB)
│
└─ SysTick  = 72 MHz (used by HAL_Delay)
```

---

## How the Counter Works

```
Timer input clock
        │
        ▼
  [ Prescaler (PSC) ]   divides clock by (PSC + 1)
        │
        ▼
  Counter clock = Timer clock / (PSC + 1)
        │
        ▼
  [ Up counter 0 → ARR ]   counts up each tick
        │
        ▼  overflow (counter resets to 0)
  Update Event / IRQ

Period = (PSC + 1) × (ARR + 1) ticks
Frequency = Timer clock / Period
```

---

## TIM1 — 1 kHz Control Loop Interrupt

```
Timer clock  =  72 MHz
PSC          =  71   →  counter clock = 72 MHz / 72 = 1 MHz
ARR          =  999  →  overflow every 1000 ticks
Frequency    =  1 MHz / 1000 = 1000 Hz = 1 kHz
Period       =  1 ms
```

---

## TIM2 / TIM4 — PWM Output (Sinusoidal FOC)

```
Timer clock  =  72 MHz   (APB1 × 2)
PSC          =  0    →  counter clock = 72 MHz / 1 = 72 MHz
ARR          =  999  →  overflow every 1000 ticks
PWM freq     =  72 MHz / 1000 = 72 kHz
CCR          =  0..999  (duty cycle resolution)
```

---

## CCR Value Controls Duty Cycle

```
 0 ──────────────────────────────── ARR(999)
 │◄── CCR ──►│
 ████████████░░░░░░░░░░░░░░░░░░░░
 HIGH        LOW
 duty = CCR / ARR × 100%
```

---

## Summary Table

| Timer | Purpose           | PSC | ARR | Freq   |
|-------|-------------------|-----|-----|--------|
| TIM1  | 1 kHz IRQ tick    | 71  | 999 | 1 kHz  |
| TIM2  | PWM phases        | 0   | 999 | 72 kHz |
| TIM4  | PWM phases        | 0   | 999 | 72 kHz |

---

## Key Takeaway

- **TIM1** uses a high prescaler (71) to slow down to a human-scale 1 ms tick.
- **TIM2/TIM4** run at full speed so the PWM frequency is high enough (72 kHz) to be inaudible and give smooth motor current.
