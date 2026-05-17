# FreeRTOS Traffic Light Simulation
 
A real-time traffic light simulation running on an STM32F4 Discovery board using FreeRTOS V9.0.0. A potentiometer controls traffic flow rate, which dynamically adjusts light timing and vehicle generation. LEDs driven through a shift register simulate cars moving through an intersection.
 
---
 
## Circuit
<![Circuit Diagram](images/circuit.jpg) -->
 
## System Overview
 
Four FreeRTOS tasks work together:
 
- **Traffic Flow Adjustment** — reads the potentiometer via ADC every 500ms and converts it to a flow percentage
- **Traffic Generator** — uses the flow percentage as a probability to randomly spawn incoming cars
- **Traffic Light State** — cycles through green → yellow → red using a one-shot software timer; green/red durations scale with the flow rate (yellow is always 3s fixed)
- **System Display** — drives the traffic light LEDs and shifts car positions through the shift register each tick; cars queue up and compress before a red light, and clear through on green


