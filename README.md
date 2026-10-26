# Pacer: Smart Alarm Clock (SIC 8 Stage 2)

## Team Members
- Person A (Hardware)
- Person B (Firmware)
- Person C (MQTT & Python)
- Person D (Docs & Testing)

## 🔌 Pin Contract
- **OLED:** SDA 21, SCL 22
- **Buttons:** GPIO 25, 26, 27
- **Potentiometer:** GPIO 34
- **Buzzer:** GPIO 18
- **LEDs:** Green 4, Yellow 5, Red 19

## 🔄 State Machine
IDLE -> PRE_ALERT -> WAIT_CONFIRM -> RUNNING -> DONE
