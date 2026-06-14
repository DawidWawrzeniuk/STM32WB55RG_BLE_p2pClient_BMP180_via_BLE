# STM32WB55 BLE P2P Client — BMP180 Sensor Transmitter
This project implements a Bluetooth Low Energy (BLE) GATT Client running on the STM32WB55 microcontroller.
The client reads environmental data from a BMP180 sensor (temperature + pressure) and transmits it to a BLE server using a custom 6‑byte payload.

The project is based on the official P2P Client example from STMicroelectronics and extends it with:

* BMP180 sensor integration

* periodic BLE data transmission

* custom 6‑byte payload format

* timer‑driven update mechanism

# 📡 BLE Architecture Overview
The client connects to a P2P Server and discovers:

Write Characteristic (Client → Server)

Notify Characteristic (Server → Client)

The client writes sensor data to the server using:

c
P2P_WRITE_CHAR_UUID
Payload size: 6 bytes

📤 Payload Format (Client → Server)
The client sends a 6‑byte packet containing:

Byte	Meaning
0	Device ID
1	Temperature (°C)
2	Pressure LSB
3	Pressure
4	Pressure
5	Pressure MSB


Encoding example (Temperature_update):
c
uint8_t payload[6];
payload[0] = 0x01;               // Device ID
payload[1] = temperature;        // 1 byte
payload[2] = (pressure >> 0) & 0xFF;
payload[3] = (pressure >> 8) & 0xFF;
payload[4] = (pressure >> 16) & 0xFF;
payload[5] = (pressure >> 24) & 0xFF;

Write_Char(P2P_WRITE_CHAR_UUID, 0, payload);
📡 BLE Write Function
The client uses:

c
aci_gatt_write_without_resp()
Correctly configured for 6‑byte payloads:

c
ret = aci_gatt_write_without_resp(
    aP2PClientContext[index].connHandle,
    aP2PClientContext[index].P2PWriteToServerCharHdle,
    6,                     // Payload length
    (uint8_t *)pPayload
);
This ensures the server receives the full 6 bytes.

🌡 BMP180 Sensor Integration
The client reads temperature and pressure in the main loop:

c
temperature = BMP180_ReadTemperature();
pressure    = BMP180_ReadPressure();
Optional conversion:

c
float pressure_hPa = pressure / 100.0f;
⏱ Timer‑Driven BLE Updates
A hardware timer (TIM17) triggers periodic BLE transmissions:

c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM17)
    {
        UTIL_SEQ_SetTask(1 << CFG_TASK_TEMPERATURE_UPDATE, CFG_SCH_PRIO_0);
    }
}
The registered task:

c
UTIL_SEQ_RegTask(1 << CFG_TASK_TEMPERATURE_UPDATE, UTIL_SEQ_RFU, Temperature_update);
This ensures stable, periodic sensor updates.

📥 Receiving Notifications (Server → Client)
The client handles server notifications in:

c
case P2P_NOTIFICATION_INFO_RECEIVED_EVT:
Example:

c
P2P_Client_App_Context.LedControl.Led1 =
    pNotification->DataTransfered.pPayload[1];

if(P2P_Client_App_Context.LedControl.Led1 == 0x00)
    BSP_LED_Off(LED_BLUE);
else
    BSP_LED_On(LED_BLUE);
🔧 BLE Service Discovery
The client automatically discovers:

P2P Service

Write Characteristic

Notify Characteristic

CCCD descriptor

This is handled in:

c
Event_Handler()
and the state machine transitions:

c
APP_BLE_DISCOVER_SERVICES
APP_BLE_DISCOVER_CHARACS
APP_BLE_DISCOVER_WRITE_DESC
APP_BLE_DISCOVER_NOTIFICATION_CHAR_DESC
APP_BLE_ENABLE_NOTIFICATION_DESC
📤 Button → Server Example
The client can also send button events:

c
Write_Char(P2P_WRITE_CHAR_UUID, 0,
           (uint8_t *)&P2P_Client_App_Context.ButtonStatus);
🧩 Hardware Requirements
STM32WB55 (e.g., Nucleo-WB55RG)

BMP180 sensor (I2C)

ST-Link V3

3.3V power supply

🛠 Software Requirements
STM32CubeIDE 1.15+

STM32CubeWB Firmware Package

BLE Stack: stm32wb5x_BLE_Stack_fw.bin

BMP180 library

🚀 How to Run
Flash BLE stack to CPU2

Flash client firmware

Power the device

Client automatically connects to the server

Sensor data is transmitted every timer tick

Server displays temperature and pressure

🔧 Debug Output Example
Kod
>> TX | ID=1, TEMP=25, PRESS=99700
