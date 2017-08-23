This project will demonstrate the CAN communication between two MCU using FreeRTOS & interrupts.
To Synchronize the task and ISR semaphores have been used
Connections
CAN1_RX -->PB8
CAN1_TX -->PB9

CAN rx goes to rx of the transreceiver

USART2 TX -->PA2
USART2 RX -->PA3

uart rx goes to tx of another uart
