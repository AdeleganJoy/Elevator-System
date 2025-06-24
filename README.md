# Elevator-System

## 2C / TWI communication basics:

### Master transmitter – Slave receiver modes (Microchip Studio)

The system builds a communication between Arduino MEGA and Arduino UNO by using I2C/TWI (2-wire Serial Interface). Arduino MEGA acts as the master, so it initiates and terminates the data transmission (SDA) between the microcontrollers, and also generates the clock (SCL). Arduino UNO acts as the slave, which is addressed by the master. This setup allows the master to send commands and receive data from the slave using just two wires, making it efficient for microcontroller communication.

## Circuit

## Components

Our circuit consisted of the following components:

- Arduino MEGA x1
- Arduino UNO x1
- 4x4 Keypad x1
- 16x2 Liquid-crystal display (LCD) x1
- HC-SR04 Ultrasonic sensor x1
- Red light-emitting diode (LED) x1
- Green LED x1
- External button x1
- Passive buzzer x1
- 10K Ohm potentiometer x1
- 5K Ohm pull-up resistor x2
- 220 Ohm resistor x3
- 10K Ohm pull-up resistor x1
- Jumper wires

## Schematic Diagram and Connections

We provide a schematic representation of the circuit in Figure 1 below. This is a visualization of the connections used in creating the elevator project. In order to understand the schematic diagram, we have provided a detailed explanation of each electronic component and their connections.

![ezcv logo](https://raw.githubusercontent.com/AdeleganJoy/Elevator-System/main/schematicDiagramCircuit.png)

### I2C connection

The Arduino MEGA (master) is connected to the Arduino UNO (slave) using the I2C serial data and serial clock lines (SDA and SCL respectively). The interface allows the Arduino MEGA to send specific tasks for the Arduino UNO to perform while the latter can send data when addressed.

The connections used to establish the connections are:

- Arduino UNO (GND) ←→ Arduino MEGA (GND)
- Arduino UNO (A4) ←→ Arduino MEGA (D20)
- Arduino UNO (A5) ←→ Arduino MEGA (D21)
- Arduino UNO (A4) ← Resistor (5KΩ) → 5V
- Arduino UNO (A5) ← Resistor (5KΩ) → 5V

### Keypad connection

The Keypad is connected to the Arduino MEGA pins. Each of the keys is connected to a separate pin on the Arduino MEGA. The pins on the keypad consist of ROW1, ROW2, ROW3, ROW4, COL1, COL2, COL3 and COL4. 

The connections used to establish the Keypad are:

- Keypad (ROW1) ←→ Arduino MEGA (A8)
- Keypad (ROW2) ←→ Arduino MEGA (A9)
- Keypad (ROW3) ←→ Arduino MEGA (A10)
- Keypad (ROW4) ←→ Arduino MEGA (A11)
- Keypad (COL1) ←→ Arduino MEGA (A12)
- Keypad (COL2) ←→ Arduino MEGA (A13)
- Keypad (COL3) ←→ Arduino MEGA (A14)
- Keypad (COL4) ←→ Arduino MEGA (A15)

### LCD connection

The LCD setup involves two electronic parts; These are the Arduino MEGA and the Potentiometer. The Potentiometer handles the contrast of the LCD display and the Arduino MEGA handles the characters displayed on the LCD.

The connections used to establish the LCD are:

- D4 – D7 pins are connected to Arduino MEGA pins (D3, D4, D5, and  D6 respectively)
- RS, RW and E are connected to Arduino MEGA pins (D9, D10, and  D11 respectively)
- V0 is connected to the center pin of the potentiometer
- Potentiometer anode is connected to 5V
- Potentiometer cathode is connected to GND
- VDD is connected to 5V
- VSS is connected to GND 
- K is connected to GND 
- A is connected to 5V through a 220Ω resistor

### Passive Buzzer Connection

The Buzzer is connected to the Arduino Uno. 

The connections used to establish the Buzzer are:

- Buzzer (Pin 1) ←→ Arduino UNO (D10)
- Buzzer (Pin 2) ←→ GND
  
### Button Connection

The Button is connected to the Arduino MEGA and 5V through a 10K Ohm pull-up resistor.

The connections used to establish the Button are:

- Button (Pin 1) ←→ Arduino MEGA (D12)
- Button (Pin 1) ←Resistor (10KΩ)→ 5V
- Button (Pin 2) ←→ GND

### Movement red LED Connection

The door LED ONs when the door is open and OFFs when the door is closed. 

The connections used to establish the door LEDs are:

- LED (Anode) ← Resistor (220Ω) →  Arduino UNO (D9)
- LED(Cathode) ←→ GND

### Door green LED Connection
The door LED ONs when the door is open and OFFs when the door is closed. 

The connections used to establish the door LEDs are:

- LED (Anode) ← Resistor (220Ω) →  Arduino UNO (D9)
- LED(Cathode) ←→ GND

### Ultrasonic Sensor

The Ultrasonic Sensor is used to detect obstructions in the elevator’s path. It works by emitting a sound pulse and measuring how long it takes to bounce back.

The connections used to establish the Ultrasonic sensor are:

- Ultrasonic Sensor (VCC) ←→ 5V
- Ultrasonic Sensor (GND) ←→ GND
- Ultrasonic Sensor  (TRIG) ←→ Arduino UNO (D4)
- Ultrasonic Sensor  (ECHO) ←→ Arduino UNO (D5)

## Code And Functionality Explanation

In this section, we provide an explanation of the code used in the demo presentation. The elevator system code consists of three files. These include the Arduino UNO code file and the Arduino MEGA code file.

### Functionality

Our project imitates the functionality of an elevator. A user is prompted on an LCD to select a floor via a keyboard (the limit is 99) and go to it by pressing “*”. The floor is then updated on the LCD while the lift is moving and the direction is indicated (an error (through ISR) gets displayed if the selected floor is the one the cabin is already on). Keyboard entries can be erased by pressing “#”. 

Elevator movement is indicated by a red LED. Additionally, the door opening sequence is signaled by a green LED, which lights up for 5 seconds after the required floor is reached, and by the LCD displaying that the door is opened.

During the movement, if an external button is pressed, the LCD shows “EMERGENCY” and the red LED blinks 3 times. Next,  “*” needs to be pressed on the keyboard for the door to open, which is then accompanied by a buzzer playing a melody. This continues until “#” is pressed, which returns the elevator to its idle state.

Furthermore, an ultrasonic sensor can detect whether an object is within 20 cm range, stopping the execution of the program until this obstruction is removed. Finally, a maintenance mode can be entered by pressing “00” and “\*” during the floor selection and exited by pressing “\*” again.

### Master/Arduino MEGA controller

The master controller serves as the brain of the elevator system, managing all the decision-making and user interface components. It reads input from a keypad where users select their desired floor, displays the elevator's status on an LCD screen, and tracks both the current and target floors. The master handles the elevator's movement logic, determining whether to go up or down, and manages special states like emergency stops, maintenance mode, and obstruction detection. When a floor is reached, it controls door operations, keeping doors open for a set time before closing them. The master communicates all these decisions to the slave controller via I2C protocol, sending specific command codes that tell the slave what physical actions to take.

### Slave/Arduino UNO controller

The slave controller acts as the physical interface of the elevator system, controlling all the hardware components based on commands received from the master controller. It manages the movement LED to indicate when the elevator is in motion, the door LED to show door status, and handles the buzzer for various alert tones. When floors are reached, it plays notification sounds, and during emergencies, it can play alarm melodies. The slave also includes an ultrasonic distance sensor that can detect obstructions in the elevator's path and report this data back to the master. This controller operates through a series of interrupt-driven functions that allow it to respond immediately to commands from the master while maintaining continuous operation of its various components.

## Areas for improvement

In terms of functionality, our team has come up with some aspects and additions we can implement to improve our project. First, an additional LED indicator of a different color (yellow) can be utilized to indicate when the elevator is moving down. This way a user can understand in what direction they are moving when inside the cabin. Second, a pressure sensor, such as HX710B can be used to perceive the weight of the people and objects inside our theoretical cabin. Through it, we can indicate when the weight limit is crossed and someone/something needs to be removed, otherwise the doors won't close and the lift will not move. Finally, we can also implement an infrared (IR) distance sensor, such as SHARP GP2Y0A21YK0F, to provide a multimodal input to the system and make the closing functionality more robust.

## 💻 Tech Stack  

![C](https://img.shields.io/badge/C-blue?style=for-the-badge&logo=c&logoColor=white)  
![Microchip Studio](https://img.shields.io/badge/Microchip%20Studio-red?style=for-the-badge&logo=microchip&logoColor=white)  
![AVR](https://img.shields.io/badge/AVR%20Microcontroller-black?style=for-the-badge)

## ⚙️ Installation & Usage

1. **Clone the repository**:
   
   ```bash
   git clone https://github.com/AdeleganJoy/Elevator-System.git
    ```
   
3. **Open the project in Microchip Studio:**:
   
   - Launch **Microchip Studio**.
   - Go to `File > Open > Project/Solution`
   - Navigate to the cloned folder and open the `.atsln` (Atmel Studio solution) file.
     
4. **Build and upload:**
   - Connect your target AVR board (e.g., ATmega328p).
   - Select the correct programmer/debugger (e.g., AVRISP mkII).
   - Build the project (`Build > Build Solution`) and then program the device.

## 🌐 Socials:

[![email](https://img.shields.io/badge/Email-D14836?logo=gmail&logoColor=white)](mailto:joyadelegan1@gmail.com) 


