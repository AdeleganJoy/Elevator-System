/*
 * Elevator Master (Arduino MEGA)
 * 
 * This program implements a master control system for an elevator using Arduino MEGA.
 * It handles user input via keypad, displays status on LCD, and communicates with a slave
 * Arduino UNO via I2C to control physical components like motors, doors, and sensors.
 * 
 * The system implements multiple states including idle, moving, emergency, and maintenance modes,
 * with safety features like obstruction detection and emergency stop capabilities.
 */

// Define CPU clock frequency for proper timing calculations
#define F_CPU 16000000UL  // 16 MHz clock speed for Arduino MEGA
#define BAUD 9600         // UART baud rate for serial communication (if needed)

// Required libraries
#include <avr/io.h>       // For AVR I/O port definitions
#include <avr/interrupt.h> // For interrupt handling
#include <util/delay.h>   // For time delay functions
#include <util/twi.h>     // For I2C/TWI communication
#include <stdlib.h>       // For general utilities like itoa()
#include <string.h>       // For string handling functions
#include "lcd.h"          // For controlling the LCD display
#include "keypad.h"       // For reading input from the keypad

// I2C/TWI communication parameters
#define SLAVE_ADDR 0x10   // Address of the slave Arduino UNO on the I2C bus

// Command codes for communication with slave Arduino
// These define the specific actions the slave should take
#define CMD_IDLE            0x00  // No action, elevator is stationary with door closed
#define CMD_MOVING_UP       0x01  // Move elevator upward toward target floor
#define CMD_MOVING_DOWN     0x02  // Move elevator downward toward target floor
#define CMD_DOOR_OPEN       0x03  // Open the elevator door
#define CMD_DOOR_CLOSE      0x04  // Close the elevator door
#define CMD_EMERGENCY       0x05  // Emergency stop, halt all operations
#define CMD_FAULT           0x06  // Fault condition detected
#define CMD_PLAY_MELODY     0x07  // Play alarm or notification sound
#define CMD_STOP_MELODY     0x08  // Stop any currently playing sound
#define CMD_FLOOR_REACHED   0x09  // Notify that elevator has reached the target floor
#define CMD_MAINTENANCE     0x0A  // Enter maintenance mode
#define CMD_CHECK_DISTANCE  0x0B  // Request distance measurement from ultrasonic sensor

// Emergency button pin configuration
// This defines the port, data direction register, and pin for the emergency button
#define EMERGENCY_BTN_PORT  PORTB  // Port B for emergency button
#define EMERGENCY_BTN_DDR   DDRB   // Data Direction Register for Port B
#define EMERGENCY_BTN_PIN   PINB   // Input register for Port B
#define EMERGENCY_BTN_BIT   6      // Digital pin 12 (PB6) - Emergency button input

// Debug LED configuration for visual feedback during operations
#define DEBUG_LED_PORT      PORTB  // Port B for debug LED
#define DEBUG_LED_DDR       DDRB   // Data Direction Register for Port B
#define DEBUG_LED_PIN       7      // Digital pin 13 (built-in LED on Arduino MEGA)

// System states enumeration
// These represent the different operational states of the elevator system
typedef enum {
    STATE_IDLE,        // Elevator is idle, waiting for input
    STATE_MOVING_UP,   // Elevator is moving upward
    STATE_MOVING_DOWN, // Elevator is moving downward
    STATE_DOOR_OPEN,   // Elevator door is open
    STATE_EMERGENCY,   // Emergency state activated
    STATE_FAULT,       // Fault condition detected
    STATE_MAINTENANCE, // Maintenance mode active
    STATE_OBSTRUCTION  // Obstruction detected in elevator path
} ElevatorState;

// Global variables for system state tracking
volatile ElevatorState current_state = STATE_IDLE;  // Current state of the elevator
volatile uint8_t current_floor = 0;                 // Current floor position (0-99)
volatile uint8_t target_floor = 0;                  // Target floor selected by user
volatile uint8_t input_buffer[3] = {0};            // Buffer for storing keypad input digits
volatile uint8_t input_index = 0;                   // Current position in input buffer
volatile uint8_t emergency_flag = 0;                // Flag indicating emergency state (1=emergency)
volatile uint8_t door_open = 0;                     // Flag indicating door state (1=open)
volatile uint8_t melody_playing = 0;                // Flag indicating if alarm melody is playing
volatile uint8_t communication_error = 0;           // Flag for I2C communication errors
volatile uint8_t maintenance_mode = 0;              // Flag for maintenance mode
volatile uint16_t distance_cm = 0;                  // Distance measured from ultrasonic sensor in cm

// Function prototypes with descriptions
// I2C/TWI communication functions
void TWI_init_master(void);                         // Initialize TWI/I2C as master
void TWI_start(void);                               // Send I2C start condition
void TWI_stop(void);                                // Send I2C stop condition
void TWI_write(uint8_t data);                       // Write byte to I2C bus
uint8_t TWI_read_ack(void);                         // Read byte from I2C bus with ACK
uint8_t TWI_read_nack(void);                        // Read byte from I2C bus with NACK
uint16_t TWI_get_distance(void);                    // Get distance from ultrasonic sensor via I2C
uint8_t TWI_send_command_safe(uint8_t command, uint8_t data); // Safely send command to slave

// Input handling functions
uint8_t safe_get_keypad_input(void);                // Get keypad input with debounce
void process_keypad_input(uint8_t key);             // Process user keypad input
void clear_input_buffer(void);                      // Clear the keypad input buffer
uint8_t convert_input_to_floor(void);               // Convert input buffer to floor number

// UI and display functions
void update_display(void);                          // Update the LCD display based on current state

// Elevator operation functions
void move_elevator(void);                           // Handle elevator movement logic
void check_for_obstruction(void);                   // Check for obstruction using ultrasonic sensor

// State management functions
void set_state(ElevatorState new_state);            // Set system state and notify slave
/**
 * Handle maintenance mode operations
 * 
 * Keeps the system in maintenance mode and updates displays/controls
 * accordingly. In this mode, the elevator is stationary and normal
 * operations are suspended.
 */
void handle_emergency(void);                        // Handle emergency situation
void handle_maintenance_mode(void);                 // Handle maintenance mode operations

// System functions
void system_init(void);                             // Initialize the entire system
void debug_blink(uint8_t times);                    // Blink debug LED for visual feedback
uint8_t is_emergency_button_pressed(void);          // Check if emergency button is pressed

/**
 * Check if emergency button is pressed
 * 
 * This function reads the emergency button input pin and returns its state.
 * The button is configured with external pull-up resistor, so it reads LOW when pressed.
 * 
 * @return 1 if button is pressed, 0 otherwise
 */
uint8_t is_emergency_button_pressed(void) {
    // With external pull-up, button reads LOW (0) when pressed
    return ((EMERGENCY_BTN_PIN & (1 << EMERGENCY_BTN_BIT)) == 0) ? 1 : 0;
}

/**
 * Initialize TWI/I2C as master
 * 
 * Sets up the TWI hardware for I2C communication where this Arduino acts as
 * the master device. Sets SCL frequency to 100kHz for reliable communication.
 */
void TWI_init_master(void) {
    // Set SCL frequency to 100kHz
    TWSR = 0x00;  // Prescaler value = 1
    TWBR = 0x48;  // SCL frequency = CPU_CLOCK/(16 + 2*TWBR*Prescaler)
    
    // Enable TWI hardware
    TWCR = (1 << TWEN);
}

/**
 * Send I2C start condition
 * 
 * Generates a START condition on the I2C bus to initiate a new transmission.
 * Waits until the operation completes before returning.
 */
void TWI_start(void) {
    // Set TWINT (interrupt flag), TWSTA (start condition), and TWEN (enable TWI)
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    // Wait for TWINT flag to set, indicating start condition was transmitted
    while (!(TWCR & (1 << TWINT)));
}

/**
 * Send I2C stop condition
 * 
 * Generates a STOP condition on the I2C bus to terminate a transmission.
 * Adds a small delay to ensure the stop condition is fully transmitted.
 */
void TWI_stop(void) {
    // Set TWINT (interrupt flag), TWSTO (stop condition), and TWEN (enable TWI)
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    _delay_ms(1);  // Small delay to ensure the stop condition is transmitted
}

/**
 * Write byte to I2C bus
 * 
 * Sends a data byte over the I2C bus.
 * Waits until the operation completes before returning.
 * 
 * @param data The byte to send
 */
void TWI_write(uint8_t data) {
    // Load data into TWI Data Register
    TWDR = data;
    // Start transmission (clear TWINT and keep TWI enabled)
    TWCR = (1 << TWINT) | (1 << TWEN);
    // Wait for TWINT flag to set, indicating data was transmitted
    while (!(TWCR & (1 << TWINT)));
}

/**
 * Read byte from I2C bus with ACK
 * 
 * Reads a byte from the I2C bus and sends an ACK to indicate more data is expected.
 * Waits until the operation completes before returning.
 * 
 * @return The received byte
 */
uint8_t TWI_read_ack(void) {
    // Enable TWI, set ACK bit, and clear interrupt flag
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    // Wait for data to be received
    while (!(TWCR & (1 << TWINT)));
    // Return received data
    return TWDR;
}

/**
 * Read byte from I2C bus with NACK
 * 
 * Reads a byte from the I2C bus and sends a NACK to indicate no more data is expected.
 * Waits until the operation completes before returning.
 * 
 * @return The received byte
 */
uint8_t TWI_read_nack(void) {
    // Enable TWI and clear interrupt flag (no ACK bit set)
    TWCR = (1 << TWINT) | (1 << TWEN);
    // Wait for data to be received
    while (!(TWCR & (1 << TWINT)));
    // Return received data
    return TWDR;
}

/**
 * Get distance from ultrasonic sensor via I2C
 * 
 * Sends a command to the slave to take a distance measurement,
 * then reads the result (16-bit value) from the slave.
 * 
 * @return Distance in centimeters (16-bit value)
 */
uint16_t TWI_get_distance(void) {
    uint16_t distance = 0;
    uint8_t low_byte, high_byte;
    
    // Send command to slave to take distance measurement
    TWI_start();
    TWI_write((SLAVE_ADDR << 1) | 0); // SLA+W (write mode)
    TWI_write(CMD_CHECK_DISTANCE);     // Command to check distance
    TWI_write(0);                      // Dummy data (not used by slave)
    TWI_stop();
    
    _delay_ms(50); // Give slave time to measure distance
    
    // Read distance data (2 bytes) from slave
    TWI_start();
    TWI_write((SLAVE_ADDR << 1) | 1); // SLA+R (read mode)
    low_byte = TWI_read_ack();        // Read low byte with ACK (more data expected)
    high_byte = TWI_read_nack();      // Read high byte with NACK (last byte)
    TWI_stop();
    
    // Combine bytes into 16-bit distance value (high byte in upper 8 bits)
    distance = (high_byte << 8) | low_byte;
    
    return distance;
}

/**
 * Safely send command and data to slave
 * 
 * Handles I2C communication with error checking to ensure commands
 * are delivered reliably to the slave device.
 * 
 * @param command The command code to send
 * @param data Additional data associated with the command
 * @return 1 if successful, 0 if communication error
 */
uint8_t TWI_send_command_safe(uint8_t command, uint8_t data) {
    // Visual feedback for I2C communication - turn on debug LED
    DEBUG_LED_PORT |= (1 << DEBUG_LED_PIN);
    
    // Send start condition
    TWI_start();
    
    // Send slave address in write mode
    TWDR = (SLAVE_ADDR << 1) | 0;  // SLA+W (Write mode)
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    
    // Check if slave acknowledged address
    if ((TWSR & 0xF8) != TW_MT_SLA_ACK) {
        // If no acknowledgment, stop transmission and signal error
        TWI_stop();
        DEBUG_LED_PORT &= ~(1 << DEBUG_LED_PIN);  // Turn off debug LED
        communication_error = 1;
        return 0;  // Return error code
    }
    
    // Send command byte
    TWDR = command;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    
    // Send data byte
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    
    // Send stop condition
    TWI_stop();
    
    // Turn off debug LED and clear error flag
    DEBUG_LED_PORT &= ~(1 << DEBUG_LED_PIN);
    communication_error = 0;
    return 1;  // Success
}

/**
 * Get keypad input with simple debounce
 * 
 * Reads the keypad and applies a simple debounce to prevent
 * multiple readings from a single key press.
 * 
 * @return The key character or 0xFF if no key pressed
 */
uint8_t safe_get_keypad_input(void) {
    uint8_t key = KEYPAD_GetKey();
    
    // If a key is detected, add debounce delay before returning it
    if (key != 0xFF) {
        _delay_ms(20);  // Debounce delay
        return key;
    }
    return 0xFF;  // No key pressed
}

/**
 * Provide visual feedback via debug LED
 * 
 * Blinks the debug LED a specified number of times for visual indication
 * of system events or errors.
 * 
 * @param times Number of times to blink the LED
 */
void debug_blink(uint8_t times) {
    for (uint8_t i = 0; i < times; i++) {
        DEBUG_LED_PORT |= (1 << DEBUG_LED_PIN);    // Turn LED on
        _delay_ms(200);
        DEBUG_LED_PORT &= ~(1 << DEBUG_LED_PIN);   // Turn LED off
        _delay_ms(200);
    }
}

/**
 * Initialize the entire elevator system
 * 
 * Sets up all hardware components, peripherals, and system variables.
 * Performs a series of self-tests and initializations for each component.
 */
void system_init(void) {
    // Setup debug LED
    DEBUG_LED_DDR |= (1 << DEBUG_LED_PIN);         // Set LED pin as output
    DEBUG_LED_PORT &= ~(1 << DEBUG_LED_PIN);       // Initialize LED as off
    
    // Initialize state variables explicitly
    emergency_flag = 0;     // No emergency
    door_open = 0;          // Door closed
    melody_playing = 0;     // No melody playing
    maintenance_mode = 0;   // Normal operation mode
    
    // Visual indication that system is starting
    debug_blink(1);
    
    // Initialize LCD display
    lcd_init(LCD_DISP_ON);
    lcd_clrscr();
    lcd_puts("Elevator System");
    lcd_gotoxy(0, 1);
    lcd_puts("Initializing...");
    _delay_ms(1000);
    
    // Initialize keypad interface
    KEYPAD_Init();
    lcd_clrscr();
    lcd_puts("Keypad Ready");
    _delay_ms(500);
    
    // Configure emergency button input (pin 12)
    EMERGENCY_BTN_DDR &= ~(1 << EMERGENCY_BTN_BIT);  // Set as input
    // Note: Not using internal pull-up since we have external resistor
    
    // Test the emergency button functionality
    lcd_clrscr();
    lcd_puts("Button test");
    lcd_gotoxy(0, 1);
    if (is_emergency_button_pressed()) {
        lcd_puts("Button pressed!");
        debug_blink(3);
    } else {
        lcd_puts("Button ready");
    }
    _delay_ms(1000);
    
    // Initialize TWI/I2C communication as master
    TWI_init_master();
    lcd_clrscr();
    lcd_puts("I2C Ready");
    _delay_ms(500);
    
    // Enable global interrupts for interrupt-driven components
    sei();
    
    // Test connection to the slave device
    if (TWI_send_command_safe(CMD_IDLE, 0)) {
        lcd_clrscr();
        lcd_puts("Slave Connected");
        _delay_ms(500);
    } else {
        lcd_clrscr();
        lcd_puts("Slave Error!");
        lcd_gotoxy(0, 1);
        lcd_puts("Check connections");
        _delay_ms(2000);
    }
    
    // Set initial system state to idle
    set_state(STATE_IDLE);
}

/**
 * Process keypad input based on current state
 * 
 * Handles different key presses based on the current system state.
 * Different keys have different functions in each state.
 * 
 * @param key The key character that was pressed
 */
void process_keypad_input(uint8_t key) {
    if (key == 0xFF) return;  // No key pressed, exit early
    
    // Visual feedback for key press using debug LED
    DEBUG_LED_PORT |= (1 << DEBUG_LED_PIN);    // Turn LED on
    _delay_ms(50);
    DEBUG_LED_PORT &= ~(1 << DEBUG_LED_PIN);   // Turn LED off
    
    // Handle different keys based on current system state
    switch (current_state) {
        case STATE_IDLE:
            // Process numeric keys (0-9) for floor selection
            if ((key >= '0' && key <= '9') && input_index < 2) {
                input_buffer[input_index++] = key - '0';  // Convert ASCII to numeric value
                update_display();
            }
            // Process '*' key as confirm/enter
            else if (key == '*' && input_index > 0) {
                target_floor = convert_input_to_floor();
                
                // Special case: 00 + * to enter maintenance mode
                if (target_floor == 0 && input_index == 2 && 
                    input_buffer[0] == 0 && input_buffer[1] == 0) {
                    maintenance_mode = 1;
                    set_state(STATE_MAINTENANCE);
                    clear_input_buffer();
                    return;
                }
                
                // Check if target floor is the same as current floor (fault condition)
                if (target_floor == current_floor) {
                    set_state(STATE_FAULT);
                }
                // Start moving to target floor
                else if (target_floor > current_floor) {
                    set_state(STATE_MOVING_UP);
                }
                else {
                    set_state(STATE_MOVING_DOWN);
                }
                
                // Clear input buffer after processing
                clear_input_buffer();
            }
            // Process '#' key as clear/cancel
            else if (key == '#') {
                clear_input_buffer();
                update_display();
            }
            break;
            
        case STATE_EMERGENCY:
            // In emergency state, '*' button opens door and activates alarm
            if (key == '*' && !door_open) {
                door_open = 1;
                melody_playing = 1;
                
                // Send door open command multiple times for reliability
                TWI_send_command_safe(CMD_DOOR_OPEN, 1);
                _delay_ms(100);
                TWI_send_command_safe(CMD_DOOR_OPEN, 1);
                _delay_ms(100);
                TWI_send_command_safe(CMD_DOOR_OPEN, 1);
                
                // Start the alarm melody
                TWI_send_command_safe(CMD_PLAY_MELODY, 1);
                
                update_display();
            }
            // '#' button stops alarm and closes door to exit emergency state
            else if (key == '#' && door_open) {
                // Stop melody and close door
                TWI_send_command_safe(CMD_STOP_MELODY, 0);
                _delay_ms(500);
                
                // Send door close command multiple times for reliability
                TWI_send_command_safe(CMD_DOOR_CLOSE, 0);
                _delay_ms(100);
                TWI_send_command_safe(CMD_DOOR_CLOSE, 0);
                _delay_ms(100);
                TWI_send_command_safe(CMD_DOOR_CLOSE, 0);
                
                door_open = 0;
                melody_playing = 0;
                
                // Return to idle state
                _delay_ms(1000);
                emergency_flag = 0;
                set_state(STATE_IDLE);
            }
            break;
            
        case STATE_MAINTENANCE:
            // Exit maintenance mode when '*' key is pressed
            if (key == '*') {
                maintenance_mode = 0;
                TWI_send_command_safe(CMD_IDLE, 0);  // Return slave to idle state
                set_state(STATE_IDLE);
            }
            break;
            
        default:
            // In other states, ignore keypad input
            // This includes MOVING_UP, MOVING_DOWN, DOOR_OPEN, etc.
            break;
    }
}

/**
 * Update LCD display based on current system state
 * 
 * Updates the LCD with different information depending on the
 * current state of the elevator system.
 */
void update_display(void) {
    lcd_clrscr();  // Clear screen before updating
    
    // If there was a communication error, show it briefly
    if (communication_error) {
        lcd_puts("I2C Error!");
        _delay_ms(500);
        communication_error = 0;
    }
    
    // Display different information based on current state
    switch (current_state) {
        case STATE_IDLE:
            lcd_puts("Choose floor:");
            lcd_gotoxy(0, 1);
            
            // Show current input if any, otherwise show current floor
            if (input_index > 0) {
                char buffer[5];
                uint8_t floor = convert_input_to_floor();
                itoa(floor, buffer, 10);  // Convert number to string
                lcd_puts(buffer);
            } else {
                lcd_puts("Current: ");
                char buffer[5];
                itoa(current_floor, buffer, 10);
                lcd_puts(buffer);
            }
            break;
            
        case STATE_MOVING_UP:
            lcd_puts("Going up");
            lcd_gotoxy(0, 1);
            lcd_puts("Floor: ");
            char up_buffer[5];
            itoa(current_floor, up_buffer, 10);
            lcd_puts(up_buffer);
            break;
            
        case STATE_MOVING_DOWN:
            lcd_puts("Going down");
            lcd_gotoxy(0, 1);
            lcd_puts("Floor: ");
            char down_buffer[5];
            itoa(current_floor, down_buffer, 10);
            lcd_puts(down_buffer);
            break;
            
        case STATE_DOOR_OPEN:
            lcd_puts("Floor: ");
            char floor_buffer[5];
            itoa(current_floor, floor_buffer, 10);
            lcd_puts(floor_buffer);
            lcd_gotoxy(0, 1);
            lcd_puts("Door is open");
            break;
            
        case STATE_EMERGENCY:
            lcd_puts("EMERGENCY");
            lcd_gotoxy(0, 1);
            
            if (!door_open) {
                lcd_puts("* to open door");
            } else {
                lcd_puts("# to stop alarm");
            }
            break;
            
        case STATE_FAULT:
            lcd_puts("ERROR");
            lcd_gotoxy(0, 1);
            lcd_puts("Already at floor");
            break;
            
        case STATE_MAINTENANCE:
            lcd_puts("Maintenance");
            lcd_gotoxy(0, 1);
            lcd_puts("* to exit");
            break;
            
        case STATE_OBSTRUCTION:
            lcd_puts("OBSTRUCTION");
            lcd_gotoxy(0, 1);
            lcd_puts("Please wait...");
            break;
    }
}

/**
 * Move elevator to target floor
 * 
 * Handles the logic for elevator movement, including floor tracking,
 * checking for obstructions, and door operations upon arrival.
 */
void move_elevator(void) {
    // Check for obstruction before attempting movement
    check_for_obstruction();
    
    // Check emergency button state
    if (is_emergency_button_pressed()) {
        emergency_flag = 1;
        DEBUG_LED_PORT |= (1 << DEBUG_LED_PIN);  // Visual feedback
    }
    
    // If emergency has been triggered, handle it first
    if (emergency_flag) {
        handle_emergency();
        return;
    }
    
    // If in obstruction state, do nothing until timeout
    if (current_state == STATE_OBSTRUCTION) {
        return;
    }
    
    // Simulate elevator movement by incrementing/decrementing floor
    if (current_state == STATE_MOVING_UP && current_floor < target_floor) {
        _delay_ms(1000);  // Delay to simulate movement time between floors
        current_floor++;  // Move up one floor
        
        // Check for obstruction again after moving
        check_for_obstruction();
        if (current_state == STATE_OBSTRUCTION) {
            return;
        }
        
        // Update display with current floor information
        lcd_clrscr();
        lcd_puts("Going up");
        lcd_gotoxy(0, 1);
        lcd_puts("Floor: ");
        char up_buffer[5];
        itoa(current_floor, up_buffer, 10);
        lcd_puts(up_buffer);
        
        // Notify slave about current floor movement
        TWI_send_command_safe(CMD_MOVING_UP, current_floor);
    }
    else if (current_state == STATE_MOVING_DOWN && current_floor > target_floor) {
        _delay_ms(1000);  // Delay to simulate movement time between floors
        current_floor--;  // Move down one floor
        
        // Check for obstruction again after moving
        check_for_obstruction();
        if (current_state == STATE_OBSTRUCTION) {
            return;
        }
        
        // Update display with current floor information
        lcd_clrscr();
        lcd_puts("Going down");
        lcd_gotoxy(0, 1);
        lcd_puts("Floor: ");
        char down_buffer[5];
        itoa(current_floor, down_buffer, 10);
        lcd_puts(down_buffer);
        
        // Notify slave about current floor movement
        TWI_send_command_safe(CMD_MOVING_DOWN, current_floor);
    }
    
    // Check if elevator has reached the target floor
    if (current_floor == target_floor) {
        // Tell slave we've reached the floor
        TWI_send_command_safe(CMD_FLOOR_REACHED, current_floor);
        
        // Change to door open state
        set_state(STATE_DOOR_OPEN);
        
        // Open door - send command multiple times for reliability
        TWI_send_command_safe(CMD_DOOR_OPEN, 1);
        _delay_ms(100);
        TWI_send_command_safe(CMD_DOOR_OPEN, 1);
        _delay_ms(100);
        TWI_send_command_safe(CMD_DOOR_OPEN, 1);
        
        // Keep door open for 5 seconds to allow entry/exit
        _delay_ms(5000);
        
        // Close door and return to idle state
        TWI_send_command_safe(CMD_DOOR_CLOSE, 0);
        _delay_ms(100);
        TWI_send_command_safe(CMD_DOOR_CLOSE, 0);
        _delay_ms(100);
        TWI_send_command_safe(CMD_DOOR_CLOSE, 0);
        
        // Return to idle state, ready for next input
        set_state(STATE_IDLE);
    }
}

/**
 * Handle emergency situation
 * 
 * Responds to emergency button press by halting elevator operation
 * and entering emergency state.
 */
void handle_emergency(void) {
    // Change to emergency state
    set_state(STATE_EMERGENCY);
    
    // Send emergency command to slave to blink movement LED
    TWI_send_command_safe(CMD_EMERGENCY, 0);
    _delay_ms(100);
    TWI_send_command_safe(CMD_EMERGENCY, 0);  // Send twice for reliability
    
    // Reset door and melody flags
    door_open = 0;
    melody_playing = 0;
    
    // Update display to show emergency message
    update_display();
}

/**
 * Handle maintenance mode operations
 * 
 * Keeps the system in maintenance mode and updates displays/controls
 * accordingly. In this mode, the elevator is stationary and normal
 * operations are suspended.
 */
void handle_maintenance_mode(void) {
    // Keep slave in maintenance mode by repeatedly sending maintenance command
    TWI_send_command_safe(CMD_MAINTENANCE, 0);
    
    // Update the display with maintenance message
    update_display();
}

/**
 * Check for obstruction using ultrasonic sensor
 * 
 * Uses the ultrasonic distance sensor to detect obstructions in the elevator path.
 * If an obstruction is detected (distance < 20cm), the elevator temporarily
 * enters the obstruction state before resuming normal operation.
 */
void check_for_obstruction(void) {
    // Only check for obstructions when elevator is moving
    if (current_state == STATE_MOVING_UP || current_state == STATE_MOVING_DOWN) {
        // Get distance from ultrasonic sensor
        distance_cm = TWI_get_distance();
        
        // Check if distance is less than 20 cm (obstruction detected)
        if (distance_cm < 20 && distance_cm > 0) {  // Also check for valid readings
            // Store the previous state to return to it after obstruction is cleared
            ElevatorState prev_state = current_state;
            
            // Set state to obstruction
            set_state(STATE_OBSTRUCTION);
            
            // Update display
            update_display();
            
            // Wait for 3 seconds
            _delay_ms(3000);
            
            // Resume previous operation
            set_state(prev_state);
            update_display();
        }
    }
}

/**
 * Clear the input buffer
 * 
 * Resets the keypad input buffer and index to initial state.
 * Called after processing input or when input is canceled.
 */
void clear_input_buffer(void) {
    for (uint8_t i = 0; i < 3; i++) {
        input_buffer[i] = 0;
    }
    input_index = 0;
}

/**
 * Convert input buffer to floor number
 * 
 * Converts the digits stored in the input buffer to a floor number.
 * Handles both single-digit and two-digit floor numbers.
 * 
 * @return Floor number as an 8-bit unsigned integer
 */
uint8_t convert_input_to_floor(void) {
    if (input_index == 0) {
        return 0;
    }
    else if (input_index == 1) {
        return input_buffer[0];  // Single-digit floor
    }
    else {
        return input_buffer[0] * 10 + input_buffer[1];  // Two-digit floor
    }
}

/**
 * Set system state and send corresponding command to slave
 * 
 * Updates the current state of the elevator system and sends the
 * appropriate command to the slave device to synchronize its state.
 * Also updates the display to reflect the new state.
 * 
 * @param new_state The new state to set the system to
 */
void set_state(ElevatorState new_state) {
    current_state = new_state;
    
    // Send command to slave based on new state
    uint8_t command = CMD_IDLE;
    uint8_t data = 0;
    
    switch (new_state) {
        case STATE_IDLE:
            command = CMD_IDLE;
            break;
        case STATE_MOVING_UP:
            command = CMD_MOVING_UP;
            data = target_floor;
            break;
        case STATE_MOVING_DOWN:
            command = CMD_MOVING_DOWN;
            data = target_floor;
            break;
        case STATE_DOOR_OPEN:
            command = CMD_DOOR_OPEN;
            data = 1;  // 1 = door open
            break;
        case STATE_EMERGENCY:
            command = CMD_EMERGENCY;
            break;
        case STATE_FAULT:
            command = CMD_FAULT;
            break;
        case STATE_MAINTENANCE:
            command = CMD_MAINTENANCE;  
            break;
        case STATE_OBSTRUCTION:
            // No specific command for obstruction state
            break;
    }
    
    // Send command to slave
    TWI_send_command_safe(command, data);
    
    // Update display
    update_display();
}

/**
 * Main function - Program entry point
 * 
 * Initializes the system and contains the main control loop that runs continuously.
 * Handles different system states and processes user input appropriately.
 * 
 * @return Never returns (infinite loop)
 */
int main(void) {
    // Initialize all system components and variables
    system_init();
    
    // EXPLICITLY reset emergency flag after initialization
    // This is critical for safety - ensures we don't start in emergency state
    emergency_flag = 0;
    
    // Main program loop - runs continuously
    while (1) {
        // Check if in maintenance mode - special operational state
        if (current_state == STATE_MAINTENANCE) {
            handle_maintenance_mode();
        }
        // Handle elevator movement if in moving state - highest priority task
        // This is handled before other operations for responsiveness
        else if (current_state == STATE_MOVING_UP || current_state == STATE_MOVING_DOWN) {
            move_elevator();
            continue;  // Skip to next iteration after handling movement
                       // This ensures movement is handled without interruption
        }
        
        // Get keypad input (non-blocking) - allows system to continue other operations
        uint8_t key = safe_get_keypad_input();
        
        // Process keypad input if a key is pressed (0xFF indicates no key pressed)
        if (key != 0xFF) {
            process_keypad_input(key);
            _delay_ms(250);  // Additional debounce delay to prevent rapid repeat processing
        }
        
        // Handle fault state - visual indication then auto-reset
        if (current_state == STATE_FAULT) {
            debug_blink(3);                // Blink LED 3 times to indicate fault
            _delay_ms(2000);               // Display fault message for 2 seconds
            set_state(STATE_IDLE);         // Return to idle state automatically
        }
    }
    
    return 0;  // Will never reach here since we have an infinite loop
}
