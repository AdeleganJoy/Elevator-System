/*
 * Elevator Slave (Arduino UNO)
 * 
 * This program implements a slave control system for an elevator using Arduino UNO.
 * It receives commands from the master Arduino MEGA via I2C communication and
 * controls the physical hardware components including LEDs, buzzer, and 
 * ultrasonic distance sensor.
 * 
 * The slave device handles the elevator movement indication, door status,
 * alarm sounds, and distance measurement functionality.
 */

// Define CPU clock frequency for proper timing calculations
#define F_CPU 16000000UL  // 16 MHz clock speed for Arduino UNO
#define BAUD 9600         // UART baud rate if serial communication is used

// Required libraries
#include <avr/io.h>           // For AVR I/O port definitions
#include <avr/interrupt.h>    // For interrupt handling
#include <util/delay.h>       // For time delay functions
#include <util/twi.h>         // For I2C/TWI communication
#include <stdlib.h>           // For general utilities

// Define I2C/TWI slave address - must match the address used by master
#define SLAVE_ADDR 0x10       // Address of this slave device on the I2C bus

// Command codes received from master - must match the master's command set
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

// Pin definitions for output devices
// Movement LED indicates when elevator is in motion
#define MOVEMENT_LED_PORT  PORTB  // Port B for movement LED
#define MOVEMENT_LED_DDR   DDRB   // Data Direction Register for Port B
#define MOVEMENT_LED_PIN   0      // Digital pin 8 (PB0) - Movement LED output

// Door LED indicates when elevator door is open
#define DOOR_LED_PORT      PORTB  // Port B for door LED
#define DOOR_LED_DDR       DDRB   // Data Direction Register for Port B
#define DOOR_LED_PIN       1      // Digital pin 9 (PB1) - Door LED output

// Buzzer for alarm and notification sounds
#define BUZZER_PORT        PORTB  // Port B for buzzer
#define BUZZER_DDR         DDRB   // Data Direction Register for Port B
#define BUZZER_PIN         2      // Digital pin 10 (PB2) - Buzzer output

// Ultrasonic sensor pins for distance measurement
// TRIG (trigger) sends the ultrasonic pulse
#define TRIG_PORT          PORTD  // Port D for trigger pin
#define TRIG_DDR           DDRD   // Data Direction Register for Port D
#define TRIG_PIN           4      // Digital pin 4 (PD4) - Trigger output

// ECHO receives the reflected ultrasonic pulse
#define ECHO_PORT          PORTD  // Port D for echo pin
#define ECHO_DDR           DDRD   // Data Direction Register for Port D
#define ECHO_PIN           PIND   // Pin Input Register for Port D
#define ECHO_BIT           5      // Digital pin 5 (PD5) - Echo input

// Global variables for system state tracking
volatile uint8_t command = CMD_IDLE;        // Current command from master
volatile uint8_t data = 0;                  // Data associated with command
volatile uint8_t command_received = 0;      // Flag indicating new command received
volatile uint8_t melody_playing = 0;        // Flag indicating if alarm melody is playing
volatile uint8_t emergency_state = 0;       // Flag indicating emergency condition
volatile uint16_t distance_cm = 0;          // Distance measured from ultrasonic sensor in cm

// Function prototypes with descriptions
void system_init(void);                     // Initialize all hardware components
void TWI_init_slave(uint8_t address);       // Initialize TWI/I2C as slave device
void handle_command(void);                  // Process received command
void blink_movement_led(uint8_t times);     // Blink movement LED specified number of times
void beep(uint8_t duration);                // Generate beep sound of specified duration
void play_emergency_melody(void);           // Start playing emergency alarm sound
void stop_melody(void);                     // Stop currently playing melody
void play_floor_reach_tone(void);           // Play tone to indicate floor arrival
void test_outputs(void);                    // Test all hardware outputs on startup
uint16_t measure_distance(void);            // Measure distance using ultrasonic sensor

// Timer1 variables for melody generation
volatile uint8_t melody_step = 0;           // Current step/note in the melody sequence
volatile uint8_t melody_enabled = 0;        // Flag to enable melody in timer ISR
volatile uint8_t continuous_melody = 1;     // Flag for continuous melody playback

/**
 * ISR for Timer1 Compare Match A - Used for melody timing
 * 
 * This interrupt service routine is triggered periodically by Timer1 to generate
 * complex melody patterns for the emergency alarm. The melody consists of multiple
 * tones of varying frequencies and durations to create an attention-grabbing pattern.
 */
ISR(TIMER1_COMPA_vect) {
    static uint8_t tone_state = 0;          // For toggling buzzer to create tones
    static uint8_t tone_counter = 0;        // Counter for note duration
    
    if (melody_enabled && melody_playing) {
        // Multi-stage melody with different tones, frequencies, and durations
        switch (melody_step) {
            case 0:  // First note - high frequency (rapid toggle)
                if (tone_state == 0) {
                    BUZZER_PORT |= (1 << BUZZER_PIN);    // Turn buzzer ON
                } else {
                    BUZZER_PORT &= ~(1 << BUZZER_PIN);   // Turn buzzer OFF
                }
                
                tone_counter++;
                if (tone_counter >= 10) {  // Duration of first note
                    tone_counter = 0;
                    melody_step = 1;      // Move to next step in sequence
                }
                break;
                
            case 1:  // Short pause between notes (silent)
                BUZZER_PORT &= ~(1 << BUZZER_PIN);       // Ensure buzzer is OFF
                tone_counter++;
                if (tone_counter >= 5) {   // Duration of pause
                    tone_counter = 0;
                    melody_step = 2;      // Move to next note
                }
                break;
                
            case 2:  // Second note - mid frequency
                if (tone_state == 0) {
                    BUZZER_PORT |= (1 << BUZZER_PIN);    // Turn buzzer ON
                } else {
                    BUZZER_PORT &= ~(1 << BUZZER_PIN);   // Turn buzzer OFF
                }
                
                tone_counter++;
                if (tone_counter >= 15) {  // Longer duration for second note
                    tone_counter = 0;
                    melody_step = 3;
                }
                break;
                
            case 3:  // Another pause (silent)
                BUZZER_PORT &= ~(1 << BUZZER_PIN);       // Ensure buzzer is OFF
                tone_counter++;
                if (tone_counter >= 5) {   // Duration of pause
                    tone_counter = 0;
                    melody_step = 4;
                }
                break;
                
            case 4:  // Third note - low frequency (slower toggle using modulo 3)
                if ((tone_state % 3) == 0) {   // Toggle buzzer less frequently
                    BUZZER_PORT |= (1 << BUZZER_PIN);    // Turn buzzer ON
                } else {
                    BUZZER_PORT &= ~(1 << BUZZER_PIN);   // Turn buzzer OFF
                }
                
                tone_counter++;
                if (tone_counter >= 20) {  // Even longer duration for third note
                    tone_counter = 0;
                    melody_step = 5;
                }
                break;
                
            case 5:  // Final pause (silent)
                BUZZER_PORT &= ~(1 << BUZZER_PIN);       // Ensure buzzer is OFF
                tone_counter++;
                if (tone_counter >= 5) {   // Duration of pause
                    tone_counter = 0;
                    melody_step = 6;
                }
                break;
                
            case 6:  // Fourth note - Urgent beeping (rapid toggle)
                if (tone_state == 0) {
                    BUZZER_PORT |= (1 << BUZZER_PIN);    // Turn buzzer ON
                } else {
                    BUZZER_PORT &= ~(1 << BUZZER_PIN);   // Turn buzzer OFF
                }
                
                tone_counter++;
                if (tone_counter >= 30) {  // Longest duration for final note
                    tone_counter = 0;
                    melody_step = 7;
                }
                break;
                
            case 7:  // End of melody sequence
                BUZZER_PORT &= ~(1 << BUZZER_PIN);       // Ensure buzzer is OFF
                
                // If continuous mode is enabled, restart the melody
                if (continuous_melody) {
                    melody_step = 0;        // Reset to first note
                    tone_counter = 0;       // Reset duration counter
                    _delay_ms(200);         // Short pause between repetitions
                } else {
                    // Otherwise, stop the melody
                    melody_playing = 0;
                    melody_enabled = 0;
                }
                break;
        }
        
        // Toggle tone state for pulse width modulation effect
        // Using modulo 4 creates different waveform patterns for variety in tones
        tone_state = (tone_state + 1) % 4;
    }
}

/**
 * ISR for TWI Communication (I2C)
 * 
 * This interrupt service routine handles I2C communication with the master device.
 * It processes incoming commands and data, and prepares responses when the master
 * requests data, such as distance measurements.
 */
ISR(TWI_vect) {
    static uint8_t twi_state = 0;       // State tracking for multi-byte commands
    
    // Get TWI status from status register (masked to remove prescaler bits)
    uint8_t status = TWSR & 0xF8;
    
    switch (status) {
        // Slave Receiver mode
        case TW_SR_SLA_ACK:       // 0x60: SLA+W received, ACK returned
            twi_state = 0;        // Reset state for new command sequence
            break;
            
        case TW_SR_DATA_ACK:      // 0x80: Data received, ACK returned
            if (twi_state == 0) {
                command = TWDR;   // First byte is command code
                twi_state = 1;
            } else if (twi_state == 1) {
                data = TWDR;      // Second byte is data value
                command_received = 1;  // Set flag that complete command is received
                twi_state = 0;
                
                // Process critical commands immediately for responsive operation
                if (command == CMD_DOOR_OPEN) {
                    DOOR_LED_PORT |= (1 << DOOR_LED_PIN);  // Turn on door LED immediately
                } else if (command == CMD_DOOR_CLOSE) {
                    DOOR_LED_PORT &= ~(1 << DOOR_LED_PIN);  // Turn off door LED immediately
                } else if (command == CMD_PLAY_MELODY) {
                    play_emergency_melody();  // Start emergency melody immediately
                } else if (command == CMD_STOP_MELODY) {
                    stop_melody();  // Stop melody immediately
                } else if (command == CMD_CHECK_DISTANCE) {
                    // Perform distance measurement immediately
                    distance_cm = measure_distance();
                }
            }
            break;
            
        case TW_ST_SLA_ACK:       // 0xA8: SLA+R received, ACK returned
            // Prepare data to send back to master (first byte)
            if (command == CMD_CHECK_DISTANCE) {
                // Return the lower byte of distance measurement first
                TWDR = (uint8_t)(distance_cm & 0xFF);
            } else {
                // Default response if no specific data requested
                TWDR = 0;
            }
            break;
            
        case TW_ST_DATA_ACK:      // 0xB8: Data transmitted, ACK received
            // If master wants more data, send the upper byte of distance
            if (command == CMD_CHECK_DISTANCE) {
                TWDR = (uint8_t)(distance_cm >> 8);  // Upper byte of 16-bit distance
            } else {
                // Default second byte
                TWDR = 0;
            }
            break;
            
        case TW_BUS_ERROR:        // 0x00: Bus error due to illegal START/STOP condition
            // Reset TWI interface to recover from error
            TWCR = (1 << TWSTO) | (1 << TWINT) | (1 << TWEN) | (1 << TWIE);
            break;
            
        default:
            // For other TWI states, just clear interrupt flag to continue
            TWCR |= (1 << TWINT);
            break;
    }
    
    // Clear interrupt flag, keep TWI enabled with acknowledgment and interrupt enabled
    TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE);
}

/**
 * Initialize the system
 * 
 * Sets up all hardware components, including I/O pins, TWI communication,
 * timer for melody generation, and initial system state.
 */
void system_init(void) {
    // Configure LED and buzzer pins as outputs
    MOVEMENT_LED_DDR |= (1 << MOVEMENT_LED_PIN);  // Movement LED pin as output
    DOOR_LED_DDR |= (1 << DOOR_LED_PIN);          // Door LED pin as output
    BUZZER_DDR |= (1 << BUZZER_PIN);              // Buzzer pin as output
    
    // Configure ultrasonic sensor pins
    TRIG_DDR |= (1 << TRIG_PIN);      // TRIG pin as output
    ECHO_DDR &= ~(1 << ECHO_BIT);     // ECHO pin as input (no pull-up)
    
    // Initialize all outputs to LOW (inactive state)
    MOVEMENT_LED_PORT &= ~(1 << MOVEMENT_LED_PIN);  // Movement LED off
    DOOR_LED_PORT &= ~(1 << DOOR_LED_PIN);          // Door LED off
    BUZZER_PORT &= ~(1 << BUZZER_PIN);              // Buzzer off
    TRIG_PORT &= ~(1 << TRIG_PIN);                  // Ensure TRIG starts LOW
    
    // Initialize TWI as slave with specified address
    TWI_init_slave(SLAVE_ADDR);
    
    // Initialize Timer1 for melody generation
    // CTC mode (Clear Timer on Compare), prescaler 64
    TCCR1A = 0;  // Normal operation, no PWM
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);  // CTC mode, prescaler 64
    OCR1A = 12499;  // For approximately 0.05 second intervals: 16MHz/64/12500 = 20Hz
    TIMSK1 |= (1 << OCIE1A);  // Enable Timer1 Compare Match A interrupt
    
    // Enable global interrupts
    sei();
    
    // Initialize state variables
    emergency_state = 0;   // Not in emergency state
    melody_playing = 0;    // No melody playing
    melody_enabled = 0;    // Melody generation disabled
    
    // Test all outputs to verify hardware connections
    test_outputs();
}

/**
 * Test all outputs to verify hardware connections
 * 
 * Sequentially activates each output device to verify hardware
 * is properly connected and functioning on startup.
 */
void test_outputs(void) {
    // Test movement LED
    MOVEMENT_LED_PORT |= (1 << MOVEMENT_LED_PIN);   // Turn on
    _delay_ms(500);                                 // Keep on for 500ms
    MOVEMENT_LED_PORT &= ~(1 << MOVEMENT_LED_PIN);  // Turn off
    
    // Test door LED
    DOOR_LED_PORT |= (1 << DOOR_LED_PIN);           // Turn on
    _delay_ms(500);                                 // Keep on for 500ms
    DOOR_LED_PORT &= ~(1 << DOOR_LED_PIN);          // Turn off
    
    // Test buzzer with two short beeps
    BUZZER_PORT |= (1 << BUZZER_PIN);               // Turn on
    _delay_ms(200);                                 // First beep
    BUZZER_PORT &= ~(1 << BUZZER_PIN);              // Turn off
    _delay_ms(200);                                 // Pause between beeps
    BUZZER_PORT |= (1 << BUZZER_PIN);               // Turn on
    _delay_ms(200);                                 // Second beep
    BUZZER_PORT &= ~(1 << BUZZER_PIN);              // Turn off
    
    // Test ultrasonic sensor by taking an initial measurement
    measure_distance();
}

/**
 * Initialize TWI as slave device
 * 
 * Sets up the TWI hardware for I2C communication where this Arduino acts as
 * a slave device listening for commands from the master.
 * 
 * @param address The 7-bit I2C slave address for this device
 */
void TWI_init_slave(uint8_t address) {
    // Set slave address (shifted left by 1 as required by AVR hardware)
    TWAR = address << 1;
    
    // Enable TWI, enable acknowledgment, and enable TWI interrupt
    TWCR = (1 << TWEN) | (1 << TWEA) | (1 << TWIE);
}

/**
 * Generate simple beep sound with specified duration
 * 
 * Activates the buzzer for a fixed duration to create a beep sound.
 * Uses fixed duration options to ensure timing accuracy.
 * 
 * @param duration_ms The duration in milliseconds (50, 100, 200, or 300)
 */
void beep(uint8_t duration_ms) {
    BUZZER_PORT |= (1 << BUZZER_PIN);  // Turn buzzer on
    
    // Uses fixed delay values for timing accuracy
    if (duration_ms == 50) {
        _delay_ms(50);
    } else if (duration_ms == 100) {
        _delay_ms(100);
    } else if (duration_ms == 200) {
        _delay_ms(200);
    } else if (duration_ms == 300) {
        _delay_ms(300);
    } else {
        // Default delay (100ms) if invalid duration specified
        _delay_ms(100);
    }
    
    BUZZER_PORT &= ~(1 << BUZZER_PIN);  // Turn buzzer off
}

/**
 * Blink movement LED specified number of times
 * 
 * Creates a visual indicator by blinking the movement LED in a
 * regular pattern. Used for status indications and alerts.
 * 
 * @param times Number of times to blink the LED
 */
void blink_movement_led(uint8_t times) {
    // First, ensure LED is off to start with consistent state
    MOVEMENT_LED_PORT &= ~(1 << MOVEMENT_LED_PIN);
    
    // Blink the specified number of times
    for (uint8_t i = 0; i < times; i++) {
        // Turn on LED
        MOVEMENT_LED_PORT |= (1 << MOVEMENT_LED_PIN);
        _delay_ms(200);  // On duration
        
        // Turn off LED
        MOVEMENT_LED_PORT &= ~(1 << MOVEMENT_LED_PIN);
        _delay_ms(200);  // Off duration
    }
}

/**
 * Measure distance using ultrasonic sensor HC-SR04
 * 
 * Sends a trigger pulse to the ultrasonic sensor and measures the time
 * until the echo is received. Converts this time to distance in centimeters.
 * 
 * @return Distance in centimeters, or 0 if measurement times out
 */
uint16_t measure_distance(void) {
    uint16_t duration = 0;    // Time counter for pulse width measurement
    uint16_t distance = 0;    // Calculated distance in centimeters
    
    // Ensures trigger pin is low before starting (required by sensor protocol)
    TRIG_PORT &= ~(1 << TRIG_PIN);
    _delay_us(2);  // Short delay to ensure low state is registered
    
    // Send 10μs pulse to trigger sensor measurement
    TRIG_PORT |= (1 << TRIG_PIN);    // Set trigger HIGH
    _delay_us(10);                   // Hold for exactly 10μs as per HC-SR04 specification
    TRIG_PORT &= ~(1 << TRIG_PIN);   // Set trigger LOW
    
    // Wait for echo pin to go HIGH (start of echo pulse)
    while (!(ECHO_PIN & (1 << ECHO_BIT))) {
        if (duration > 30000) {  // Timeout after ~30ms (corresponds to ~5m range)
            return 0;            // Return 0 if no echo detected
        }
        duration++;
        _delay_us(1);  // 1μs increment for timing
    }
    
    // Reset duration counter for measuring pulse width
    duration = 0;
    
    // Measure how long the echo pin stays HIGH
    while (ECHO_PIN & (1 << ECHO_BIT)) {
        duration++;
        _delay_us(1);  // 1μs increment for timing
        if (duration > 30000) {  // Timeout after ~30ms (corresponds to ~5m range)
            return 0;            // Return 0 if measurement exceeds valid range
        }
    }
    
    // Calculate distance in centimeters
    // Sound travels at 343m/s, or 34300cm/s
    // Time is in microseconds, so formula is:
    // distance = (duration * 34300) / 2 / 1000000
    // Simplified to: distance = duration / 58
    distance = duration / 58;
    
    return distance;
}

/**
 * Start playing emergency melody
 * 
 * Initiates the alarm sequence for emergency situations.
 * The melody will play continuously until explicitly stopped.
 */
void play_emergency_melody(void) {
    melody_step = 0;          // Start from the first note in sequence
    melody_enabled = 1;       // Enable melody generation in timer ISR
    melody_playing = 1;       // Set flag that melody is active
    continuous_melody = 1;    // Set to continuous mode for emergency
    
    // Make an immediate confirmation sound to indicate alarm activation
    BUZZER_PORT |= (1 << BUZZER_PIN);   // On
    _delay_ms(200);                     // Initial beep
    BUZZER_PORT &= ~(1 << BUZZER_PIN);  // Off
    _delay_ms(100);                     // Short pause
    BUZZER_PORT |= (1 << BUZZER_PIN);   // On
    _delay_ms(100);                     // Second shorter beep
    BUZZER_PORT &= ~(1 << BUZZER_PIN);  // Off
}

/**
 * Stop currently playing melody
 * 
 * Halts any active alarm or notification sound and ensures
 * the buzzer is turned off.
 */
void stop_melody(void) {
    melody_enabled = 0;       // Disable melody generation in timer ISR
    melody_playing = 0;       // Clear melody active flag
    continuous_melody = 0;    // Disable continuous mode
    BUZZER_PORT &= ~(1 << BUZZER_PIN);  // Ensure buzzer is turned off
}

/**
 * Play a tone pattern when elevator reaches a floor
 * 
 * Generates a distinctive three-beep pattern to notify
 * passengers that the elevator has arrived at the destination floor.
 */
void play_floor_reach_tone(void) {
    // Three beeps for floor reached notification
    beep(100);      // First short beep
    _delay_ms(50);  // Short pause
    beep(100);      // Second short beep
    _delay_ms(50);  // Short pause
    beep(200);      // Final longer beep
}

/**
 * Handle received command from master
 * 
 * Processes commands received via I2C and performs the
 * corresponding actions on hardware outputs.
 */
void handle_command(void) {
    switch (command) {
        case CMD_IDLE:
            // Turn off movement LED in idle state
            MOVEMENT_LED_PORT &= ~(1 << MOVEMENT_LED_PIN);
            emergency_state = 0;  // Clear emergency state
            break;
            
        case CMD_MOVING_UP:
        case CMD_MOVING_DOWN:
            // Turn on movement LED to indicate elevator is moving
            MOVEMENT_LED_PORT |= (1 << MOVEMENT_LED_PIN);
            break;
            
        case CMD_DOOR_OPEN:
            // Turn on door LED to indicate door is open
            DOOR_LED_PORT |= (1 << DOOR_LED_PIN);
            break;
            
        case CMD_DOOR_CLOSE:
            // Turn off door LED to indicate door is closed
            DOOR_LED_PORT &= ~(1 << DOOR_LED_PIN);
            break;
            
        case CMD_EMERGENCY:
            // Set emergency state and blink movement LED 3 times
            emergency_state = 1;
            blink_movement_led(3);  // Visual indication of emergency
            break;
            
        case CMD_FAULT:
            // Blink movement LED 3 times to indicate fault condition
            blink_movement_led(3);
            break;
            
        case CMD_PLAY_MELODY:
            // Start playing emergency melody
            play_emergency_melody();
            break;
            
        case CMD_STOP_MELODY:
            // Stop the melody
            stop_melody();
            break;
            
        case CMD_FLOOR_REACHED:
            // Turn off movement LED and play floor reach notification
            MOVEMENT_LED_PORT &= ~(1 << MOVEMENT_LED_PIN);
            play_floor_reach_tone();
            break;
            
        case CMD_MAINTENANCE:
            // Enter maintenance mode - blink both LEDs twice as visual indicator
            for (uint8_t i = 0; i < 2; i++) {
                MOVEMENT_LED_PORT |= (1 << MOVEMENT_LED_PIN);    // Both LEDs on
                DOOR_LED_PORT |= (1 << DOOR_LED_PIN);
                _delay_ms(300);
                MOVEMENT_LED_PORT &= ~(1 << MOVEMENT_LED_PIN);   // Both LEDs off
                DOOR_LED_PORT &= ~(1 << DOOR_LED_PIN);
                _delay_ms(300);
            }
            break;
            
        case CMD_CHECK_DISTANCE:
            // Measurement already done in ISR when command was received
            // Nothing more to do here
            break;
            
        default:
            // Unknown command - do nothing
            break;
    }
    
    // Reset command received flag to indicate command has been processed
    command_received = 0;
}

/**
 * Main function - Program entry point
 * 
 * Initializes the system and contains the main control loop
 * that continuously processes commands and maintains system state.
 * 
 * @return Never returns (infinite loop)
 */
int main(void) {
    // Initialize system hardware and state
    system_init();
    
    // Main control loop - runs continuously
    while (1) {
        // Process command from master if one was received
        if (command_received) {
            handle_command();
        }
        
        // If in emergency state and LED is not currently on, keep it blinking
        // This creates a continuous visual indication of emergency condition
        if (emergency_state && !(MOVEMENT_LED_PORT & (1 << MOVEMENT_LED_PIN))) {
            // Blink the LED if it's not already on from another operation
            MOVEMENT_LED_PORT |= (1 << MOVEMENT_LED_PIN);    // Turn on
            _delay_ms(500);                                  // Half-second on
            MOVEMENT_LED_PORT &= ~(1 << MOVEMENT_LED_PIN);   // Turn off
            _delay_ms(500);                                  // Half-second off
        }
    }
    
    return 0;  // Will never reach here due to infinite loop
}
