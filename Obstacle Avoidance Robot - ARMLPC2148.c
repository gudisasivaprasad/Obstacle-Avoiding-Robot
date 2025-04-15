#include <lpc214x.h>
#include <stdint.h>

// Define sensor and motor pins
#define TRIG_PIN    (1 << 16)  // P0.16 - Ultrasonic Trigger
#define ECHO_PIN    (1 << 17)  // P0.17 - Ultrasonic Echo
#define IR_LEFT     (1 << 18)  // P0.18 - Left IR sensor
#define IR_RIGHT    (1 << 19)  // P0.19 - Right IR sensor

#define MOTOR1_A    (1 << 0)   // P0.0  - Motor 1 Pin A
#define MOTOR1_B    (1 << 1)   // P0.1  - Motor 1 Pin B
#define MOTOR2_A    (1 << 4)   // P0.4  - Motor 2 Pin A
#define MOTOR2_B    (1 << 5)   // P0.5  - Motor 2 Pin B

// Function prototypes
void delay_ms(uint32_t milliseconds);
void ultrasonic_init(void);
uint32_t get_distance(void);
void motor_init(void);
void move_forward(void);
void move_backward(void);
void turn_left(void);
void turn_right(void);
void stop_motors(void);

int main(void) {
    uint32_t distance = 0;
    uint32_t ir_left_status, ir_right_status;
    
    // Initialize peripherals
    ultrasonic_init();
    motor_init();
    
    // Set IR sensor pins as input
    IODIR0 &= ~(IR_LEFT | IR_RIGHT);
    
    while(1) {
        // Read IR sensors
        ir_left_status = (IOPIN0 & IR_LEFT) ? 0 : 1;    // 0 when obstacle detected
        ir_right_status = (IOPIN0 & IR_RIGHT) ? 0 : 1;  // 0 when obstacle detected
        
        // Get distance from ultrasonic sensor
        distance = get_distance();
        
        // Decision making
        if(distance < 20 || ir_left_status || ir_right_status) {
            // Obstacle detected
            stop_motors();
            delay_ms(500);
            
            // Move backward briefly
            move_backward();
            delay_ms(300);
            stop_motors();
            delay_ms(200);
            
            // Check which side is clearer
            if(ir_left_status && !ir_right_status) {
                // Left IR detects obstacle, turn right
                turn_right();
                delay_ms(400);
            }
            else if(!ir_left_status && ir_right_status) {
                // Right IR detects obstacle, turn left
                turn_left();
                delay_ms(400);
            }
            else {
                // Both IRs detect obstacle or ultrasonic detects front obstacle
                // Turn 180 degrees
                turn_right();
                delay_ms(800);
            }
        }
        else {
            // No obstacle, move forward
            move_forward();
        }
        
        delay_ms(100);  // Small delay between readings
    }
    
    return 0;
}

// Initialize ultrasonic sensor
void ultrasonic_init(void) {
    IODIR0 |= TRIG_PIN;   // Trigger as output
    IODIR0 &= ~ECHO_PIN;  // Echo as input
}

// Get distance from ultrasonic sensor in cm
uint32_t get_distance(void) {
    uint32_t duration = 0;
    uint32_t distance = 0;
    
    // Send trigger pulse
    IOSET0 = TRIG_PIN;
    delay_ms(10);         // 10us trigger pulse
    IOCLR0 = TRIG_PIN;
    
    // Wait for echo to go high
    while(!(IOPIN0 & ECHO_PIN));
    
    // Measure echo pulse width
    while((IOPIN0 & ECHO_PIN)) {
        duration++;
        delay_ms(1);      // Each count = 1us (approximate)
    }
    
    // Calculate distance in cm (speed of sound = 343 m/s = 0.0343 cm/us)
    // Distance = (duration * 0.0343) / 2 (round trip)
    distance = (duration * 17) / 1000;  // Simplified calculation
    
    return distance;
}

// Initialize motor control pins
void motor_init(void) {
    IODIR0 |= (MOTOR1_A | MOTOR1_B | MOTOR2_A | MOTOR2_B);
    IOCLR0 = (MOTOR1_A | MOTOR1_B | MOTOR2_A | MOTOR2_B);  // Initially stop motors
}

// Motor control functions
void move_forward(void) {
    IOSET0 = MOTOR1_A | MOTOR2_A;
    IOCLR0 = MOTOR1_B | MOTOR2_B;
}

void move_backward(void) {
    IOSET0 = MOTOR1_B | MOTOR2_B;
    IOCLR0 = MOTOR1_A | MOTOR2_A;
}

void turn_left(void) {
    IOSET0 = MOTOR1_B | MOTOR2_A;
    IOCLR0 = MOTOR1_A | MOTOR2_B;
}

void turn_right(void) {
    IOSET0 = MOTOR1_A | MOTOR2_B;
    IOCLR0 = MOTOR1_B | MOTOR2_A;
}

void stop_motors(void) {
    IOCLR0 = (MOTOR1_A | MOTOR1_B | MOTOR2_A | MOTOR2_B);
}

// Simple delay function
void delay_ms(uint32_t milliseconds) {
    uint32_t i, j;
    for(i = 0; i < milliseconds; i++)
        for(j = 0; j < 2000; j++);
}