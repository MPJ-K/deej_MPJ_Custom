#include <avr/sleep.h>


// Specify the configuration of the potentiometers.
#define NUM_POTS 5
#define POT_ANALOG_PINS \
  { A0, A1, A2, A3, A4 }

// Specify the serial BAUD rate.
#define BAUD 115200

// Specify the ADC resolution.
#define ADC_MAX 1023

// Specify the maximum distance from the top and bottom of the potentiometer range where the measured value will snap to its limits.
#define TOP_EDGE_SNAP_RANGE 0
#define BOTTOM_EDGE_SNAP_RANGE 0

// Set the potentiometer measurement interval in milliseconds.
#define SAMPLING_INTERVAL 48
// Set the interrupt interval mode, which must be an integer in the range [0, 4].
// Interrupts will happen every 2^INTERRUPT_INTERVAL_MODE milliseconds.
// The number of interrupts per sample is calculated according to: max(SAMPLING_INTERVAL / 2^INTERRUPT_INTERVAL_MODE, 1).
#define INTERRUPT_INTERVAL_MODE 4


// Set up global variables.

// The analog input pins from which to read the potentiometer values.
const int POT_PINS[NUM_POTS] = POT_ANALOG_PINS;

// The current potentiometer values.
int pot_values[NUM_POTS] = { 0 };
// The current potentiometer values scaled by a factor of 100.
long pot_values_scaled[NUM_POTS] = { 0 };
// The current potentiometer values expressed as a percentage of the maximum.
int pot_percentages[NUM_POTS] = { 0 };
// The potentiometer values obtained during the most recent sample.
int new_pot_values[NUM_POTS];

// The power of two that determines the interval between interrupts. For details, see INTERRUPT_INTERVAL_MODE.
const int LOG2_INTERRUPT_INTERVAL = max(min(INTERRUPT_INTERVAL_MODE, 4), 0);
// The interval between interrupts in milliseconds.
const int INTERRUPT_INTERVAL = (1 << LOG2_INTERRUPT_INTERVAL);
// The number of Timer2 compare interrupts that should occur per potentiometer sample.
const int INTERRUPTS_PER_SAMPLE = max(SAMPLING_INTERVAL / INTERRUPT_INTERVAL, 1);

// A counter that will increment by one for every Timer2 compare interrupt.
volatile int interrupt_counter = 0;
// Whether a new potentiometer sample is due.
volatile bool sample_due = false;

// The number of consecutive samples taken without any potentiometer value changing significantly.
int unchanged_sample_counter = 0;
// The maximum allowed number of consecutive samples taken without any potentiometer value changing significantly before sending the values over serial.
const int MAX_UNCHANGED_SAMPLES_BEFORE_TRANSMISSION = 10000 / (INTERRUPT_INTERVAL * INTERRUPTS_PER_SAMPLE);


void sample_pots() {
  // Do a dummy analogRead() to prevent stabilization issues after waking from sleep.
  analogRead(POT_PINS[0]);

  for (int i = 0; i < NUM_POTS; i++) {
    new_pot_values[i] = analogRead(POT_PINS[i]);
  }
}


bool parse_new_samples() {
  // A flag that will be true if any new potentiometer value is significantly different from its old value, and false otherwise.
  bool any_value_changed = false;

  for (int i = 0; i < NUM_POTS; i++) {
    int new_percentage = pot_percentages[i];
    if (new_pot_values[i] + TOP_EDGE_SNAP_RANGE >= ADC_MAX) {
      new_percentage = 100;
    } else if (new_pot_values[i] - BOTTOM_EDGE_SNAP_RANGE <= 0) {
      new_percentage = 0;
    } else if (abs(pot_values_scaled[i] - 100L * new_pot_values[i]) >= ADC_MAX) {
      // To avoid floating point operations, the potentiometer values are scaled by a factor of 100.
      // Integer arithmetic is used to replace the following floating point arithmetic: percentage = round(100 * value / ADC_MAX).
      new_percentage = (100L * new_pot_values[i] + (ADC_MAX / 2)) / ADC_MAX;
    }

    if (new_percentage != pot_percentages[i]) {
      pot_percentages[i] = new_percentage;
      pot_values_scaled[i] = (long)pot_percentages[i] * ADC_MAX;
      pot_values[i] = pot_values_scaled[i] / 100;
      any_value_changed = true;
    }
  }

  return any_value_changed;
}


void send_pot_values() {
  for (int i = 0; i < NUM_POTS; i++) {
    Serial.print(pot_values[i]);
    // Separate each value with a vertical bar.
    if (i < NUM_POTS - 1) {
      Serial.print("|");
    }
  }
  Serial.println();
}


ISR(TIMER2_COMPA_vect) {
  // This is the interrupt service routine for Timer2 compare match interrupts.
  interrupt_counter++;
  if (!(interrupt_counter % INTERRUPTS_PER_SAMPLE)) { sample_due = true; }
}


void setup() {
  Serial.begin(BAUD);

  for (int i = 0; i < NUM_POTS; i++) {
    pinMode(POT_PINS[i], INPUT);
    // Take some void analogRead() measurements to 'warm up' the ADC.
    analogRead(POT_PINS[i]);
  }

  set_sleep_mode(SLEEP_MODE_IDLE);

  // Set up Timer2 to run sample_pots() periodically using interrupts.
  TCCR2B = 0;           // Ensure that Timer2 is not running.
  TCNT2 = 0;            // Ensure that Timer2 starts at zero.
  ASSR &= ~(1 << AS2);  // Ensure that Timer2 is not in asynchronous mode.

  TCCR2A = (1 << WGM21);                             // Set Timer2 to Clear Timer on Compare Match (CTC) mode.
  OCR2A = LOG2_INTERRUPT_INTERVAL == 3 ? 124 : 249;  // Set the compare register such that interrupts happen once every 2^LOG2_INTERRUPT_INTERVAL milliseconds.

  TIFR2 |= (1 << OCF2A);   // Ensure that no interrupt flag is set.
  TIMSK2 = (1 << OCIE2A);  // Enable Timer2 compare interrupts.

  interrupts();
  TCCR2B = min(LOG2_INTERRUPT_INTERVAL + 4, 7);  // Start Timer2 with a prescaler such that interrupts happen once every 2^LOG2_INTERRUPT_INTERVAL milliseconds.
}


void loop() {
  // Sleep until the next interrupt. Execution will continue after an interrupt has been handled.
  sleep_mode();

  // Check whether a new sample is due.
  if (!sample_due) { return; }
  sample_due = false;

  sample_pots();

  // Check whether any potentiometer value changed significantly.
  if (parse_new_samples()) {
    unchanged_sample_counter = 0;
    send_pot_values();
  } else {
    unchanged_sample_counter++;

    // Send the potentiometer values every now and then regardless of whether they have changed.
    if (!(unchanged_sample_counter % MAX_UNCHANGED_SAMPLES_BEFORE_TRANSMISSION)) {
      send_pot_values();
    }
  }
}
