#include "PPMEncoder.h"

#define OUTPUT_PIN 10
#define NUM_CHANNELS 4  // Total number of channels

const int MIN_VALUE = 1000;  // Minimum value for PPM signal
const int MAX_VALUE = 2000;  // Maximum value for PPM signal

void setup() {
  Serial.begin(9600);  // Initialize serial communication
  ppmEncoder.begin(OUTPUT_PIN);
  Serial.println("Enter channel (0-3) and value (0-100) in the format: channel,value");
}

void loop() {
  takeInput();  // Call the takeInput function to read from serial
}

// Function to take input from Serial and set the corresponding channel value
void takeInput() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');  // Read input string until newline
    input.trim();  // Remove any extra whitespace

    // Parse the input to get channel and value
    int commaIndex = input.indexOf(',');
    if (commaIndex > 0) {
      String channelStr = input.substring(0, commaIndex);
      String valueStr = input.substring(commaIndex + 1);

      int channel = channelStr.toInt();  // Convert channel to an integer
      int percent = valueStr.toInt();    // Convert value to an integer (0-100%)

      // Ensure channel and percent are within valid range
      if (channel >= 0 && channel < NUM_CHANNELS && percent >= 0 && percent <= 100) {
        // Map the percentage (0-100) to the range (1000-2000)
        int mappedValue = map(percent, 0, 100, MIN_VALUE, MAX_VALUE);
        
        // Set the channel to the mapped value
        ppmEncoder.setChannel(channel, mappedValue);
        
        Serial.print("Channel ");
        Serial.print(channel);
        Serial.print(" set to ");
        Serial.print(mappedValue);
        Serial.print(" (from ");
        Serial.print(percent);
        Serial.println("%)");
      } else {
        Serial.println("Invalid input. Please enter channel (0-3) and value (0-100).");
      }
    } else {
      Serial.println("Invalid format. Use: channel,value (e.g., 2,75)");
    }
  }
}



void sweep() {
  for (int channel = 0; channel < NUM_CHANNELS; channel++) {  // Loop through each channel (0 to 3)
    
    // Sweep from 0% to 100%
    for (int percent = 0; percent <= 100; percent++) {
      ppmEncoder.setChannelPercent(channel, percent);  // Set the channel to the current percent value
      delay(50);  // Adjust the delay for smoother/faster transitions
    }
    
    // Sweep back from 100% to 0%
    for (int percent = 100; percent >= 0; percent--) {
      ppmEncoder.setChannelPercent(channel, percent);  // Set the channel to the current percent value
      delay(50);  // Adjust the delay for smoother/faster transitions
    }
    
    delay(1000);  // Delay between each channel sweep (optional)
  }
}


  // Min value
  // ppmEncoder.setChannel(0, 500);
  // ppmEncoder.setChannel(0, PPMEncoder::MIN);

  // Max value
  // ppmEncoder.setChannel(0, 2500);
  // ppmEncoder.setChannel(0, PPMEncoder::MAX);
  // ppmEncoder.setChannelPercent(0, 75);
  // delay(1000);
