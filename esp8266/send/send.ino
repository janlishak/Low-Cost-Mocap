#include <ESP8266WiFi.h>
#include <espnow.h>

// REPLACE WITH RECEIVER MAC Address
uint8_t broadcastAddress[] = {0x2C, 0x3A, 0xE8, 0x2F, 0x3A, 0xA2};
int ledState = LOW;

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  char msg[128];
} struct_message;

// Create a struct_message called myData
struct_message myData;

unsigned long lastTime = 0;  
unsigned long timerDelay = 20000;  // send readings timer

// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  // Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0){
    digitalWrite(LED_BUILTIN, HIGH);
    // Serial.println("Delivery success");
  }
  else{
    digitalWrite(LED_BUILTIN, LOW);
    // Serial.println("Delivery fail");
  }
}
 
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);  // Initialize the LED_BUILTIN pin as an output
  
  // Init Serial Monitor
  Serial.begin(115200);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
}
 
void loop() {
  if ((millis() - lastTime) > timerDelay) {
    //   // if the LED is off turn it on and vice-versa:
    //   if (ledState == LOW) {
    //     ledState = HIGH;
    //   } else {
    //     ledState = LOW;
    //   }
    // digitalWrite(LED_BUILTIN, ledState);
    lastTime = millis();

    Serial.println("AT+switchdis=1");
  }

    // Check if data is available to read from serial input
  if (Serial.available()) {
    String inputMessage = Serial.readString();
    inputMessage.trim(); 

    // Set values to send
    inputMessage.toCharArray(myData.msg, sizeof(myData.msg));

    // Send message via ESP-NOW
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

    // Print out what we are sending
    // Serial.print("Sending message: ");
    // Serial.println(inputMessage);
  }
}