#include <WiFi.h>
#include <Servo.h>

const char* ssid = "your_SSID";  // Replace with your Wi-Fi network name
const char* password = "your_PASSWORD";  // Replace with your Wi-Fi password

WiFiServer server(80); // Server will listen on port 80

// Defining servos
Servo base_servo, arm1_servo, arm2_servo;

// Angle limits
#define BASE_MIN 10
#define BASE_MAX 170
#define ARM1_MIN 90
#define ARM1_MAX 175
#define ARM2_MIN 0
#define ARM2_MAX 100

void setup() {
  Serial.begin(115200);  // Start serial communication
  
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  
  Serial.println("Connected to WiFi");
  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.localIP());  // Print the ESP32's IP address

  server.begin();  // Start the server

  // Attach servos to their respective pins
  base_servo.attach(5);
  arm1_servo.attach(19);
  arm2_servo.attach(18);

  // Set servos to 90 degrees initially
  base_servo.write(90);
  arm1_servo.write(90);
  arm2_servo.write(90);
}

void loop() {
  WiFiClient client = server.available();  // Check if a client has connected
  
  if (client) {
    Serial.println("New Client Connected");
    String request = "";  // String to store the HTTP request from the client
    
    // Read the HTTP request from the client
    while (client.available()) {
      char c = client.read();
      request += c;
    }
    
    Serial.println("Request received: " + request);  // Debugging output
    
    // Parse angles and speed from the HTTP request
    int base_angle = 90, arm1_angle = 90, arm2_angle = 90, speed = 10;  // Default values
    if (request.indexOf("base_angle=") != -1 && request.indexOf("arm1_angle=") != -1 && request.indexOf("arm2_angle=") != -1 && request.indexOf("speed=") != -1) {
      sscanf(request.c_str(), "base_angle=%d&arm1_angle=%d&arm2_angle=%d&speed=%d", &base_angle, &arm1_angle, &arm2_angle, &speed);
      
      base_angle = adjustAngleWithinLimits(base_angle, BASE_MIN, BASE_MAX, "Base Servo");
      arm1_angle = adjustAngleWithinLimits(arm1_angle, ARM1_MIN, ARM1_MAX, "Arm1 Servo");
      arm2_angle = adjustAngleWithinLimits(arm2_angle, ARM2_MIN, ARM2_MAX, "Arm2 Servo");

      // Move servos smoothly
      bool movementExecuted = true;
      movementExecuted &= moveServoSmoothly(base_servo, base_angle, speed);
      movementExecuted &= moveServoSmoothly(arm1_servo, arm1_angle, speed);
      movementExecuted &= moveServoSmoothly(arm2_servo, arm2_angle, speed);

      // Send response back to the client
      if (movementExecuted) {
        client.println("Angles executed successfully.");
      } else {
        client.println("Movement not executed.");
      }
    } else {
      client.println("Invalid request format.");
    }

    // Close the client connection
    client.stop();
    Serial.println("Client Disconnected");
  }
}

// Function to adjust an angle within its limits and send a message if adjusted
int adjustAngleWithinLimits(int angle, int minLimit, int maxLimit, const char *servoName) {
  if (angle < minLimit) {
    Serial.printf("%s angle below minimum limit, adjusting to %d.\n", servoName, minLimit);
    return minLimit;  // Adjust to minimum limit
  }
  if (angle > maxLimit) {
    Serial.printf("%s angle above maximum limit, adjusting to %d.\n", servoName, maxLimit);
    return maxLimit;  // Adjust to maximum limit
  }
  return angle;  // Angle is within limits
}

// Function to move a servo smoothly to a target angle
bool moveServoSmoothly(Servo &servo, int targetAngle, int speed) {
  int currentAngle = servo.read();  // Get the current angle of the servo
  int stepDelay = 1000 / speed;     // Calculate delay between steps based on speed

  // Smoothly move the servo to the target angle
  if (currentAngle < targetAngle) {
    for (int pos = currentAngle; pos <= targetAngle; pos++) {
      servo.write(pos);       // Move servo to the next position
      delay(stepDelay);       // Wait to control speed
    }
  } else {
    for (int pos = currentAngle; pos >= targetAngle; pos--) {
      servo.write(pos);       // Move servo to the next position
      delay(stepDelay);       // Wait to control speed
    }
  }
  
  // Return true if servo moved, false otherwise
  return currentAngle != targetAngle;
}
