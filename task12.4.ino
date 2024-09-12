// PID Control Class Definition
class PID {
  private:
    double Kp;  
    double Ki;  
    double Kd;  
    double setPoint;  
    double integral;  
    double prevError;  
    unsigned long lastTime;  
    double alpha;  
    double filteredOutput;  


  public:
    // Constructor
    PID(double Kp_in, double Ki_in, double Kd_in,double alpha=0.5) {
      Kp = Kp_in;
      Ki = Ki_in;
      Kd = Kd_in;
      integral = 0;
      prevError = 0;
      lastTime = millis();
    }

    void setSetPoint(double setPoint_in) {
      setPoint = setPoint_in;
    }

    // Update the PID controller
    double compute(double input) {
      unsigned long now = millis();
      double timeChange = (double)(now - lastTime) / 1000.0;  // Time difference in seconds
      lastTime = now;

      double error = setPoint - input;  // Calculate error

      // Proportional term
      double Pout = Kp * error;

      // Integral term
      integral += error * timeChange;
      double Iout = Ki * integral;

      // Derivative term
      double derivative = (error - prevError) / timeChange;
      double Dout = Kd * derivative;

      // Total output
      double output = Pout + Iout + Dout;

      filteredOutput = alpha * output + (1 - alpha) * filteredOutput;

      // Save error for next loop
      prevError = error;
      return filteredOutput;
    }
};


PID motorPID(1.0, 0.5, 0.1);  // Initialize PID with Kp, Ki, Kd values
double motorSpeed;  // Current motor speed
double desiredSpeed = 100;  // Desired speed 
double pidOutput;  // PID controller output

void setup() {
  Serial.begin(9600);  
  motorPID.setSetPoint(desiredSpeed); 
  pinMode(9, OUTPUT);  // Set motor control pin as output
}

void loop() {
  // Get the current speed 
  motorSpeed = analogRead(A0); 
  // Convert the sensor value to a speed range (0-255)
  motorSpeed = map(motorSpeed, 0, 1023, 0, 255);

  // Compute the PID output to adjust motor speed
  pidOutput = motorPID.compute(motorSpeed);

  // Control the motor using PWM based on PID output
  analogWrite(9, pidOutput);  // Output to motor (pin 9)

  
  Serial.print("Motor Speed: ");
  Serial.println(motorSpeed);
  Serial.print("PID Output: ");
  Serial.println(pidOutput);

  delay(100);  // Delay for stability
}
