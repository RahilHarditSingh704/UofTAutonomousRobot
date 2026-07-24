/*==========================================================================
// Dual BTS7960 robot: BLE control + current protection (ESP32)
//
// BLE commands (single chars, matching the App Inventor blocks):
//   F forward   B backward   C rotate CW   X rotate CCW   S stop
//
// Current protection is UNCHANGED from the serial version:
//   - Instant trip ignored for first 300 ms after a command
//   - Sustained trip needs 12 samples over limit (~250 ms)
//   - Either trip stops BOTH motors
//==========================================================================*/

#include <BLEDevice.h>
#include <BLEServer.h>

// ===== BLE identifiers - must match the App Inventor blocks =====
#define DEVICE_NAME "MyRobot"
#define SERVICE_UUID "19B10000-E8F2-537E-4F6C-D104768A1214"
#define CHARACTERISTIC_UUID "19B10001-E8F2-537E-4F6C-D104768A1214"

// ---------------- Pins (from the current-sensing code) ----------------
const int SenseM1 = 34; // M1 current sense
const int SenseM2 = 35; // M2 current sense

const int M1_RPWM = 12; // M1 forward
const int M1_LPWM = 13; // M1 reverse
const int M2_RPWM = 18; // M2 forward
const int M2_LPWM = 19; // M2 reverse

// ---------------- PWM ----------------
const uint32_t PWM_FREQ = 20000;
const uint8_t PWM_BITS = 8;
const int DUTY = 100; // 0-255

// ---------------- Protection settings ----------------
const float SENSE_R = 1000.0;
const float K_ILIS = 8500.0;
const float ADC_FS_V = 3.1;

const float SUSTAIN_AMPS = 1.5;
const float INSTANT_AMPS = 45.0;
const int SUSTAIN_MS = 250;
const int GRACE_MS = 300;
const int SAMPLE_MS = 20;

const int SUSTAIN_SAMPLES = SUSTAIN_MS / SAMPLE_MS;
const int CLEAR_SAMPLES = 3;

// ---------------- State ----------------
bool m1On = false;
bool m2On = false;
unsigned long commandTime = 0;
unsigned long lastSample = 0;
String mode = "STOP";

int m1OverCount = 0;
int m2OverCount = 0;
int m1UnderRun = 0;
int m2UnderRun = 0;

int sustainCounts;
int instantCounts;

// Command received over BLE, handled in loop(). volatile: written from
// the BLE task, read from loop().
volatile char pendingCmd = 0;

// ================= MOTOR CONTROL (LEDC) ================================
void motor1(int fwd, int rev)
{
    ledcWrite(M1_RPWM, fwd);
    ledcWrite(M1_LPWM, rev);
}
void motor2(int fwd, int rev)
{
    ledcWrite(M2_RPWM, fwd);
    ledcWrite(M2_LPWM, rev);
}

void stopAll()
{
    motor1(0, 0);
    motor2(0, 0);
    m1On = false;
    m2On = false;
    m1OverCount = 0;
    m2OverCount = 0;
    m1UnderRun = 0;
    m2UnderRun = 0;
}

// ==================== CURRENT PROTECTION (UNCHANGED) ====================
// Convert a given current in amps to counts (the metric given by the current sensing circuit setup)
int ampsToCounts(float amps)
{
    float volts = amps * (SENSE_R / K_ILIS) * (DUTY / 255.0);
    int counts = volts * 4095.0 / ADC_FS_V;
    if (counts > 4090)
        counts = 4090;
    return counts;
}

// Given analogRead values from current-sensing pins on the motor driver for M1 & M2, it checks if a trip is required for M1, M2, or both, and initiates it accordingly.
void checkProtection(int c1, int c2)
{

    // MOTOR STATE GUARD
    // If both motors are currently turned off, skip protection checks entirely.
    if (!m1On && !m2On)
        return;

    // GRACE PERIOD CHECK
    // Check if at least 300 ms (GRACE_MS) have passed since the last command was issued.
    // During this 300 ms window, the motor's initial inrush current spike is ignored.
    bool pastGrace = (millis() - commandTime >= GRACE_MS);

    // INSTANT TRIP PROTECTION (Hard Cutoff / Short Circuit Protection)
    // Only evaluated AFTER the 300 ms grace period has expired.
    if (pastGrace)
    {
        // If Motor 1 is active and current exceeds the instant limit (~45A equivalent)
        if (m1On && c1 >= instantCounts)
        {
            Serial.print("INSTANT TRIP - M1 counts=");
            Serial.println(c1);
            stopAll();
            mode = "TRIPPED (M1 instant)";
            return;
        }
        // If Motor 2 is active and current exceeds the instant limit (~45A equivalent)
        if (m2On && c2 >= instantCounts)
        {
            Serial.print("INSTANT TRIP - M2 counts=");
            Serial.println(c2);
            stopAll();
            mode = "TRIPPED (M2 instant)";
            return;
        }
    }

    // SUSTAINED TRIP PROTECTION - MOTOR 1 (Motor Stall / Overload Protection)
    if (m1On)
    {
        if (c1 >= sustainCounts)
        {
            // Current exceeds sustained threshold (~1.5A): increment breach count & reset clear counter
            m1OverCount++;
            m1UnderRun = 0;

            // If over-current persists for 12 consecutive samples (~250 ms), trigger trip
            if (m1OverCount >= SUSTAIN_SAMPLES)
            {
                Serial.print("SUSTAINED TRIP - M1 counts=");
                Serial.println(c1);
                stopAll();
                mode = "TRIPPED (M1 sustained)";
                return;
            }
        }
        else
        {
            // Current is within safe limits: count clean samples to filter out transient noise spikes
            m1UnderRun++;

            // If current remains normal for 3 consecutive samples, clear the accumulated breach count
            if (m1UnderRun >= CLEAR_SAMPLES)
            {
                m1OverCount = 0;
                m1UnderRun = 0;
            }
        }
    }

    // SUSTAINED TRIP PROTECTION - MOTOR 2 (Motor Stall / Overload Protection)
    if (m2On)
    {
        if (c2 >= sustainCounts)
        {
            // Current exceeds sustained threshold (~1.5A): increment breach count & reset clear counter
            m2OverCount++;
            m2UnderRun = 0;

            // If over-current persists for 12 consecutive samples (~250 ms), trigger trip
            if (m2OverCount >= SUSTAIN_SAMPLES)
            {
                Serial.print("SUSTAINED TRIP - M2 counts=");
                Serial.println(c2);
                stopAll();
                mode = "TRIPPED (M2 sustained)";
                return;
            }
        }
        else
        {
            // Current is within safe limits: count clean samples to filter out transient noise spikes
            m2UnderRun++;

            // If current remains normal for 3 consecutive samples, clear the accumulated breach count
            if (m2UnderRun >= CLEAR_SAMPLES)
            {
                m2OverCount = 0;
                m2UnderRun = 0;
            }
        }
    }
}
// ================== END CURRENT PROTECTION ==============================

// ==================== BLE COMMAND HANDLING =============================
// F/B move both motors together. C/X rotate by driving the motors opposite.
void handleCommand(char cmd)
{
    stopAll(); // clean slate; also resets protection counters

    switch (cmd)
    {
    case 'F':
        motor1(DUTY, 0);
        motor2(DUTY, 0);
        m1On = m2On = true;
        mode = "FWD";
        break;
    case 'B':
        motor1(0, DUTY);
        motor2(0, DUTY);
        m1On = m2On = true;
        mode = "BACK";
        break;
    case 'C':
        motor1(DUTY, 0);
        motor2(0, DUTY);
        m1On = m2On = true;
        mode = "CW";
        break;
    case 'X':
        motor1(0, DUTY);
        motor2(DUTY, 0);
        m1On = m2On = true;
        mode = "CCW";
        break;
    case 'S':
        mode = "STOP";
        break;
    default:
        return; // ignore stray characters, don't restamp the timer
    }

    commandTime = millis();
}

class CommandCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *c)
    {
        String value = c->getValue().c_str();
        if (value.length() > 0)
        {
            pendingCmd = value.charAt(0); // store only; loop() does the work
        }
    }
};

class ServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *s) {}
    void onDisconnect(BLEServer *s)
    {
        pendingCmd = 'S';              // loop() stops the motors
        BLEDevice::startAdvertising(); // allow reconnect
    }
};
// ================== END BLE COMMAND HANDLING ===========================

void setup()
{
    Serial.begin(115200);

    // Set up specific GPIO pins for PWM output to control the motors
    ledcAttach(M1_RPWM, PWM_FREQ, PWM_BITS);
    ledcAttach(M1_LPWM, PWM_FREQ, PWM_BITS);
    ledcAttach(M2_RPWM, PWM_FREQ, PWM_BITS);
    ledcAttach(M2_LPWM, PWM_FREQ, PWM_BITS);

    stopAll();

    // Convert reference current limit values from amps to counts
    sustainCounts = ampsToCounts(SUSTAIN_AMPS);
    instantCounts = ampsToCounts(INSTANT_AMPS);

    Serial.print("Trip points: sustained=");
    Serial.print(sustainCounts);
    Serial.print("   instant=");
    Serial.println(instantCounts);

    // ----- Bluetooth Low Energy (BLE) setup -----
    // This code sets up the BLE stack on the ESP32 so the MIT App Inventor can find the robot, connect to it, and send movement commands

    // Initialize Bluetooth radio with the broadcast name (DEVICE_NAME)
    // (Gives the robot its visible name so you can identify it in your phone's scan list)
    BLEDevice::init(DEVICE_NAME);

    // Create the BLE Server (host) and attach connection/disconnection handlers
    // (Prepares the robot to accept a phone connection and safety-stops if the link drops)
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    // Create a unique service container using the custom SERVICE_UUID
    // (A unique ID code that acts as a folder holding all of the robot's specific features)
    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Create a writable characteristic ("mailbox") and attach incoming data listener
    // (Creates a specific data slot where the phone writes commands like 'F' or 'S')
    BLECharacteristic *pChar = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_WRITE);
    pChar->setCallbacks(new CommandCallbacks());

    // Activate the service
    // (Turns on this specific feature folder so it's ready to handle incoming data)
    pService->start();

    // Broadcast the service UUID so phone apps can discover the robot
    // (Shouts "I am here!" into the air so your mobile app can automatically find it)
    BLEAdvertising *pAdv = BLEDevice::getAdvertising();
    pAdv->addServiceUUID(SERVICE_UUID);
    pAdv->setScanResponse(true);   // Provide extra device info on scan (helps phones connect faster)
    BLEDevice::startAdvertising(); // Start wireless advertising (begins transmitting the signal)

    Serial.println("BLE ready as MyRobot");
}

void loop()
{
    // If the user gives a command, process it and move the motors accordingly
    if (pendingCmd != 0)
    {
        char cmd = pendingCmd;
        pendingCmd = 0;
        handleCommand(cmd);
    }

    // Every SAMPLE_MS amount of time, check if current usage exceeds limits and stop the motors if needed
    if (millis() - lastSample >= SAMPLE_MS)
    {
        lastSample = millis();

        int c1 = analogRead(SenseM1);
        int c2 = analogRead(SenseM2);

        checkProtection(c1, c2);
    }
}