/*==========================================================================
// Dual BTS7960 motor test with current protection (ESP32)
// Commands: 1F 1R 2F 2R BF BR S   (+ Enter, line ending = New Line)
//
// Protection rules:
//   - Instant trip: ignored for the first 300 ms after a command
//   - Sustained trip: needs 12 samples over the limit (~250 ms).
//     Resets after 3 consecutive samples under the limit, so scattered
//     spikes do not slowly accumulate into a false trip.
//   - Either trip stops BOTH motors
//==========================================================================*/

// ---------------- Pins ----------------
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
const float SENSE_R = 1000.0; // ohms
const float K_ILIS = 8500.0;  // BTS7960 mirror ratio
const float ADC_FS_V = 3.1;   // ADC full scale volts

const float SUSTAIN_AMPS = 1.5;  // sustained limit  -> ~90 counts
const float INSTANT_AMPS = 45.0; // instant limit
const int SUSTAIN_MS = 250;      // how long sustained limit must be exceeded
const int GRACE_MS = 300;        // instant trip ignored this long after start
const int SAMPLE_MS = 20;        // how often we check

const int SUSTAIN_SAMPLES = SUSTAIN_MS / SAMPLE_MS; // 12 samples
const int CLEAR_SAMPLES = 3;                        // under-limit samples needed to reset

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

// ---------------- Motor control ----------------
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

// ---------------- Convert amps to ADC counts ----------------
int ampsToCounts(float amps)
{
    float volts = amps * (SENSE_R / K_ILIS) * (DUTY / 255.0);
    int counts = volts * 4095.0 / ADC_FS_V;
    if (counts > 4090)
        counts = 4090;
    return counts;
}

// ---------------- Protection ----------------
void checkProtection(int c1, int c2)
{

    if (!m1On && !m2On)
        return;

    bool pastGrace = (millis() - commandTime >= GRACE_MS);

    // ---- Instant trip - ignored for the first 300 ms ----
    if (pastGrace)
    {
        if (m1On && c1 >= instantCounts)
        {
            Serial.print("INSTANT TRIP - M1 counts=");
            Serial.println(c1);
            stopAll();
            mode = "TRIPPED (M1 instant)";
            return;
        }
        if (m2On && c2 >= instantCounts)
        {
            Serial.print("INSTANT TRIP - M2 counts=");
            Serial.println(c2);
            stopAll();
            mode = "TRIPPED (M2 instant)";
            return;
        }
    }

    // ---- Sustained trip ----
    if (m1On)
    {
        if (c1 >= sustainCounts)
        {
            m1OverCount++;
            m1UnderRun = 0;
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
            m1UnderRun++;
            if (m1UnderRun >= CLEAR_SAMPLES)
            {
                m1OverCount = 0;
                m1UnderRun = 0;
            }
        }
    }

    if (m2On)
    {
        if (c2 >= sustainCounts)
        {
            m2OverCount++;
            m2UnderRun = 0;
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
            m2UnderRun++;
            if (m2UnderRun >= CLEAR_SAMPLES)
            {
                m2OverCount = 0;
                m2UnderRun = 0;
            }
        }
    }
}

// ---------------- Commands ----------------
void runCommand(String cmd)
{
    cmd.trim();
    cmd.toUpperCase();

    stopAll();

    if (cmd == "1F")
    {
        motor1(DUTY, 0);
        m1On = true;
        mode = "M1 FWD";
    }
    else if (cmd == "1R")
    {
        motor1(0, DUTY);
        m1On = true;
        mode = "M1 REV";
    }
    else if (cmd == "2F")
    {
        motor2(DUTY, 0);
        m2On = true;
        mode = "M2 FWD";
    }
    else if (cmd == "2R")
    {
        motor2(0, DUTY);
        m2On = true;
        mode = "M2 REV";
    }
    else if (cmd == "BF")
    {
        motor1(DUTY, 0);
        motor2(DUTY, 0);
        m1On = true;
        m2On = true;
        mode = "BOTH FWD";
    }
    else if (cmd == "BR")
    {
        motor1(0, DUTY);
        motor2(0, DUTY);
        m1On = true;
        m2On = true;
        mode = "BOTH REV";
    }
    else if (cmd == "S")
    {
        mode = "STOP";
    }
    else
    {
        Serial.println("Unknown. Use: 1F 1R 2F 2R BF BR S");
        return;
    }

    commandTime = millis();
    Serial.print(">> ");
    Serial.println(mode);
}

// ---------------- Setup ----------------
void setup()
{
    Serial.begin(115200);

    ledcAttach(M1_RPWM, PWM_FREQ, PWM_BITS);
    ledcAttach(M1_LPWM, PWM_FREQ, PWM_BITS);
    ledcAttach(M2_RPWM, PWM_FREQ, PWM_BITS);
    ledcAttach(M2_LPWM, PWM_FREQ, PWM_BITS);

    stopAll();

    sustainCounts = ampsToCounts(SUSTAIN_AMPS);
    instantCounts = ampsToCounts(INSTANT_AMPS);

    Serial.println("Commands: 1F 1R 2F 2R BF BR S");
    Serial.print("Trip points: sustained=");
    Serial.print(sustainCounts);
    Serial.print(" counts   instant=");
    Serial.print(instantCounts);
    Serial.println(" counts");
    Serial.println("mode / M2 / M1 / on / overcounts");
}

// ---------------- Main loop ----------------
void loop()
{
    if (Serial.available())
    {
        runCommand(Serial.readStringUntil('\n'));
    }

    if (millis() - lastSample >= SAMPLE_MS)
    {
        lastSample = millis();

        int c1 = analogRead(SenseM1);
        int c2 = analogRead(SenseM2);

        checkProtection(c1, c2);

        Serial.print(mode);
        Serial.print("\t");
        Serial.print(c2);
        Serial.print("\t");
        Serial.print(c1);
        Serial.print("\t");
        Serial.print(m1On);
        Serial.print(m2On);
        Serial.print("\t");
        Serial.print(m2OverCount);
        Serial.print("/");
        Serial.println(m1OverCount);
    }
}
