/*
# Implements a PID controller for the pH sensor feedback control system
# Product: analog pH meter
# SKU : SEN0161
*/

#define PWM1 9 // pH up solution
#define PWM2 3 // pH down solution

#define phSensorPin A0
#define phOffset 0.05 // deviation compensate

#define phTarget 5.8 // set target pH

int sensorValue = 0;
unsigned long int avgValue;
float b;
int buf[10];
int temp;

// PID variables
int phValue = 0;
long prevT = 0;
float ePrev = 0;
float eIntegral = 0;

void setup()
{
    Serial.begin(9600);
    Serial.println("target pH");
}

void loop()
{
    for (int i = 0; i < 10; i++)
    {
        buf[i] = analogRead(phSensorPin);
        delay(10);
    }

    for (int i = 0; i < 9; i++)
    {
        for (int j = i+1; j < 10; j++)
        {
            if (buf[i] > buf[j])
            {
                temp = buf[j];
                buf[i] = buf[j];
                buf[j] = temp;
            }
        }
    }

    avgValue = 0;
    // avgValue is composed of the 6 center samples
    for (int i = 2; i < 8; i ++)
    {
        avgValue += buf[i];
    }
    avgValue /= 6;
    float pHVoltage = (float)(avgValue * 5.0 / 1024);
    float phValue = 3.5 * pHVoltage + phOffset;
    delay(20);

    // PID constants
    float kp = 100;
    float kd = 0.00001;
    float ki = 0.3;

    // time difference
    long currT = micros();
    float deltaT = ((float) (currT - prevT) / 1000000);
    prevT = currT;

    // error
    float e = phValue - phTarget;

    // derivative
    float dedt = (e - ePrev) / (deltaT);

    // integral
    eIntegral += e * deltaT;

    // control signal
    float u = kp*e + kd*dedt + ki*eIntegral;

    // pump power
    float power = fabs(u);
    if (power > 255)
    {
        power = 255;
    }
    else if (power < 76.5)
    {
        power = 0;
    }


    // select which pump to control
    if (u < 0)
    {
        setPump1(power);
        setPump2(0);
    }
    else
    {
        setPump1(0);
        setPump2(power);
    }

    ePrev = e;

    Serial.print(phTarget);
    Serial.print(" ");
    Serial.print(phValue);
    Serial.println();
}

void setPump1(int power)
{
    analogWrite(PWM1, power);
}

void setPump2(int power)
{
    analogWrite(PWM2, power);
}
