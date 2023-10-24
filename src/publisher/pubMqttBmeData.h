//-----------------------------------------------------------------------------
// 2023 Ahoy, https://ahoydtu.de
// Creative Commons - https://creativecommons.org/licenses/by-nc-sa/4.0/deed
//-----------------------------------------------------------------------------

#ifndef __PUB_MQTT_BME_DATA_H__
#define __PUB_MQTT_BME_DATA_H__
#include <Wire.h>

#include <Adafruit_BME280.h>

#include "../utils/dbg.h"
#include "pubMqttDefs.h"

#define ALTITUDE 123 //Altitude of your location (m above sea level)

const float cToKOffset = 273.15;
const uint8_t SDA_PIN = 4;
const uint8_t SCL_PIN = 5;

//Adafruit_BME280 bme(5);

typedef std::function<void(const char *subTopic, const char *payload, bool retained, uint8_t qos)> pubMqttPublisherType;

class PubMqttBmeData {
    public:
        void setup(uint32_t *utcTs) {
            mUtcTimestamp = utcTs;
            mState        = IDLE;

            Wire.begin(SDA_PIN, SCL_PIN);
            status = bme.begin(0x76);

            if (!status) {
                Serial.println("Could not find a valid BME280 sensor, check wiring!");
            }

            mTable[IDLE]        = &PubMqttBmeData::stateIdle;
            mTable[START]       = &PubMqttBmeData::stateStart;
            mTable[SEND_DATA]   = &PubMqttBmeData::stateSend;
        }

        void loop() {
            (this->*mTable[mState])();
            yield();
        }

        bool start(uint32_t uptime) {
            if(mState != IDLE)
                return false;

            if (uptime % 30 == 0) {
                mState = START;
            }

            return true;
        }

        void setPublishFunc(pubMqttPublisherType cb) {
            mPublish = cb;
        }

    private:
        enum State {IDLE, START, SEND_DATA, NUM_STATES};
        typedef void (PubMqttBmeData::*StateFunction)();

        void stateIdle() {
            ; // nothing to do
        }

        void stateStart() {
            mState = SEND_DATA;
        }

        void stateSend() {
            Serial.print(bme.readTemperature());
            bool retained = false;
            uint8_t qos = QOS_0;

            float temperature = bme.readTemperature();
            float humidity_r = bme.readHumidity();
            float humidity = absoluteHumidity(temperature, humidity_r);
            float pressure = bme.readPressure() / 100.0F;
            float pressure_r = bme.seaLevelForAltitude(ALTITUDE, pressure);
            float dew = dewPoint(temperature, humidity_r);

            snprintf(mSubTopic, 32 + MAX_NAME_LENGTH, "%s/%s", "bme", "temperature");
            snprintf(mVal, 40, "%f", temperature);
            mPublish(mSubTopic, mVal, retained, qos);

            snprintf(mSubTopic, 32 + MAX_NAME_LENGTH, "%s/%s", "bme", "dewpoint");
            snprintf(mVal, 40, "%f", dew);
            mPublish(mSubTopic, mVal, retained, qos);

            snprintf(mSubTopic, 32 + MAX_NAME_LENGTH, "%s/%s", "bme", "humidity_abs");
            snprintf(mVal, 40, "%f", humidity);
            mPublish(mSubTopic, mVal, retained, qos);

            snprintf(mSubTopic, 32 + MAX_NAME_LENGTH, "%s/%s", "bme", "humidity");
            snprintf(mVal, 40, "%f", humidity_r);
            mPublish(mSubTopic, mVal, retained, qos);

            snprintf(mSubTopic, 32 + MAX_NAME_LENGTH, "%s/%s", "bme", "pressure");
            snprintf(mVal, 40, "%f", pressure);
            mPublish(mSubTopic, mVal, retained, qos);

            snprintf(mSubTopic, 32 + MAX_NAME_LENGTH, "%s/%s", "bme", "pressure_rel");
            snprintf(mVal, 40, "%f", pressure_r);
            mPublish(mSubTopic, mVal, retained, qos);

            mState = IDLE;
        }

        // Relative to absolute humidity
        // Based on https://carnotcycle.wordpress.com/2012/08/04/how-to-convert-relative-humidity-to-absolute-humidity/
        float absoluteHumidity(float temperature, float humidity) {
          return (13.2471*pow(EULER,17.67*temperature/(temperature+243.5))*humidity/(cToKOffset+temperature));
        }

        // Calculate saturation vapor pressure
        // Based on dew.js, Copyright 2011 Wolfgang Kuehn, Apache License 2.0
        float saturationVaporPressure(float temperature) {
          if(temperature < 173 || temperature > 678) return -112; //Temperature out of range

          float svp = 0;
          if(temperature <= cToKOffset) {
            /**
              * -100-0°C -> Saturation vapor pressure over ice
              * ITS-90 Formulations by Bob Hardy published in
              * "The Proceedings of the Third International
              * Symposium on Humidity & Moisture",
              * Teddington, London, England, April 1998
              */

            svp = exp(-5.8666426e3/temperature + 2.232870244e1 + (1.39387003e-2 + (-3.4262402e-5 + (2.7040955e-8*temperature)) * temperature) * temperature + 6.7063522e-1 * log(temperature));
          }else{
            /**
              * 0°C-400°C -> Saturation vapor pressure over water
              * IAPWS Industrial Formulation 1997
              * for the Thermodynamic Properties of Water and Steam
              * by IAPWS (International Association for the Properties
              * of Water and Steam), Erlangen, Germany, September 1997.
              * Equation 30 in Section 8.1 "The Saturation-Pressure
              * Equation (Basic Equation)"
              */

            const float th = temperature + -0.23855557567849 / (temperature - 0.65017534844798e3);
            const float a  = (th + 0.11670521452767e4) * th + -0.72421316703206e6;
            const float b  = (-0.17073846940092e2 * th + 0.12020824702470e5) * th + -0.32325550322333e7;
            const float c  = (0.14915108613530e2 * th + -0.48232657361591e4) * th + 0.40511340542057e6;

            svp = 2 * c / (-b + sqrt(b * b - 4 * a * c));
            svp *= svp;
            svp *= svp;
            svp *= 1e6;
          }

          yield();

          return svp;
        }


        // Calculate dew point in °C
        // Based on dew.js, Copyright 2011 Wolfgang Kuehn, Apache License 2.0
        float dewPoint(float temperature, float humidity)
        {
          temperature += cToKOffset; //Celsius to Kelvin

          if(humidity < 0 || humidity > 100) return -111; //Invalid humidity
          if(temperature < 173 || temperature > 678) return -112; //Temperature out of range

          humidity = humidity / 100 * saturationVaporPressure(temperature);

          byte mc = 10;

          float xNew;
          float dx;
          float z;

          do {
            dx = temperature / 1000;
            z = saturationVaporPressure(temperature);
            xNew = temperature + dx * (humidity - z) / (saturationVaporPressure(temperature + dx) - z);
            if (abs((xNew - temperature) / xNew) < 0.0001) {
                return xNew - cToKOffset;
            }
            temperature = xNew;
            mc--;
          } while(mc > 0);

          return -113; //Solver did not get a close result
        }

        uint32_t *mUtcTimestamp;
        pubMqttPublisherType mPublish;
        State mState;
        StateFunction mTable[NUM_STATES];

        uint8_t mCmd;
        uint32_t status;
        Adafruit_BME280 bme;
        float mTotal[4];

        uint8_t mPos;

        char mSubTopic[32 + MAX_NAME_LENGTH + 1];
        char mVal[40];
        bool mZeroValues; // makes sure that yield day is sent even if no inverter is online
};

#endif /*__PUB_MQTT_BME_DATA_H__*/