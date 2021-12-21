
#ifndef __CONFIG_H_
#define __CONFIG_H_

/**
 * Wi-Fi SSID
 */
#ifndef WIFI_SSID
#define WIFI_SSID "VIETTEL_ Thien"
#endif

/**
 * Wi-Fi password
 */
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "0918489189"
#endif

/**
 * MQTT broker hostname or IP
 */
#ifndef MQTT_HOSTNAME
#define MQTT_HOSTNAME "mqtt.flespi.io"
#endif

/**
 * MQTT port
 */
#ifndef MQTT_PORT
#define MQTT_PORT 1883
#endif

/**
 * MQTT token - MQTT username
 */
#ifndef MQTT_USERNAME
#define MQTT_USERNAME "FlespiToken 2g0FTgXUK2vCDhILxKms889h0q5ZFojvu9YcdMULCdd5lLEGjRWiNTnC9grCuyVN"
#endif                  //nho cap nhat token o ca 2 code

/**
 * Topic name for MQTT published
 */
#ifndef MQTT_TOPIC
#define MQTT_TOPIC "test"
#endif

/**
 * Unique ID of this device in the system
 */
#ifndef DEVICE_ID
#define DEVICE_ID "e0xv2695"
#endif

/**
 * Measurement period in ms
 */
#ifndef MEASUREMENT_INTERVAL
#define MEASUREMENT_INTERVAL 60000
#endif

/**
 * Offset in ms to measurement period calculated as: sample_utc_ms % MEASUREMENT_INTERVAL
 */
#ifndef MEASUREMENT_OFFSET
#define MEASUREMENT_OFFSET 0
#endif

#endif /* MAIN_CONFIG_H_ */