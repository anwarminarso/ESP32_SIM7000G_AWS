// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon					Serial
#define TINY_GSM_USE_GPRS			true
#define TINY_GSM_USE_WIFI			false
#define TINY_GSM_MODEM_SIM7000SSL
#define TINY_GSM_RX_BUFFER			1024		// Set RX buffer to 1Kb
#define SerialAT					Serial1
#define uS_TO_S_FACTOR				1000000ULL  // Conversion factor for micro seconds to seconds
#define TIME_TO_SLEEP				60          // Time ESP32 will go to sleep (in seconds)

#define GSM_AUTOBAUD_MIN 9600
#define GSM_AUTOBAUD_MAX 115200


//Arduino JSON Library
//https://www.arduino.cc/reference/en/libraries/arduinojson/
#include <ArduinoJson.h>


//GSM Library
//https://www.arduino.cc/reference/en/libraries/tinygsm/
#include <TinyGsmClient.h>

//Set your AWS Certificate
#include "AWS_Certificate.h"


//I2C Library
// Built-in Library
#include <Wire.h>


//Adafruit Unitifed Library
//https://www.arduino.cc/reference/en/libraries/adafruit-unified-sensor/
#include <Adafruit_Sensor.h>

//Adafruit BME680 Library
//https://www.arduino.cc/reference/en/libraries/adafruit-bme680-library/
#include <Adafruit_BME680.h>

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME680 bme; // I2C

#define GSM_PIN					""					//set GSM PIN Code, if any
#define GPRS_APN				"m2mautotronic"		//SET TO YOUR APN, required
#define GPRS_USER				""					//SET TO YOUR GPRS USERNAME, if any
#define GPRS_PASS				""					//SET TO YOUR GPRS PASSWORD, if any

//Assign SerialAT Pin (Modem)
#define UART_BAUD           9600
#define PIN_DTR             25
#define PIN_TX              27
#define PIN_RX              26
#define PWR_PIN             4




const char* ThingName = "esp32_sim7000g";			// your thing name on AWS
//const char* mqttServer = "a3a7hex4sympda-ats.iot.us-east-1.amazonaws.com";
const char* mqttServer = "a3a7hex4sympda.iot.us-east-1.amazonaws.com";
const uint16_t mqttPort = 8883;						// default port AWS IoT
const char* mqttPubTopic = "esp32/pub";				// your policy for publish
const char* mqttSubTopic = "esp32/sub";				// your policy for subscribe

#define PUBLISH_INTERVAL	3000 //every 3 seconds


TinyGsm modem(SerialAT);
TinyGsmClient client(modem);

void enableGPS(void)
{
	// Set SIM7000G GPIO4 LOW ,turn on GPS power
	// CMD:AT+SGPIO=0,4,1,1
	// Only in version 20200415 is there a function to control GPS power
	modem.sendAT("+SGPIO=0,4,1,1");
	if (modem.waitResponse(10000L) != 1) {
		DBG(" SGPIO=0,4,1,1 false ");
	}
	modem.enableGPS();
}

void disableGPS(void)
{
	modem.disableGPS();
	// Set SIM7000G GPIO4 LOW ,turn off GPS power
	// CMD:AT+SGPIO=0,4,1,0
	// Only in version 20200415 is there a function to control GPS power
	modem.sendAT("+SGPIO=0,4,1,0");
	if (modem.waitResponse(10000L) != 1) {
		DBG(" SGPIO=0,4,1,0 false ");
	}
}

void modemPowerOn()
{
	pinMode(PWR_PIN, OUTPUT);
	digitalWrite(PWR_PIN, LOW);
	delay(1000);    //Datasheet Ton mintues = 1S
	digitalWrite(PWR_PIN, HIGH);
}

void modemPowerOff()
{
	pinMode(PWR_PIN, OUTPUT);
	digitalWrite(PWR_PIN, LOW);
	delay(1500);    //Datasheet Ton mintues = 1.2S
	digitalWrite(PWR_PIN, HIGH);
}

void modemRestart()
{
	modemPowerOff();
	delay(1000);
	modemPowerOn();
}
void initGSM() {
	String res;
	if (!modem.init()) {
		modemRestart();
		delay(2000);
		SerialMon.println("Failed to restart modem, attempting to continue without restarting");
	}
	String name = modem.getModemName();
	SerialMon.println("Modem Name: " + name);

	String modemInfo = modem.getModemInfo();
	SerialMon.println("Modem Info: " + modemInfo);

	// Unlock your SIM card with a PIN if needed
	if (GSM_PIN && modem.getSimStatus() != 3) {
		modem.simUnlock(GSM_PIN);
	}
	bool isConnected = false;
	for (int i = 0; i <= 4; i++) {
		uint8_t network[] = {
			2,  /*Automatic*/
			13, /*GSM only*/
			38, /*LTE only*/
			51  /*GSM and LTE only*/
		};
		SerialMon.printf("Try %d method\n", network[i]);
		modem.setNetworkMode(network[i]);
		delay(3000);
		bool isConnected = false;
		int tryCount = 60;
		while (tryCount--) {
			int16_t signal = modem.getSignalQuality();
			SerialMon.print("Signal: ");
			SerialMon.print(signal);
			SerialMon.print(" ");
			SerialMon.print("Status: ");
			isConnected = modem.isNetworkConnected();
			SerialMon.println(isConnected ? "CONNECTED" : "FAIL");
			if (isConnected) {
				break;
			}
			delay(1000);
		}
		if (isConnected) {
			break;
		}
	}

	if (!modem.gprsConnect(GPRS_APN, GPRS_USER, GPRS_PASS)) {
		SerialMon.println("Failed to connect GPRS");
		delay(3000);
		return;
	}
	if (modem.isGprsConnected()) {
		Serial.println("GPRS connected");
	}
	res = "";
	//// set Timezone GMT+7 and sync system date & time
	//modem.sendAT("+CNTP=pool.ntp.org,7,1,2");
	//if (modem.waitResponse(1000L, res) == 1) {
	//	res.replace(GSM_NL "OK" GSM_NL, "");
	//	SerialMon.println(res);
	//}
	
	// set Timezone GMT+2 and sync system date & time
	modem.sendAT("+CNTP=pool.ntp.org,2,1,2");
	if (modem.waitResponse(1000L, res) == 1) {
		res.replace(GSM_NL "OK" GSM_NL, "");
		SerialMon.println(res);
	}
	

	res = "";
	// get Current Date & Time with selected timezone from server
	modem.sendAT("+CNTP");
	if (modem.waitResponse(1000L, res) == 1) {
		res.replace(GSM_NL "OK" GSM_NL, "");
		SerialMon.println(res);
	}

	res = "";
	// get System date & time
	modem.sendAT("+CCLK?");
	if (modem.waitResponse(1000L, res) == 1) {
		res.replace(GSM_NL "OK" GSM_NL, "");
		SerialMon.println(res);
	}

	SerialMon.println("Init Modem Completed");
}

void waitModemResponse(uint32_t timeout, String& data, char* terminateString) {
	uint32_t startMillis = millis();
	do {
		while (modem.stream.available() > 0) {
			int8_t a = modem.stream.read();
			if (a <= 0)
				continue;
			data += static_cast<char>(a);
			if (terminateString != nullptr) {
				if (data.endsWith(GF(terminateString)))
					break;
			}
		}
	} while (millis() - startMillis < timeout);
}


bool initFile() {
	String res = "";
	modem.sendAT("+CFSTERM");
	if (modem.waitResponse(10000L, res) == 1) {
	}
	SerialMon.println(res);
	res = "";
	modem.sendAT("+CFSINIT");
	if (modem.waitResponse(10000L, res) != 1) {
		SerialMon.println("INIT File error");
		return false;
	}
	return true;
}
void writeFile(char* fileName, unsigned int fileSize, char* fileData, bool overwrite = false) {
	String res;
	res = "";
	modem.sendAT("+CFSGFIS=3,\"", fileName, "\"");
	if (modem.waitResponse(10000L, res) == 1) {
		if (!overwrite)
			return;
	}
	SerialMon.print("Begin Writing File: ");
	SerialMon.println(fileName);
	modem.sendAT("+CFSWFILE=3,\"", fileName, "\",0,", fileSize, ",10000");
	delay(100);
	res = "";

	modem.stream.write(fileData, fileSize);
	SerialMon.println("Waiting for response");
	waitModemResponse(10000, res, "DOWNLOAD");
	SerialMon.println(res);
	res = "";
	modem.sendAT("+CFSGFIS=3,\"", fileName, "\"");
	if (modem.waitResponse(15000L, res) != 1) {
		SerialMon.print("ERROR READ File ");
		SerialMon.println(fileName);
		SerialMon.println(res);
		return;
	}

}
bool assignCertAndBroker() {
	
	String res;
	res = "";
	modem.sendAT("+CSSLCFG=\"convert\",2,\"VeriSignCA.pem\"");
	if (modem.waitResponse(10000L, res) != 1) {
		SerialMon.println("Failure to assign VeriSignCA");
		return false;
	}
	modem.sendAT("+CSSLCFG=\"convert\",1,\"certificate.crt\",\"private.key\"");
	if (modem.waitResponse(10000L, res) != 1) {
		SerialMon.println("Failure to assign Device Certificate");
		return false;
	}
	modem.sendAT("+SMSSL=1,VeriSignCA.pem,certificate.crt");
	if (modem.waitResponse(10000L, res) != 1) {
		SerialMon.println("Failure to assign Device Certificate");
		return false;
	}
	modem.sendAT("+CSSLCFG=\"sslversion\",0,3");
	if (modem.waitResponse(10000L, res) != 1) {
		SerialMon.println("Failure to assign SSL Version");
		return false;
	}
	modem.sendAT("+SMCONF=url,", mqttServer ,",", mqttPort);
	if (modem.waitResponse(10000L, res) != 1) {
		SerialMon.println("Failure to set MQTT Server");
		return false;
	}
	modem.sendAT("+SMCONF=\"clientid\",\"", ThingName, "\"");
	if (modem.waitResponse(10000L, res) != 1) {
		SerialMon.println("Failure to set MQTT Client Id");
		return false;
	}
	modem.sendAT("+SMCONF=\"KEEPTIME\",60");
	if (modem.waitResponse(10000L, res) != 1) {
		SerialMon.println("Failure to set Keep Alive");
		return false;
	}
	res = "";
	modem.sendAT("+SMCONF?");
	if (modem.waitResponse(10000L, res) == 1) {
		SerialMon.println(res);
	}
	return true;
}
bool connectToAWS() {
	modem.sendAT("+SMCONN");
	if (modem.waitResponse(10000L) != 1) {
		SerialMon.println("Failure to connect MQTT Server");
		return false;
	}
	return true;
}
inline int16_t streamGetIntBefore(char lastChar) {
	char   buf[7];
	size_t bytesRead = modem.stream.readBytesUntil(
		lastChar, buf, static_cast<size_t>(7));
	// if we read 7 or more bytes, it's an overflow
	if (bytesRead && bytesRead < 7) {
		buf[bytesRead] = '\0';
		int16_t res = atoi(buf);
		return res;
	}
	return -9999;
}
bool reconnectAWS(uint count = 0) {
	// Loop until we're reconnected
	uint val = 0;
	bool result = IsAWSConnected();
	while (!result) {
		SerialMon.println("Attempting MQTT connection...");
		// Attempt to connect
		val++;
		if (connectToAWS()) {
			SerialMon.println("connected");
			result = true;
			break;
		}
		else {
			if (count > 0 && val == count) {
				result = false;
				break;
			}
			SerialMon.println("Try again in 5 seconds");
			// Wait 5 seconds before retrying
			delay(5000);
		}
	}
	return result;
}

bool IsAWSConnected() {
	modem.sendAT(GF("+SMSTATE?"));
	if (modem.waitResponse(GF("+SMSTATE:")) != 1) { return false; }
	int16_t res = streamGetIntBefore('\n');
	modem.waitResponse();
	if (res == 1)
		return true;
	else
		return false;
}
bool disconnectAWS() {
	modem.sendAT(GF("+SMDISC"));
	return modem.waitResponse(10000L) == 1;
}

bool publishMQTT(const char* topic, const char* message, unsigned int messageLength) {
	modem.sendAT("+SMPUB=\"", topic, "\",", messageLength, ",0,0");
	modem.stream.readStringUntil('>');
	SerialAT.println(message);
	String res;
	if (modem.waitResponse(10000L, res) != 1) {
		SerialMon.println("Failure to publish");
		SerialMon.println(res);
		return false;
	}
	return true;
}


void initBME680() {
	if (!bme.begin()) {
		SerialMon.println("Could not find a valid BME680 sensor, check wiring!");
		while (1);
	}
	// Set up oversampling and filter initialization
	bme.setTemperatureOversampling(BME680_OS_8X);
	bme.setHumidityOversampling(BME680_OS_2X);
	bme.setPressureOversampling(BME680_OS_4X);
	bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
	bme.setGasHeater(320, 150); // 320*C for 150 ms
}
void setup() {
	SerialMon.begin(115200);
	initBME680();
	
	modemPowerOn();
	SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
	initGSM();
	enableGPS();
	
	if (initFile()) {
		writeFile((char*)&VERISIGN_CERT_CA_FILE_NAME, sizeof(VERISIGN_CERT_CA), (char*)&VERISIGN_CERT_CA, false);
		writeFile((char*)&AWS_CERT_CA_FILE_NAME, sizeof(AWS_CERT_CA), (char*)&AWS_CERT_CA, false);
		writeFile((char*)&AWS_CERT_CRT_FILE_NAME, sizeof(AWS_CERT_CRT), (char*)&AWS_CERT_CRT, false);
		writeFile((char*)&AWS_CERT_PRIVATE_FILE_NAME, sizeof(AWS_CERT_PRIVATE), (char*)&AWS_CERT_PRIVATE, false);
		assignCertAndBroker();
	}
	if (IsAWSConnected())
		disconnectAWS();
}

void loop() {
	if (reconnectAWS(5)) {
		DynamicJsonDocument doc(512);
		String jsonMessage;

		if (!bme.performReading()) {
			Serial.println("Failed to perform reading");
		}
		doc["bme"]["temperature"] = bme.temperature; // °C
		doc["bme"]["humidity"] = bme.humidity;
		doc["bme"]["pressure"] = bme.pressure / 100; //hpa
		doc["bme"]["gas_resistance"] = bme.gas_resistance / 1000.0; //KOhms
		doc["bme"]["altitude"] = bme.readAltitude(SEALEVELPRESSURE_HPA);

		float lat, lon, speed, alt, acc;
		int vsat, usat, year, month, day, hour, minute, second;
		if (!modem.getGPS(&lat, &lon, &speed, &alt, &vsat, &usat, &acc, &year, &month, &day, &hour, &minute, &second)) {
			Serial.println("Failed to get GPS Position");
		}
		doc["gps"]["latitude"] = lat;
		doc["gps"]["longitute"] = lon;
		doc["gps"]["altitude"] = alt;
		doc["gps"]["speed"] = speed;
		doc["gps"]["accuracy"] = acc;
		doc["gps"]["year"] = year;
		doc["gps"]["month"] = month;
		doc["gps"]["day"] = day;
		doc["gps"]["hour"] = hour;
		doc["gps"]["minute"] = minute;
		doc["gps"]["second"] = acc;
		
		serializeJson(doc, jsonMessage);
		if (publishMQTT(mqttPubTopic, jsonMessage.c_str(), jsonMessage.length())) {
			SerialMon.print("Published : ");
			SerialMon.println(jsonMessage);
		}
		else
			SerialMon.println("Failure to publish MQTT");
	}
	else {
		SerialMon.println("RESTART ESP");
		ESP.restart();
	}
	delay(PUBLISH_INTERVAL);
}