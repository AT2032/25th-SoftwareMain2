//Analog UARTの宣言
#define SerialMain Serial1 //Main1層目からのUART
#define SerialWireless Serial1 //TWELITE

char TWE_BUF[256]; //TWELITE BUF

#include <TORICA_SD.h>
int cs_SD = 28;
TORICA_SD sd(cs_SD, false);
char SD_BUF[256];

#include <TORICA_UART.h>
TORICA_UART Main_UART(&SerialMain);

const int O_SPK = 28;
const int L_SD = 27;

#include <Adafruit_NeoPixel.h>
int NEO_Power = 11;
int NEO_PIN = 12;
#define NUMPIXELS 1

Adafruit_NeoPixel pixels(NUMPIXELS, NEO_PIN, NEO_GBR + NEO_KHZ800);

volatile int time_ms = 0;
volatile int air_is_alive = 0;
volatile int under_is_alive = 0;
volatile int flight_phase = 0;
volatile int speed_level = 0;

volatile float estimated_altitude_lake_m = 0;
volatile float altitude_dps_urm_offset_m = 0;

// ---- sensor data value  ----
//    data_マイコン名_センサー名_データ種類_単位
//main電装
//BNO055
volatile float data_main_bno_accx_mss = 0;
volatile float data_main_bno_accy_mss = 0;
volatile float data_main_bno_accz_mss = 0;
volatile float data_main_bno_qw = 0;
volatile float data_main_bno_qx = 0;
volatile float data_main_bno_qy = 0;
volatile float data_main_bno_qz = 0;
volatile float data_main_bno_roll = 0;
volatile float data_main_bno_pitch = 0;
volatile float data_main_bno_yaw = 0;
//DPS310
volatile float data_main_dps_pressure_hPa = 0;
volatile float data_main_dps_temperature_deg = 0;
volatile float data_main_dps_altitude_m = 0;
//GPS
volatile uint8_t data_main_gps_hour = 0;
volatile uint8_t data_main_gps_minute = 0;
volatile uint8_t data_main_gps_second = 0;
volatile uint8_t data_main_gps_centisecond = 0;
volatile double data_main_gps_latitude_deg = 0;
volatile double data_main_gps_longitude_deg = 0;
volatile double data_main_gps_altitude_m = 0;

//Under電装部
volatile float data_under_dps_pressure_hPa = 0;
volatile float data_under_dps_temperature_deg = 0;
volatile float data_under_dps_altitude_m = 0;
volatile float data_under_urm_altitude_m = 0;

//Airdata電装部
volatile float data_air_dps_pressure_hPa = 0;
volatile float data_air_dps_temperature_deg = 0;
volatile float data_air_dps_altitude_m = 0;
volatile float data_air_sdp_differentialPressure_Pa = 0;
volatile float data_air_sdp_airspeed_ms = 0;
volatile float data_air_AoA_angle_deg = 0;
volatile float data_air_AoS_angle_deg = 0;
volatile float data_air_bno_accx_mss = 0;
volatile float data_air_bno_accy_mss = 0;
volatile float data_air_bno_accz_mss = 0;
volatile float data_air_bno_qw = 0;
volatile float data_air_bno_qx = 0;
volatile float data_air_bno_qy = 0;
volatile float data_air_bno_qz = 0;
volatile float data_air_bno_roll = 0;
volatile float data_air_bno_pitch = 0;
volatile float data_air_bno_yaw = 0;
//ICS基盤
volatile int data_ics_angle = 0;

// ----------------------------


void setup() {
  SerialMain.setFIFOSize(1024);
  Serial.begin(115200);
  SerialMain.begin(115200);  //Main1+TWELITE
  SerialWireless.print("loading...\n\n");

  pinMode(NEO_Power, OUTPUT);
  digitalWrite(NEO_Power, HIGH);
  pixels.begin();

  pinMode(L_SD,OUTPUT);

  sd.begin();

  delay(100); //delay for setup1
}


void setup1() {
  pinMode(16, OUTPUT);
  pinMode(25, OUTPUT);
  pinMode(17, OUTPUT);
  pinMode(16, INPUT);
  pinMode(25, INPUT);
  pinMode(17, INPUT);
}

void loop() {
  uint32_t ISR_now_time = millis();
  static uint32_t ISR_last_time = 0;

  Serial.println(millis());
  polling_UART();

  ISR_last_time = millis();


}

void loop1() {
  digitalWrite(16, !digitalRead(16));
  digitalWrite(25, !digitalRead(25));
  digitalWrite(17, !digitalRead(17));

  speaker();

  TWE_downlink();

}

void polling_UART() {
  static int UART_count = 0;
  while (Main_UART.readUART()) {
    digitalWrite(L_SD, !digitalRead(L_SD));
    pixels.clear();
    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    pixels.show();
    if (UART_count == 0) {
      int readnum = Main_UART.readUART();
      int loop_count_data_num = 11;
      if (readnum == loop_count_data_num) {
        time_ms = Main_UART.UART_data[0];
        data_main_bno_accx_mss = Main_UART.UART_data[1];
        data_main_bno_accy_mss = Main_UART.UART_data[2];
        data_main_bno_accz_mss = Main_UART.UART_data[3];
        data_main_bno_qw = Main_UART.UART_data[4];
        data_main_bno_qx = Main_UART.UART_data[5];
        data_main_bno_qy = Main_UART.UART_data[6];
        data_main_bno_qz = Main_UART.UART_data[7];
        data_main_bno_roll = Main_UART.UART_data[8];
        data_main_bno_pitch = Main_UART.UART_data[9];
        data_main_bno_yaw = Main_UART.UART_data[10];
        sprintf(SD_BUF, "%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",  //11個
            time_ms, data_main_bno_accx_mss, data_main_bno_accy_mss, data_main_bno_accz_mss, 
            data_main_bno_qw, data_main_bno_qx, data_main_bno_qy, data_main_bno_qz, 
            data_main_bno_roll, data_main_bno_pitch, data_main_bno_yaw);
      }
    }
    if (UART_count == 1) {
      int readnum = Main_UART.readUART();
      int loop_count_data_num = 11;
      if (readnum == loop_count_data_num) {
        estimated_altitude_lake_m = Main_UART.UART_data[0];
        altitude_dps_urm_offset_m = Main_UART.UART_data[1];
        flight_phase = Main_UART.UART_data[2];
        speed_level = Main_UART.UART_data[3];
        data_main_dps_pressure_hPa = Main_UART.UART_data[4];
        data_main_dps_temperature_deg = Main_UART.UART_data[5];
        data_main_dps_altitude_m = Main_UART.UART_data[6];
        data_under_dps_pressure_hPa = Main_UART.UART_data[7];
        data_under_dps_temperature_deg = Main_UART.UART_data[8];
        data_under_dps_altitude_m = Main_UART.UART_data[9];
        data_under_urm_altitude_m = Main_UART.UART_data[10];
        sprintf(SD_BUF, "%.2f,%.2f,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,", //11個
            estimated_altitude_lake_m, altitude_dps_urm_offset_m, flight_phase, speed_level,
            data_main_dps_pressure_hPa, data_main_dps_temperature_deg, data_main_dps_altitude_m, data_under_dps_pressure_hPa,
            data_under_dps_temperature_deg, data_under_dps_altitude_m, data_under_urm_altitude_m);
      }
    }
    if (UART_count == 2) {
      int readnum = Main_UART.readUART();
      int loop_count_data_num = 8;
      if (readnum == loop_count_data_num) {
        data_air_dps_pressure_hPa = Main_UART.UART_data[0];
        data_air_dps_temperature_deg = Main_UART.UART_data[1];
        data_air_dps_altitude_m = Main_UART.UART_data[2];
        data_air_sdp_differentialPressure_Pa = Main_UART.UART_data[3];
        data_air_sdp_airspeed_ms = Main_UART.UART_data[4];
        data_air_AoA_angle_deg = Main_UART.UART_data[5];
        data_air_AoS_angle_deg = Main_UART.UART_data[6];
        data_ics_angle = Main_UART.UART_data[7];
        sprintf(SD_BUF, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,",  //8個
            data_air_dps_pressure_hPa, data_air_dps_temperature_deg, data_air_dps_altitude_m,
            data_air_sdp_differentialPressure_Pa, data_air_sdp_airspeed_ms, data_air_AoA_angle_deg,
            data_air_AoS_angle_deg, data_ics_angle);
      }
    }
    if (UART_count == 3) {
      int readnum = Main_UART.readUART();
      int loop_count_data_num = 10;
      if (readnum == loop_count_data_num) {
        data_air_bno_accx_mss = Main_UART.UART_data[0];
        data_air_bno_accy_mss = Main_UART.UART_data[1];
        data_air_bno_accz_mss = Main_UART.UART_data[2];
        data_air_bno_qw = Main_UART.UART_data[3];
        data_air_bno_qx = Main_UART.UART_data[4];
        data_air_bno_qy = Main_UART.UART_data[5];
        data_air_bno_qz = Main_UART.UART_data[6];
        data_air_bno_roll = Main_UART.UART_data[7];
        data_air_bno_pitch = Main_UART.UART_data[8];
        data_air_bno_yaw = Main_UART.UART_data[9];
        sprintf(SD_BUF, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,",   //10個
            data_air_bno_accx_mss, data_air_bno_accy_mss, data_air_bno_accz_mss,
            data_air_bno_qw, data_air_bno_qx, data_air_bno_qy, data_air_bno_qz,
            data_air_bno_roll, data_air_bno_pitch, data_air_bno_yaw);
      }
    }
    if (UART_count == 4) {
      int readnum = Main_UART.readUART();
      int loop_count_data_num = 9;
      if (readnum == loop_count_data_num) {
        data_main_gps_hour = Main_UART.UART_data[0];
        data_main_gps_minute = Main_UART.UART_data[1];
        data_main_gps_second = Main_UART.UART_data[2];
        data_main_gps_centisecond = Main_UART.UART_data[3];
        data_main_gps_latitude_deg = Main_UART.UART_data[4];
        data_main_gps_longitude_deg = Main_UART.UART_data[5];
        data_main_gps_altitude_m = Main_UART.UART_data[6];
        under_is_alive = Main_UART.UART_data[7];
        air_is_alive = Main_UART.UART_data[8];
       sprintf(SD_BUF, "%u,%u,%u.%u,%10.7lf,%10.7lf,%5.2lf,%d,%d\n",    //9個
            data_main_gps_hour, data_main_gps_minute, data_main_gps_second, data_main_gps_centisecond,
            data_main_gps_latitude_deg, data_main_gps_longitude_deg, data_main_gps_altitude_m, under_is_alive, air_is_alive);
      }
      UART_count = -1;
    }
    sd.add_str(SD_BUF);
    sd.flash();
    pixels.clear();
    pixels.show();
    UART_count++;
  }
}


void TWE_downlink() {
  static uint8_t TWE_downlink_type = 0;
  static uint32_t TWE_last_send_time = millis() - 1000;
  if (TWE_downlink_type == 0 && millis() - TWE_last_send_time >= 2000) {
    SerialWireless.print("\n\n\n");
    SerialWireless.print("MAIN\n");
    sprintf(TWE_BUF, "accX    accY    accZ\n%+06.2f  %+06.2f  %+06.2f\n", data_main_bno_accx_mss, data_main_bno_accy_mss, data_main_bno_accz_mss);
    SerialWireless.print(TWE_BUF);
    sprintf(TWE_BUF, "roll(left+)   pitch   yaw\n %+06.2f        %+06.2f  %+06.2f\n", data_main_bno_roll, data_main_bno_pitch, data_main_bno_yaw);
    SerialWireless.print(TWE_BUF);
    TWE_downlink_type++;
    TWE_last_send_time = millis();
  } else if (TWE_downlink_type == 1 && millis() - TWE_last_send_time >= 400) {
    sprintf(TWE_BUF, "pressure        temp    alt\n%+06.2f        %+06.2f  %+06.2f\n", data_main_dps_pressure_hPa, data_main_dps_temperature_deg, data_main_dps_altitude_m);
    SerialWireless.print(TWE_BUF);
    SerialWireless.print("\n");
    TWE_downlink_type++;
    TWE_last_send_time = millis();
  } else if (TWE_downlink_type == 2 && millis() - TWE_last_send_time >= 400) {
    SerialWireless.print("UNDER : ");
    if (under_is_alive) {
      SerialWireless.print("alive\n");
    } else {
      SerialWireless.print("dead\n");
    }
    sprintf(TWE_BUF, "pressure        temp    alt\n%+08.2f        %+06.2f  %+06.2f\n", data_under_dps_pressure_hPa, data_under_dps_temperature_deg, data_under_dps_altitude_m);
    SerialWireless.print(TWE_BUF);
    sprintf(TWE_BUF, "sonic_alt\n%+06.2f\n", data_under_urm_altitude_m);
    SerialWireless.print(TWE_BUF);
    //sprintf(TWE_BUF, "%+06.2f\n", filtered_under_urm_altitude_m.get());
    //SerialWireless.print(TWE_BUF);
    SerialWireless.print("\n");
    TWE_downlink_type++;
    TWE_last_send_time = millis();
  } else if (TWE_downlink_type == 3 && millis() - TWE_last_send_time >= 400) {
    SerialWireless.print("AIR : ");
    if (air_is_alive) {
      SerialWireless.print("alive\n");
    } else {
      SerialWireless.print("dead\n");
    }
    sprintf(TWE_BUF, "accX    accY    accZ\n%+06.2f  %+06.2f  %+06.2f\n", data_air_bno_accx_mss, data_air_bno_accy_mss, data_air_bno_accz_mss);
    SerialWireless.print(TWE_BUF);
    sprintf(TWE_BUF, "roll(left+)   pitch   yaw\n %+06.2f        %+06.2f  %+06.2f\n", data_air_bno_roll, data_air_bno_pitch, data_air_bno_yaw);
    SerialWireless.print(TWE_BUF);
    sprintf(TWE_BUF, "AoA   AoS\n %+06.2f        %+06.2f\n", data_air_AoA_angle_deg, data_air_AoS_angle_deg);
    SerialWireless.print(TWE_BUF);
    sprintf(TWE_BUF, "pressure        temp    alt\n%+08.2f        %+06.2f  %+06.2f\n", data_air_dps_pressure_hPa, data_air_dps_temperature_deg, data_air_dps_altitude_m);
    SerialWireless.print(TWE_BUF);
    sprintf(TWE_BUF, "diffPressure    AirSpeed\n%+09.3f       %+06.2f\n", data_air_sdp_differentialPressure_Pa, data_air_sdp_airspeed_ms);
    SerialWireless.print(TWE_BUF);
    //sprintf(TWE_BUF, "                %+06.2f\n", filtered_airspeed_ms.get());
    //SerialWireless.print(TWE_BUF);
    SerialWireless.print("\n");
    TWE_downlink_type++;
    TWE_last_send_time = millis();
  } else if (TWE_downlink_type == 4 && millis() - TWE_last_send_time >= 400) {
    SerialWireless.print("ICS(joystick)\n");
    sprintf(TWE_BUF, "angle (center=7500)\n%d\n", data_ics_angle);
    SerialWireless.print(TWE_BUF);

    SerialWireless.print("estimated_altitude_lake_m\n");
    sprintf(TWE_BUF, "%+08.3f\n", estimated_altitude_lake_m);
    SerialWireless.print(TWE_BUF);

    SerialWireless.print("Flight Phase\n");
    sprintf(TWE_BUF, "%d\n",flight_phase);
    SerialWireless.print(TWE_BUF);

   SerialWireless.print("Speed Level\n");
    sprintf(TWE_BUF, "%d\n",speed_level);
    SerialWireless.print(TWE_BUF);


    sprintf(TWE_BUF, "latitude:%10.7lf longitude:%10.7lf", data_main_gps_latitude_deg, data_main_gps_longitude_deg);
    SerialWireless.println(TWE_BUF);

    //SerialWireless.flush();

    //Reset downlink type
    TWE_downlink_type = 0;
    TWE_last_send_time = millis();
  }
}



void speaker() {
  static int sound_freq = 440;
  static int last_flight_phase = -1;
  static int spk_flag = 0;
  static uint32_t speaker_last_change_time = millis();
  static uint32_t sound_duration = 100; // 音が出ている時間

  switch (speed_level) {
    case 0:
      sound_freq = 440;
      break;
    case 1:
      sound_freq = 880;
      break;
    case 2:
      sound_freq = 1320;
      break;
    default:
      break;
  }

  int interval;
  switch (flight_phase) {
    case 0:
      interval = 1000;
      break;  
    case 1:
      interval = 500;
      break;
    case 2:    
      interval = 250;
      break;
    case 3:
      interval = 125;
      break;
    default:
      interval = 0;
      break;
  }

  uint32_t current_time = millis();
  uint32_t off_duration = interval - sound_duration;

  if (flight_phase != 0) {
    if (spk_flag == 0 && (current_time - speaker_last_change_time) > off_duration) {
      tone(O_SPK, sound_freq);
      speaker_last_change_time = current_time;
      spk_flag = 1;
    } else if (spk_flag == 1 && (current_time - speaker_last_change_time) > sound_duration) {
      noTone(O_SPK);
      speaker_last_change_time = current_time;
      spk_flag = 0;
    }
  }
}
