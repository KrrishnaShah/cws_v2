#include <cstring>
#include "position_sensor.h"
#include "trace.h"
#include "Arduino.h"

static s_position_sensor_data_t position_data;
static MT6701I2C position_sensor(&Wire);

void position_sensor_app_init(void)
{
  position_sensor.begin();
  position_sensor.setClock();
  memset(&position_data, 0, sizeof(s_position_sensor_data_t));
}

int position_sensor_is_connected(void)
{
  return position_sensor.isConnected();
}


void position_sensor_read_values(void) {

  position_data.raw_angle = position_sensor.getRawAngle();
  position_data.output_type = position_sensor.getOutputType();
  position_data.output_mode = position_sensor.getOutputMode();
  position_data.radians_angle = position_sensor.getRadiansAngle();
  position_data.degrees_angle = position_sensor.getDegreesAngle();
  position_data.output_resolution_ABZ = position_sensor.getOutputResolutionABZ();
  position_data.output_resolution_UVW = position_sensor.getOutputResolutionUVW();
  position_data.output_rotation_direction = position_sensor.getOutputRotationDirection();

  TRACE("Raw: %d", position_data.raw_angle);
  TRACE("Degrees: %f", position_data.degrees_angle);
  TRACE("Radians: %f", position_data.radians_angle);
  TRACE("Output type: %s", position_data.output_type == MT6701I2_OUTPUT_TYPE_ABZ ? "ABZ" : "UVW");
  TRACE("Output mode: %s", position_data.output_mode == MT6701I2_OUTPUT_MODE_ANALOG ? "ANALOGUE": "PWM");
  TRACE("Resolution UVW: %d", position_data.output_resolution_UVW);
  TRACE("Resolution ABZ: %d", position_data.output_resolution_ABZ);
  TRACE("Rotation direction: %s", position_data.output_rotation_direction == MT6701I2_DIRECTION_CLOCKWISE ? "CLOCKWISE" : "ANTICLOCKWISE");
}

void position_sensor_save_new_settings(void) {
  Serial.print("Write Resolution ABZ: ");
  word new_res_abz = 732;
  if(position_sensor.setOutputResolutionABZVerify(new_res_abz)) {
    Serial.println("OK");
  } else {
    Serial.println("ERROR");
  }

  Serial.print("Write Output Mode PWM: ");
  if(position_sensor.setOutputModePWMVerify()) {
    Serial.println("OK");
  } else {
    Serial.println("ERROR");
  }

  Serial.print("Write Rotation Direction Clockwise: ");
  if(position_sensor.setOutputRotationDirectionClockwiseVerify()) {
    Serial.println("OK");
  } else {
    Serial.println("ERROR");
  }

  Serial.println("Saving New Values...");
  position_sensor.saveNewValues();
  delay(700); // >600мс
  Serial.println("Saved Successfully. Reconnect Power");
  
  while(1);
}
