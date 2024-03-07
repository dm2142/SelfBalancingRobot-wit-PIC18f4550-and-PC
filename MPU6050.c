
#define W_DATA         0xD0
#define R_DATA         0xD1
#define PWR_MGMT_1     0x6B
#define PWR_MGMT_2     0x6C
#define SMPRT_DIV      0x19
#define CONFIG_R       0x1A
#define GYRO_CONFIG    0x1B
#define ACCEL_CONFIG   0x1C
#define ACCEL_XOUT_H   0x3B
#define ACCEL_XOUT_L   0x3C
#define ACCEL_YOUT_H   0x3D
#define ACCEL_YOUT_L   0x3E
#define ACCEL_ZOUT_H   0x3F
#define ACCEL_ZOUT_L   0x40
#define TEMP_OUT_H     0x41
#define TEMP_OUT_L     0x42
#define GYRO_XOUT_H    0x43
#define GYRO_XOUT_L    0x44
#define GYRO_YOUT_H    0x45
#define GYRO_YOUT_L    0x46
#define GYRO_ZOUT_H    0x47
#define GYRO_ZOUT_L    0x48

void MPU6050_write(int add, int data)
{
   i2c_start();
   i2c_write(W_DATA);
   i2c_write(add);
   i2c_write(data);
   i2c_stop();
}
      
int16 MPU6050_read(int add)
{
   int retval;
   i2c_start();
   i2c_write(W_DATA);
   i2c_write(add);
   i2c_start();
   i2c_write(R_DATA);
   retval = i2c_read(0);
   i2c_stop();
   return retval;
}

void MPU6050_init()
{
   MPU6050_write(PWR_MGMT_1, 0x80);
   delay_ms(100);
   MPU6050_write(PWR_MGMT_1, 0x00);
   delay_ms(100);
   MPU6050_write(CONFIG_R, 0x01);
   delay_ms(10);
   MPU6050_write(GYRO_CONFIG, 0x00);
}

float MPU6050_get_Ax()
{
   signed int8 A_data_x[2];
   signed int16 accel_value_x;
   A_data_x[0] = MPU6050_read(ACCEL_XOUT_H);
   A_data_x[1] = MPU6050_read(ACCEL_XOUT_L);
   accel_value_x = make16(A_data_x[0], A_data_x[1]);
   float acx = (float)accel_value_x/(float)16384;
   return acx;
}

float MPU6050_get_Ay()
{
   signed int8 A_data_y[2];
   signed int16 accel_value_y;
   A_data_y[0] = MPU6050_read(ACCEL_YOUT_H);
   A_data_y[1] = MPU6050_read(ACCEL_YOUT_L);
   accel_value_y = make16(A_data_y[0], A_data_y[1]);
   float acy = (float)accel_value_y/(float)16384;
   return acy;
}

float MPU6050_get_Az()
{
   signed int8 A_data_z[2];
   signed int16 accel_value_z;
   A_data_z[0] = MPU6050_read(ACCEL_ZOUT_H);
   A_data_z[1] = MPU6050_read(ACCEL_ZOUT_L);
   accel_value_z = make16(A_data_z[0], A_data_z[1]);
   float acz = (float)accel_value_z/(float)16384;
   return acz;
}

float MPU6050_get_Gx()
{
   signed int8 G_data_x[2];
   signed int16 gyro_value_x;
   G_data_x[0] = MPU6050_read(GYRO_XOUT_H);
   G_data_x[1] = MPU6050_read(GYRO_XOUT_L);
   gyro_value_x = make16(G_data_x[0], G_data_x[1]);
   float gcx = (float)gyro_value_x/(float)1031;
   return gcx;
}

float MPU6050_get_Gy()
{
   signed int8 G_data_y[2];
   signed int16 gyro_value_y;
   G_data_y[0] = MPU6050_read(GYRO_YOUT_H);
   G_data_y[1] = MPU6050_read(GYRO_YOUT_L);
   gyro_value_y = make16(G_data_y[0], G_data_y[1]);
   float gcy = (float)gyro_value_y/(float)131;
   return gcy;
}

float MPU6050_get_Gz()
{
   signed int8 G_data_z[2];
   signed int16 gyro_value_z;
   G_data_z[0] = MPU6050_read(GYRO_ZOUT_H);
   G_data_z[1] = MPU6050_read(GYRO_ZOUT_L);
   gyro_value_z = make16(G_data_z[0], G_data_z[1]);
   float gcz = (float)gyro_value_z/(float)131;
   return gcz;
}

float MPU6050_get_Temp()
{
   signed int8 temp_data[2];
   signed int16 temp;
   temp_data[0] = mpu6050_read(TEMP_OUT_H);
   temp_data[1] = mpu6050_read(TEMP_OUT_l);
   temp = make16(temp_data[0],temp_data[1]);
   float temp_mpu = (float)temp/(float)340 + (float)36.53;
   return temp_mpu;
}
