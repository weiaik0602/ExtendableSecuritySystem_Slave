#include "unity.h"
#include "SlaveFunc.h"
#include "mock_mockFunc.h"

void setUp(void){}

void tearDown(void){}

void test_SPI_Receive_Buffer_spi_receive_position_reset(void)
{
  spi_receive_position = BUFFER_SIZE-1;
  spi_receive[0] = MODULE_Self;
  spi_receive[1] = 1;
  spi_receive[2] = ACTION_Open;

  SPI_Receive_Buffer();
  TEST_ASSERT_EQUAL(spi_receive_position,1);
  TEST_ASSERT_EQUAL(spi_receive_buffer[0].module,MODULE_Self);
  TEST_ASSERT_EQUAL(spi_receive_buffer[0].size,1);
  TEST_ASSERT_EQUAL(spi_receive_buffer[0].data,ACTION_Open);
}

void test_SPI_Receive_Buffer_spi_receive_position_increment(void)
{
  spi_receive_position = 0;
  spi_receive[0] = MODULE_Led;
  spi_receive[1] = 1;
  spi_receive[2] = ACTION_Close;

  SPI_Receive_Buffer();
  TEST_ASSERT_EQUAL(spi_receive_position,1);
  TEST_ASSERT_EQUAL(spi_receive_buffer[0].module,MODULE_Led);
  TEST_ASSERT_EQUAL(spi_receive_buffer[0].size,1);
  TEST_ASSERT_EQUAL(spi_receive_buffer[0].data,ACTION_Close);
}

void test_SPI_Receive_Buffer_spi_receive_position_increment_agn(void)
{
  spi_receive[0] = MODULE_Buzzer;
  spi_receive[1] = 1;
  spi_receive[2] = ACTION_Close;

  SPI_Receive_Buffer();
  TEST_ASSERT_EQUAL(spi_receive_position,2);
  TEST_ASSERT_EQUAL(spi_receive_buffer[1].module,MODULE_Buzzer);
  TEST_ASSERT_EQUAL(spi_receive_buffer[1].size,1);
  TEST_ASSERT_EQUAL(spi_receive_buffer[1].data,ACTION_Close);
}

void test_IDLE_Func_Module_Self_Here(void){
  spi_use_position = 0;
  spi_receive_position = 1;
  spi_receive_buffer[0].module = MODULE_Self;
  spi_receive_buffer[0].data = ACTION_Open;
  SPI_Reply_Expect(MODULE_Self,ACTION_Open);
  IDLE_Func();
  TEST_ASSERT_EQUAL(spi_use_position, 1);
}
void test_IDLE_Func_Module_Self_NA(void){
  spi_use_position = 0;
  spi_receive_position = 1;
  spi_receive_buffer[0].module = MODULE_Self;
  spi_receive_buffer[0].data = ACTION_Close;
  SPI_Reply_Expect(MODULE_Self,REPLY_NA);
  IDLE_Func();
  TEST_ASSERT_EQUAL(spi_use_position, 1);
}

void test_IDLE_Func_Module_Buzzer_Open(void){
  spi_use_position = 0;
  spi_receive_position = 1;
  spi_receive_buffer[0].module = MODULE_Buzzer;
  spi_receive_buffer[0].data = ACTION_Open;
  Open_Buzzer_Expect();
  //SPI_Reply_Expect(MODULE_Buzzer,ACTION_Open);
  IDLE_Func();
  TEST_ASSERT_EQUAL(spi_use_position, 1);
}

void test_IDLE_Func_Module_Buzzer_Close(void){
  spi_use_position = 0;
  spi_receive_position = 1;
  spi_receive_buffer[0].module = MODULE_Buzzer;
  spi_receive_buffer[0].data = ACTION_Close;
  Close_Buzzer_Expect();
  //SPI_Reply_Expect(MODULE_Buzzer,ACTION_Open);
  IDLE_Func();
  TEST_ASSERT_EQUAL(spi_use_position, 1);
}

void test_IDLE_Func_MODULE_Led_Open(void){
  spi_use_position = 0;
  spi_receive_position = 1;
  spi_receive_buffer[0].module = MODULE_Led;
  spi_receive_buffer[0].data = ACTION_Open;
  Open_LED_Expect();
  //SPI_Reply_Expect(MODULE_Buzzer,ACTION_Open);
  IDLE_Func();
  TEST_ASSERT_EQUAL(spi_use_position, 1);
}

void test_IDLE_Func_MODULE_Led_Close(void){
  spi_use_position = 0;
  spi_receive_position = 1;
  spi_receive_buffer[0].module = MODULE_Led;
  spi_receive_buffer[0].data = ACTION_Close;
  Close_LED_Expect();
  //SPI_Reply_Expect(MODULE_Buzzer,ACTION_Open);
  IDLE_Func();
  TEST_ASSERT_EQUAL(spi_use_position, 1);
}

void test_IDLE_Func_MODULE_Lock_OpenThenClose(void){
  spi_use_position = 0;
  spi_receive_position = 1;
  spi_receive_buffer[0].module = MODULE_Lock;
  spi_receive_buffer[0].data = ACTION_Open;
  OpenThenClose_Lock_Expect();
  //SPI_Reply_Expect(MODULE_Buzzer,ACTION_Open);
  IDLE_Func();
  TEST_ASSERT_EQUAL(spi_use_position, 1);
}
