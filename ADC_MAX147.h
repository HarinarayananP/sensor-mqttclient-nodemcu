/*
    SPI Master Demo Sketch
    Connect the SPI Master device to the following pins on the esp8266:
    GPIO    NodeMCU   Name  |   Uno
   ===================================
     15       D8       SS   |   D10
     13       D7      MOSI  |   D11
     12       D6      MISO  |   D12
     14       D5      SCK   |   D13
    Note: If the ESP is booting at a moment when the SPI Master has the Select line HIGH (deselected)
    the ESP8266 WILL FAIL to boot!
    See SPISlave_SafeMaster example for possible workaround
*/
#include <SPI.h>

class ADCmax147 {
  private:
    uint8_t _ss_pin;

  public:
    ADCmax147(uint8_t pin): _ss_pin(pin) {}
    void begin() {
      pinMode(_ss_pin, OUTPUT);
      digitalWrite(_ss_pin, HIGH);
    }

     unsigned int ADC_readch(uint8_t ch) {
      unsigned int adc_res=0;
      uint8_t data[2];
      uint8_t startbyte = 0x8F ;  // 0b 1 A2 A1 A0  1   1   1  1
                                  //    S  ADC CH  Bin Sin Ext.Clk
      startbyte = startbyte | ((ch&0x07)<<4);                            
      digitalWrite(_ss_pin, LOW);
      SPI.transfer(startbyte);
      //SPI.transfer(0x00);
      for (uint8_t i = 0; i < 2; i++) {
        data[i] = SPI.transfer(0);
      }
      digitalWrite(_ss_pin, HIGH);
      adc_res = data[0]&0x7F;
      adc_res = (adc_res<<8)| (data[1]);
      adc_res = adc_res >>3;
      return adc_res; 
    }

    uint32_t readStatus() {
      digitalWrite(_ss_pin, LOW);
      SPI.transfer(0x04);
      uint32_t status = (SPI.transfer(0) | ((uint32_t)(SPI.transfer(0)) << 8) | ((uint32_t)(SPI.transfer(0)) << 16) | ((uint32_t)(SPI.transfer(0)) << 24));
      digitalWrite(_ss_pin, HIGH);
      return status;
    }

    void writeStatus(uint32_t status) {
      digitalWrite(_ss_pin, LOW);
      SPI.transfer(0x01);
      SPI.transfer(status & 0xFF);
      SPI.transfer((status >> 8) & 0xFF);
      SPI.transfer((status >> 16) & 0xFF);
      SPI.transfer((status >> 24) & 0xFF);
      digitalWrite(_ss_pin, HIGH);
    }

    void readData(uint8_t * data) {
      digitalWrite(_ss_pin, LOW);
      SPI.transfer(0x03);
      SPI.transfer(0x00);
      for (uint8_t i = 0; i < 32; i++) {
        data[i] = SPI.transfer(0);
      }
      digitalWrite(_ss_pin, HIGH);
    }

    void writeData(uint8_t * data, size_t len) {
      uint8_t i = 0;
      digitalWrite(_ss_pin, LOW);
      SPI.transfer(0x02);
      SPI.transfer(0x00);
      while (len-- && i < 32) {
        SPI.transfer(data[i++]);
      }
      while (i++ < 32) {
        SPI.transfer(0);
      }
      digitalWrite(_ss_pin, HIGH);
    }

    String readData() {
      char data[33];
      data[32] = 0;
      readData((uint8_t *)data);
      return String(data);
    }

    void writeData(const char * data) {
      writeData((uint8_t *)data, strlen(data));
    }
};
