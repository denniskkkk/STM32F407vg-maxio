#define PWMIN_PRIORITY        0x01
// blink all LEDs
// STM32F4 Discovery ( STM32F407VG )



void setup()
{

  SerialUSB.end();
  SerialUSB.begin (115200);
  delay(1000);
  SerialUSB.println ("start");
  delay(1000);
  // toggle port E at 12Mhz
  RCC->AHB1ENR |= 1 << 3;  // AHB1 reg GPIOD_ENS
  RCC->AHB1ENR |= 1 << 4;  // AHB1 reg GPIOE_ENS
  GPIOD->MODER &= 0x00ffffff;  // 0x00 = input , 0x01 = goutput, 0x10 alt, 11 analog
  GPIOD->MODER |= 0x55000000;  // 0x00 = input , 0x01 = goutput, 0x10 alt, 11 analog
  GPIOE->MODER = 0x55555555;  // 0x00 = input , 0x01 = goutput, 0x10 alt, 11 analog
  GPIOD->OTYPER &= 0x00ffffff; // 17-31bit=undefine,  0= pushpull, 1= opendrain
  GPIOE->OTYPER =  0x00000000;  // 17-31bit=undefine,  0= pushpull, 1= opendrain
  GPIOE->OSPEEDR = 0xffffffff;  // 00 = 2Mhz 01=25Mhz 10=50Mhz 11 = 100Mhz
  GPIOE->LCKR = 0x0001ffff;   // lock remained set
  GPIOE->LCKR = 0x0000ffff;   // lockreg bit31-17=rev, bit16=lockall, bit15-0 clock
  GPIOE->LCKR = 0x0001ffff;   // lock remained set
  GPIOD->ODR |=  0x0000f000;   //LED high
  delay (500);
  GPIOD->ODR &= ~0x0000f000;
  delay (500);
  GPIOD->ODR |=  0x0000f000;   //LED high
  while (1) {
    GPIOE->ODR |= 0x0000ffff;      // 0 =low 1=high bit 17-31=reserve
    GPIOE->ODR &= ~0x0000FFFF;
    //uint16_t pe = GPIOE->IDR;        // read port E inputs
    //Serial.println (pe, 16);
    // atomic  reset bit 8 will require setting bit 8+16 =24; ie
    //GPIOE->BSRR = 1<<(8+16);
    // atomic To set bit 8 the code will be
    //GPIOE->BSRR = 1<<8;
  }

}


void loop()
{
  //------------------------//

  GPIOD->ODR |=  0x0000f000;
  GPIOE->ODR |= 0x0000ffff;      // 0 =low 1=high bit 17-31=reserve
  delay (1);
  GPIOD->ODR &= ~0x0000f000;
  GPIOE->ODR &= ~0x0000FFFF;
  delay (1);


}
