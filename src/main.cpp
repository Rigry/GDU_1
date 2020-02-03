#define STM32F405xx
#define F_OSC   8000000UL
#define F_CPU   48000000UL

#include "init_clock.h"
#include "periph_rcc.h"
#include "periph_flash.h"
#include "pin.h"
#include "literals.h"
#include "generator.h"
#include "communication.h"

using DO  = mcu::PA7;
using TX  = mcu::PA9;
using RX  = mcu::PA10;
using RTS = mcu::PA11;
using PWM_pin = mcu::PA6;
using LED_red = mcu::PB0;
using LED_green = mcu::PC5;
using FACTORY   = mcu::PD2;

extern "C" void init_clock () { init_clock<F_OSC, F_CPU>(); }

int main()
{
   struct Flash_data {
      uint16_t factory_number = 0;
      UART::Settings uart_set = {
         .parity_enable  = false,
         .parity         = USART::Parity::even,
         .data_bits      = USART::DataBits::_8,
         .stop_bits      = USART::StopBits::_1,
         .baudrate       = USART::Baudrate::BR9600,
         .res            = 0
      };
      uint8_t  modbus_address  = 1;
      uint16_t model_number    = 0;
      uint16_t work_frequency  = 18_kHz;
      uint16_t max_current     = 0;
      uint16_t a_current       = 0;
      uint16_t m_current       = 0;
      uint16_t m_resonance     = 18_kHz;
      uint16_t a_resonance     = 18_kHz;
      uint16_t range_deviation = 200;
      uint16_t  time            = 200_ms;
      uint8_t  qty_changes     = 2;
      uint8_t  power           = 100_percent;
      uint8_t  temperatura     = 65;
      uint8_t  recovery        = 45;
      bool     m_search        = false;
      bool     m_control       = false;
      bool     search          = false;
      bool     deviation       = false;
   } flash;
   
   [[maybe_unused]] auto _ = Flash_updater<
        mcu::FLASH::Sector::_9
      , mcu::FLASH::Sector::_8
   >::make (&flash);

   
   decltype(auto) led_green = Pin::make<LED_green, mcu::PinMode::Output>();
   decltype(auto) led_red   = Pin::make<LED_red, mcu::PinMode::Output>();

   volatile decltype(auto) modbus = Modbus_slave<In_regs, Out_regs, coils_qty>
                 ::make<mcu::Periph::USART1, TX, RX, RTS>
                       (flash.modbus_address, flash.uart_set);

   auto& work_flags = modbus.outRegs.flags;

   // управление по модбас
   modbus.force_single_coil_05[0] = [&](bool on) {
      if (on)
         work_flags.on = true;
      if (not on)
         work_flags.on = false;
   };

   modbus.force_single_coil_05[1] = [&](bool on) {
      if (on)
         flash.search = true;
      if (not on)
         flash.search = false;
   };

   #define ADR(reg) GET_ADR(In_regs, reg)
   modbus.outRegs.device_code       = 10;
   modbus.outRegs.factory_number    = flash.factory_number;
   modbus.outRegs.modbus_address    = flash.modbus_address;
   modbus.outRegs.uart_set          = flash.uart_set;    
   modbus.arInRegsMax[ADR(uart_set)]= 0b11111111;
   modbus.inRegsMin.modbus_address  = 1;
   modbus.inRegsMax.modbus_address  = 255;
   
   volatile decltype(auto) pwm = PWM::make<mcu::Periph::TIM3, PWM_pin>(490);

   ADC_ adc;

   using Flash  = decltype(flash);
   using Modbus = Modbus_slave<In_regs, Out_regs, coils_qty>;
   using Generator = Generator<Flash>;

   Generator generator {adc, pwm, led_green, led_red, work_flags, flash};
   Communication<Modbus, Generator> communication{modbus, generator};
   
   while(1){
      generator();
      communication();
      __WFI();
   }
}
