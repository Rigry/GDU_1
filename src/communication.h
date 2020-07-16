#pragma once

#include "flash.h"
#include "modbus_slave.h"

struct In_regs {
   
   UART::Settings uart_set;         // 0
   uint16_t modbus_address;         // 1
   uint16_t password;               // 2
   uint16_t factory_number;         // 3
   uint16_t frequency;              // 4
   uint16_t work_frequency;         // 5
   uint16_t power;                  // 6
   uint16_t max_current;            // 7
   uint16_t max_temp;               // 8
   uint16_t recovery_temp;          // 9
   Flags flags;                     // 10

}__attribute__((packed));

struct Out_regs {

   uint16_t device_code;            // 0
   uint16_t factory_number;         // 1
   UART::Settings uart_set;         // 2
   uint16_t modbus_address;         // 3
   uint16_t power;                  // 4
   uint16_t duty_cycle;             // 5
   uint16_t work_frequency;         // 6
   uint16_t frequency;              // 7
   uint16_t m_resonance;            // 8
   uint16_t a_resonance;            // 9
   uint16_t current;                // 10
   uint16_t max_current;            // 11
   uint16_t a_current;              // 12
   uint16_t m_current;              // 13
   uint16_t temperatura;            // 14
   uint16_t max_temp;               // 15
   uint16_t recovery_temp;          // 16
   Flags flags;                     // 17

};//__attribute__((packed));

#define ADR(reg) GET_ADR(In_regs, reg)

// колбеки для коилов далее
constexpr auto coils_qty {2};


template<class Modbus, class Generator>
class Communication
{
   Modbus& modbus;
   Generator& generator;

public:
   
   Communication(Modbus& modbus, Generator& generator)
      : modbus   {modbus}
      , generator{generator}
      
   {}

   void operator() (){

      modbus.outRegs.work_frequency    = generator.flash.work_frequency;
      modbus.outRegs.frequency         = generator.pwm.frequency;
      modbus.outRegs.a_resonance       = generator.flash.a_resonance;
      modbus.outRegs.m_resonance       = generator.flash.m_resonance;
      modbus.outRegs.max_current       = generator.flash.max_current;
      modbus.outRegs.current           = generator.current_mA;
      modbus.outRegs.temperatura       = generator.temperatura;
      modbus.outRegs.max_temp          = generator.flash.temperatura;
      modbus.outRegs.recovery_temp     = generator.flash.recovery;
      modbus.outRegs.a_current         = generator.milliamper(generator.flash.a_current);
      modbus.outRegs.m_current         = generator.milliamper(generator.flash.m_current);
      modbus.outRegs.power             = generator.flash.power;
      modbus.outRegs.duty_cycle        = generator.pwm.duty_cycle / 5;
     
      // modbus.outRegs.flags.search      = generator.flash.search;
      modbus.outRegs.flags.manual       = generator.flash.m_control;
      modbus.outRegs.flags.manual_tune  = generator.flash.m_search;
      modbus.outRegs.flags.connect      = true;
      modbus.outRegs.flags.overheat     = generator.flags.overheat;
      modbus.outRegs.flags.no_load      = generator.flags.no_load;
      modbus.outRegs.flags.overload     = generator.flags.overload;
      // modbus.outRegs.flags.research     = generator.flags.research;
      modbus.outRegs.flags.end_research = generator.flags.end_research;


      modbus([&](uint16_t registrAddress) {
            static bool unblock = false;
         switch (registrAddress) {
            case ADR(uart_set):
               generator.flash.uart_set
                  = modbus.outRegs.uart_set
                  = modbus.inRegs.uart_set;
            break;
            case ADR(modbus_address):
               generator.flash.modbus_address 
                  = modbus.outRegs.modbus_address
                  = modbus.inRegs.modbus_address;
            break;
            case ADR(password):
               unblock = modbus.inRegs.password == 1207;
            break;
            case ADR(factory_number):
               if (unblock) {
                  unblock = false;
                  generator.flash.factory_number 
                     = modbus.outRegs.factory_number
                     = modbus.inRegs.factory_number;
               }
               unblock = true;
            break;
            case ADR(frequency):
               generator.frequency = modbus.inRegs.frequency;
            break;
            case ADR(work_frequency):
               generator.flash.work_frequency = modbus.inRegs.work_frequency;
            break;
            case ADR(power):
               generator.flash.power = modbus.inRegs.power;
            break;
            case ADR(max_current):
               generator.flash.max_current = modbus.inRegs.max_current;
            break;
            case ADR(max_temp):
               generator.flash.temperatura = modbus.inRegs.max_temp;
            break;
            case ADR(recovery_temp):
               generator.flash.recovery = modbus.inRegs.recovery_temp;
            break;
            case ADR(flags):
               // generator.mode.on          = modbus.inRegs.flags.on;
               // generator.flash.search     = modbus.inRegs.flags.search;
               generator.flash.m_control  = modbus.inRegs.flags.manual;
               generator.flash.m_search   = modbus.inRegs.flags.manual_tune;
               generator.flags.research   = modbus.inRegs.flags.research;
            break;
         } // switch
      }, [&](auto registr){}
      );
   } //void operator() ()
};