#pragma once

#include "pin.h"
#include "pwm_.h"
#include "adc.h"
#include "flash.h"
#include "timers.h"
#include "modbus_slave.h"
#include "literals.h"
#include "NTC_table.h"

struct Flags {
   bool on            :1; 
   bool search        :1;
   bool manual        :1;
   bool manual_tune   :1;
   bool overheat      :1;
   bool no_load       :1;
   bool overload      :1;
   bool connect       :1;
   bool research      :1;
   bool end_research  :1;
   bool deviation     :1;
   bool boost         :1;
   uint16_t           :4; //Bits 12:15 res: Reserved, must be kept cleared
   bool is_alarm() { return overheat or no_load or overload; }
};

constexpr auto conversion_on_channel {16};
constexpr auto conversion {(3.3 * 1000 * 1.65) / (conversion_on_channel * 4095)}; // 1.65 - R54 - 4,7kOm(0.125 - R54 56kOm) коэффициент для пересчета, 1000 - для передачи в мА
struct ADC_{
   ADC_average& control     = ADC_average::make<mcu::Periph::ADC1>(conversion_on_channel);
   ADC_channel& power       = control.add_channel<mcu::PA0>();
   ADC_channel& current     = control.add_channel<mcu::PA1>();
   ADC_channel& temperatura = control.add_channel<mcu::PA2>();
};

template<class Flash_data>
class Generator
{
   template<class Modbus, class Generator>
   friend class Communication;
   
   enum State_algo {on_off_pause, scan_below, scan_above, resonance_set} algo{State_algo::on_off_pause};
   enum State {wait_, auto_search, manual_search, auto_control, manual_control, set_power, emergency} state{State::wait_}; 
   enum State_scan {wait, pause, scan_down, set_resonance} state_scan{State_scan::wait};
   enum State_deviation {calculation, dev_down, dev_up} state_dev{State_deviation::calculation};
   enum State_booster {start, work_boost, pause_boost, stop} state_boost{State_booster::stop};
   State last_state{State::wait_};

   ADC_& adc;
   PWM& pwm;
   Pin& led_green;
   Pin& led_red;
   Flags& flags;
   Flash_data& flash;

   Timer timer{200_ms};
   Timer blink{500_ms};
   Timer s{1_s};
   Timer set{1_s};
   Timer test{};
   Timer on_power{50_ms};
   Timer delay {};
   Timer dev{flash.time};
   Timer on_off{};
   Timer work_booster{};
   Timer pause_booster{};
   uint16_t current_1s_ago{0};
   uint16_t temperatura{0};
   uint16_t frequency{0};
   uint16_t current{0};
   uint16_t cur{0};
   uint16_t current_mA{0};
   uint16_t current_up{0};
   uint16_t current_down{0};
   uint16_t resonance{0};
   uint16_t resonance_up{0};
   uint16_t resonance_down{0};
   uint16_t resonance_dev{0};
   uint16_t resonance_select{0};
   uint16_t range_frequency{ 1_kHz};
   uint16_t work_frequency {18_kHz};
   uint16_t min_frequency  {19_kHz};
   uint16_t max_frequency  {25_kHz};
   uint16_t duty_cycle {200};
   uint16_t power_ {0};
   int16_t  step {10_Hz};
   int16_t  dev_step{0_Hz};
   bool uz{false};
   bool enable{true};
   bool tested{false};
   bool power();
   bool scanning();
   void select_mode();
   bool scanning_up();
   bool scanning_down ();
   bool is_resonance();
   std::array<uint16_t, 10> arr {0};
   uint8_t index{0};

   void algorithm();
   void deviation();
   void boost();

   uint16_t milliamper(uint16_t adc) { return uz ? (adc * conversion) : 0;} 
   void temp(uint16_t adc) {
      adc = adc / conversion_on_channel;
      auto p = std::lower_bound(
          std::begin(NTC::u2904<33,5100>),
          std::end(NTC::u2904<33,5100>),
          adc,
          std::greater<uint32_t>());
         temperatura = (p - NTC::u2904<33,5100>);
   }

   void is_no_load() {
      if (pwm)
         test.start();
      
      if (test.done() and (current_mA < 10_mA)) {
         test.stop();
         flags.on = false;
         flags.no_load = pwm ? true : flags.no_load;
         state = State::emergency;
         last_state = State::wait_;
      }
      flags.on = flags.no_load ? false : flags.on; // чтобы не тыкали кпонку
   }

   void is_overload(){
      if (pwm and (current_mA > flash.max_current)) {
         flags.on = false;
         flags.overload = true;
         state = State::emergency;
         last_state = State::wait_;
      }
      flags.on = flags.overload ? false : flags.on;
   }

   void switch_state(State s) {
      state = s;
      last_state = state;
   }

   size_t U = 33;
   size_t R = 5100;

public:
   
   Generator(ADC_& adc, PWM& pwm, Pin& led_green, Pin& led_red, Flags& flags
           , Flash_data& flash) 
      : adc {adc}
      , pwm {pwm}
      , led_green {led_green}
      , led_red {led_red}
      , flags {flags}
      , flash {flash}
   {
      adc.control.start();
      test.timeSet = 3_ms;
   }

   void operator()() {

      if (flags.overheat |= temperatura > flash.temperatura) {
            flags.overheat = temperatura > flash.recovery;
            state = State::emergency;
      }

      uz = flags.on and not flags.is_alarm();
      (uz and enable) ? pwm.out_enable() : pwm.out_disable();

      // is_no_load();
      is_overload();

      current_mA = milliamper(adc.current);
      temp(adc.temperatura);

      flags.search = flash.search ? flags.search : false; 
      if (flash.search and pwm)
         led_green ^= blink.event(); 
      else
         led_green = pwm;
      
      work_frequency = flash.work_frequency;
      min_frequency  = work_frequency - range_frequency;
      max_frequency  = work_frequency + range_frequency;
      duty_cycle = flash.power * 4;
      
      switch (state)
      {
         case wait_:
            if (tested) {
               if (flash.search) {
                  flags.search = true;
                  if (flash.m_search) {
                     pwm.duty_cycle = duty_cycle;
                     pwm.frequency  = work_frequency;
                     switch_state(State::manual_search);
                  } else {
                     current = 0;
                     current_up = 0;
                     current_down = 0;
                     pwm.duty_cycle = 100;
                     pwm.frequency = max_frequency;
                     switch_state(State::auto_search);
                  }
               } else if(not flash.search and pwm) {
                  select_mode();
               } else {
                  if (flash.m_search) {
                     pwm.frequency = flash.m_resonance;
                  } else {
                     pwm.frequency = flash.a_resonance;
                  }
               }
            } else {
               pwm.duty_cycle = 100;
               pwm.frequency  = work_frequency;
               if (pwm) {
                  tested = true;
               }
            }

            if (flash.boost) {
               work_booster.stop();
               pause_booster.stop();
               state_boost = State_booster::stop;
            }
            
         break;
         case auto_search:
            if (pwm) {
               if (not scanning()) {
                  flags.search = false;
                  select_mode();
               }
            }
            if (not pwm) state = State::wait_;
         break;
         case manual_search:
            pwm.frequency = frequency;
            if (not flash.search) {
               flags.search = false;
               flash.m_resonance = pwm.frequency;
               flash.m_current = current;
               select_mode();
            }
            if (not flash.m_search) state = State::wait_;
         break;
         case set_power:
            if (power()) 
               switch_state(State::auto_control);
            if (not pwm) {
               switch_state(State::wait_);
            }
         break;
         case auto_control:
            if (flash.deviation)
               deviation();
            else if (flash.boost)
               boost();
            else {
               if (flags.research) {
                  algorithm();
               } else {
                  flags.end_research = false;
               }
            }
            if (not flags.on) {
               switch_state(State::wait_);
               algo = State_algo::on_off_pause;
               state_dev = State_deviation::calculation;
            }
         break;
         case manual_control:
            pwm.frequency = frequency;
            if (not pwm) switch_state(State::wait_);
         break;
         case emergency:
            if (flags.is_alarm()) {
               led_red ^= blink.event(); 
            } else {
               led_red = false;
               state = last_state;
            }
         break;
      } //switch (state)

   } //void operator()()
};

template<class Flash>
void Generator<Flash>::algorithm()
{
   switch(algo){
      case on_off_pause:
         if (not flags.end_research ) {
            on_off.start(3_s);
            enable = false;
            if (on_off.done()) {
               on_off.stop();
               enable = true;
               uz ? pwm.out_enable() : pwm.out_disable();
               algo = State_algo::scan_below;
            }
         }
      break;
      case scan_below:
         if (adc.power > current_down) {
            current_down = adc.power;
            resonance_down = pwm.frequency;
         }
         if (timer.event())
            pwm.frequency += pwm.frequency > (resonance_select - 300) ? -step : 0;

         if (pwm.frequency <= (resonance_select - 300))
            algo = State_algo::scan_above;
      break;
      case scan_above:
         if (adc.power > current_up) {
            current_up = adc.power;
            resonance_up = pwm.frequency;
         }
         if (timer.event())
            pwm.frequency += pwm.frequency < (resonance_select + 300) ? step : 0;
            
         if (pwm.frequency >= resonance_select + 300){
            resonance = current_up > current_down ? resonance_up : resonance_down;
            algo = State_algo::resonance_set;
         }   
      break;
      case resonance_set:
         if (timer.event()) {
            if (pwm.frequency >= resonance)
               pwm.frequency += pwm.frequency > resonance ? -step : 0;
            else
               pwm.frequency += pwm.frequency < resonance ? step : 0;
         }     
         if(pwm.frequency == resonance) {
            flags.end_research = true;

            if (flash.m_search) {
               flash.m_resonance = resonance;
            } else {
               flash.a_resonance = resonance;
            }

            resonance_select = resonance;
            current_down = 0;
            current_up = 0;

            algo = State_algo::on_off_pause;
         }
      break;
   } //switch(algo)
}

template<class Flash>
void Generator<Flash>::deviation()
{
   switch(state_dev){
      case calculation:
         if (flash.m_search) {
            resonance_dev = flash.m_resonance;
         } else {
            resonance_dev = flash.a_resonance;
         }
         dev_step = flash.range_deviation / flash.qty_changes;
         state_dev = State_deviation::dev_up;
      break;
      case dev_up:
         if (dev.event())
            pwm.frequency += pwm.frequency < (resonance_dev + flash.range_deviation / 2) ? dev_step : 0;

         if (pwm.frequency >= (resonance_dev + flash.range_deviation / 2))
            state_dev = State_deviation::dev_down;
      break;
      case dev_down:
         if (dev.event())
            pwm.frequency += pwm.frequency > (resonance_dev - flash.range_deviation / 2) ? -dev_step : 0;
            
         if (pwm.frequency <= resonance_dev - flash.range_deviation / 2){
            state_dev = State_deviation::dev_up;
         }   
      break;
   } //switch(state_dev)
}

template<class Flash>
void Generator<Flash>::boost()
{
   switch (state_boost) {
      case start:
         work_booster.start(flash.work_time);
         state_boost = State_booster::work_boost;
         pwm.duty_cycle = duty_cycle;
      break;
      case work_boost:
         if (work_booster.done()) {
            pwm.duty_cycle = 100;
            work_booster.stop();
            pause_booster.start(flash.pause_time);
            state_boost = State_booster::pause_boost;
         }
      break;
      case pause_boost:
         if (pause_booster.done()) {
            pwm.duty_cycle = duty_cycle;
            pause_booster.stop();
            work_booster.start(flash.work_time);
            state_boost = State_booster::work_boost;
         }
      break;
      case stop:
         work_booster.stop();
         pause_booster.stop();
         state_boost = State_booster::start;
      break;

   }
}

template<class Flash>
void Generator<Flash>::select_mode()
{
   if (flash.m_search) {
         resonance_select = flash.m_resonance;
      } else {
         resonance_select = flash.a_resonance;
   }

   pwm.frequency = resonance_select;
      
   if (flash.m_control) {
      pwm.duty_cycle = duty_cycle;
      switch_state(State::manual_control);
   } else {
      pwm.duty_cycle = 100;
      switch_state(State::set_power);
   }
}

template<class Flash>
bool Generator<Flash>::scanning()
{
   bool tmp{false};
   switch (state_scan){
      
      case wait:
         delay.start(1000_ms);
         state_scan = State_scan::pause;
      break;
      case pause:
         if (delay.done()) {
            delay.stop();
            current = 0;
            current_up = 0;
            current_down = 0;
            resonance = 0;
            pwm.frequency = max_frequency;
            state_scan = State_scan::scan_down;
         }
      break;
      case scan_down:
         if (not scanning_down()) {
            flash.a_resonance = resonance_down;
            state_scan = State_scan::set_resonance;
         }
      break;
      case set_resonance:
         if (is_resonance()) {
            tmp = true;
            state_scan = State_scan::wait;
         }
      break;

   } //switch (state_scan)

   return not tmp;
}

template<class Flash>
bool Generator<Flash>::scanning_down ()
{  
   if (adc.power > current_down) {
      current_down = adc.power;
      resonance_down = pwm.frequency;
   }
   
   if (timer.event())
      pwm.frequency += pwm.frequency > min_frequency ? -step : 0;
   
   return pwm.frequency != min_frequency ? true : false;      
}

template<class Flash>
bool Generator<Flash>::scanning_up ()
{  
   if (adc.power > current_up) {
      current_up = adc.power;
      resonance_up = pwm.frequency;
   }
   
   if (timer.event())
      pwm.frequency += pwm.frequency < max_frequency ? step : 0;
   
   return pwm.frequency != max_frequency ? true : false;      
}

template<class Flash>
bool Generator<Flash>::is_resonance ()
{
   if (timer.event()) {
      if (pwm.frequency >= flash.a_resonance)
         pwm.frequency += pwm.frequency > flash.a_resonance ? -step : 0;
      else
         pwm.frequency += pwm.frequency < flash.a_resonance ? step : 0;
   }     
   return pwm.frequency == flash.a_resonance;
}

template<class Flash>
bool Generator<Flash>::power ()
{
   if (adc.power > current) {
      flash.a_current = current = adc.power;
   }
   if (on_power.event())   
      pwm.duty_cycle += pwm.duty_cycle < duty_cycle ? 1 : -1;

   return pwm.duty_cycle >= duty_cycle;
}