/*
	Библиотека работы с силовыми выходами платы переднего и заднего освещения.

	@Dragon_Knight https://github.com/Dragon-Knight, 2023.
*/

#pragma once

#include <inttypes.h>

extern ADC_HandleTypeDef hadc1;

template <uint8_t _ports_max = 1> 
class PowerOutputs
{
	using event_short_circuit_t = void (*)(uint8_t num, uint16_t current);
	
	typedef struct
	{
		GPIO_TypeDef * port;
		uint16_t digitalpin;
		uint16_t analogpin;
		bool state;
	} ports_t;
	
	public:
		
		PowerOutputs()
		{
			memset(&_ports, 0x00, sizeof(_ports));
			
			HAL_ADCEx_Calibration_Start(&hadc1);
			
			return;
		}
		
		void AddPort(ports_t port)
		{
			if(_ports_idx == _ports_max) return;
			
			_ports[_ports_idx++] = port;
			
			return;
		}
		
		void RegShortCircuitEvent(event_short_circuit_t event)
		{
			_event_short_circuit = event;
			
			return;
		}
		
		bool On(uint8_t out)
		{
			if(out > _ports_max) return false;
			
			ports_t &port_data = _ports[out-1];
			
			HAL_GPIO_WritePin(port_data.port, port_data.digitalpin, GPIO_PIN_SET);
			port_data.state = true;
			
			_delayTick(10000);
			
			uint16_t current = _GetCurrent(port_data.analogpin);
			if( _CheckCurrent(current) == 1 )
			{
				Off(out);
				
				return false;
			}
			
			return true;
		}
		
		void Off(uint8_t out)
		{
			if(out > _ports_max) return;
			
			ports_t &port_data = _ports[out-1];
			
			HAL_GPIO_WritePin(port_data.port, port_data.digitalpin, GPIO_PIN_RESET);
			port_data.state = false;
			
			return;
		}
		
		uint16_t Current(uint8_t out)
		{
			if(out > _ports_max) return 0;

			ports_t &port_data = _ports[out-1];
			
			return _GetCurrent(port_data.analogpin);
		}
		
		void Processing(uint32_t current_time)
		{
			if(current_time - _last_time < 20) return;
			
			uint16_t current;
			for(uint8_t i = 0; i < _ports_max; ++i)
			{
				ports_t &port_data = _ports[i];
				
				if(port_data.port == NULL) continue;
				if(port_data.state == false) continue;
				
				current = _GetCurrent(port_data.analogpin);
				if( _CheckCurrent(current) == 1 )
				{
					Off(i);
					
					if(_event_short_circuit != nullptr)
					{
						_event_short_circuit( (i + 1), current );
					}
				}
			}
			
			_last_time = current_time;
			
			return;
		}
		
	private:
		
		uint16_t _GetCurrent(uint16_t channel)
		{
			_adc_config.Channel = channel;
			
			HAL_ADC_ConfigChannel(&hadc1, &_adc_config);
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, 5);
			uint16_t adc = HAL_ADC_GetValue(&hadc1);
			//HAL_ADC_Stop(&hadc1);
			
			return (((3300000 / 4095) * adc) / 50) / 5;
		}
		
		int8_t _CheckCurrent(uint16_t current)
		{
			if(current < 50) return -1;
			else if(current > 5000) return 1;
			else return 0;
		}
		
		void _delayTick(uint16_t nop_tick)
		{
			while(--nop_tick) { asm("NOP"); }
			
			return;
		}
		
		ADC_ChannelConfTypeDef _adc_config = { ADC_CHANNEL_1, ADC_REGULAR_RANK_1, ADC_SAMPLETIME_1CYCLE_5 };
		
		event_short_circuit_t _event_short_circuit = nullptr;
		uint32_t _last_time = 0;
		
		ports_t _ports[_ports_max];
		uint8_t _ports_idx = 0;
};
