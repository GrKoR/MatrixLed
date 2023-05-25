#pragma once

#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>

extern UART_HandleTypeDef huart1;

namespace Serial
{
	HAL_StatusTypeDef _PrintNumber(uint32_t num, uint8_t radix);
	
	
	HAL_StatusTypeDef Print(const char *str)
	{
		HAL_StatusTypeDef result = HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 1000);
		if(result != HAL_OK)
		{
			HAL_UART_AbortTransmit(&huart1);
		}
		
		return result;
	}

	HAL_StatusTypeDef Print(const uint8_t *str, uint16_t length)
	{
		HAL_StatusTypeDef result = HAL_UART_Transmit(&huart1, (uint8_t*)str, length, 1000);
		if(result != HAL_OK)
		{
			HAL_UART_AbortTransmit(&huart1);
		}
		
		return result;
	}
	
	HAL_StatusTypeDef Print(uint32_t num, uint8_t radix = 10)
	{
		return _PrintNumber(num, radix);
	}
	
	HAL_StatusTypeDef Print(int32_t num, uint8_t radix = 10)
	{
		if(radix == 10)
		{
			if(num < 0)
			{
				Print("-");
				
				return _PrintNumber(abs(num), radix);
			}
			
			return _PrintNumber(num, radix);
		}
		else
		{
			return _PrintNumber(num, radix);
		}
	}

	HAL_StatusTypeDef Println()
	{
		return Print("\r\n");
	}
	
	template <uint16_t length = 255> 
	HAL_StatusTypeDef Printf(const char *str, ...)
	{
		char buffer[length];
		
		va_list argptr;
		va_start(argptr, str);
		vsprintf(buffer, str, argptr);
		va_end(argptr);
		
		return Print(buffer);
	}
	
	HAL_StatusTypeDef _PrintNumber(uint32_t num, uint8_t radix)
	{
		if(radix < 2) radix = 10;
		
		char buf[8 * sizeof(long) + 1];
		char *str = &buf[sizeof(buf) - 1];
		
		*str = '\0';
		
		do
		{
			char c = num % radix;
			num /= radix;
			
			*--str = c < 10 ? c + '0' : c + 'A' - 10;
		} while(num);
		
		return Print(str);
	}

}


class DebugSerial
{
	public:
		DebugSerial() = delete;
		
		DebugSerial(UART_HandleTypeDef *huart) : _huart(huart)
		{
			return;
		}

		DebugSerial& SetTopic(const char *topic)
		{
			_topic = topic;
			_topic_print = true;
			
			return *this;
		}
		
		DebugSerial& Print(const char *str)
		{
			if(_topic_print == true)
			{
				_HW_Print((uint8_t*)_topic, strlen(_topic));
				_topic_print = false;
			}
			_HW_Print((uint8_t*)str, strlen(str));
			
			return *this;
		}
		
		DebugSerial& Print(const uint8_t *str, uint16_t length)
		{
			if(_topic_print == true)
			{
				_HW_Print((uint8_t*)_topic, strlen(_topic));
				_topic_print = false;
			}
			_HW_Print((uint8_t*)str, length);
			
			return *this;
		}
		
		template <uint16_t length = 255> 
		DebugSerial& Printf(const char *str, ...)
		{
			char buffer[length];
			
			va_list argptr; va_start(argptr, str); vsprintf(buffer, str, argptr); va_end(argptr);
			
			_HW_Print((uint8_t*)buffer, strlen(buffer));
			
			return *this;
		}
		
		DebugSerial& NewLine()
		{
			_HW_Print((uint8_t*)_new_line_str, strlen(_new_line_str));
			
			return *this;
		}
		
	private:
		HAL_StatusTypeDef _HW_Print(uint8_t *pData, uint16_t Size)
		{
			HAL_StatusTypeDef result = HAL_UART_Transmit(_huart, pData, Size, 64);
			if(result != HAL_OK)
			{
				HAL_UART_AbortTransmit(_huart);
			}

			return result;
		}
		
		UART_HandleTypeDef *_huart;
		
		const char *_topic;
		bool _topic_print;

		const char _new_line_str[3] = "\r\n";
};

DebugSerial serial(&huart1);
