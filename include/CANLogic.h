#pragma once

#include <CANLibrary.h>

void HAL_CAN_Send(can_object_id_t id, uint8_t *data, uint8_t length);

extern CAN_HandleTypeDef hcan;

namespace CANLib
{
	//*********************************************************************
	// CAN Library settings
	//*********************************************************************

	/// @brief Number of CANObjects in CANManager
	static constexpr uint8_t CFG_CANObjectsCount = 12;

	/// @brief The size of CANManager's internal CAN frame buffer
	static constexpr uint8_t CFG_CANFrameBufferSize = 16;


	static constexpr uint16_t CFG_TurnTimeOn = 550;
	static constexpr uint16_t CFG_TurnTimeOf = 400;


	//*********************************************************************
	// CAN Manager & CAN Object configuration
	//*********************************************************************

	// structure for all data fields of CANObject
	// rear_light_can_data_t light_ecu_can_data;

	CANManager<CFG_CANObjectsCount, CFG_CANFrameBufferSize> can_manager(&HAL_CAN_Send);

	// ******************** common blocks ********************
	// 0x00E0	BlockInfo
	// request | timer:15000
	// byte	1 + 7	{ type[0] data[1..7] }
	// Основная информация о блоке. См. "Системные параметры".
	CANObject<uint8_t, 7> obj_block_info(0x00E0);

	// 0x00E1	BlockHealth
	// request | event
	// byte	1 + 7	{ type[0] data[1..7] }
	// Информация о здоровье блока. См. "Системные параметры".
	CANObject<uint8_t, 7> obj_block_health(0x00E1);

	// 0x00E2	BlockCfg
	// request
	// byte	1 + 1 + X	{ type[0] param[1] data[2..7] }
	// Чтение и запись настроек блока. См. "Системные параметры".
	CANObject<uint8_t, 7> obj_block_features(0x00E2);


	// 0x00E3	BlockError
	// request | event
	// byte	1 + X	{ type[0] data[1..7] }
	// Ошибки блока. См. "Системные параметры".
	CANObject<uint8_t, 7> obj_block_error(0x00E3);

	// ******************** specific blocks ********************

	// 0x00E4	SideBeam
	// set | request | event
	// uint8_t	0..255	1 + 1	{ type[0] val[1] }
	// Управление габаритами.
	CANObject<uint8_t, 1> obj_side_beam(0x00E4, CAN_TIMER_DISABLED, 300);


	// 0x00E5	BrakeLight
	// set | request | event
	// uint8_t	0..255	1 + 1	{ type[0] val[1] }
	// Управление стоп-сигналами.
	CANObject<uint8_t, 1> obj_brake_light(0x00E5, CAN_TIMER_DISABLED, 300);


	// 0x00E6	ReverseLight
	// set | request | event
	// uint8_t	0..255	1 + 1	{ type[0] val[1] }
	// Управление задним ходом.
	CANObject<uint8_t, 1> obj_reverse_light(0x00E6, CAN_TIMER_DISABLED, 300);


	// 0x00E7	LeftIndicator
	// set | request | event
	// uint8_t	0..255	1 + 1	{ type[0] val[1] }
	// Управление левым поворотником.
	CANObject<uint8_t, 1> obj_left_indicator(0x00E7, CAN_TIMER_DISABLED, 300);


	// 0x00E8	RightIndicator
	// set | request | event
	// uint8_t	0..255	1 + 1	{ type[0] val[1] }
	// Управление правым поворотником.
	CANObject<uint8_t, 1> obj_right_indicator(0x00E8, CAN_TIMER_DISABLED, 300);


	// 0x00E9	HazardBeam
	// set | request | event
	// uint8_t	0..255	1 + 1	{ type[0] val[1] }
	// Управление аварийным сигналом.
	CANObject<uint8_t, 1> obj_hazard_beam(0x00E9, CAN_TIMER_DISABLED, 300);


	// 0x00EA	CustomBeam
	// set | request | event
	// uint8_t	0..255	1 + 1	{ type[0] val[1] }
	// Управление пользовательским светом.
	CANObject<uint8_t, 1> obj_custom_beam(0x00EA, CAN_TIMER_DISABLED, 300);


	// 0x00EB	CustomImage
	// set | request | event
	// uint8_t	0..255	1 + 1	{ type[0] val[1] }
	// Управление пользовательскими изображениями ( для WS2812b ).
	CANObject<uint8_t, 1> obj_custom_image(0x00EB, CAN_TIMER_DISABLED, 300);

	inline uint8_t on_off_validator(uint8_t value)
	{
		return (value > 0) ? 0xFF : 0;
	}

	// вызывается, если по CAN пришла команда включения/выключения габаритов
	can_result_t side_beam_set_handler(can_frame_t &can_frame, can_error_t &error)
	{
		if (can_frame.data[0] == 0)
		{
			Matrix::matrixObj.HideLayer(2);
			Outputs::outObj.SetOff(2);
		}
		else
		{
			Matrix::matrixObj.ShowLayer(2);
			Outputs::outObj.SetOn(2);
		}
		obj_side_beam.SetValue(0, on_off_validator(can_frame.data[0]), CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);

		return CAN_RESULT_IGNORE;
	}

	// вызывается, если по CAN пришла команда включения/выключения стоп-сигналов
	can_result_t brake_light_set_handler(can_frame_t &can_frame, can_error_t &error)
	{
		if (can_frame.data[0] == 0)
		{
			Matrix::matrixObj.HideLayer(4);
			Outputs::outObj.SetOff(4);
		}
		else
		{
			Matrix::matrixObj.ShowLayer(4);
			Outputs::outObj.SetOn(4);
		}
		obj_brake_light.SetValue(0, on_off_validator(can_frame.data[0]), CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);

		return CAN_RESULT_IGNORE;
	}

	// вызывается, если по CAN пришла команда включения/выключения заднего хода
	can_result_t reverse_light_set_handler(can_frame_t &can_frame, can_error_t &error)
	{
		if (can_frame.data[0] == 0)
		{
			Matrix::matrixObj.HideLayer(3);
			Outputs::outObj.SetOff(3);
		}
		else
		{
			Matrix::matrixObj.ShowLayer(3);
			Outputs::outObj.SetOn(3);
		}
		obj_reverse_light.SetValue(0, on_off_validator(can_frame.data[0]), CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);

		return CAN_RESULT_IGNORE;
	}

	// вызывается, если по CAN пришла команда включения/выключения левого поворотника
	can_result_t turn_left_set_handler(can_frame_t &can_frame, can_error_t &error)
	{
		if (can_frame.data[0] == 0)
		{
			Matrix::matrixObj.HideLayer(5);
			Outputs::outObj.SetOff(5);
		}
		else
		{
			Matrix::matrixObj.ShowLayer(5);
			Outputs::outObj.SetOn(5, CFG_TurnTimeOn, CFG_TurnTimeOf);
		}
		obj_left_indicator.SetValue(0, on_off_validator(can_frame.data[0]), CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);

		return CAN_RESULT_IGNORE;
	}

	// вызывается, если по CAN пришла команда включения/выключения правого поворотника
	can_result_t turn_right_set_handler(can_frame_t &can_frame, can_error_t &error)
	{
		if (can_frame.data[0] == 0)
		{
			Matrix::matrixObj.HideLayer(6);
			Outputs::outObj.SetOff(6);
		}
		else
		{
			Matrix::matrixObj.ShowLayer(6);
			Outputs::outObj.SetOn(6, CFG_TurnTimeOn, CFG_TurnTimeOf);
		}
		obj_right_indicator.SetValue(0, on_off_validator(can_frame.data[0]), CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);

		return CAN_RESULT_IGNORE;
	}

	// вызывается, если по CAN пришла команда включения/выключения аварийного сигнала
	can_result_t hazard_beam_set_handler(can_frame_t &can_frame, can_error_t &error)
	{
		if (can_frame.data[0] == 0)
		{
			Matrix::matrixObj.HideLayer(7);
			Outputs::outObj.SetOff(5);
			Outputs::outObj.SetOff(6);
		}
		else
		{
			Matrix::matrixObj.ShowLayer(7);
			Outputs::outObj.SetOn(5, CFG_TurnTimeOn, CFG_TurnTimeOf);
			Outputs::outObj.SetOn(6, CFG_TurnTimeOn, CFG_TurnTimeOf);
		}
		obj_hazard_beam.SetValue(0, on_off_validator(can_frame.data[0]), CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);

		return CAN_RESULT_IGNORE;
	}

	// вызывается, если по CAN пришла команда включения/выключения пользовательского света
	can_result_t custom_beam_set_handler(can_frame_t &can_frame, can_error_t &error)
	{
		if (can_frame.data[0] == 0)
		{
			Outputs::outObj.SetOff(1);
		}
		else
		{
			Outputs::outObj.SetOn(1);
		}
		obj_custom_beam.SetValue(0, on_off_validator(can_frame.data[0]), CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);

		return CAN_RESULT_IGNORE;
	}

	// вызывается, если по CAN пришла команда включения/выключения пользовательского изображения на панели
	can_result_t custom_image_set_handler(can_frame_t &can_frame, can_error_t &error)
	{
		if (can_frame.data[0] == 0)
		{
			//Matrix::matrixObj.HideLayer(1);
			Matrix::matrixObj.RegLayer("layer1.pxl", 1);
			Matrix::matrixObj.ShowLayer(1);
		}
		else
		{
			char filename[13];
			sprintf(filename, "user%03d.pxl", can_frame.data[0]);
			Matrix::matrixObj.RegLayer(filename, 1);
			Matrix::matrixObj.ShowLayer(1);
		}
		obj_custom_image.SetValue(0, on_off_validator(can_frame.data[0]), CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);

		return CAN_RESULT_IGNORE;
	}

	inline void Setup()
	{
		obj_side_beam.RegisterFunctionSet(&side_beam_set_handler);
		obj_brake_light.RegisterFunctionSet(&brake_light_set_handler);
		obj_reverse_light.RegisterFunctionSet(&reverse_light_set_handler);
		obj_left_indicator.RegisterFunctionSet(&turn_left_set_handler);
		obj_right_indicator.RegisterFunctionSet(&turn_right_set_handler);
		obj_hazard_beam.RegisterFunctionSet(&hazard_beam_set_handler);
		obj_custom_beam.RegisterFunctionSet(&custom_beam_set_handler);
		obj_custom_image.RegisterFunctionSet(&custom_image_set_handler);

		// system blocks
		set_block_info_params(obj_block_info);
		set_block_health_params(obj_block_health);
		set_block_features_params(obj_block_features);
		set_block_error_params(obj_block_error);

		// common blocks
		can_manager.RegisterObject(obj_block_info);
		can_manager.RegisterObject(obj_block_health);
		can_manager.RegisterObject(obj_block_features);
		can_manager.RegisterObject(obj_block_error);

		// specific blocks
		can_manager.RegisterObject(obj_side_beam);
		can_manager.RegisterObject(obj_brake_light);
		can_manager.RegisterObject(obj_reverse_light);
		can_manager.RegisterObject(obj_left_indicator);
		can_manager.RegisterObject(obj_right_indicator);
		can_manager.RegisterObject(obj_hazard_beam);
		can_manager.RegisterObject(obj_custom_beam);
		can_manager.RegisterObject(obj_custom_image);

		// Set versions data to block_info.
		obj_block_info.SetValue(0, (About::board_type << 3 | About::board_ver), CAN_TIMER_TYPE_NORMAL);
		obj_block_info.SetValue(1, (About::soft_ver << 2 | About::can_ver), CAN_TIMER_TYPE_NORMAL);
		
		return;
	}

	inline void Loop(uint32_t &current_time)
	{
		can_manager.Process(current_time);
		
		// Set uptime to block_info.
		static uint32_t iter = 0;
		if(current_time - iter > 1000)
		{
			iter = current_time;

			uint8_t *data = (uint8_t *)&current_time;
			obj_block_info.SetValue(2, data[0], CAN_TIMER_TYPE_NORMAL);
			obj_block_info.SetValue(3, data[1], CAN_TIMER_TYPE_NORMAL);
			obj_block_info.SetValue(4, data[2], CAN_TIMER_TYPE_NORMAL);
			obj_block_info.SetValue(5, data[3], CAN_TIMER_TYPE_NORMAL);
		}
		
		current_time = HAL_GetTick();

		return;
	}
}
