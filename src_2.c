#include "radio.h"
#include "main.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "time.h"
#include "uart.h"
#include "i2c.h"
#include "frag_data_func.h"
#include <math.h>


uint8_t* dev_id   = (uint8_t*)ID_BASE_ADDRESS;

char MODEM_ANSWER[250] 			= {0}; //буфер для ответа от модема LoRa в чистом виде
char *MODEM_SEND[130]           = {0}; //буфер для посылки команд в модем LoRa
uint8_t lora_rec_buf[100]   	= {0}; //буфер для команды от модема LoRa, "переведвенный" в hex формат
uint8_t coded_fragment[60]      = {0}; //буфер для закодированного фрагмента в сессии фрагментированной передачи данных

uint8_t lora_confirm_cmd[130]   = {'A', 'T', '+', 'C', 'M', 'S', 'G', 'H', 'E', 'X', '=', '"'}; //команда на отправку данных с подтверждением от сервера
uint8_t lora_unconfirm_cmd[130] = {'A', 'T', '+', 'M', 'S', 'G', 'H', 'E', 'X', '=', '"'};//команда на отправку данных без подтверждения
uint8_t mcast_lw_cmd[120]       = {'A', 'T', '+', 'L', 'W', '=', 'M', 'C', ',', 'O', 'N', ',', '"'}; //команда для включения мультикаст вещания
uint8_t dev_ui[100] 			= {'A', 'T', '+', 'I', 'D', '=', 'D', 'e', 'v', 'E', 'u', 'i', '"'};


uint8_t frag_session_msg_queue[FRAG_SESSION_BUF_COUNT][FRAG_SESSION_BUF_SIZE]   = {0};
uint8_t resend_msg_queue[RESEND_QUEUE_BUF_COUNT][RESEND_QUEUE_BUF_SIZE] 		= {0};
uint8_t mcast_msg_queue[MCAST_QUEUE_BUF_COUNT][MCAST_QUEUE_BUF_SIZE]			= {0};


/* Основные сообщение протокола информационного обмена
 * Описание смотреть в документе
 * Все поля структур и названия соотвествуют тому, что расписано в документе
 */
CoordinateAndRTCStruct  coordinate_and_rtc;
GPSSetReqAns            gps_set_req_ans;
LumMaxCurrentReqAns     lum_max_current_req_ans;
DevStatusIntReqAns      dev_status_int_req_ans;
SunRSSetReqAns          sun_rs_set_req_ans;
LumAutoDimSetReqAns     lum_auto_dim_set_req_ans;
McastAnsIntReqAns       mcast_ans_int_req_ans;
TimeRiseSetStruct       time_rise_set_struct;
TimeScheduleStruct      time_schedule_struct;
LumDimStatusRep         lum_dim_status_rep;
DevStatusReqAns         dev_status_req_ans;
ModeSetReq              mode_set_req;
TimeZoneSetReqAns       time_zone_set_req_ans;
LumDimSetReq 			lum_dim_set_req_ans;
OperTimeRep				oper_time_rep;
NewMcDevAddrSetReq      new_mc_dev_addr_set_req;
ScenarioDimReq          scenario_dim_req;
MotionOptSetReq         motion_opt_set_req;
MotSenSetReq            mot_sen_set_req_ans;
ProfileOptSetReq        profile_opt_set_req;
PackageVsersionAns      package_version_ans;
FragSessionSetupAns     frag_session_setup_ans;
FragSessionSetupReq     frag_session_setup_req;
FragSessionStatusAns    frag_session_status_ans;
FragSessionStatusReq    frag_session_status_req;
ShedSetReqAns           shed_set_req_ans;
ProfileSetAns           profile_set_ans;
FragSessionDeleteAns    frag_session_delete_ans;
ShedSetReq              shed_set_req;
ProfileSetReq           profile_set_req;
DeviceParamsAns         device_params_ans;
StatusLighterReqAns     status_lighter_req_ans;
FragSessionDeleteReq    frag_session_delete_req;
DevStatusSettingsReq    dev_status_settings_req_ans;


/*Отправка сообщения на модем LoRa
 * p_buf - указатель на полезные данные
 * buf_size - количество байт полезной информации
 * port_num - порт, через который нужно отправить данные
 * confirm_flag - посылать сообщение с подтверждением от сервера или без подтверждения
 */
void SendMsgLora(uint8_t* p_buf, uint32_t buf_size, uint8_t port_num, uint8_t confirm_flag) {

	uint32_t data_iter = 0;
	ack_flag           = 0;

	if (port_num == 2)          *MODEM_SEND = "AT+PORT=2\r\n";

	else if (port_num == 200)	*MODEM_SEND = "AT+PORT=200\r\n";

	else if (port_num == 201)   *MODEM_SEND = "AT+PORT=201\r\n";

	else if (port_num == 202)   *MODEM_SEND = "AT+PORT=202\r\n";

	//Посылаем данные через порты 200 и 2, если не установлена сессия фрагментированной передачи данных,
	//или если сессия установлена, то есть данные идут через порты 201 и 202
	if ((!flags_and_states.frag_session_is_setup_flag) || (port_num == 201) || (port_num == 202)) {

		HAL_UART_Transmit(&LORA_UART, (uint8_t*)*MODEM_SEND, strlen(*MODEM_SEND), 0x200);
		HAL_Delay(100);

		//Если нужно отправить сообщение с подтверждением от сервера
		if (confirm_flag) {

			memset(&lora_confirm_cmd[12], 0 , (sizeof(lora_confirm_cmd)-12));

			//Заполняем команду полезными данными из буфера
			for (data_iter = 12; data_iter<buf_size*2+12; data_iter += 2) {

				sprintf(&lora_confirm_cmd[data_iter], "%02X", *p_buf++);
			}

			//Добавялем в конец необходимые служебеные символы, чтобы команда была завершенной
			lora_confirm_cmd[buf_size*2+12] = '\"';
			lora_confirm_cmd[buf_size*2+13] = '\r';
			lora_confirm_cmd[buf_size*2+14] = '\n';

			bytes_need_to_send = buf_size*2 + 15;

			//Отправляем команду в модем LoRa
			HAL_UART_Transmit(&LORA_UART, lora_confirm_cmd, bytes_need_to_send, 0x200);
			HAL_Delay(300);

			receive_msg_delay = 4;

			//Ждем 4 секунды подтверждение от сервера
			while(receive_msg_delay) {

				//Если получено подтверждение, то поднимаем флаг, и обновляем время проверки соединения между светильником и сервером
				if (strstr(MODEM_ANSWER, "ACK Received")) {
					ack_flag = 1;
					power_error_ack_flag = 1;
					receive_msg_delay = 0;
					link_check_time = 3600;
					memset(MODEM_ANSWER,0,sizeof(MODEM_ANSWER));
					HAL_Delay(500);
					break;
				}

			}

			//Если же подтверждение не было получено от сервера, то помещаем сообщение в очередь на переотправку
			if (!ack_flag) {

				if (need_to_resend_count == RESEND_QUEUE_BUF_COUNT)
					need_to_resend_count = 0;

				if (general_resend_count == RESEND_QUEUE_BUF_COUNT)
					general_resend_count = RESEND_QUEUE_BUF_COUNT;
				else
					general_resend_count += 1;

				need_to_resend_count += 1;

				memcpy(&resend_msg_queue[need_to_resend_count-1][0], p_buf-buf_size, buf_size);
				resend_msg_queue[need_to_resend_count-1][RESEND_QUEUE_BUF_SIZE-2] = port_num;
				resend_msg_queue[need_to_resend_count-1][RESEND_QUEUE_BUF_SIZE-1] = buf_size;

			}

		//Если не нуждаемся в подтверждении от сервера, то просто отправляем команду
		} else if (confirm_flag == 0) {

			memset(&lora_unconfirm_cmd[11], 0 , (sizeof(lora_unconfirm_cmd)-11));

			for (data_iter = 11; data_iter<buf_size*2+11; data_iter += 2) {

				sprintf(&lora_unconfirm_cmd[data_iter], "%02X", *p_buf++);
			}

			lora_unconfirm_cmd[buf_size*2+11] = '\"';
			lora_unconfirm_cmd[buf_size*2+12] = '\r';
			lora_unconfirm_cmd[buf_size*2+13] = '\n';

			bytes_need_to_send = buf_size*2 + 14;

			HAL_UART_Transmit(&LORA_UART, lora_unconfirm_cmd, bytes_need_to_send, 0x300);
		}

	//Если же активен режим сесси фрагментированной передачи данных,  то отправляем сообщения в очередь на переотправку, чтобы
	//после окончания сессии их отправить
	} else {

		if (need_to_resend_count == RESEND_QUEUE_BUF_COUNT)
			need_to_resend_count = 0;

		if (general_resend_count == RESEND_QUEUE_BUF_COUNT)
			general_resend_count = RESEND_QUEUE_BUF_COUNT;
		else
			general_resend_count += 1;

		need_to_resend_count += 1;

		memcpy(&resend_msg_queue[need_to_resend_count-1][0], p_buf, buf_size);
		resend_msg_queue[need_to_resend_count-1][RESEND_QUEUE_BUF_SIZE-2] = port_num;
		resend_msg_queue[need_to_resend_count-1][RESEND_QUEUE_BUF_SIZE-1] = buf_size;

	}

	uart_receive_flag = 0;

	return;

	//memset(MODEM_ANSWER, 0 ,sizeof(MODEM_ANSWER));

}

//Функция разбора сообщения, принятого от модема LoRa
void ParseReceivedMsgLora() {

	char* inc_ptr_data    	 = 0;

	uint8_t  data_byte    	 = 0; //байт полезных данных из сообщений от модема
	uint8_t  half_byte_count = 0; //переменная, обозначающая старший или младший полубайт сейчас собирается (0 - старший, младший)
	uint8_t  port_num 		 = 0; //порт, который используется для приема/посылки сообщения
	uint8_t  payload_bytes 	 = 0; //количество байт полезной информации от модема
	uint32_t rec_buf_iter 	 = 0; //количество байт
	uint32_t rec_data_pos 	 = 0;

	uart_receive_flag 		 = 0;

	//Смотрим есть ли в данных от модема полезная информация (см. в документации на модем форматы сообщений)
	if (strstr(MODEM_ANSWER, "RX:")) {

		//Обновляем таймер на провеку соединения между светильником и сервером
		link_check_time = 3600;

		//вычисляем указатель, который указывает на полезные данные в сообщений от модема
		inc_ptr_data = strstr(MODEM_ANSWER, "RX:");
		//вычисляем индекс в массиве с данными, откуда начинаются полезные данные
		rec_data_pos = inc_ptr_data - MODEM_ANSWER + 5;

		//Так как данные, полученные от модема в символьном виде (1 байт - на самом деле полубайт значения в hex), то собираем
		//данные в hex вид
		while(MODEM_ANSWER[rec_data_pos] != '\"') {

			//старший полубайт получаем
			if (!half_byte_count) {

				data_byte = SymToHex(MODEM_ANSWER[rec_data_pos]) << 4;
				half_byte_count += 1;

			//младший полубайт получаем
			} else {

				data_byte |= SymToHex(MODEM_ANSWER[rec_data_pos]);
				lora_rec_buf[rec_buf_iter] = data_byte;
				rec_buf_iter += 1;
				payload_bytes += 1;
				half_byte_count = 0;
			}

			rec_data_pos += 1;

		}

		//Получаем номер порта из служебной информации из сообщения от модема
		inc_ptr_data = strstr(MODEM_ANSWER, "PORT:");
		rec_data_pos = inc_ptr_data - MODEM_ANSWER + 6;
		port_num = atoi(&MODEM_ANSWER[rec_data_pos]);

		//Если сообщение было принято по порту 2 (Unicast) или порту 200(Multicast)
		if (((port_num == 2) || (port_num == 200))) {

			/*Если не установлена сессия фрагментированной передачи данных, то обрабатываем полученные сообщения, иначе игнорируем
			* Все поля сообщений соотвествуют ПИО SmartLigth-LW, смотреть описание там.
			* Если плолезные данные больше одного байта, то значит пишли параметры, которые надо применить и сохранить
			* Если байт полезной информации всего один, то значит надо выдать на сервер текущие параметры, которые заправшивают
			* если запрос пришел по порту 2, то отправляем сразу ответ без подтверждения
			* если запрос пришел по порту 200, то отправляем ответ в очередь мультикаст сообщений с соотвествующей паузой на отправку
			* */
			if (!flags_and_states.frag_session_is_setup_flag) {

				switch(lora_rec_buf[0]) {

					case MODE_SET_REQ_ANS:
					{

						mode_set_req.cid = MODE_SET_REQ_ANS;

						if (payload_bytes > 1) {

							if ((lora_rec_buf[1] == 0x0) || (lora_rec_buf[1] == 0x1) || (lora_rec_buf[1] == 0x2)) {

								active_parameters.mode = lora_rec_buf[1];
								mode_set_req.mode = lora_rec_buf[1];
								flags_and_states.change_mode_flag = 1;

								if (active_parameters.mode != MANUAL_MODE) {
									WriteToEEPROM((uint8_t*)&active_parameters, PARAMETERS_STRUCT_ADDRESS, sizeof(active_parameters));
									HAL_Delay(100);
								}

							}

						}

						if (lora_rec_buf[1] == SCHEDULE_MODE) {

							if (flags_and_states.permission_dim_level_flag)
								SetDimLevel(time_schedule_struct.hour_on, time_schedule_struct.minutes_on,
											time_schedule_struct.hour_off, time_schedule_struct.minutes_off, coordinate_and_rtc.day_num);

						} else if (lora_rec_buf[1] == RISE_SET_MODE) {

							sunrise_hour_result    = time_rise_set_struct.sunrise_hour    + time_rise_set_struct.sunrise_time_offset/60;
							sunrise_minutes_result = time_rise_set_struct.sunrise_minutes + time_rise_set_struct.sunrise_time_offset%60;

							sunset_hour_result     = time_rise_set_struct.sunset_hour    + time_rise_set_struct.sunset_time_offset/60;
							sunset_minutes_result  = time_rise_set_struct.sunset_minutes + time_rise_set_struct.sunset_time_offset%60;

							CheckResultRiseSetTime();

							if (flags_and_states.permission_dim_level_flag)
								SetDimLevel(sunset_hour_result, sunset_minutes_result, sunrise_hour_result,
											sunrise_minutes_result, coordinate_and_rtc.day_num);
						}

						FillDevStatus(MODE_SET_REQ_ANS);

						flags_and_states.change_mode_flag = 0;

						if (port_num == 2) {

							SendMsgLora((uint8_t*)&dev_status_req_ans, sizeof(dev_status_req_ans), port_num, UNCONFIRM_CMD);

						} else if (port_num == 200) {

							FillMcastQueue((uint8_t*)&dev_status_req_ans, sizeof(dev_status_req_ans));

						}

						break;
					}


					case LUM_DIM_SET_REQ_ANS:
					{
						lum_dim_set_req_ans.cid = LUM_DIM_SET_REQ_ANS;

						if (payload_bytes == 1) {

							lum_dim_set_req_ans.dim_level = active_parameters.current_dim_level;

						} else if ((payload_bytes > 1) && (active_parameters.mode == MANUAL_MODE)) {

								if (lora_rec_buf[1] <= 100) {

									lum_dim_set_req_ans.dim_level = lora_rec_buf[1];
									flags_and_states.need_to_set_dim_level = 1;
									active_parameters.current_dim_level = lum_dim_set_req_ans.dim_level;
									WriteToEEPROM((uint8_t*)&active_parameters, PARAMETERS_STRUCT_ADDRESS, sizeof(active_parameters));
									HAL_Delay(100);

									if (!flags_and_states.permission_dim_level_flag) {

										CheckMetro();
									}

							}

						}

						FillDevStatus(LUM_DIM_SET_REQ_ANS);

						if (port_num == 2) {

							SendMsgLora((uint8_t*)&dev_status_req_ans, sizeof(dev_status_req_ans), port_num, UNCONFIRM_CMD);

						} else if (port_num == 200) {

							FillMcastQueue((uint8_t*)&dev_status_req_ans, sizeof(dev_status_req_ans));

						}

						break;

					}


					case TIME_ZONE_SET_REQ_ANS:
					{

						time_zone_set_req_ans.cid = TIME_ZONE_SET_REQ_ANS;

						if (payload_bytes == 1) {

							if (coordinate_and_rtc.UTC < 0)
								time_zone_set_req_ans.time_zone = (1 << 4) | (-coordinate_and_rtc.UTC);
							else
								time_zone_set_req_ans.time_zone  = coordinate_and_rtc.UTC & 0xF;

						} else if (payload_bytes > 1) {

							int8_t time_zone = lora_rec_buf[1];

							if (((time_zone & 0xF) <= 12) && ((time_zone & 0xF) >= -12)) {

								if (((lora_rec_buf[1] >> 4) & 0x1) == 0x1) {

									coordinate_and_rtc.UTC = -(lora_rec_buf[1] & 0xF);
									time_zone_set_req_ans.time_zone = (1 << 4) | (-coordinate_and_rtc.UTC);

								} else {
									coordinate_and_rtc.UTC = (lora_rec_buf[1] & 0xF);
									time_zone_set_req_ans.time_zone  = coordinate_and_rtc.UTC & 0xF;
								}

								CalculateRiseSetTime(&coordinate_and_rtc, &time_rise_set_struct);
								WriteToEEPROM((uint8_t*)&coordinate_and_rtc, COORDINATES_AND_RTC_ADDRESS, sizeof(coordinate_and_rtc));
								HAL_Delay(100);

								flags_and_states.set_dim_level_flag = 1;
							}

						}

						if (port_num == 2) {

							SendMsgLora((uint8_t*)&time_zone_set_req_ans, 2, port_num, UNCONFIRM_CMD);

						} else if (port_num == 200) {

							FillMcastQueue((uint8_t*)&time_zone_set_req_ans, sizeof(time_zone_set_req_ans));

						}

						break;

					}


					case DEV_STATUS_REQ_ANS:
					{

							FillDevStatus(DEV_STATUS_REQ_ANS);

							if (port_num == 2) {

								SendMsgLora((uint8_t*)&dev_status_req_ans, sizeof(dev_status_req_ans), port_num, UNCONFIRM_CMD);

							} else if (port_num == 200) {

								FillMcastQueue((uint8_t*)&dev_status_req_ans, sizeof(dev_status_req_ans));

							}


						break;

					}

					case PROFILE_OPT_SET_REQ:
					{

						dev_status_req_ans.cid = DEV_STATUS_REQ_ANS;

						if (payload_bytes > 1) {

							if (lora_rec_buf[1] < 0x2) {

								active_parameters.dim_active_profile = lora_rec_buf[1];

								WriteToEEPROM((uint8_t*)&active_parameters, PARAMETERS_STRUCT_ADDRESS, sizeof(active_parameters));
								HAL_Delay(100);

								need_to_set_profile = 1;

								if (active_parameters.mode == SCHEDULE_MODE) {

									if (flags_and_states.permission_dim_level_flag)
										SetDimLevel(time_schedule_struct.hour_on, time_schedule_struct.minutes_on,
													time_schedule_struct.hour_off, time_schedule_struct.minutes_off, coordinate_and_rtc.day_num);

								} else if (active_parameters.mode == RISE_SET_MODE) {

									sunrise_hour_result    = time_rise_set_struct.sunrise_hour    + time_rise_set_struct.sunrise_time_offset/60;
									sunrise_minutes_result = time_rise_set_struct.sunrise_minutes + time_rise_set_struct.sunrise_time_offset%60;

									sunset_hour_result     = time_rise_set_struct.sunset_hour    + time_rise_set_struct.sunset_time_offset/60;
									sunset_minutes_result  = time_rise_set_struct.sunset_minutes + time_rise_set_struct.sunset_time_offset%60;

									CheckResultRiseSetTime();

									if (flags_and_states.permission_dim_level_flag)
										SetDimLevel(sunset_hour_result, sunset_minutes_result, sunrise_hour_result,
													sunrise_minutes_result, coordinate_and_rtc.day_num);
								}

								need_to_set_profile = 0;

							}

						}


						FillDevStatus(PROFILE_OPT_SET_REQ);

						if (port_num == 2) {

							SendMsgLora((uint8_t*)&dev_status_req_ans, sizeof(dev_status_req_ans), port_num, UNCONFIRM_CMD);

						} else if (port_num == 200) {

							FillMcastQueue((uint8_t*)&dev_status_req_ans, sizeof(dev_status_req_ans));

						}

						break;

					}

					case GPS_SET_REQ_ANS:
					{

						gps_set_req_ans.cid = GPS_SET_REQ_ANS;
						int8_t sign = 1;

						if (payload_bytes == 1) {

							ReadFromEEPROM((uint8_t*)&gps_set_req_ans, GPS_COORDINATES_ADDRESS, sizeof(gps_set_req_ans));
							gps_set_req_ans.cid = GPS_SET_REQ_ANS;


						} else if (payload_bytes > 1) {

							memcpy((void*)&gps_set_req_ans, (void*)&lora_rec_buf, sizeof(gps_set_req_ans));

							gps_set_req_ans.lat = SwapInt32(gps_set_req_ans.lat);
							gps_set_req_ans.longt = SwapInt32(gps_set_req_ans.longt);

							if ((gps_set_req_ans.lat <= 9000000) && (gps_set_req_ans.longt <= 18000000) &&
								(gps_set_req_ans.lat >= -9000000) && (gps_set_req_ans.longt >= -18000000)) {

								lat = (float)gps_set_req_ans.lat / 100000.0f;
								longt = (float)gps_set_req_ans.longt / 100000.0f;

								if (longt < 0)
									sign = -1;
								else
									sign = 1;

								if (lat < 0)
									sign = -1;
								else
									sign = 1;

								lat_minutes = (int16_t)(sign*(lat - (int16_t)lat)*60.0f);
								lat_seconds = ((sign*(lat - (int16_t)lat)*60.0f - lat_minutes)*60.0f);

								long_minutes = (int16_t)(sign*(longt - (int16_t)longt)*60.0f);
								long_seconds = ((sign*(longt - (int16_t)longt)*60.0f - long_minutes)*60.0f);

								if (CheckCoordinates()) {

									ReadFromEEPROM((uint8_t*)&gps_set_req_ans, GPS_COORDINATES_ADDRESS, sizeof(gps_set_req_ans));
									gps_set_req_ans.cid = GPS_SET_REQ_ANS;

								} else {

									gps_set_req_ans.cid = GPS_SET_REQ_ANS;

									coordinate_and_rtc.lat_degrees = (int16_t)lat;
									coordinate_and_rtc.lat_minutes = (int16_t)lat_minutes;
									coordinate_and_rtc.lat_seconds = (int16_t)lat_seconds;

									coordinate_and_rtc.long_degrees = (int16_t)longt;
									coordinate_and_rtc.long_minutes = (int16_t)long_minutes;
									coordinate_and_rtc.long_seconds = (int16_t)long_seconds;

									gps_set_req_ans.lat = SwapInt32(gps_set_req_ans.lat);
									gps_set_req_ans.longt = SwapInt32(gps_set_req_ans.longt);

									CalculateRiseSetTime(&coordinate_and_rtc, &time_rise_set_struct);
									WriteToEEPROM((uint8_t*)&coordinate_and_rtc, COORDINATES_AND_RTC_ADDRESS, sizeof(coordinate_and_rtc));
									HAL_Delay(100);
									WriteToEEPROM((uint8_t*)&gps_set_req_ans, GPS_COORDINATES_ADDRESS, sizeof(gps_set_req_ans));
									HAL_Delay(100);

								}

							} else {

								ReadFromEEPROM((uint8_t*)&gps_set_req_ans, GPS_COORDINATES_ADDRESS, sizeof(gps_set_req_ans));
								gps_set_req_ans.cid = GPS_SET_REQ_ANS;

							}

						}

						if (port_num == 2) {

							SendMsgLora((uint8_t*)&gps_set_req_ans, sizeof(gps_set_req_ans), port_num, UNCONFIRM_CMD);

						} else if (port_num == 200) {

							FillMcastQueue((uint8_t*)&gps_set_req_ans, sizeof(gps_set_req_ans));

						}

						break;

					}


					case MOTION_OPT_SET_REQ:
					{

						dev_status_req_ans.cid = DEV_STATUS_REQ_ANS;

						if (payload_bytes > 1) {

							if ((lora_rec_buf[1] < 0x8)) {

								active_parameters.mot_option_active_profile = (lora_rec_buf[1] & 0x2) >> 1;
								active_parameters.mot_optiom_set_event      = lora_rec_buf[1] & 0x1;
								active_parameters.motion_mode               = (lora_rec_buf[1] & 0x4) >> 2;

								WriteToEEPROM((uint8_t*)&active_parameters, PARAMETERS_STRUCT_ADDRESS, sizeof(active_parameters));
								HAL_Delay(100);

								need_to_set_mot_opt = 1;
								flags_and_states.detect_movement = 1;

								if (active_parameters.mode == SCHEDULE_MODE) {

									if (flags_and_states.permission_dim_level_flag)
										SetDimLevel(time_schedule_struct.hour_on, time_schedule_struct.minutes_on,
													time_schedule_struct.hour_off, time_schedule_struct.minutes_off, coordinate_and_rtc.day_num);

								} else if (active_parameters.mode == RISE_SET_MODE) {

									sunrise_hour_result    = time_rise_set_struct.sunrise_hour    + time_rise_set_struct.sunrise_time_offset/60;
									sunrise_minutes_result = time_rise_set_struct.sunrise_minutes + time_rise_set_struct.sunrise_time_offset%60;

									sunset_hour_result     = time_rise_set_struct.sunset_hour    + time_rise_set_struct.sunset_time_offset/60;
									sunset_minutes_result  = time_rise_set_struct.sunset_minutes + time_rise_set_struct.sunset_time_offset%60;

									CheckResultRiseSetTime();

									if (flags_and_states.permission_dim_level_flag)
										SetDimLevel(sunset_hour_result, sunset_minutes_result, sunrise_hour_result,
													sunrise_minutes_result, coordinate_and_rtc.day_num);
								}

								need_to_set_mot_opt = 0;
								flags_and_states.detect_movement = 0;

							}

						}

						FillDevStatus(MOTION_OPT_SET_REQ);

						if (port_num == 2) {

							SendMsgLora((uint8_t*)&dev_status_req_ans, sizeof(dev_status_req_ans), port_num, UNCONFIRM_CMD);

						} else if (port_num == 200) {

							FillMcastQueue((uint8_t*)&dev_status_req_ans, sizeof(dev_status_req_ans));

						}


						break;
					}

					case MOT_SEN_SET_REQ_ANS:
					{

						mot_sen_set_req_ans.cid = MOT_SEN_SET_REQ_ANS;

						if (payload_bytes > 1) {

							memcpy((uint8_t*)&mot_sen_set_req_ans.time, &lora_rec_buf[1], 3);

							if ((lora_rec_buf[3] <= 100) && (SwapUInt16(mot_sen_set_req_ans.time) <= 1800) && (SwapUInt16(mot_sen_set_req_ans.time) >= 1)) {

								active_parameters.motion_dim_level = mot_sen_set_req_ans.dim_level;
								active_parameters.motion_sensor_time = SwapUInt16(mot_sen_set_req_ans.time);
								WriteToEEPROM((uint8_t*)&active_parameters, PARAMETERS_STRUCT_ADDRESS, sizeof(active_parameters));

							} else {

								mot_sen_set_req_ans.time      = SwapUInt16(active_parameters.motion_sensor_time);
								mot_sen_set_req_ans.dim_level = active_parameters.motion_dim_level;

							}

						} else if (payload_bytes == 1) {

							mot_sen_set_req_ans.time      = SwapUInt16(active_parameters.motion_sensor_time);
							mot_sen_set_req_ans.dim_level = active_parameters.motion_dim_level;

						}

						if (port_num==2) {

							SendMsgLora((uint8_t*)&mot_sen_set_req_ans, sizeof(mot_sen_set_req_ans), port_num, UNCONFIRM_CMD);

						} else if (port_num == 200) {

							FillMcastQueue((uint8_t*)&mot_sen_set_req_ans, sizeof(mot_sen_set_req_ans));

						}

						break;
					}

					case STATUS_LIGHTER_REQ_ANS:
					{

						status_lighter_req_ans.cid = STATUS_LIGHTER_REQ_ANS;

						if (payload_bytes == 1) {

							status_lighter_req_ans.max_current = SwapUInt16(active_parameters.max_current);
							status_lighter_req_ans.max_voltage = SwapUInt16(active_parameters.max_voltage);
							status_lighter_req_ans.min_current = SwapUInt16(active_parameters.min_current);
							status_lighter_req_ans.min_voltage = SwapUInt16(active_parameters.min_voltage);

						} else if (payload_bytes > 1) {

							memcpy((uint8_t*)&status_lighter_req_ans.max_current, (uint8_t*)&lora_rec_buf[1], 8);

							if ((SwapUInt16(status_lighter_req_ans.max_current) > 1500)  || (SwapUInt16(status_lighter_req_ans.min_current) < 10)
							   || (SwapUInt16(status_lighter_req_ans.max_current) < 10)  || (SwapUInt16(status_lighter_req_ans.min_current) > 1500)
							   || (SwapUInt16(status_lighter_req_ans.max_voltage) > 275) || (SwapUInt16(status_lighter_req_ans.min_voltage) < 85 )
							   || (SwapUInt16(status_lighter_req_ans.max_voltage) < 85)  || (SwapUInt16(status_lighter_req_ans.min_voltage) > 275)
							   || (SwapUInt16(status_lighter_req_ans.min_current) >= SwapUInt16(status_lighter_req_ans.max_current))
							   || (SwapUInt16(status_lighter_req_ans.min_voltage) >= SwapUInt16(status_lighter_req_ans.max_voltage))) {

								status_lighter_req_ans.max_current = SwapUInt16(active_parameters.max_current);
								status_lighter_req_ans.max_voltage = SwapUInt16(active_parameters.max_voltage);
								status_lighter_req_ans.min_current = SwapUInt16(active_parameters.min_current);
								status_lighter_req_ans.min_voltage = SwapUInt16(active_parameters.min_voltage);

							} else {

								active_parameters.max_current = SwapUInt16(status_lighter_req_ans.max_current);
								active_parameters.min_current = SwapUInt16(status_lighter_req_ans.min_current);
								active_parameters.max_voltage = SwapUInt16(status_lighter_req_ans.max_voltage);
								active_parameters.min_voltage = SwapUInt16(status_lighter_req_ans.min_voltage);

								flags_and_states.check_metro_flag = 1;

								WriteToEEPROM((uint8_t*)&active_parameters, PARAMETERS_STRUCT_ADDRESS, sizeof(active_parameters));
								HAL_Delay(100);

							}

						}

						if (port_num == 2) {

							SendMsgLora((uint8_t*)&status_lighter_req_ans, sizeof(status_lighter_req_ans), port_num, UNCONFIRM_CMD);

						} else if (port_num == 200) {

							FillMcastQueue((uint8_t*)&status_lighter_req_ans, sizeof(status_lighter_req_ans));

						}

						HAL_Delay(2000);

						break;
					}

					case MCAST_ANS_INT_REQ_ANS:
					{

						mcast_ans_int_req_ans.cid = MCAST_ANS_INT_REQ_ANS;

						if (payload_bytes == 1) {

							mcast_ans_int_req_ans.mcast_interval = SwapUInt16(active_parameters.mcast_interval);

						} else if (payload_bytes > 1) {

							active_parameters.mcast_interval = SwapUInt16(mcast_ans_int_req_ans.mcast_interval);

							memcpy((uint8_t*)&mcast_ans_int_req_ans.mcast_interval, &lora_rec_buf[1], 2);

							if ((SwapUInt16(mcast_ans_int_req_ans.mcast_interval) < 3601) && (SwapUInt16(mcast_ans_int_req_ans.mcast_interval) >= 0)) {

								active_parameters.mcast_interval = SwapUInt16(mcast_ans_int_req_ans.mcast_interval);

								WriteToEEPROM((uint8_t*)&active_parameters, PARAMETERS_STRUCT_ADDRESS, sizeof(active_parameters));
								HAL_Delay(100);

							} else {

								mcast_ans_int_req_ans.mcast_interval = SwapUInt16(active_parameters.mcast_interval);
							}

						}

						if (port_num == 2) {

							SendMsgLora((uint8_t*)&mcast_ans_int_req_ans, sizeof(mcast_ans_int_req_ans), port_num, UNCONFIRM_CMD);

						} else if (port_num == 200) {

							FillMcastQueue((uint8_t*)&mcast_ans_int_req_ans, sizeof(mcast_ans_int_req_ans));

						}

						break;

					}

					case SHED_SET_REQ_ANS:
					{

						shed_set_req_ans.cid = SHED_SET_REQ_ANS;

						if (payload_bytes == 1) {

							shed_set_req_ans.hash = SwapUInt32(CalcHash(SCHEDULE_START_ADDRESS, 2196));

						} else if (payload_bytes > 1) {

							uint8_t  data_schedule_size = (payload_bytes-1);
							uint8_t  days_count         = 0;
							uint16_t day                = 0;
							uint16_t  iter               = 0;

							for (days_count=0;days_count<data_schedule_size/6;days_count++) {

								memcpy(&shed_set_req, &lora_rec_buf[days_count*6+1], 6);

								day = SwapUInt16(shed_set_req.day_num);

								if (((day >= 1) && (day <= 366)
									&& (shed_set_req.hour_on < 24) && (shed_set_req.hour_off < 24)
									&& (shed_set_req.minutes_on < 60) && (shed_set_req.minutes_off < 60))) {

									result = 0;

								} else {

									result = 1;
									break;

								}

								if 	((shed_set_req.hour_on == shed_set_req.hour_off) && (shed_set_req.minutes_off == shed_set_req.minutes_on)) {

									result = 1;
									break;

								}

							}

							if (!result) {

								for (days_count=0;days_count<data_schedule_size/6;days_count++) {

									memcpy(&shed_set_req, &lora_rec_buf[days_count*6+1], 6);

									day = SwapUInt16(shed_set_req.day_num);

									if (coordinate_and_rtc.day_num == day) {

										time_schedule_struct.day = day;
										time_schedule_struct.hour_on = shed_set_req.hour_on;
										time_schedule_struct.hour_off = shed_set_req.hour_off;
										time_schedule_struct.minutes_on = shed_set_req.minutes_on;
										time_schedule_struct.minutes_off = shed_set_req.minutes_off;

										flags_and_states.set_dim_level_flag = 1;

										HAL_Delay(2000);

									}

									for (iter = 0; iter < sizeof(shed_set_req); iter++) {

										WriteToEEPROM((((uint8_t*)(&shed_set_req)) + iter), (SCHEDULE_START_ADDRESS + (day-1)*6 + iter) , 1);
										HAL_Delay(10);

									}

								}

								shed_set_req_ans.hash = SwapUInt32(CalcHash(SCHEDULE_START_ADDRESS, 2196));

							}

						}

						shed_set_req_ans.cid = SHED_SET_REQ_ANS;

						if (port_num == 2) {

							shed_set_req_ans.cid = SHED_SET_REQ_ANS;

							SendMsgLora((uint8_t*)&shed_set_req_ans, sizeof(shed_set_req_ans), port_num, UNCONFIRM_CMD);

						} else if (port_num == 200) {

							FillMcastQueue((uint8_t*)&shed_set_req_ans, sizeof(shed_set_req_ans));

						}

						break;
					}

					case PROFILE_SET_REQ_ANS:
					{

						profile_set_ans.cid = PROFILE_SET_REQ_ANS;

						if (payload_bytes == 1) {

							profile_set_ans.hash = SwapUInt32(CalcHash(DIM_PROFILE_START_ADDRESS, 288));

						} else if (payload_bytes > 1) {

							uint8_t data_profile_size = (payload_bytes - 1);
							uint8_t interval_count    =  0;

							for(interval_count = 0; interval_count<data_profile_size/2; interval_count++) {

								memcpy(&profile_set_req, &lora_rec_buf[interval_count*2 + 1], 2);

								if ((profile_set_req.int_num < 144) && (profile_set_req.dim_level <= 100)) {

									result = 0;

								} else {

									result = 1;
									break;

								}

							}

							if (!result) {

								for(interval_count = 0; interval_count<data_profile_size/2; interval_count++) {

									memcpy(&profile_set_req, &lora_rec_buf[interval_count*2 + 1], 2);

									WriteToEEPROM((uint8_t*)&profile_set_req, (DIM_PROFILE_START_ADDRESS + (profile_set_req.int_num)*2) , 2);
									HAL_Delay(100);

								}

								flags_and_states.set_dim_level_flag = 1;

								profile_set_ans.hash = SwapUInt32(CalcHash(DIM_PROFILE_START_ADDRESS, 288));


							}

						}

						profile_set_ans.cid = PROFILE_SET_REQ_ANS;

						if (port_num == 2) {

							SendMsgLora((uint8_t*)&profile_set_ans, sizeof(profile_set_ans), port_num, UNCONFIRM_CMD);

						} else if (port_num == 200) {

							FillMcastQueue((uint8_t*)&profile_set_ans, sizeof(profile_set_ans));

						}

						break;

					}


					case SUN_RS_SET_REQ_ANS:
					{

						int8_t current_sunrise_time_offset = time_rise_set_struct.sunrise_time_offset;
						int8_t current_sunset_time_offset  = time_rise_set_struct.sunset_time_offset;

						sun_rs_set_req_ans.cid = SUN_RS_SET_REQ_ANS;


						if (payload_bytes == 1) {

							sun_rs_set_req_ans.sunrise_time_offset = time_rise_set_struct.sunrise_time_offset;
							sun_rs_set_req_ans.sunset_time_offset  = time_rise_set_struct.sunset_time_offset;


						} else if (payload_bytes > 1) {

							time_rise_set_struct.sunrise_time_offset = lora_rec_buf[1];
							time_rise_set_struct.sunset_time_offset  = lora_rec_buf[2];

							sunrise_hour_result    = time_rise_set_struct.sunrise_hour    + time_rise_set_struct.sunrise_time_offset/60;
							sunrise_minutes_result = time_rise_set_struct.sunrise_minutes + time_rise_set_struct.sunrise_time_offset%60;

							sunset_hour_result     = time_rise_set_struct.sunset_hour    + time_rise_set_struct.sunset_time_offset/60;
							sunset_minutes_result  = time_rise_set_struct.sunset_minutes + time_rise_set_struct.sunset_time_offset%60;

							CheckResultRiseSetTime();

							if (!((sunset_hour_result == sunrise_hour_result) && (sunrise_minutes_result == sunset_minutes_result))) {

								sun_rs_set_req_ans.sunrise_time_offset = time_rise_set_struct.sunrise_time_offset;
								sun_rs_set_req_ans.sunset_time_offset  = time_rise_set_struct.sunset_time_offset;

								WriteToEEPROM((uint8_t*)&time_rise_set_struct, TIME_RISE_SET_DAY_ADDRESS, sizeof(time_rise_set_struct));
								HAL_Delay(100);

								flags_and_states.set_dim_level_flag = 1;

							} else {

								time_rise_set_struct.sunrise_time_offset = current_sunrise_time_offset;
								time_rise_set_struct.sunset_time_offset  = current_sunset_time_offset;

								sun_rs_set_req_ans.sunrise_time_offset = time_rise_set_struct.sunrise_time_offset;
								sun_rs_set_req_ans.sunset_time_offset  = time_rise_set_struct.sunset_time_offset;

							}



						}

						if (port_num == 2){

							SendMsgLora((uint8_t*)&sun_rs_set_req_ans, sizeof(sun_rs_set_req_ans), port_num, UNCONFIRM_CMD);

						} else if (port_num == 200){

							FillMcastQueue((uint8_t*)&sun_rs_set_req_ans, sizeof(sun_rs_set_req_ans));

						}

						break;

					}

					case SCENARIO_DIM_REQ:
					{

						scenario_dim_req.cid = SCENARIO_DIM_REQ;

						if (payload_bytes > 1) {

							if ((lora_rec_buf[1] <= 100) || (lora_rec_buf[1] == 0xFF)) {

								scenario_dim_req.dim_level = lora_rec_buf[1];
								active_parameters.scenario_dim_level = scenario_dim_req.dim_level;

								if (scenario_dim_req.dim_level != 0xFF)
									active_parameters.scenario_active_profile = 1;
								else if (scenario_dim_req.dim_level == 0xFF)
									active_parameters.scenario_active_profile = 0;

								WriteToEEPROM((uint8_t*)&active_parameters, PARAMETERS_STRUCT_ADDRESS, sizeof(active_parameters));
								HAL_Delay(100);

								need_to_set_scen = 1;

								if (active_parameters.mode == SCHEDULE_MODE) {

									if (flags_and_states.permission_dim_level_flag)
										SetDimLevel(time_schedule_struct.hour_on, time_schedule_struct.minutes_on,
													time_schedule_struct.hour_off, time_schedule_struct.minutes_off, coordinate_and_rtc.day_num);

								} else if (active_parameters.mode == RISE_SET_MODE) {

									sunrise_hour_result    = time_rise_set_struct.sunrise_hour    + time_rise_set_struct.sunrise_time_offset/60;
									sunrise_minutes_result = time_rise_set_struct.sunrise_minutes + time_rise_set_struct.sunrise_time_offset%60;

									sunset_hour_result     = time_rise_set_struct.sunset_hour    + time_rise_set_struct.sunset_time_offset/60;
									sunset_minutes_result  = time_rise_set_struct.sunset_minutes + time_rise_set_struct.sunset_time_offset%60;

									CheckResultRiseSetTime();

									if (flags_and_states.permission_dim_level_flag)
										SetDimLevel(sunset_hour_result, sunset_minutes_result, sunrise_hour_result,
													sunrise_minutes_result, coordinate_and_rtc.day_num);
								}

								need_to_set_scen = 0;

							}

						}

						FillDevStatus(SCENARIO_DIM_REQ);

						if (port_num == 2)

							SendMsgLora((uint8_t*)&dev_status_req_ans, sizeof(dev_status_req_ans), port_num, UNCONFIRM_CMD);

						else if (port_num == 200) {

							FillMcastQueue((uint8_t*)&dev_status_req_ans, sizeof(dev_status_req_ans));

						}

						break;
					}

					case DEV_STATUS_SETTTING_REQ_ANS:
					{

						dev_status_settings_req_ans.cid = DEV_STATUS_SETTTING_REQ_ANS;

							if (payload_bytes > 1) {

								memcpy((uint8_t*)&dev_status_settings_req_ans.status_interval, (uint8_t*)&lora_rec_buf[1], 3);

								if ((SwapUInt16(dev_status_settings_req_ans.status_interval) >= 0) && (dev_status_settings_req_ans.response_set<0x2) &&
									((SwapUInt16(dev_status_settings_req_ans.status_interval) < 3601) && (dev_status_settings_req_ans.response_set >= 0))) {

									active_parameters.response_set = dev_status_settings_req_ans.response_set & 0x1;
									active_parameters.status_interval = SwapUInt16(dev_status_settings_req_ans.status_interval);
									//active_parameters.status_mode     = (dev_status_settings_req_ans.response_set & 0x2) >> 1;

									WriteToEEPROM((uint8_t*)&active_parameters, PARAMETERS_STRUCT_ADDRESS, sizeof(active_parameters));
									HAL_Delay(100);

								}

							}

							dev_status_settings_req_ans.response_set = active_parameters.response_set;
							dev_status_settings_req_ans.status_interval = SwapUInt16(active_parameters.status_interval);
							//dev_status_settings_req_ans.response_set |= (active_parameters.status_mode << 1);


							if (port_num == 2)

								SendMsgLora((uint8_t*)&dev_status_settings_req_ans, sizeof(dev_status_settings_req_ans), port_num, UNCONFIRM_CMD);

							else if (port_num == 200) {

								FillMcastQueue((uint8_t*)&dev_status_settings_req_ans, sizeof(dev_status_settings_req_ans));

							}

							break;

					}


					case LUM_AUTO_DIM_SET_REQ_ANS:
					{
						lum_auto_dim_set_req_ans.cid = LUM_AUTO_DIM_SET_REQ_ANS;

						if (payload_bytes == 1) {

							lum_auto_dim_set_req_ans.dim_level_auto = active_parameters.dim_auto_level;

						} else if (payload_bytes > 1) {

							if (lora_rec_buf[1] <= 100) {

								lum_auto_dim_set_req_ans.dim_level_auto = lora_rec_buf[1];

								if (lum_auto_dim_set_req_ans.dim_level_auto != active_parameters.dim_auto_level) {
									active_parameters.dim_auto_level = lum_auto_dim_set_req_ans.dim_level_auto;
								}

								flags_and_states.set_dim_level_flag = 1;

								HAL_Delay(2000);

								WriteToEEPROM((uint8_t*)&active_parameters, PARAMETERS_STRUCT_ADDRESS, sizeof(active_parameters));
								HAL_Delay(100);

							}

						}


						if (port_num == 2) {

							SendMsgLora((uint8_t*)&lum_auto_dim_set_req_ans, sizeof(lum_auto_dim_set_req_ans), port_num, UNCONFIRM_CMD);

						} else if (port_num == 200) {

							FillMcastQueue((uint8_t*)&lum_auto_dim_set_req_ans, sizeof(lum_auto_dim_set_req_ans));

						}

						break;

					}

					case RESET_OPER_TIME_LIGHT_REQ:
					{

						if (lora_rec_buf[1] == 0xFF) {

							active_parameters.lamp_running_time = 0;
							WriteToEEPROM((uint8_t*)&active_parameters, PARAMETERS_STRUCT_ADDRESS, sizeof(active_parameters));
							HAL_Delay(100);
							//ReadFromEEPROM((uint8_t*)&active_parameters.lamp_running_time, LAMP_RUNNING_TIME_ADDRESS, 4);
							//HAL_Delay(100);//сбросить счетчик наработки светильника

							GetModemRTC(&coordinate_and_rtc);

							oper_time_rep.event_time          = SwapUInt32(coordinate_and_rtc.gps_epoch_time);
							oper_time_rep.cid                 = RESET_OPER_TIME_LIGHT_REQ;
							oper_time_rep.controller_run_time = SwapUInt32(active_parameters.controller_running_time);
							oper_time_rep.lamp_run_time       = SwapUInt32(active_parameters.lamp_running_time);
							oper_time_rep.power_consumption   = SwapUInt32(active_parameters.power_consumption);

							if (port_num == 2) {

								SendMsgLora((uint8_t*)&oper_time_rep, sizeof(oper_time_rep), port_num, CONFIRM_CMD);

							} else if (port_num == 200) {

								FillMcastQueue((uint8_t*)&oper_time_rep, sizeof(oper_time_rep));

							}
						}

						break;

					}

					case RESET_OPER_TIME_ENERGO_REQ:
					{

						if (lora_rec_buf[1] == 0xFF) {
								//сбросить счетчик наработки светильника
							active_parameters.power_consumption = 0;
							WriteToEEPROM((uint8_t*)&active_parameters, PARAMETERS_STRUCT_ADDRESS, sizeof(active_parameters));
							HAL_Delay(100);
							//ReadFromEEPROM((uint8_t*)&active_parameters.power_consumption, POWER_CONSUMPTION_ADDRESS, 4);
							//HAL_Delay(100);//сбросить счетчик наработки светильника

							GetModemRTC(&coordinate_and_rtc);

							oper_time_rep.event_time          = SwapUInt32(coordinate_and_rtc.gps_epoch_time);
							oper_time_rep.cid                 = RESET_OPER_TIME_ENERGO_REQ;
							oper_time_rep.controller_run_time = SwapUInt32(active_parameters.controller_running_time);
							oper_time_rep.lamp_run_time       = SwapUInt32(active_parameters.lamp_running_time);
							oper_time_rep.power_consumption   = SwapUInt32(active_parameters.power_consumption);

							if (port_num == 2)
								SendMsgLora((uint8_t*)&oper_time_rep, sizeof(oper_time_rep), port_num, CONFIRM_CMD);
							else {

								FillMcastQueue((uint8_t*)&oper_time_rep, sizeof(oper_time_rep));

							}

						}

						break;

					}

					case OPER_TIME_REP:
					{

						GetModemRTC(&coordinate_and_rtc);

						oper_time_rep.event_time          = SwapUInt32(coordinate_and_rtc.gps_epoch_time);
						oper_time_rep.cid                 = OPER_TIME_REP;
						oper_time_rep.controller_run_time = SwapUInt32(active_parameters.controller_running_time);
						oper_time_rep.lamp_run_time       = SwapUInt32(active_parameters.lamp_running_time);
						oper_time_rep.power_consumption   = SwapUInt32(active_parameters.power_consumption);

						if (port_num == 2)

							SendMsgLora((uint8_t*)&oper_time_rep, sizeof(oper_time_rep), port_num, CONFIRM_CMD);
						else {

							FillMcastQueue((uint8_t*)&oper_time_rep, sizeof(oper_time_rep));

						}

						break;


					}


					case FIRMWARE_VERSION_REQ:
					{

						ReadFromEEPROM((uint8_t*)&firmware_version, FIRMWARE_VERSION_ADDRESS, sizeof(firmware_version));
						HAL_Delay(50);

						firmware_version.cid = FIRMWARE_VERSION_REQ;

						if (port_num == 2) {

							SendMsgLora((uint8_t*)&firmware_version, sizeof(firmware_version), port_num, UNCONFIRM_CMD);

						} else if (port_num == 200){

							FillMcastQueue((uint8_t*)&firmware_version, sizeof(firmware_version));

						}

						break;

					}


					case DEVICE_PARAMS_REQ_ANS:
					{

						device_params_ans.cid = DEVICE_PARAMS_REQ_ANS;

						device_params_ans.temperature = CheckModemTemp();
						device_params_ans.current     = SwapUInt16((uint16_t)STPM_RMS_AMPS);
						device_params_ans.voltage     = SwapUInt16((uint16_t)STPM_RMS_VOLTS);


						if (port_num == 2) {

							SendMsgLora((uint8_t*)&device_params_ans, sizeof(device_params_ans), port_num, UNCONFIRM_CMD);

						} else if (port_num == 200){

							FillMcastQueue((uint8_t*)&device_params_ans, sizeof(device_params_ans));

						}

						break;

					}

					case NEW_MC_DEV_ADDR_SET_REQ:
					{

						 new_mc_dev_addr_set_req.cid = NEW_MC_DEV_ADDR_SET_REQ;

						if (payload_bytes == 1) {

							ReadFromEEPROM((uint8_t*)&new_mc_dev_addr_set_req.mcast_addr, MCAST_DEVADDR_ADDRESS, 4);

							if (port_num == 2) {

								SendMsgLora((uint8_t*)&new_mc_dev_addr_set_req, sizeof(new_mc_dev_addr_set_req), port_num, UNCONFIRM_CMD);

							} else if (port_num == 200){

								FillMcastQueue((uint8_t*)&new_mc_dev_addr_set_req, sizeof(new_mc_dev_addr_set_req));

							}

						} else if (payload_bytes > 1) {

							memcpy((void*)&new_mc_dev_addr_set_req.mcast_addr, (void*)&lora_rec_buf[1], 4);

							WriteToEEPROM((uint8_t*)&new_mc_dev_addr_set_req.mcast_addr, MCAST_DEVADDR_ADDRESS, 4);

							SetMcastParams();

							if (port_num == 2) {

								SendMsgLora((uint8_t*)&new_mc_dev_addr_set_req, sizeof(new_mc_dev_addr_set_req), port_num, UNCONFIRM_CMD);

							} else if  (port_num == 200) {

								FillMcastQueue((uint8_t*)&new_mc_dev_addr_set_req, sizeof(new_mc_dev_addr_set_req));

							}

						}

						break;

					}

					default:
						break;

				}

			}

			memset(lora_rec_buf, 0 ,sizeof(lora_rec_buf));
			memset(MODEM_ANSWER,0,sizeof(MODEM_ANSWER));
			HAL_Delay(100);

		//Если порт 201 или 202, то это касается сессии фрагментированной передачи данных (см. документацию на фрагменитированную передачу данных)
		} else if ((port_num == 201) || (port_num == 202)) {

			//Определяем тип сообщения
			switch(lora_rec_buf[0]) {

				//Установка сессии фрагментированной передачи данных
				case FRAG_SESSION_SETUP_REQ_ANS:
				{

					if (!flags_and_states.frag_session_is_setup_flag) {

						flags_and_states.frag_session_is_setup_flag = 1;

						memset((uint8_t*)&frag_session_setup_ans, 0, sizeof(frag_session_setup_ans));
						memset((uint8_t*)&frag_session_status_ans, 0, sizeof(frag_session_status_ans));

						memcpy((uint8_t*)&frag_session_setup_req, &lora_rec_buf, 11);

						frag_session_setup_req.nb_frag = SwapUInt16(frag_session_setup_req.nb_frag);
						frag_session_setup_req.descriptor = SwapUInt32(frag_session_setup_req.descriptor);

						if (((frag_session_setup_req.nb_frag*frag_session_setup_req.frag_size) > 5120) ||
								(frag_session_setup_req.nb_frag < 1) || (frag_session_setup_req.frag_size < 1)) {

							memset((void*)&frag_session_setup_req, 0, sizeof(frag_session_setup_req));

							frag_session_setup_ans.status_bit_mask |= 1 << 1;
							flags_and_states.frag_session_is_setup_flag = 0;

						}

						if ((frag_session_setup_req.control & 0x38) != 0x0) {

							frag_session_setup_ans.status_bit_mask |= 1;
							flags_and_states.frag_session_is_setup_flag = 0;

						}

						if (((frag_session_setup_req.frag_session & 0x30) >> 4) > 0x1) {

							frag_session_setup_ans.status_bit_mask |= 1 << 2;
							flags_and_states.frag_session_is_setup_flag = 0;

						}

						if (frag_session_setup_ans.status_bit_mask == 0) {

							if (CreateCheckParityMatrix(frag_session_setup_req.nb_frag, frag_session_setup_req.nb_frag)) {
								frag_session_setup_ans.status_bit_mask |= 1 << 1;
								flags_and_states.frag_session_is_setup_flag = 0;
							} else {

								frag_session_status_ans.missing_frag = frag_session_setup_req.nb_frag;
							}

						}

						frag_session_setup_ans.cid = FRAG_SESSION_SETUP_REQ_ANS;

						if (((frag_session_setup_req.frag_session & 0x10) >> 4) == 0x0) {

							SendMsgLora((uint8_t*)&frag_session_setup_ans, sizeof(frag_session_setup_ans), port_num, UNCONFIRM_CMD);

						} else if (((frag_session_setup_req.frag_session & 0x10) >> 4) == 0x1) {

							frag_session_setup_ans.status_bit_mask |= 1 << 6;

							FillFragSessionQueue((uint8_t*)&frag_session_setup_ans, sizeof(frag_session_setup_ans), port_num);

						}

					} else if (flags_and_states.frag_session_is_setup_flag) {

						frag_session_status_ans.cid = FRAG_STATUS_REQ_ANS;


						frag_session_status_ans.received_and_index = (frag_session_status_ans.received_and_index & 0x3FFF);

						if (((lora_rec_buf[1] & 0x10) >> 4) == 0x0) {

							frag_session_status_ans.received_and_index &= ~(0x4000);

							frag_session_status_ans.received_and_index = SwapUInt16(frag_session_status_ans.received_and_index);

							SendMsgLora((uint8_t*)&frag_session_status_ans, sizeof(frag_session_status_ans), port_num, UNCONFIRM_CMD);

						} else if (((lora_rec_buf[1] & 0x10) >> 4) == 0x1) {

							frag_session_status_ans.received_and_index |= 1 << 14;

							frag_session_status_ans.received_and_index = SwapUInt16(frag_session_status_ans.received_and_index);

							FillFragSessionQueue((uint8_t*)&frag_session_status_ans, sizeof(frag_session_status_ans), port_num);

						}

						frag_session_status_ans.received_and_index = SwapUInt16(frag_session_status_ans.received_and_index);


					}

					break;

				}

				case FRAG_SESSION_DELETE_REQ_ANS:
				{
					memcpy((uint8_t*)&frag_session_delete_req, lora_rec_buf, 2);

					memset((uint8_t*)&frag_session_delete_ans, 0, 2);

					if (!flags_and_states.frag_session_is_setup_flag) {

						frag_session_delete_ans.status |= 1 << 2;

					} else {

						flags_and_states.frag_session_is_setup_flag = 0;
						memset((uint8_t*)&frag_session_setup_req, 0, sizeof(frag_session_setup_req));
						memset((uint8_t*)&frag_session_status_ans, 0, sizeof(frag_session_status_ans));
						memset((uint8_t*)&frag_session_status_req, 0, sizeof(frag_session_status_req));
						memset((uint8_t*)&frag_session_setup_ans, 0, sizeof(frag_session_setup_ans));
						if (frag_session_status_ans.missing_frag)
							DeleteCheckParityMatrix(frag_session_setup_req.nb_frag);

					}

					frag_session_delete_ans.cid = FRAG_SESSION_DELETE_REQ_ANS;

					if ((frag_session_delete_req.param & 0x1) == 0x0) {

						SendMsgLora((uint8_t*)&frag_session_delete_ans, sizeof(frag_session_delete_ans), port_num, UNCONFIRM_CMD);

					} else if ((frag_session_delete_req.param & 0x1) == 0x1) {

						frag_session_delete_ans.status |= 1;

						FillFragSessionQueue((uint8_t*)&frag_session_delete_ans, sizeof(frag_session_delete_ans), port_num);
					}

					break;
				}

				case DATA_FRAGMENT:
				{

					if (flags_and_states.frag_session_is_setup_flag && (frag_session_status_ans.missing_frag != 0)) {

						frag_session_status_ans.received_and_index += 1;

						if (((frag_session_status_ans.received_and_index & 0x3FFF)*frag_session_setup_req.frag_size) > 5120) {
							frag_session_status_ans.status |= 1;
							//flags_and_states.frag_session_is_setup_flag = 0;

							break;

						}

						memcpy(&index_and_n, &lora_rec_buf[1], 2);
						index_and_n = SwapUInt16(index_and_n);
						frag_index  = index_and_n & 0x3FFF;
						memset(coded_fragment, 0 , sizeof(coded_fragment));
						memcpy(coded_fragment, &lora_rec_buf[3],frag_session_setup_req.frag_size);

						DecodeFragment(frag_index-1);

						//
						CheckMatrixA();

						if (frag_session_status_ans.missing_frag == 0) {

							ReassembleDataBlock(port_num);
							DeleteCheckParityMatrix(frag_session_setup_req.nb_frag);

						}

					}

					break;
				}

				case FRAG_STATUS_REQ_ANS:
				{

					frag_session_status_ans.cid = FRAG_STATUS_REQ_ANS;

					memcpy((uint8_t*)&frag_session_status_req, lora_rec_buf, 2);

					if (flags_and_states.frag_session_is_setup_flag) {

						if (((frag_session_status_req.frag_status_req_param & 0x1) == 0x1) ||
							(((frag_session_status_req.frag_status_req_param & 0x1) == 0x0) && (frag_session_status_ans.missing_frag))) {

							frag_session_status_ans.received_and_index = (frag_session_status_ans.received_and_index & 0x3FFF);
																		 //| ((frag_session_status_req.frag_status_req_param & 0x6) << 13);




							if (((frag_session_status_req.frag_status_req_param & 0x2)) == 0x0) {

								frag_session_status_ans.received_and_index &= ~(0x4000);

								frag_session_status_ans.received_and_index = SwapUInt16(frag_session_status_ans.received_and_index);

								SendMsgLora((uint8_t*)&frag_session_status_ans, sizeof(frag_session_status_ans), port_num, UNCONFIRM_CMD);

							} else if (((frag_session_status_req.frag_status_req_param & 0x2)) == 0x2) {

								frag_session_status_ans.received_and_index |= 1 << 14;

								frag_session_status_ans.received_and_index = SwapUInt16(frag_session_status_ans.received_and_index);

								FillFragSessionQueue((uint8_t*)&frag_session_status_ans, sizeof(frag_session_status_ans), port_num);

							}

							frag_session_status_ans.received_and_index = SwapUInt16(frag_session_status_ans.received_and_index);
						}

					} else if (!flags_and_states.frag_session_is_setup_flag)  {

						memset((uint8_t*)&frag_session_status_ans, 0, sizeof(frag_session_status_ans));

						frag_session_status_ans.cid = FRAG_STATUS_REQ_ANS;

						frag_session_status_ans.status |= 1 << 1;

						frag_session_status_ans.received_and_index = (frag_session_status_ans.received_and_index & 0x3FFF)
																	 | ((frag_session_status_req.frag_status_req_param & 0x6) << 13);

						if (((frag_session_status_req.frag_status_req_param & 0x2)) == 0x0) {

							frag_session_status_ans.received_and_index &= ~(0x4000);

							frag_session_status_ans.received_and_index = SwapUInt16(frag_session_status_ans.received_and_index);

							SendMsgLora((uint8_t*)&frag_session_status_ans, sizeof(frag_session_status_ans), port_num, UNCONFIRM_CMD);

						} else if (((frag_session_status_req.frag_status_req_param & 0x2)) == 0x2) {

							frag_session_status_ans.received_and_index |= 1 << 14;

							frag_session_status_ans.received_and_index = SwapUInt16(frag_session_status_ans.received_and_index);

							FillFragSessionQueue((uint8_t*)&frag_session_status_ans, sizeof(frag_session_status_ans), port_num);

						}

						frag_session_status_ans.received_and_index = SwapUInt16(frag_session_status_ans.received_and_index);

					}

					break;
				}

				default:
					break;

			}

			memset(lora_rec_buf, 0 ,sizeof(lora_rec_buf));
			memset(MODEM_ANSWER,0,sizeof(MODEM_ANSWER));
			HAL_Delay(100);

		}

	}

}

//Запрос температуры от модема
int8_t CheckModemTemp() {

	int8_t temp = 0x0;

	*MODEM_SEND = "AT+TEMP\r\n";
	HAL_UART_Transmit(&LORA_UART, (uint8_t*)*MODEM_SEND, strlen( *MODEM_SEND ),0x100);
	HAL_Delay(300);

	temp = atoi(&MODEM_ANSWER[7]);

	return temp;

}

//Запрос текущей скорости передачи данных модема LoRa
uint8_t GetModemDataRate() {

	uint8_t rate = 0;

	*MODEM_SEND = "AT+DR\r\n";
	HAL_UART_Transmit(&LORA_UART, (uint8_t*)*MODEM_SEND, strlen( *MODEM_SEND ),0x100);
	HAL_Delay(300);

	rate = atoi(&MODEM_ANSWER[7]);

	return rate;

}

//Установка скорости передачи данных модема LoRa
void SetModemDataRate(uint8_t data_rate) {

	switch(data_rate) {

		case 1:
		{
			*MODEM_SEND = "AT+DR=DR1\r\n";
			break;
		}

		case 2:
		{
			*MODEM_SEND = "AT+DR=DR2\r\n";
			break;
		}

		case 3:
		{
			*MODEM_SEND = "AT+DR=DR3\r\n";
			break;
		}

		case 4:
		{
			*MODEM_SEND = "AT+DR=DR4\r\n";
			break;
		}

		case 5:
		{
			*MODEM_SEND = "AT+DR=DR5\r\n";
			break;
		}

		case 6:
		{
			*MODEM_SEND = "AT+DR=DR6\r\n";
			break;
		}

	}

	HAL_UART_Transmit(&LORA_UART, (uint8_t*)*MODEM_SEND, strlen(*MODEM_SEND), 0x100);
	HAL_Delay(100);


	return;
}


//Установка DevUI в модеме
void SetDevUi() {

	uint8_t id_buf[8] = {0};
	uint8_t cmd_iter  = 0;
	uint8_t buf_iter  = 0;

	memcpy(id_buf, dev_id, sizeof(id_buf));

	for(buf_iter=0; buf_iter<4; buf_iter++) {

		id_buf[buf_iter] = id_buf[buf_iter] ^ id_buf[7-buf_iter];
		id_buf[7-buf_iter] = id_buf[7-buf_iter] ^ id_buf[buf_iter];

	}


	buf_iter = 0;

	memset(&dev_ui[13], 0 , sizeof(dev_ui)-13);

	for (cmd_iter = 13; cmd_iter<8*2+13; cmd_iter += 2) {

		sprintf(&dev_ui[cmd_iter], "%02X", id_buf[buf_iter]);
		buf_iter += 1;
	}

	dev_ui[8*2+13] = '\"';
	dev_ui[8*2+14] = '\r';
	dev_ui[8*2+15] = '\n';

	HAL_UART_Transmit(&LORA_UART, dev_ui, 8*2+16, 0x200);

	return;

}

//Установка парметров мультикаст вещания в модеме (мультикаст адрес, ключ приложения, ключ сети)
uint8_t SetMcastParams() {

	new_mc_dev_addr_set_req.mcast_addr = SwapUInt32(new_mc_dev_addr_set_req.mcast_addr);

	uint8_t data_iter = 0;
	uint8_t buf_iter  = 0;
	uint8_t result = 0;
	uint32_t addr = new_mc_dev_addr_set_req.mcast_addr;

	PRNG_AppSKey(new_mc_dev_addr_set_req.mcast_addr);
	PRNG_NwkSKey(new_mc_dev_addr_set_req.mcast_addr);

	//for (buf_iter = 0; buf_iter < 8; buf_iter+=2) {
	sprintf(&mcast_lw_cmd[13], "%02X", new_mc_dev_addr_set_req.mcast_addr);
		//data_iter += 1;

	//}

	mcast_lw_cmd[21] = '"';
	mcast_lw_cmd[22] = ',';
	mcast_lw_cmd[23] = '"';

	data_iter = 0;

	for(buf_iter=24;buf_iter<24+32;buf_iter += 2) {

		sprintf(&mcast_lw_cmd[buf_iter], "%02X", new_mc_dev_addr_set_req.nwskey[data_iter++]);
	}

	mcast_lw_cmd[56] = '"';
	mcast_lw_cmd[57] = ',';
	mcast_lw_cmd[58] = '"';

	data_iter = 0;

	for(buf_iter=59;buf_iter<59+32;buf_iter += 2) {

		sprintf(&mcast_lw_cmd[buf_iter], "%02X", new_mc_dev_addr_set_req.appskey[data_iter++]);
	}

	mcast_lw_cmd[91] = '"';
	mcast_lw_cmd[92] = ',';
	mcast_lw_cmd[93] = '0';
	mcast_lw_cmd[94] = '\r';
	mcast_lw_cmd[95] = '\n';

	HAL_UART_Transmit(&LORA_UART, mcast_lw_cmd, 96, 0x200);
	HAL_Delay(1000);
	if (strstr(MODEM_ANSWER, "ERROR")) {
		new_mc_dev_addr_set_req.mcast_addr = SwapUInt32(new_mc_dev_addr_set_req.mcast_addr);
		result = 0;
		return result;
	}
	HAL_UART_Transmit(&BT_UART, MODEM_ANSWER, byte_send_bt, 0x100);


	result = 1;
	new_mc_dev_addr_set_req.mcast_addr = SwapUInt32(new_mc_dev_addr_set_req.mcast_addr);

	return result;

}

//Добавление сообщения в мультикаст очередь
void FillMcastQueue(uint8_t* msg_buf, uint16_t msg_size) {

	uint8_t  iter_msg         		 = 0;
	uint8_t  need_to_copy_msg 		 = 1;
	uint16_t mcast_interval_time_msg = 0;

	//Проверяем нет ли идентичного сообшения в очереди, если есть то не добавляем
	for (iter_msg = 0; iter_msg<MCAST_QUEUE_BUF_COUNT; iter_msg++) {

		if (memcmp((void*)msg_buf, (void*)&mcast_msg_queue[iter_msg][0], msg_size) == 0)
		{
			need_to_copy_msg = 0;
			break;
		}

	}

	//Если надо доабвить
	if (need_to_copy_msg) {

		//высчитываем интервал, через который будет отправлено сообщение
		mcast_interval_time_msg = RandNum(0, active_parameters.mcast_interval);

		//
		if (general_mcast_count == 0)
			mcast_interval_time = mcast_interval_time_msg;

		//Перезаписываем сообщения в очереди по "кругу"
		if (mcast_msg_count == MCAST_QUEUE_BUF_COUNT)
			mcast_msg_count = 0;

		//Обновляем общий счетчик сообщений в очереди
		if (general_mcast_count == MCAST_QUEUE_BUF_COUNT)
			general_mcast_count = MCAST_QUEUE_BUF_COUNT;
		else
			general_mcast_count += 1;

		//Увеличиваем счетчик-указатель на позицию в очереди текущего сообщения
		mcast_msg_count += 1;

		//копируем сообщение в очередь мультикаст сообщений
		memcpy(&mcast_msg_queue[mcast_msg_count-1][0], msg_buf, msg_size);
		mcast_msg_queue[mcast_msg_count-1][MCAST_QUEUE_BUF_SIZE-2] = 200;
		mcast_msg_queue[mcast_msg_count-1][MCAST_QUEUE_BUF_SIZE-1] = msg_size;
		memcpy(&mcast_msg_queue[mcast_msg_count-1][MCAST_QUEUE_BUF_SIZE-4],(uint8_t*)&mcast_interval_time_msg, 2);

	}

	return;

}

/*Добавление сообщения из сесси фрагменитрованной передачи данных в очередь на отправку.*/

void FillFragSessionQueue(uint8_t* msg_buf, uint8_t msg_size, uint8_t port_num) {

	uint8_t  iter_msg 		 = 0;
	uint8_t  need_to_copy_msg = 1;
	uint16_t max_time 	     = 0;

	//Проверяем ли нет идентичного сообщения в очереди, если есть, то не добавляем сообщение в очередь
	for (iter_msg = 0; iter_msg<FRAG_SESSION_BUF_COUNT; iter_msg++) {

		if (memcmp((void*)msg_buf, (void*)&frag_session_msg_queue[iter_msg][0], msg_size) == 0)
		{
			need_to_copy_msg = 0;
			break;
		}

	}

	if (need_to_copy_msg) {

		//рассчитываем паузу до отправления сообщения
		max_time = pow(2, (frag_session_setup_req.control & 0x7) + 4);

		if (!frag_session_interval_time_msg)
			frag_session_interval_time_msg = RandNum(0, max_time);

		//переписываем сообщения по "кругу"
		if (frag_session_msg_ptr == FRAG_SESSION_BUF_COUNT)
			frag_session_msg_ptr = 0;

		//Подсчитываем обшее количество сооббщений в очереди
		if (frag_session_msg_count == FRAG_SESSION_BUF_COUNT)
			frag_session_msg_count = FRAG_SESSION_BUF_COUNT;
		else
			frag_session_msg_count += 1;

		//увеличиваем счетчик-указатель на текущую позицию, добавляемого сообщения в очередь
		frag_session_msg_ptr += 1;

		//копируем сообщение в очередь
		memcpy(&frag_session_msg_queue[frag_session_msg_ptr-1][0], msg_buf, msg_size);
		frag_session_msg_queue[frag_session_msg_ptr-1][FRAG_SESSION_BUF_SIZE-2] = port_num;
		frag_session_msg_queue[frag_session_msg_ptr-1][FRAG_SESSION_BUF_SIZE-1] = msg_size;

	}


	return;

}

//Резет модема пином
void ResetModemByPin() {

	HAL_GPIO_WritePin(LoRa_RESET_GPIO_Port, LoRa_RESET_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);

	HAL_GPIO_WritePin(LoRa_RESET_GPIO_Port, LoRa_RESET_Pin, GPIO_PIN_SET);
	HAL_Delay(100);

	flags_and_states.network_joined_flag = 0;
	flags_and_states.modem_config_flag = 0;

	join_in_process = 0;
	network_join_flag = 0;
	join_interval = 0;

	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

	return;

}
