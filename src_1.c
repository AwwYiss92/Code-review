#include <stdlib.h>
#include "stdio.h"
#include "main.h"
#include "string.h"
#include "i2c.h"
#include "math.h"
#include "frag_data_func.h"

uint8_t* parity_matrix_ptr    = NULL;
uint8_t* matrix_a_ptr         = NULL;
uint8_t* c                    = NULL;

//буферы используются для восстановления потерянных данных во время сесси фрагментированной передачи данных
uint8_t fragments_memory_ptr_first[32]  = {0}; //буфер для хранения одного фрагмента данных
uint8_t fragments_memory_ptr_second[32] = {0}; //буфер для хранения второго фрагмента данных

uint16_t row_bytes = 0;

extern uint8_t                 coded_fragment[60];
extern FragSessionSetupReq     frag_session_setup_req;
extern ShedSetReq              shed_set_req;
extern ProfileSetReq           profile_set_req;
extern FragSessionStatusAns    frag_session_status_ans;
extern IWDG_HandleTypeDef      hiwdg;


uint16_t bindex(uint16_t bit)  {return bit / 8;}
uint16_t boffset(uint16_t bit) {return bit % 8;}

//Установка бита в матрице четности А
void SetMatrixBit(uint8_t* matrix_ptr, uint8_t bit) {

	matrix_ptr[bindex(bit)] |= 1 << (boffset(bit));

}

//Очитка бита в матрице четности А
void ClearMatrixBit(uint8_t* matrix_ptr, uint8_t bit) {

	matrix_ptr[bindex(bit)] &= ~(1 << (boffset(bit)));

}

//Получения значения бита из матрицы четности А
uint8_t GetMatrixBit(uint8_t* matrix_ptr, uint8_t bit) {

	return ((matrix_ptr[bindex(bit)]) & ((1 << (boffset(bit)))));

}

//Выделение памяти для матрицы четности А, и очистка памяти во внешней EEPROM для хранения полученных фрагментов данных
uint8_t CreateCheckParityMatrix(uint16_t number_of_line, uint16_t number_of_row) {

	row_bytes			    = number_of_row/8;
	uint16_t clean_mem_size = 0;


	if ((number_of_row%8) != 0) {

		row_bytes += 1;
	}

	 clean_mem_size = number_of_line*row_bytes;

	 if (clean_mem_size > 65535)
		 return 1;
	 else {

		 if (!(CleanEEPROM(MATRIX_A_BASE_ADDR, clean_mem_size)))
			 return 1;
	 }

	c = (uint8_t*)malloc(row_bytes);

	if (c == NULL) {

		//free (c);

		return 1;
	}

	matrix_a_ptr = (uint8_t*)malloc(row_bytes);

	if (matrix_a_ptr == NULL)
		return 1;
	else {

		memset(matrix_a_ptr,0,row_bytes);

	}

	return 0;

}

//Освобождение памяти выделенной для матрицы четности
void DeleteCheckParityMatrix(uint16_t number_of_line) {

	free(matrix_a_ptr);

	memset((void*)fragments_memory_ptr_first, 0, 32);
	HAL_Delay(100);
	memset((void*)fragments_memory_ptr_second, 0, 32);
	HAL_Delay(100);

	free(c);

	return;

}

/*Декодирование принятого фрагмента данных fragmented_data_block_transport_v1.0.0 см пункты 1-4 на стр. 20*/
void DecodeFragment(uint16_t index) {

    uint16_t iter 		  = 0;
	uint16_t row 		  = 0;
	uint8_t  byte_xor 	  = 0;
	uint16_t  page_number = 0;
	uint16_t byte_amount  = 0;

	memset((uint8_t*)c, 0, row_bytes);
	HAL_Delay(100);

	if (index < frag_session_setup_req.nb_frag) {

		SetMatrixBit(c, index);

	} else {

		GenerateParMatrixLine(index);

	}


	for (row = 0; row<frag_session_setup_req.nb_frag; row++) {

		if (GetMatrixBit(c, row)){

			ReadFromEEPROM(matrix_a_ptr, MATRIX_A_BASE_ADDR + row*row_bytes, row_bytes);
			HAL_Delay(20);

			if (GetMatrixBit(matrix_a_ptr, row)) {

				for (iter = 0; iter < row_bytes; iter++) {

					byte_xor = c[iter] ^ matrix_a_ptr[iter];

					c[iter] = byte_xor;

				}

				ReadFromEEPROM(fragments_memory_ptr_first, START_FRAGMENTS_ADDR+row*frag_session_setup_req.frag_size, frag_session_setup_req.frag_size);
				HAL_Delay(20);

				for (iter = 0; iter < frag_session_setup_req.frag_size; iter++) {

					byte_xor = coded_fragment[iter] ^ fragments_memory_ptr_first[iter];

					coded_fragment[iter] = byte_xor;

				}
			}
		}
	}

	for (iter=0;iter<frag_session_setup_req.nb_frag;iter++) {

		if (GetMatrixBit(c, iter)) {

			ReadFromEEPROM(matrix_a_ptr, MATRIX_A_BASE_ADDR + iter*row_bytes, row_bytes);
			HAL_Delay(20);

			if (CheckMatrixLine(iter)) {

				memcpy(matrix_a_ptr, c, row_bytes);

				page_number = (iter*row_bytes)/256;

				byte_amount = ((page_number+1)*256) - iter*row_bytes;

				if (byte_amount >= row_bytes) {

					WriteToEEPROM(matrix_a_ptr,  MATRIX_A_BASE_ADDR + iter*row_bytes, row_bytes);
					HAL_Delay(100);

				} else if (byte_amount < row_bytes) {

					WriteToEEPROM(matrix_a_ptr,  MATRIX_A_BASE_ADDR + iter*row_bytes, byte_amount);
					HAL_Delay(100);
					WriteToEEPROM((uint8_t*)(matrix_a_ptr + byte_amount), (MATRIX_A_BASE_ADDR + iter*row_bytes + byte_amount), (row_bytes-byte_amount));
					HAL_Delay(100);

				}

				WriteToEEPROM(coded_fragment, START_FRAGMENTS_ADDR + (iter)*frag_session_setup_req.frag_size, frag_session_setup_req.frag_size);
				HAL_Delay(100);

			}

			break;

			}
	}


	return;
}

/*Функция пересборки исходных данных из принятых пакетов во время сессии фрагементированной передачи данных
 * Алгоритм пересборки описан в документе на стр. 20 (пункты 5-7)
 *
 */
void ReassembleDataBlock(uint8_t port_num) {

	uint8_t  byte_xor 			= 0;
	int16_t  line 				= 0;
	uint16_t row 				= 0;
	uint16_t byte_count 		= 0;
	uint16_t iter 				= 0;
	uint16_t interval_iter 		= 0;
	uint16_t day_iter 			= 0;
	uint16_t day_num 		   	= 0;
	uint32_t data_block_hash   	= 0;
	uint8_t	 correct_data_flag 	= 1;
	uint16_t days = (frag_session_setup_req.nb_frag*frag_session_setup_req.frag_size - frag_session_setup_req.padding)/6;

	//Выполняем переборку данных по алгоритму восстановления из документа (пункты 5-7 на стр.20)
	for(line=frag_session_setup_req.nb_frag - 2; line >= 0; line--) {

		HAL_IWDG_Refresh(&hiwdg);

		//Вычитваем строку матрицы
		ReadFromEEPROM(matrix_a_ptr, MATRIX_A_BASE_ADDR+line*row_bytes, row_bytes);
		HAL_Delay(20);

		//Вычитываем фрагмент данных, принятый от сервера
		ReadFromEEPROM(fragments_memory_ptr_first, START_FRAGMENTS_ADDR+line*frag_session_setup_req.frag_size, frag_session_setup_req.frag_size);
		HAL_Delay(20);

		//Пробегаем по строке матрицы, начиная с позиции line+1 до конца, и если бит в строке равен 1, то вычитываем второй фрагмент данных из памяти
		//с индексом row и выполняем операцию xor между байтами двух фрагментов
		for (row = line+1; row < frag_session_setup_req.nb_frag; row++) {

			if (GetMatrixBit(matrix_a_ptr, row)) {

				ReadFromEEPROM(fragments_memory_ptr_second, START_FRAGMENTS_ADDR+row*frag_session_setup_req.frag_size, frag_session_setup_req.frag_size);
				HAL_Delay(10);

				for (byte_count=0; byte_count<frag_session_setup_req.frag_size; byte_count++) {
					byte_xor = fragments_memory_ptr_first[byte_count] ^ fragments_memory_ptr_second[byte_count];
					fragments_memory_ptr_first[byte_count] = byte_xor;
				}

				HAL_IWDG_Refresh(&hiwdg);

			}

			//Когда доходим до конца строки матрицы А, то записываем получившийся фрагмент данных обратно в память на старую позицию
			if (row == (frag_session_setup_req.nb_frag-1)) {

				WriteToEEPROM(fragments_memory_ptr_first, START_FRAGMENTS_ADDR+line*frag_session_setup_req.frag_size, frag_session_setup_req.frag_size);
				HAL_Delay(100);

			}

		}

	}

	//Проверяем контрольную сумму получившихся данных после пересборки
	data_block_hash = CalcHash(START_FRAGMENTS_ADDR, (frag_session_setup_req.frag_size*frag_session_setup_req.nb_frag-frag_session_setup_req.padding));

	//Если получившаяся контрольная сумма совпала с заявленной сервером, то записываем данные по нужному адресу
	if (data_block_hash == frag_session_setup_req.descriptor) {

		//Порт 201 используется для перадачи профиля диммирования
		if (port_num == 201) {

			//Вычитываем фрагмент данных и проверяем корректность данных, если данные неверные, то данные игнорируем целиком
			for (iter=0;iter<frag_session_setup_req.nb_frag;iter++) {

				if (iter < (frag_session_setup_req.nb_frag - 1)) {

					ReadFromEEPROM(fragments_memory_ptr_first, START_FRAGMENTS_ADDR + iter*frag_session_setup_req.frag_size, frag_session_setup_req.frag_size);
					HAL_Delay(10);

					for(interval_iter = 0;interval_iter<frag_session_setup_req.frag_size/2;interval_iter++) {

						memcpy((uint8_t*)&profile_set_req,(uint8_t*)(fragments_memory_ptr_first + interval_iter*sizeof(profile_set_req)),sizeof(profile_set_req));

						if ((profile_set_req.int_num > 143) || (profile_set_req.int_num < 0) || (profile_set_req.dim_level > 100) || (profile_set_req.dim_level < 0)) {
							correct_data_flag = 0;
							break;
						}
					}

				} else {

					ReadFromEEPROM(fragments_memory_ptr_first, START_FRAGMENTS_ADDR + iter*frag_session_setup_req.frag_size, frag_session_setup_req.frag_size);
					HAL_Delay(10);

					for(interval_iter = 0;interval_iter<frag_session_setup_req.frag_size/2;interval_iter++) {

						memcpy((uint8_t*)&profile_set_req,(uint8_t*)(fragments_memory_ptr_first + interval_iter*sizeof(profile_set_req)),sizeof(profile_set_req));

						if ((profile_set_req.int_num > 143) || (profile_set_req.int_num < 0) || (profile_set_req.dim_level > 100) || (profile_set_req.dim_level < 0)) {
							correct_data_flag = 0;
							break;
						}
					}
				}
			}

			HAL_IWDG_Refresh(&hiwdg);

			//Если ошибок в данных не было обнаружено, то записываем данные профиля диммирования на место старого профиля
			if (correct_data_flag) {

				for (iter = 0;iter<frag_session_setup_req.nb_frag;iter++) {

					//Записываем в память целые кадры
					if (iter < (frag_session_setup_req.nb_frag - 1)) {

						ReadFromEEPROM(fragments_memory_ptr_first, START_FRAGMENTS_ADDR + iter*frag_session_setup_req.frag_size, frag_session_setup_req.frag_size);
						HAL_Delay(10);

						for(interval_iter = 0;interval_iter<frag_session_setup_req.frag_size/2;interval_iter++) {

							memcpy((uint8_t*)&profile_set_req,(uint8_t*)(fragments_memory_ptr_first + interval_iter*sizeof(profile_set_req)),sizeof(profile_set_req));

							WriteToEEPROM((uint8_t*)&profile_set_req, DIM_PROFILE_START_ADDRESS + profile_set_req.int_num*2, sizeof(profile_set_req));
							HAL_Delay(10);

						}

					//Тут записываем остаток
					} else {

						ReadFromEEPROM(fragments_memory_ptr_first, START_FRAGMENTS_ADDR + iter*frag_session_setup_req.frag_size, frag_session_setup_req.frag_size-frag_session_setup_req.padding);
						HAL_Delay(10);

						for(interval_iter = 0;interval_iter<((frag_session_setup_req.frag_size-frag_session_setup_req.padding)/2);interval_iter++) {

							memcpy((uint8_t*)&profile_set_req,(uint8_t*)(fragments_memory_ptr_first + interval_iter*sizeof(profile_set_req)),sizeof(profile_set_req));

							WriteToEEPROM((uint8_t*)&profile_set_req, DIM_PROFILE_START_ADDRESS + profile_set_req.int_num*2, sizeof(profile_set_req));
							HAL_Delay(10);

						}
					}
				}

				HAL_IWDG_Refresh(&hiwdg);
			}

		//Если порт 202, то это годовое расписание
		} else if (port_num == 202) {

			//Вычитваем фрагменты из памяти и проверяем корректность данных, если они не верные, то сбрасываем флаг, и данные игнорируются целиком
			for (day_iter = 0; day_iter<days;day_iter++) {

				ReadFromEEPROM((uint8_t*)&shed_set_req, START_FRAGMENTS_ADDR + day_iter*sizeof(shed_set_req), sizeof(shed_set_req));
				HAL_Delay(5);

				day_num = SwapUInt16(shed_set_req.day_num);

				if ((day_num >= 1) && (day_num <= 366)
					&& (shed_set_req.hour_on < 24) && (shed_set_req.hour_off < 24)
					&& (shed_set_req.minutes_on < 60) && (shed_set_req.minutes_off < 60)) {

					correct_data_flag = 1;

				} else {

					correct_data_flag = 0;
					break;

				}

			}

			HAL_IWDG_Refresh(&hiwdg);

			//Если в данных не было обнаружено неверных значений, то записываем фрагменты годового расписания на соответствующие позиции
			if (correct_data_flag) {

				for (day_iter = 0;day_iter<days;day_iter++) {

					ReadFromEEPROM((uint8_t*)&shed_set_req, START_FRAGMENTS_ADDR + day_iter*sizeof(shed_set_req), sizeof(shed_set_req));
					HAL_Delay(5);

					day_num = SwapUInt16(shed_set_req.day_num);

					for (iter = 0; iter < sizeof(shed_set_req); iter++) {
						WriteToEEPROM((((uint8_t*)(&shed_set_req)) + iter), (SCHEDULE_START_ADDRESS + (day_num-1)*6 + iter) , 1);
						HAL_Delay(10);
					}

					HAL_IWDG_Refresh(&hiwdg);

				}
			}
		}
	}

	return;

}


uint32_t PRBS23(uint32_t start) {

	uint32_t x 	 = start;
	uint32_t b_0 = x & 0x00000001;
	uint32_t b_1 = (x & 0x00000020)/32;

	float r = x/2;

	x = floorf(r) + (b_0 ^ b_1)*pow(2, 22);

	return x;
}

/*Функция генерации строки матрицы четности, алгоритм описан в документе fragmented_data_block_transport_v1.0.0 на стр. 26.*/
void GenerateParMatrixLine(uint32_t index) {

	uint32_t seed     = 1+1001*index;
	uint32_t nb_coeff = 0;
	uint32_t r        = 0;

	float number_of_row = floorf(((float)frag_session_setup_req.nb_frag)/2);

	while(nb_coeff<number_of_row) {

		r=pow(2, 16);

		while(r>=frag_session_setup_req.nb_frag) {

			seed =  PRBS23(seed);
			r = seed%(frag_session_setup_req.nb_frag);

		}

		SetMatrixBit((uint8_t*)c, r);
		nb_coeff += 1;

	}

	return;
}

/* Проверка битовой матрицы А четности */
uint8_t CheckMatrixA() {

	uint8_t  result_check_diag      = 0;
	uint8_t  result_check_left_half = 1;
	uint16_t matrix_line 			= 0;
	uint16_t bit_count   			= 0;
	uint16_t iter        			= 0;

	for (matrix_line=0; matrix_line<frag_session_setup_req.nb_frag; matrix_line++) {

		ReadFromEEPROM(matrix_a_ptr, MATRIX_A_BASE_ADDR + matrix_line*row_bytes, row_bytes);
		HAL_Delay(20);

		result_check_diag = 0;
		result_check_left_half = 1;

		if (matrix_line > 0) {

			if (GetMatrixBit(matrix_a_ptr, matrix_line)) {
				result_check_diag = 1;
			}

			for(iter=0;iter<matrix_line;iter++) {

				if(GetMatrixBit(matrix_a_ptr, iter)) {
					result_check_left_half = 0;
				}
			}

		} else {

			if (GetMatrixBit(matrix_a_ptr, matrix_line)) {
				result_check_diag = 1;
			}

		}

		if (result_check_diag && result_check_left_half)
			bit_count += 1;
	}

	frag_session_status_ans.missing_frag = frag_session_setup_req.nb_frag - bit_count;

	//Если количество верных строк равно количеству заявленных фрагментов данных, то значит матрица А верная, и мы приняли все заявленные фрагменты
	if (bit_count == frag_session_setup_req.nb_frag)
		return 1;
	else {
		return 0;
	}

}

/* Проверка строки битовой матрицы А
 * Если левая часть строки нулевая, а начиная с позиции, равной индексу строки, все биты единичные,
 * то не трогаем эту строку, она верная
 */
uint8_t CheckMatrixLine(uint16_t line) {

	uint8_t result_check_diag 	   = 0;
	uint8_t result_check_left_half = 1;
	uint8_t iter 				   = 0;

	if (GetMatrixBit(matrix_a_ptr, line)) {
		result_check_diag = 1;
	}

	for(iter=0;iter<line;iter++) {

		if(GetMatrixBit(matrix_a_ptr, iter)) {
			result_check_left_half = 0;
		}
	}

	if (result_check_left_half && result_check_diag)
		return 0;
	else
		return 1;


}

