#include <linux/clk.h>
#include "sd_port.h"
#include "sd_misc.h"
#include "sd_protocol.h"

//Global struct variable, to hold all card information need to operate card
/*static SD_MMC_Card_Info_t _sd_mmc_info = {CARD_TYPE_NONE,           //card_type
										  CARD_TYPE_NONE_SDIO,		//sdio_card_type
										 CARD_INDENTIFICATION_MODE, //operation_mode
										 SD_BUS_SINGLE,             //bus_width
										 SPEC_VERSION_10_101,       //spec_v1.0
										 SPEC_VERSION_10_12,        //spec_v1.0-v1.2
										 NORMAL_SPEED,              //normal_speed
										 {0},						//raw cid
										 0,                         //card_rca
										 0,                         //blk_len
										 0,                         //blk_nums
										 SD_MMC_TIME_NAC_DEFAULT,   //clks_nac
										 0,							//function_no
										 3000,						//clk_unit(default 3000ns)
										 0,                         //write_protected_flag
										 0,                         //inited_flag
										 0,                         //removed_flag
										 0,							//init_retry
										 0,							//single_blk_failed
										 0,                         //sdio_init_flag
										 NULL,						//sd_mmc_power
										 NULL,						//sd_mmc_get_ins
										 NULL,						//sd_get_wp
										 NULL						//sd_mmc_io_release
										 };*/
//SD_MMC_Card_Info_t *sd_mmc_info = &_sd_mmc_info;

extern unsigned sdio_timeout_int_times;

static char * sd_error_string[]={
	"SD_MMC_NO_ERROR",
	"SD_MMC_ERROR_OUT_OF_RANGE",        //Bit 31
	"SD_MMC_ERROR_ADDRESS",             //Bit 30 
	"SD_MMC_ERROR_BLOCK_LEN",           //Bit 29
	"SD_MMC_ERROR_ERASE_SEQ",           //Bit 28
	"SD_MMC_ERROR_ERASE_PARAM",         //Bit 27
	"SD_MMC_ERROR_WP_VIOLATION",        //Bit 26
	"SD_ERROR_CARD_IS_LOCKED",          //Bit 25
	"SD_ERROR_LOCK_UNLOCK_FAILED",      //Bit 24
	"SD_MMC_ERROR_COM_CRC",             //Bit 23
	"SD_MMC_ERROR_ILLEGAL_COMMAND",     //Bit 22
	"SD_ERROR_CARD_ECC_FAILED",         //Bit 21
	"SD_ERROR_CC",                      //Bit 20
	"SD_MMC_ERROR_GENERAL",             //Bit 19
	"SD_ERROR_Reserved1",               //Bit 18
	"SD_ERROR_Reserved2",               //Bit 17
	"SD_MMC_ERROR_CID_CSD_OVERWRITE",   //Bit 16
	"SD_ERROR_AKE_SEQ",                 //Bit 03
	"SD_MMC_ERROR_STATE_MISMATCH",
	"SD_MMC_ERROR_HEADER_MISMATCH",
	"SD_MMC_ERROR_DATA_CRC",
	"SD_MMC_ERROR_TIMEOUT", 
	"SD_MMC_ERROR_DRIVER_FAILURE",
	"SD_MMC_ERROR_WRITE_PROTECTED",
	"SD_MMC_ERROR_NO_MEMORY",
	"SD_ERROR_SWITCH_FUNCTION_COMUNICATION",
	"SD_ERROR_NO_FUNCTION_SWITCH",
	"SD_MMC_ERROR_NO_CARD_INS",
	"SD_MMC_ERROR_READ_DATA_FAILED",
	"SD_SDIO_ERROR_NO_FUNCTION"
};

static unsigned char char_mode[4][3] = {{0x10, 0x01, 0},
										{0x20, 0x02, 0},
										{0x40, 0x04, 0},
										{0x80, 0x08, 0}};
						
//All local function definitions, only used in this .C file
char * sd_error_to_string(int errcode);

void sd_delay_clocks_z(SD_MMC_Card_Info_t *sd_mmc_info, int num_clk);
void sd_delay_clocks_h(SD_MMC_Card_Info_t *sd_mmc_info, int num_clk);

void sd_clear_response(unsigned char * res_buf);
int sd_write_cmd_data(SD_MMC_Card_Info_t *sd_mmc_info, unsigned long byte_cnt, unsigned char * data_buf);
int sd_get_dat0_data(SD_MMC_Card_Info_t *sd_mmc_info, unsigned long byte_cnt, unsigned char * data_buf, unsigned short * crc16);
int sd_get_response_length(SD_Response_Type_t res_type);
int sd_read_response_data(SD_MMC_Card_Info_t *sd_mmc_info, unsigned long byte_cnt, unsigned char * res_buf);
#ifdef SD_MMC_SW_CONTROL
int sd_send_cmd_sw(SD_MMC_Card_Info_t *sd_mmc_info, unsigned char cmd, unsigned long arg, SD_Response_Type_t res_type, unsigned char * res_buf);
#endif
#ifdef SD_MMC_HW_CONTROL
int sd_send_cmd_hw(SD_MMC_Card_Info_t *sd_mmc_info, unsigned char cmd, unsigned long arg, SD_Response_Type_t res_type, unsigned char * res_buf, unsigned char *data_buf, unsigned long data_cnt, int retry_flag);
#endif

int sd_check_response_r1(unsigned char cmd, SD_Response_R1_t * r1);
int sd_check_response_r3(unsigned char cmd, SD_Response_R3_t * r3);
int sd_check_response_r4(unsigned char cmd, SDIO_Response_R4_t * r4);
int sd_check_response_r5(unsigned char cmd, SDIO_RW_CMD_Response_R5_t * r5);
int sd_check_response_r6(unsigned char cmd, SD_Response_R6_t * r6);
int sd_check_response_r7(unsigned char cmd, SD_Response_R7_t * r7);
int sd_check_response_r2_cid(unsigned char cmd, SD_Response_R2_CID_t * r2_cid);
int sd_check_response_r2_csd(unsigned char cmd, SD_Response_R2_CSD_t * r2_csd);
int sd_check_response(unsigned char cmd, SD_Response_Type_t res_type, unsigned char * res_buf);

int sd_hw_reset(SD_MMC_Card_Info_t *sd_mmc_info);
int sd_sw_reset(SD_MMC_Card_Info_t *sd_mmc_info);
int sd_voltage_validation(SD_MMC_Card_Info_t *sd_mmc_info);
int sd_identify_process(SD_MMC_Card_Info_t *sd_mmc_info);
int sd_mmc_switch_function(SD_MMC_Card_Info_t *sd_mmc_info);
int sd_check_sdio_card_type(SD_MMC_Card_Info_t *sd_mmc_info);
int sdio_data_transfer_abort(SD_MMC_Card_Info_t *sd_mmc_info, int function_no);
int sdio_card_reset(SD_MMC_Card_Info_t *sd_mmc_info);
void sd_mmc_set_input(SD_MMC_Card_Info_t *sd_mmc_info);

//Read single block data from SD card
int sd_read_single_block(SD_MMC_Card_Info_t *sd_mmc_info, unsigned long lba, unsigned char * data_buf);
//Read multi block data from SD card
int sd_read_multi_block(SD_MMC_Card_Info_t *sd_mmc_info, unsigned long lba, unsigned long lba_cnt, unsigned char * data_buf);
//Write single block data to SD card
int sd_write_single_block(SD_MMC_Card_Info_t *sd_mmc_info, unsigned long lba, unsigned char * data_buf);
//Write multi block data to SD card
int sd_write_multi_block(SD_MMC_Card_Info_t *sd_mmc_info, unsigned long lba, unsigned long lba_cnt, unsigned char * data_buf);

//Read Operation Conditions Register
int sd_read_reg_ocr(SD_MMC_Card_Info_t *sd_mmc_info, SD_REG_OCR_t * ocr);
//Read Card_Identification Register
int sd_read_reg_cid(SD_MMC_Card_Info_t *sd_mmc_info, SD_REG_CID_t * cid);
//Read Card-Specific Data Register
int sd_read_reg_csd(SD_MMC_Card_Info_t *sd_mmc_info, SD_REG_CSD_t * csd);
//Read Relative Card Address Register
int sd_read_reg_rca(SD_MMC_Card_Info_t *sd_mmc_info, unsigned short * rca);
//Read Driver Stage Register
int sd_read_reg_dsr(SD_MMC_Card_Info_t *sd_mmc_info, SD_REG_DSR_t * dsr);
//Read SD CARD Configuration Register
int sd_read_reg_scr(SD_MMC_Card_Info_t *sd_mmc_info, SD_REG_SCR_t * scr);

int sd_check_data_consistency(SD_MMC_Card_Info_t *sd_mmc_info);

void sd_mmc_prepare_power(SD_MMC_Card_Info_t *sd_mmc_info);
void sd_mmc_io_config(SD_MMC_Card_Info_t *sd_mmc_info);
int sd_mmc_staff_init(SD_MMC_Card_Info_t *sd_mmc_info);
int sd_mmc_cmd_test(SD_MMC_Card_Info_t *sd_mmc_info);

int sd_mmc_check_wp(SD_MMC_Card_Info_t *sd_mmc_info);

//Return the string buf address of specific errcode
char * sd_error_to_string(int errcode)
{
	return sd_error_string[errcode];
}

//Set clock delay, Z-bit is driven to (respectively kept) HIGH by the pull-up resistors RCMD respectively RDAT.
void sd_delay_clocks_z(SD_MMC_Card_Info_t *sd_mmc_info, int num_clk)
{
	int i;
	
	sd_set_cmd_input();
	
	if(sd_mmc_info->operation_mode == CARD_INDENTIFICATION_MODE)
	{
		for(i = 0; i < num_clk; i++)
		{
			sd_clk_identify_low();
			sd_clk_identify_high();
		}
	}
	else    // Tranfer mode
	{
		for(i = 0; i < num_clk; i++)
		{
			sd_clk_transfer_low();
			sd_clk_transfer_high();
		}
	}
}

//Set clock delay, P-bit is actively driven to HIGH by the card respectively host output driver.
void sd_delay_clocks_h(SD_MMC_Card_Info_t *sd_mmc_info, int num_clk)
{
	int i;
	
	sd_set_cmd_output();
	sd_set_cmd_value(1);
	
	if(sd_mmc_info->operation_mode == CARD_INDENTIFICATION_MODE)
	{
		for(i = 0; i < num_clk; i++)
		{
			sd_clk_identify_low();
			sd_clk_identify_high();
		}
	}
	else    // Tranfer mode
	{
		for(i = 0; i < num_clk; i++)
		{
			sd_clk_transfer_low();
			sd_clk_transfer_high();
		}
	}
}

//Clear response data buffer
void sd_clear_response(unsigned char * res_buf)
{
	int i;

	if(res_buf == NULL)
		return;
	
	for(i = 0; i < MAX_RESPONSE_BYTES; i++)
		res_buf[i]=0;
}

//Put data bytes to cmd line
int sd_write_cmd_data(SD_MMC_Card_Info_t *sd_mmc_info, unsigned long byte_cnt, unsigned char * data_buf)
{
	unsigned long data_cnt,data;
	int i;

	sd_set_cmd_output();

	if(sd_mmc_info->operation_mode == CARD_INDENTIFICATION_MODE)
	{
		for(data_cnt = 0; data_cnt < byte_cnt; data_cnt++)
		{
			for(i=7; i>=0; i--)
			{
				sd_clk_identify_low();
				
				data = (*data_buf >> i) & 0x01;
				sd_set_cmd_value(data);
				
				sd_clk_identify_high();
			}
			
			data_buf++;
		}
	}
	else    // Tranfer mode
	{
		for(data_cnt = 0; data_cnt < byte_cnt; data_cnt++)
		{
			for(i=7; i>=0; i--)
			{
				sd_clk_transfer_low();
				
				data = (*data_buf >> i) & 0x01;
				sd_set_cmd_value(data);
				
				sd_clk_transfer_high();
			}
			
			data_buf++;
		}
	}
	
	return SD_MMC_NO_ERROR; 
}

//Get data bytes from data0 line
int sd_get_dat0_data(SD_MMC_Card_Info_t *sd_mmc_info, unsigned long byte_cnt, unsigned char * data_buf, unsigned short * crc16)
{
	unsigned long data_cnt,data,temp, num_nac=0;
	int busy = 1, i;
	
	if(!byte_cnt)
		return SD_MMC_NO_ERROR;
		
	memset(data_buf, 0, byte_cnt);
	*crc16 = 0;
	
	//wait until data is valid
	sd_set_dat0_input();
	
	if(sd_mmc_info->operation_mode == CARD_INDENTIFICATION_MODE)
	{
		do
		{
			sd_clk_identify_low();
		
			data = sd_get_dat0_value();
			if(!data)
			{
				busy = 0;
			}
			
			sd_clk_identify_high();
			
			num_nac++;
		
		}while(busy && (num_nac < sd_mmc_info->clks_nac));

		if(num_nac >= sd_mmc_info->clks_nac)
			return SD_MMC_ERROR_TIMEOUT;

		//read data
		for(data_cnt = 0; data_cnt < byte_cnt; data_cnt++)
		{
			temp = 0;
			
			for(i=0; i<8; i++)
			{
				sd_clk_identify_low();
				
				data = sd_get_dat0_value();
				temp <<= 1;
				temp |= data;
				
				sd_clk_identify_high();
			}
			
			*data_buf = temp;
			data_buf++;
		}
	
		//Read CRC16 data
		for(data_cnt = 0; data_cnt < 16; data_cnt++)    // 16 bits CRC
		{
			sd_clk_identify_low();
		
			data = sd_get_dat0_value();
			*crc16 <<= 1;
			*crc16 |= data;
					
			sd_clk_identify_high();
		}
		
		//for end bit
		sd_clk_identify_low();
		sd_clk_identify_high();
	}
	else    // Tranfer mode
	{
		do
		{
			sd_clk_transfer_low();
		
			data = sd_get_dat0_value();
			if(!data)
			{
				busy = 0;
			}
			
			sd_clk_transfer_high();
			
			num_nac++;
		
		}while(busy && (num_nac < sd_mmc_info->clks_nac));

		if(num_nac >= sd_mmc_info->clks_nac)
			return SD_MMC_ERROR_TIMEOUT;

		//read data
		for(data_cnt = 0; data_cnt < byte_cnt; data_cnt++)
		{
			temp = 0;
			
			for(i=0; i<8; i++)
			{
				sd_clk_transfer_low();
				
				data = sd_get_dat0_value();
				temp <<= 1;
				temp |= data;
				
				sd_clk_transfer_high();
			}
			
			*data_buf = temp;
			data_buf++;
		}
	
		//Read CRC16 data
		for(data_cnt = 0; data_cnt < 16; data_cnt++)    // 16 bits CRC
		{
			sd_clk_transfer_low();
		
			data = sd_get_dat0_value();
			*crc16 <<= 1;
			*crc16 |= data;
					
			sd_clk_transfer_high();
		}
		
		//for end bit
		sd_clk_transfer_low();
		sd_clk_transfer_high();
	}

	return SD_MMC_NO_ERROR;
}

//Get response length according to response type
int sd_get_response_length(SD_Response_Type_t res_type)
{
	int num_res;
	
	switch(res_type)
	{
		case RESPONSE_R1:
		case RESPONSE_R1B:
		case RESPONSE_R3:
		case RESPONSE_R4:
		case RESPONSE_R5:
		case RESPONSE_R6:
		case RESPONSE_R7:
			num_res = RESPONSE_R1_R3_R4_R5_R6_R7_LENGTH;
			break;
		case RESPONSE_R2_CID:
		case RESPONSE_R2_CSD:
			num_res = RESPONSE_R2_CID_CSD_LENGTH;
			break;
		case RESPONSE_NONE:
			num_res = RESPONSE_NONE_LENGTH;
			break;
		default:
			num_res = RESPONSE_NONE_LENGTH;
			break;
	 }
	 
	 return num_res;
}

//Send command with response
#ifdef SD_MMC_SW_CONTROL
int sd_send_cmd_sw(SD_MMC_Card_Info_t *sd_mmc_info, unsigned char cmd, unsigned long arg, SD_Response_Type_t res_type, unsigned char * res_buf)
{
	int ret = SD_MMC_NO_ERROR, num_res;
	unsigned char cmd_buf[6];

	cmd_buf[0] = 0x40 | cmd;                //0x40: host command, command: 6 bits
	cmd_buf[1] = arg >> 24;                 //Command argument: 32 bits
	cmd_buf[2] = arg >> 16;
	cmd_buf[3] = arg >> 8;
	cmd_buf[4] = (unsigned char)arg;
	cmd_buf[5] = sd_cal_crc7(cmd_buf, 5) | 0x01;            //Calculate CRC checksum, 7 bits
	
	ret = sd_verify_crc7(cmd_buf, 6);
	if(ret)
		return SD_MMC_ERROR_COM_CRC;

	sd_write_cmd_data(sd_mmc_info, 6, cmd_buf);
	
	if(res_type == RESPONSE_NONE)
	{
		sd_delay_clocks_z(sd_mmc_info, SD_MMC_TIME_NRC_NCC);
		return SD_MMC_NO_ERROR;
	}

	//A delay before dealing with response
	sd_delay_clocks_z(sd_mmc_info,SD_MMC_Z_CMD_TO_RES);
	
	num_res = sd_get_response_length(res_type);
	
	sd_clear_response(res_buf);
	ret = sd_read_response_data(sd_mmc_info, num_res, res_buf); 
	if(ret)
		return ret;

	ret = sd_check_response(cmd, res_type, res_buf);

	sd_delay_clocks_z(sd_mmc_info, SD_MMC_TIME_NRC_NCC);
	
	return ret;
}
#endif


//Send command with response
#ifdef SD_MMC_HW_CONTROL
int sd_send_cmd_hw( SD_MMC_Card_Info_t  *   sd_mmc_info, 
                    unsigned char           cmd, 
                    unsigned long           arg, 
                    SD_Response_Type_t      res_type, 
                    unsigned char       *   res_buf, 
                    unsigned char       *   data_buf, 
                    unsigned long           data_cnt, 
                    int                     retry_flag )
{
	int ret = SD_MMC_NO_ERROR, num_res;
	unsigned char *buffer = NULL;
	unsigned int cmd_ext, cmd_send;

	MSHW_IRQ_Config_Reg_t *irq_config_reg;
	SDIO_Status_IRQ_Reg_t *status_irq_reg;
	SDHW_CMD_Send_Reg_t *cmd_send_reg;
	SDHW_Extension_Reg_t *cmd_ext_reg;
	unsigned int irq_config, status_irq, timeout;
	dma_addr_t data_dma_to_device_addr=0;
	dma_addr_t data_dma_from_device_addr=0;

	cmd_send = 0;
	cmd_send_reg = (void *)&cmd_send;
	if ((cmd == SD_SWITCH_FUNCTION) || (cmd == MMC_SEND_EXT_CSD))
		cmd_send_reg->cmd_data  = 0x40 | (cmd-40);          //for distinguish ACMD6 and CMD6,Maybe more good way but now I cant find
	else	
	    cmd_send_reg->cmd_data = 0x40 | cmd;
	cmd_send_reg->use_int_window = 1;

	cmd_ext = 0;
	cmd_ext_reg = (void *)&cmd_ext;
	//if(sd_mmc_info->bus_width == SD_BUS_WIDE)
	//	cmd_ext_reg->crc_status_4line = 1;

	sd_clear_response(res_buf);

	switch(res_type)
	{
		case RESPONSE_R1:
		case RESPONSE_R1B:
		case RESPONSE_R3:
		case RESPONSE_R4:
		case RESPONSE_R6:
		case RESPONSE_R5:
		case RESPONSE_R7:
			cmd_send_reg->cmd_res_bits = 45;		// RESPONSE have 7(cmd)+32(respnse)+7(crc)-1 data
			break;
		case RESPONSE_R2_CID:
		case RESPONSE_R2_CSD:
			cmd_send_reg->cmd_res_bits = 133;		// RESPONSE have 7(cmd)+120(respnse)+7(crc)-1 data
			cmd_send_reg->res_crc7_from_8 = 1;
			break;
		case RESPONSE_NONE:
			cmd_send_reg->cmd_res_bits = 0;			// NO_RESPONSE
			break;
		default:
			cmd_send_reg->cmd_res_bits = 0;			// NO_RESPONSE
			break;
	 }

	//cmd with adtc
	switch(cmd)
	{
		case SD_MMC_READ_SINGLE_BLOCK:
		case SD_MMC_READ_MULTIPLE_BLOCK:
			cmd_send_reg->res_with_data = 1;
			cmd_send_reg->repeat_package_times = data_cnt/sd_mmc_info->blk_len - 1;
			if(sd_mmc_info->bus_width == SD_BUS_WIDE)
				cmd_ext_reg->data_rw_number = sd_mmc_info->blk_len * 8 + (16 - 1) * 4;
			else
				cmd_ext_reg->data_rw_number = sd_mmc_info->blk_len * 8 + 16 - 1;

			buffer = sd_mmc_info->sd_mmc_phy_buf;
			break;

        case SD_SWITCH_FUNCTION:
        case MMC_SEND_EXT_CSD:
            //inv_dcache_range((unsigned long)sd_mmc_buf, ((unsigned long)sd_mmc_buf + data_cnt));
			cmd_send_reg->res_with_data = 1;
			cmd_send_reg->repeat_package_times = 0;
			if(sd_mmc_info->bus_width == SD_BUS_WIDE)
				cmd_ext_reg->data_rw_number = data_cnt * 8 + (16 - 1) * 4;
			else
				cmd_ext_reg->data_rw_number = data_cnt * 8 + 16 - 1;
			buffer = sd_mmc_info->sd_mmc_phy_buf;
			break;

		case SD_MMC_WRITE_BLOCK:
		case SD_MMC_WRITE_MULTIPLE_BLOCK:
			cmd_send_reg->cmd_send_data = 1;
			cmd_send_reg->repeat_package_times = data_cnt/sd_mmc_info->blk_len - 1;
			if(sd_mmc_info->bus_width == SD_BUS_WIDE)
				cmd_ext_reg->data_rw_number = sd_mmc_info->blk_len * 8 + (16 - 1) * 4;
			else
				cmd_ext_reg->data_rw_number = sd_mmc_info->blk_len * 8 + 16 - 1;
			
			buffer = sd_mmc_info->sd_mmc_phy_buf;
                        wmb();
			break;

		case IO_RW_EXTENDED:
			if(arg & (1<<27))
			{
				cmd_send_reg->repeat_package_times = data_cnt/sd_mmc_info->blk_len - 1;
				if(sd_mmc_info->bus_width == SD_BUS_WIDE)
					cmd_ext_reg->data_rw_number = sd_mmc_info->blk_len * 8 + (16 - 1) * 4;
				else
					cmd_ext_reg->data_rw_number = sd_mmc_info->blk_len * 8 + (16 - 1);
			}
			else
			{
				cmd_send_reg->repeat_package_times = 0;
				if(sd_mmc_info->bus_width == SD_BUS_WIDE)
					cmd_ext_reg->data_rw_number = data_cnt * 8 + (16 - 1) * 4;
				else
					cmd_ext_reg->data_rw_number = data_cnt * 8 + (16 - 1);
			}

			if(arg & (1<<31))
			{
				cmd_send_reg->cmd_send_data = 1;
				//memcpy(sd_mmc_info->sd_mmc_buf, data_buf, data_cnt);
				//buffer = sd_mmc_info->sd_mmc_phy_buf;
				data_dma_to_device_addr=dma_map_single(NULL, (void *)data_buf, data_cnt, DMA_TO_DEVICE);	
				buffer = (unsigned char*)data_dma_to_device_addr;
			}
			else
			{
				cmd_send_reg->res_with_data = 1;
				//buffer = sd_mmc_info->sd_mmc_phy_buf;
				data_dma_from_device_addr = dma_map_single(NULL, (void *)data_buf, data_cnt, DMA_FROM_DEVICE );
				buffer = (unsigned char*)data_dma_from_device_addr;
			}
			break;

		case SD_READ_DAT_UNTIL_STOP:
		case SD_SEND_NUM_WR_BLOCKS:
		
		case SD_MMC_PROGRAM_CSD:
		case SD_MMC_SEND_WRITE_PROT:
		case MMC_LOCK_UNLOCK:
		case SD_SEND_SCR:
		case SD_GEN_CMD:
			cmd_send_reg->res_with_data = 1;
			if(sd_mmc_info->bus_width == SD_BUS_WIDE)
				cmd_ext_reg->data_rw_number = data_cnt * 8 + (16 - 1) * 4;
			else
				cmd_ext_reg->data_rw_number = data_cnt * 8 + 16 - 1;
			buffer = sd_mmc_info->sd_mmc_phy_buf;
			break;
			
		default:
			break;
			
	}

	//cmd with R1b
	switch(cmd)
	{
		case SD_MMC_STOP_TRANSMISSION:
		case SD_MMC_SET_WRITE_PROT:
		case SD_MMC_CLR_WRITE_PROT:
		case SD_MMC_ERASE:
		case MMC_LOCK_UNLOCK:
			cmd_send_reg->check_dat0_busy = 1;
			break;
		default:
			break;
			
	}

	//cmd with R3
	switch(cmd)
	{
		case MMC_SEND_OP_COND:
		case SD_APP_OP_COND:
		case IO_SEND_OP_COND:
			cmd_send_reg->res_without_crc7 = 1;
			break;
		default:
			break;
			
	}

	#define SD_MMC_CMD_COUNT			20000//20
	#define SD_MMC_READ_BUSY_COUNT		2000000//20
	#define SD_MMC_WRITE_BUSY_COUNT		50000000//500000
	#define SD_MMC_WAIT_STOP_COUNT		50000000
	#define SD_MMC_RETRY_COUNT			2
    
    if(cmd_send_reg->cmd_send_data)
    {    
        if(cmd == SD_MMC_WRITE_MULTIPLE_BLOCK)
    	    timeout = SD_MMC_WRITE_BUSY_COUNT * (data_cnt/512);
    	else if(cmd == IO_RW_EXTENDED)
    		timeout = SD_MMC_WRITE_BUSY_COUNT * (cmd_send_reg->repeat_package_times + 1);
    	else
    	    timeout = SD_MMC_WRITE_BUSY_COUNT;
    }
    else
    {    
        if(cmd == SD_MMC_READ_MULTIPLE_BLOCK)
    	    timeout = SD_MMC_READ_BUSY_COUNT * (data_cnt/512);
    	else if(cmd == IO_RW_EXTENDED)
    		timeout = SD_MMC_READ_BUSY_COUNT * (cmd_send_reg->repeat_package_times + 1);
        else
    	    timeout = SD_MMC_CMD_COUNT;
    }
    
    if(cmd == SD_MMC_STOP_TRANSMISSION)
        timeout = SD_MMC_WAIT_STOP_COUNT;

	irq_config = READ_CBUS_REG(SDIO_IRQ_CONFIG);
	irq_config_reg = (void *)&irq_config;

	irq_config_reg->soft_reset = 1;
	WRITE_CBUS_REG(SDIO_IRQ_CONFIG, irq_config);

	status_irq = 0;
	status_irq_reg = (void *)&status_irq;
	status_irq_reg->if_int = 1;
	status_irq_reg->cmd_int = 1;
	status_irq_reg->timing_out_int = 1;
	if(timeout > (sd_mmc_info->sdio_clk_unit*0x1FFF)/1000)
	{
		status_irq_reg->timing_out_count = 0x1FFF;
		//sdio_timeout_int_times = (timeout*1000)/(sd_mmc_info->sdio_clk_unit*0x1FFF);
		sdio_timeout_int_times = timeout/(sd_mmc_info->sdio_clk_unit*0x1FFF/1000);
	}
	else
	{
		status_irq_reg->timing_out_count = (timeout/sd_mmc_info->sdio_clk_unit)*1000;
		sdio_timeout_int_times = 1;
	}
	WRITE_CBUS_REG(SDIO_STATUS_IRQ, status_irq);

	WRITE_CBUS_REG(CMD_ARGUMENT, arg);
	WRITE_CBUS_REG(SDIO_EXTENSION, cmd_ext);
	if(buffer != NULL)
	{
		WRITE_CBUS_REG(SDIO_M_ADDR, (unsigned long)buffer);
	}

	init_completion(&sdio_int_complete);
	sdio_open_host_interrupt(SDIO_CMD_INT);
	sdio_open_host_interrupt(SDIO_TIMEOUT_INT);

	WRITE_CBUS_REG(CMD_SEND, cmd_send);

	timeout =500;/*5s*/
	timeout = wait_for_completion_timeout(&sdio_int_complete,timeout);
	//wait_for_completion(&sdio_int_complete);

	if(sdio_timeout_int_times == 0 || timeout == 0){
		ret = SD_MMC_ERROR_TIMEOUT;
		if(timeout == 0)
			printk("[sd_send_cmd_hw] wait_for_completion_timeout\n");
		sdio_close_host_interrupt(SDIO_CMD_INT);
		sdio_close_host_interrupt(SDIO_TIMEOUT_INT);
		goto error;
	}

	status_irq = READ_CBUS_REG(SDIO_STATUS_IRQ);
	if(cmd_send_reg->cmd_res_bits && !cmd_send_reg->res_without_crc7 && !status_irq_reg->res_crc7_ok && !sd_mmc_info->sdio_read_crc_close){
		ret = SD_MMC_ERROR_COM_CRC;
		goto error;
	}

	num_res = sd_get_response_length(res_type);
	
	if(num_res > 0)
	{
		unsigned long multi_config = 0;
		SDIO_Multi_Config_Reg_t *multi_config_reg = (void *)&multi_config;
		multi_config_reg->write_read_out_index = 1;
		WRITE_CBUS_REG(SDIO_MULT_CONFIG, multi_config);

		num_res--;		// Minus CRC byte
	}
	while(num_res)
	{
		unsigned long data_temp = READ_CBUS_REG(CMD_ARGUMENT);
		
		res_buf[--num_res] = data_temp & 0xFF;
		if(num_res <= 0)
			break;
		res_buf[--num_res] = (data_temp >> 8) & 0xFF;
		if(num_res <= 0)
			break;
		res_buf[--num_res] = (data_temp >> 16) & 0xFF;
		if(num_res <= 0)
			break;
		res_buf[--num_res] = (data_temp >> 24) & 0xFF;
	}
	
	ret = sd_check_response(cmd, res_type, res_buf);
	if(ret)
		goto error;

	//cmd with adtc
	switch(cmd)
	{
		case SD_READ_DAT_UNTIL_STOP:
		case SD_MMC_READ_SINGLE_BLOCK:
		case SD_MMC_READ_MULTIPLE_BLOCK:
		case SD_SWITCH_FUNCTION:
                case MMC_SEND_EXT_CSD:
			if(!status_irq_reg->data_read_crc16_ok){
				ret = SD_MMC_ERROR_DATA_CRC;
				goto error;
			}
			break;
		case SD_MMC_WRITE_BLOCK:
		case SD_MMC_WRITE_MULTIPLE_BLOCK:
		case SD_MMC_PROGRAM_CSD:
			if(!status_irq_reg->data_write_crc16_ok){
				ret =  SD_MMC_ERROR_DATA_CRC;
				goto error;
			}
			break;
		case SD_SEND_NUM_WR_BLOCKS:
		case SD_MMC_SEND_WRITE_PROT:
		case MMC_LOCK_UNLOCK:
		case SD_SEND_SCR:
		case SD_GEN_CMD:
			if(!status_irq_reg->data_read_crc16_ok){
				ret = SD_MMC_ERROR_DATA_CRC;
				goto error;
			}
			break;
		case IO_RW_EXTENDED:
			if(arg & (1<<31))
			{
				if(!status_irq_reg->data_write_crc16_ok){
					ret =  SD_MMC_ERROR_DATA_CRC;
					goto error;
				}
			}
			else
			{
				if(!sd_mmc_info->sdio_read_crc_close)
				{
					if(!status_irq_reg->data_read_crc16_ok){
						ret = SD_MMC_ERROR_DATA_CRC;
						goto error;
					}
				}
			}
			break;
		default:
			break;
			
	}
	/*error need dma_unmap_single also*/
error:
	if(data_dma_from_device_addr)
	{
		dma_unmap_single(NULL, data_dma_from_device_addr, data_cnt, DMA_FROM_DEVICE);
	}
	if(data_dma_to_device_addr)
	{
		dma_unmap_single(NULL, data_dma_to_device_addr, data_cnt, DMA_TO_DEVICE);
	}
	if(cmd_send_reg->res_with_data && buffer && (data_buf != sd_mmc_info->sd_mmc_buf)
		&& (!data_dma_from_device_addr) && (!data_dma_to_device_addr))
	{
		memcpy(data_buf, sd_mmc_info->sd_mmc_buf, data_cnt);
	}

	return ret;
}
#endif

//Read SD Response Data
int sd_read_response_data(SD_MMC_Card_Info_t *sd_mmc_info, unsigned long byte_cnt, unsigned char * res_buf)
{
	unsigned long data_cnt, num_ncr = 0;
	unsigned char data, temp;
	int busy = 1, i;
	
	if(!byte_cnt)
		return SD_MMC_NO_ERROR;

	memset(res_buf, 0, byte_cnt);
	
	sd_set_cmd_input();
	
	if(sd_mmc_info->operation_mode == CARD_INDENTIFICATION_MODE)
	{
		//wait until cmd line is valid
		do
		{
			sd_clk_identify_low();
		
			data = sd_get_cmd_value();
			if(!data)
			{
				busy = 0;
				break;
			}
		
			sd_clk_identify_high();
			
			num_ncr++;
		
		}while(busy && (num_ncr < SD_MMC_TIME_NCR_MAX));

		if(num_ncr >= SD_MMC_TIME_NCR_MAX)
			return SD_MMC_ERROR_TIMEOUT;

		for(data_cnt = 0; data_cnt < byte_cnt; data_cnt++)
		{
			temp = 0;
			
			for(i=0; i<8; i++)
			{
				sd_clk_identify_low();
				
				data = sd_get_cmd_value();
				temp <<= 1;
				temp |= data;
							
				sd_clk_identify_high();
			}
			
			*res_buf = temp;
			res_buf++;
		}
	}
	else    // Tranfer mode
	{
		//wait until cmd line is valid
		do
		{
			sd_clk_transfer_low();
		
			data = sd_get_cmd_value();
			if(!data)
			{
				busy = 0;
				break;
			}
		
			sd_clk_transfer_high();
			
			num_ncr++;
		
		}while(busy && (num_ncr < SD_MMC_TIME_NCR_MAX));

		if(num_ncr >= SD_MMC_TIME_NCR_MAX)
			return SD_MMC_ERROR_TIMEOUT;

		for(data_cnt = 0; data_cnt < byte_cnt; data_cnt++)
		{
			temp = 0;
			
			for(i=0; i<8; i++)
			{
				sd_clk_transfer_low();
				
				data = sd_get_cmd_value();
				temp <<= 1;
				temp |= data;
							
				sd_clk_transfer_high();
			}
			
			*res_buf = temp;
			res_buf++;
		}
	}

	return SD_MMC_NO_ERROR;
}

//Check R1 response and return the result
int sd_check_response_r1(unsigned char cmd, SD_Response_R1_t * r1)
{
#ifdef SD_MMC_HW_CONTROL
	if(SD_WORK_MODE == CARD_HW_MODE)
	{
		return SD_MMC_NO_ERROR;
		//if(0)//if ((r1->command & 0x3F) != cmd)
	}
#endif		
#ifdef SD_MMC_SW_CONTROL
	if(SD_WORK_MODE == CARD_SW_MODE)
	{	
		if (r1->command != cmd)
			return SD_MMC_ERROR_HEADER_MISMATCH;		
		if (r1->card_status.OUT_OF_RANGE)
			return SD_MMC_ERROR_OUT_OF_RANGE;
		else if (r1->card_status.ADDRESS_ERROR)
			return SD_MMC_ERROR_ADDRESS;
		else if (r1->card_status.BLOCK_LEN_ERROR)
			return SD_MMC_ERROR_BLOCK_LEN;
		else if (r1->card_status.ERASE_SEQ_ERROR)
			return SD_MMC_ERROR_ERASE_SEQ;
		else if (r1->card_status.ERASE_PARAM)
			return SD_MMC_ERROR_ERASE_PARAM;
		else if (r1->card_status.WP_VIOLATION)
			return SD_MMC_ERROR_WP_VIOLATION;
		else if (r1->card_status.CARD_IS_LOCKED)
			return SD_ERROR_CARD_IS_LOCKED;
		else if (r1->card_status.LOCK_UNLOCK_FAILED)
			return SD_ERROR_LOCK_UNLOCK_FAILED;
		else if (r1->card_status.COM_CRC_ERROR)
			return SD_MMC_ERROR_COM_CRC;
		else if (r1->card_status.ILLEGAL_COMMAND)
			return SD_MMC_ERROR_ILLEGAL_COMMAND;
		else if (r1->card_status.CARD_ECC_FAILED)
			return SD_ERROR_CARD_ECC_FAILED;
		else if (r1->card_status.CC_ERROR)
			return SD_ERROR_CC;
		else if (r1->card_status.ERROR)
			return SD_MMC_ERROR_GENERAL;
		else if (r1->card_status.CID_CSD_OVERWRITE)
			return SD_MMC_ERROR_CID_CSD_OVERWRITE;
		else if (r1->card_status.AKE_SEQ_ERROR)
			return SD_ERROR_AKE_SEQ;	
	}
#endif

	return SD_MMC_NO_ERROR;	
}

//Check R3 response and return the result
int sd_check_response_r3(unsigned char cmd, SD_Response_R3_t * r3)
{
#ifdef SD_MMC_HW_CONTROL
	if(SD_WORK_MODE == CARD_HW_MODE)
	{
		if(0)//if ((r3->reserved1 & 0x3F) != 0x3F)
			return SD_MMC_ERROR_HEADER_MISMATCH;
	}
#endif		
#ifdef SD_MMC_SW_CONTROL
	if(SD_WORK_MODE == CARD_SW_MODE)
	{	
		if ((r3->reserved1 != 0x3F))// || (r3->reserved2 != 0x7F))
		return SD_MMC_ERROR_HEADER_MISMATCH;
	}
#endif	

	return SD_MMC_NO_ERROR;
}

int sd_check_response_r4(unsigned char cmd, SDIO_Response_R4_t * r4)
{
#ifdef SD_MMC_HW_CONTROL
	if(SD_WORK_MODE == CARD_HW_MODE)
	{
		if(0)//if ((r3->reserved1 & 0x3F) != 0x3F)
			return SD_MMC_ERROR_HEADER_MISMATCH;
	}
#endif		
#ifdef SD_MMC_SW_CONTROL
	if(SD_WORK_MODE == CARD_SW_MODE)
	{	
		if ((r4->reserved1 != 0x3F))// || (r3->reserved2 != 0x7F))
		return SD_MMC_ERROR_HEADER_MISMATCH;
	}
#endif	

	return SD_MMC_NO_ERROR;
}

int sd_check_response_r5(unsigned char cmd, SDIO_RW_CMD_Response_R5_t * r5)
{
#ifdef SD_MMC_HW_CONTROL
	if(SD_WORK_MODE == CARD_HW_MODE)
	{	
		if(0)//if ((r6->command & 0x3F) != SD_MMC_SEND_RELATIVE_ADDR)
			return SD_MMC_ERROR_HEADER_MISMATCH;
	}
#endif		
#ifdef SD_MMC_SW_CONTROL
	if(SD_WORK_MODE == CARD_SW_MODE)
	{
		if ((r5->command == IO_RW_DIRECT) || (r5->command == IO_RW_EXTENDED))
			return SD_MMC_NO_ERROR;
		
		return SD_MMC_ERROR_HEADER_MISMATCH;
	}
#endif

	return SD_MMC_NO_ERROR;
}

//Check R6 response and return the result
int sd_check_response_r6(unsigned char cmd, SD_Response_R6_t * r6)
{
#ifdef SD_MMC_HW_CONTROL
	if(SD_WORK_MODE == CARD_HW_MODE)
	{	
		if(0)//if ((r6->command & 0x3F) != SD_MMC_SEND_RELATIVE_ADDR)
			return SD_MMC_ERROR_HEADER_MISMATCH;
	}
#endif		
#ifdef SD_MMC_SW_CONTROL
	if(SD_WORK_MODE == CARD_SW_MODE)
	{
		if (r6->command != SD_MMC_SEND_RELATIVE_ADDR)
			return SD_MMC_ERROR_HEADER_MISMATCH;
	}
#endif

	return SD_MMC_NO_ERROR;
}

int sd_check_response_r7(unsigned char cmd, SD_Response_R7_t * r7)
{
#ifdef SD_MMC_HW_CONTROL
	if(SD_WORK_MODE == CARD_HW_MODE)
	{	
		if(0)//if ((r6->command & 0x3F) != SD_MMC_SEND_RELATIVE_ADDR)
			return SD_MMC_ERROR_HEADER_MISMATCH;
	}
#endif		
#ifdef SD_MMC_SW_CONTROL
	if(SD_WORK_MODE == CARD_SW_MODE)
	{
		if (r7->command != SD_SEND_IF_COND)
			return SD_MMC_ERROR_HEADER_MISMATCH;
	}
#endif

	return SD_MMC_NO_ERROR;	
}

//Check R2_CID response and return the result
int sd_check_response_r2_cid(unsigned char cmd, SD_Response_R2_CID_t * r2_cid)
{
#ifdef SD_MMC_HW_CONTROL
	if(SD_WORK_MODE == CARD_HW_MODE)
	{	
		if(0)//if ((r2_cid->reserved & 0x3F) != 0x3F)
			return SD_MMC_ERROR_HEADER_MISMATCH;
	}
#endif		
#ifdef SD_MMC_SW_CONTROL
	if(SD_WORK_MODE == CARD_SW_MODE)
	{
		if (r2_cid->reserved != 0x3F)
			return SD_MMC_ERROR_HEADER_MISMATCH;
	}
#endif	

	return SD_MMC_NO_ERROR;
}

//Check R2_CSD response and return the result
int sd_check_response_r2_csd(unsigned char cmd, SD_Response_R2_CSD_t * r2_csd)
{
#ifdef SD_MMC_HW_CONTROL
	if(SD_WORK_MODE == CARD_HW_MODE)
	{
		if(0)//if ((r2_csd->reserved & 0x3F)!= 0x3F)
			return SD_MMC_ERROR_HEADER_MISMATCH;
	}
#endif		
#ifdef SD_MMC_SW_CONTROL
	if(SD_WORK_MODE == CARD_SW_MODE)
	{
		if(r2_csd->reserved != 0x3F)
			return SD_MMC_ERROR_HEADER_MISMATCH;
	}
#endif	

	return SD_MMC_NO_ERROR;
}

//Check response and return the result
int sd_check_response(unsigned char cmd, SD_Response_Type_t res_type, unsigned char * res_buf)
{
	int ret = SD_MMC_NO_ERROR;
	
	switch(res_type)
	{
		case RESPONSE_R1:
		case RESPONSE_R1B:
			ret = sd_check_response_r1(cmd, (SD_Response_R1_t *)res_buf);
			break;
		
		case RESPONSE_R3:
			ret = sd_check_response_r3(cmd, (SD_Response_R3_t *)res_buf);
			break;
	   
		case RESPONSE_R4:
			ret = sd_check_response_r4(cmd, (SDIO_Response_R4_t *)res_buf);
			break;
		case RESPONSE_R5:
			ret = sd_check_response_r5(cmd, (SDIO_RW_CMD_Response_R5_t *)res_buf);
			break;
		case RESPONSE_R6:
			ret = sd_check_response_r6(cmd, (SD_Response_R6_t *)res_buf);
			break;
		
		case RESPONSE_R2_CID:
			ret = sd_check_response_r2_cid(cmd, (SD_Response_R2_CID_t *)res_buf);
			break;
		 
		case RESPONSE_R2_CSD:
			ret = sd_check_response_r2_csd(cmd, (SD_Response_R2_CSD_t *)res_buf);
			break;
		
		case RESPONSE_NONE:
			break;
			
		default:
			break;
	}
	
	return ret;
}

//Read single block data from SD card
#ifdef SD_MMC_HW_CONTROL
int sd_read_single_block_hw(SD_MMC_Card_Info_t *sd_mmc_info, unsigned long lba, unsigned char * data_buf)
{
	int ret, read_retry_count, read_single_block_hw_failed = 0;
	unsigned long data_addr;
	unsigned char response[MAX_RESPONSE_BYTES];

	if ((sd_mmc_info->card_type == CARD_TYPE_SDHC) || (sd_mmc_info->card_type == CARD_TYPE_EMMC))
	{
		data_addr = lba;
	}
	else
	{
	    data_addr = sd_mmc_info->blk_len;
	    data_addr *= lba;
	}

    for(read_retry_count=0; read_retry_count<3; read_retry_count++)
    {
	    ret = sd_send_cmd_hw(sd_mmc_info, SD_MMC_READ_SINGLE_BLOCK, data_addr, RESPONSE_R1, response, data_buf, sd_mmc_info->blk_len, 1);
	    if(ret)
	    {
	    	printk(" sd_read_single_block_hw ret %d \n", ret);
	        read_single_block_hw_failed++;
	        continue;
	    }
	    else
		    break;
    }

    if(read_single_block_hw_failed >= 3)
        return SD_MMC_ERROR_READ_DATA_FAILED;

    if(ret)
        return ret;

	return SD_MMC_NO_ERROR;
}
#endif

//Read single block data from SD card
#ifdef SD_MMC_SW_CONTROL
int sd_read_single_block_sw(SD_MMC_Card_Info_t *sd_mmc_info, unsigned long lba, unsigned char * data_buf)
{
	unsigned long data = 0, res = 0, temp = 0;
	int ret, data_busy = 1, res_busy = 1;
	
	unsigned long res_cnt = 0, data_cnt = 0, num_nac = 0, num_ncr = 0;
	
	unsigned short crc16_array[4] = {0, 0, 0, 0};
	unsigned short crc16 = 0;
	
	unsigned char response[MAX_RESPONSE_BYTES];
	
	unsigned long data_addr, loop_num;
	
	int i,j;

#ifdef SD_MMC_CRC_CHECK
	unsigned short crc_check = 0, crc_check_array[4]={0,0,0,0};
	int error=0;
	//unsigned char *org_buf=data_buf;
#endif
	
	if(sd_mmc_info->bus_width == SD_BUS_WIDE)
	{
		sd_set_dat0_3_input();
	}
	else
	{
		sd_set_dat0_input();
	}

	if ((sd_mmc_info->card_type == CARD_TYPE_SDHC) || (sd_mmc_info->card_type == CARD_TYPE_EMMC))
	{
		data_addr = lba;
	}
	else
	{
	data_addr = sd_mmc_info->blk_len;
	data_addr *= lba;
	}
	
	ret = sd_send_cmd_sw(sd_mmc_info, SD_MMC_READ_SINGLE_BLOCK, data_addr, RESPONSE_NONE, 0);
	if(ret)
		return ret;
	
	sd_clear_response(response);
	sd_delay_clocks_z(sd_mmc_info, SD_MMC_Z_CMD_TO_RES);
	
	sd_set_cmd_input();
	//wait until both response and data is valid    
	do
	{
		sd_clk_transfer_low();
		
		res = sd_get_cmd_value();
		data = sd_get_dat0_value();
		
		if (res_busy)
		{
			if (res)
				num_ncr++;
			else
				res_busy = 0;
		}
		else
		{
			if (res_cnt < (RESPONSE_R1_R3_R4_R5_R6_R7_LENGTH*8))
			{
				response[res_cnt>>3] <<= 1;
				response[res_cnt>>3] |= res;
				
				res_cnt++;
			}
		}
		
		if (data_busy)
		{
			if (data)
				num_nac++;
			else
				data_busy = 0;
		}
		else
		{
			if(sd_mmc_info->bus_width == SD_BUS_WIDE)          //4 data lines
			{
				data = sd_get_dat0_3_value();
				temp <<= 4;
				temp |= data;
#ifdef SD_MMC_CRC_CHECK
				SD_CAL_BIT_CRC(crc_check_array[0],data&0x01);
				SD_CAL_BIT_CRC(crc_check_array[1],data&0x02);
				SD_CAL_BIT_CRC(crc_check_array[2],data&0x04);
				SD_CAL_BIT_CRC(crc_check_array[3],data&0x08);
#endif
				if((data_cnt & 0x01) == 1)
				{
#ifdef AMLOGIC_CHIP_SUPPORT
					if((unsigned long)data_buf == 0x3400000)
					{
						WRITE_BYTE_TO_FIFO(temp);
					}
					else
#endif
					{
						*data_buf = temp;
						data_buf++;
					}

					temp = 0;   //one byte received, clear temp varialbe
				}                   
			}
			else                //only data0 lines
			{
				data = sd_get_dat0_value();
				temp <<= 1;
				temp |= data;
				if((data_cnt & 0x07) == 7)
				{
#ifdef AMLOGIC_CHIP_SUPPORT
					if((unsigned)data_buf == 0x3400000)
					{
						WRITE_BYTE_TO_FIFO(temp);
					}
					else
#endif
					{
						*data_buf = temp;
						data_buf++;
					}

#ifdef SD_MMC_CRC_CHECK
					crc_check = (crc_check << 8) ^ sd_crc_table[((crc_check >> 8) ^ temp) & 0xff];
#endif
					temp = 0;   //one byte received, clear temp varialbe
				}
			}
			data_cnt++;
		}
		
		sd_clk_transfer_high();
		
		if(!res_busy && !data_busy)
		{
			if(sd_mmc_info->bus_width == SD_BUS_WIDE)          //4 data lines
			{
				if((res_cnt >= (RESPONSE_R1_R3_R4_R5_R6_R7_LENGTH*8)) && ((data_cnt&0x01) == 0))
				{
					data_cnt >>= 1;
					break;
				}
			}
			else
			{
				if((res_cnt >= (RESPONSE_R1_R3_R4_R5_R6_R7_LENGTH*8)) && ((data_cnt&0x07) == 0))
				{
					data_cnt >>= 3;
					break;
				}
			}
		}

	}while((num_ncr < SD_MMC_TIME_NCR_MAX) && (num_nac < sd_mmc_info->clks_nac));
	
	if((num_ncr >= SD_MMC_TIME_NCR_MAX) || (num_nac >= sd_mmc_info->clks_nac))
		return SD_MMC_ERROR_TIMEOUT;

	//Read data and response
	loop_num = sd_mmc_info->blk_len;
	if(sd_mmc_info->bus_width == SD_BUS_WIDE)          //4 data lines
	{
#ifdef AMLOGIC_CHIP_SUPPORT
		if((unsigned long)data_buf == 0x3400000)
		{
			for(; data_cnt < loop_num; data_cnt++)
			{
				temp = 0;   //clear temp varialbe
			
				for(i = 0; i < 2; i++)
				{
					sd_clk_transfer_low();
		
					data = sd_get_dat0_3_value();
					temp <<= 4;
					temp |= data;
				
					sd_clk_transfer_high();
					
#ifdef SD_MMC_CRC_CHECK
					SD_CAL_BIT_CRC(crc_check_array[0],data&0x01);
					SD_CAL_BIT_CRC(crc_check_array[1],data&0x02);
					SD_CAL_BIT_CRC(crc_check_array[2],data&0x04);
					SD_CAL_BIT_CRC(crc_check_array[3],data&0x08);
#endif
				}
				
				WRITE_BYTE_TO_FIFO(temp);
			}
		}
		else
#endif
		{
			for(; data_cnt < loop_num; data_cnt++)
			{
				temp = 0;   //clear temp varialbe
			
				for(i = 0; i < 2; i++)
				{
					sd_clk_transfer_low();
		
					data = sd_get_dat0_3_value();
					temp <<= 4;
					temp |= data;
				
					sd_clk_transfer_high();
					
#ifdef SD_MMC_CRC_CHECK
					SD_CAL_BIT_CRC(crc_check_array[0],data&0x01);
					SD_CAL_BIT_CRC(crc_check_array[1],data&0x02);
					SD_CAL_BIT_CRC(crc_check_array[2],data&0x04);
					SD_CAL_BIT_CRC(crc_check_array[3],data&0x08);
#endif
				}
				
				*data_buf = temp;
				data_buf++;
			}
		}
		
		//Read CRC16 data
		for(data_cnt = 0; data_cnt < 16; data_cnt++)    // 16 bits CRC
		{
			sd_clk_transfer_low();
		
			crc16_array[0] <<= 1;
			crc16_array[1] <<= 1;
			crc16_array[2] <<= 1;
			crc16_array[3] <<= 1;
			data = sd_get_dat0_3_value();
			crc16_array[0] |= (data & 0x01);
			crc16_array[1] |= ((data >> 1) & 0x01);
			crc16_array[2] |= ((data >> 2) & 0x01);
			crc16_array[3] |= ((data >> 3) & 0x01);

			sd_clk_transfer_high();
		}
		
#ifdef SD_MMC_CRC_CHECK
		for(i=0; i<4; i++)
		{
			//crc_check_array[i] = sd_cal_crc_mode(org_buf, sd_mmc_info->blk_len, char_mode[i]);
			if(crc16_array[i] != crc_check_array[i])
			{
				error = SD_MMC_ERROR_DATA_CRC;
				break;
			}
		}
#endif
	}
	else    //only data0 lines
	{
#ifdef AMLOGIC_CHIP_SUPPORT
		if((unsigned)data_buf == 0x3400000)
		{
			for(; data_cnt < loop_num; data_cnt++)
			{
				temp = 0;   //clear temp varialbe
			
				for(j = 0; j < 8; j++)
				{
					sd_clk_transfer_low();
				
					data = sd_get_dat0_value();
					temp <<= 1;
					temp |= data;
				
					sd_clk_transfer_high();
				}
				
				WRITE_BYTE_TO_FIFO(temp);
				
#ifdef SD_MMC_CRC_CHECK
				crc_check = (crc_check << 8) ^ sd_crc_table[((crc_check >> 8) ^ temp) & 0xff];
#endif
			}
		}
		else
#endif
		{
			for(; data_cnt < loop_num; data_cnt++)
			{
				temp = 0;   //clear temp varialbe
			
				for(j = 0; j < 8; j++)
				{
					sd_clk_transfer_low();
				
					data = sd_get_dat0_value();
					temp <<= 1;
					temp |= data;
				
					sd_clk_transfer_high();
				}
				
				*data_buf = temp;
				data_buf++;
				
#ifdef SD_MMC_CRC_CHECK
				crc_check = (crc_check << 8) ^ sd_crc_table[((crc_check >> 8) ^ temp) & 0xff];
#endif
			}
		}

		//Read CRC16 data
		for(data_cnt = 0; data_cnt < 16; data_cnt++)    // 16 bits CRC
		{
			sd_clk_transfer_low();
		
			data = sd_get_dat0_value();
			crc16 <<= 1;
			crc16 |= data;

			sd_clk_transfer_high();
		}
		
#ifdef SD_MMC_CRC_CHECK
		if(crc16 != crc_check)
			error = SD_MMC_ERROR_DATA_CRC;
#endif
	}

	sd_clk_transfer_low();      //for end bit
	sd_clk_transfer_high();
	
	sd_delay_clocks_z(sd_mmc_info, SD_MMC_TIME_NRC_NCC);     //Clock delay, Z type

#ifdef SD_MMC_CRC_CHECK
	if(error == SD_MMC_ERROR_DATA_CRC)
	{
		//#ifdef  SD_MMC_DEBUG
		//Debug_Printf("#%s error occured in sd_read_single_block()!\n", sd_error_to_string(error));
		//#endif
		return error;
	}
#endif

	return SD_MMC_NO_ERROR;
}
#endif

#ifdef SD_MMC_HW_CONTROL
int sd_read_multi_block_hw(SD_MMC_Card_Info_t *sd_mmc_info, unsigned long lba, unsigned long lba_cnt, unsigned char * data_buf)
{
	int ret, read_retry_count, i, read_multi_block_hw_failed = 0;
	unsigned long data_addr, lba_num, data_offset = 0;
	unsigned char response[MAX_RESPONSE_BYTES];
	
	if(lba_cnt == 0)
		return SD_MMC_ERROR_BLOCK_LEN;

	if ((sd_mmc_info->card_type == CARD_TYPE_SDHC) || (sd_mmc_info->card_type == CARD_TYPE_EMMC))
	{
		data_addr = lba;
	}
	else
	{
		data_addr = sd_mmc_info->blk_len;
		data_addr *= lba;
	}
	
    if(sd_mmc_info->read_multi_block_failed == 0)
	{	
        while(lba_cnt)
        {
            if(lba_cnt > sd_mmc_info->max_blk_count)
                lba_num = sd_mmc_info->max_blk_count;
            else
                lba_num = lba_cnt;

            if((sd_mmc_info->card_type == CARD_TYPE_SDHC) || (sd_mmc_info->card_type == CARD_TYPE_EMMC))
                data_addr += data_offset/512;
            else
                data_addr += data_offset;

            data_buf += data_offset;
            for(read_retry_count=0; read_retry_count<3; read_retry_count++)
            {
	            ret = sd_send_cmd_hw(sd_mmc_info, SD_MMC_READ_MULTIPLE_BLOCK, data_addr, RESPONSE_R1, response, data_buf, sd_mmc_info->blk_len*lba_num, 1);
	            if(ret)
	            {
	                read_multi_block_hw_failed++;
	                ret = sd_send_cmd_hw(sd_mmc_info, SD_MMC_STOP_TRANSMISSION, 0, RESPONSE_R1B, response, NULL, 0, 0);
	                if(ret)
	                    return ret;
                }
                else
                    break;
            }

            if(read_multi_block_hw_failed >= 3)
            {
                sd_mmc_info->read_multi_block_failed = 1;
                return SD_MMC_ERROR_READ_DATA_FAILED;
            }	

	        if(ret)
	        { 
                return ret;
            }
            else
            {        
	            ret = sd_send_cmd_hw(sd_mmc_info, SD_MMC_STOP_TRANSMISSION, 0, RESPONSE_R1B, response, NULL, 0, 0);
	            if(ret)
	                return ret;
	        }

            lba_cnt -= lba_num;
            data_offset = lba_num*512;
        }
    }
    else
    {
	    for(i=0; i<lba_cnt; i++)
	    {
		    ret = sd_read_single_block_hw(sd_mmc_info, lba++, data_buf);
		    if(ret)
			    return ret;
		
#ifdef AMLOGIC_CHIP_SUPPORT
		    data_buf += (((unsigned long)data_buf == 0x3400000) ? 0 : sd_mmc_info->blk_len);
#else
		    data_buf += 512;
#endif
	    }
    }
	return SD_MMC_NO_ERROR;
}
#endif

#ifdef SD_MMC_SW_CONTROL
int sd_read_multi_block_sw(SD_MMC_Card_Info_t *sd_mmc_info, unsigned long lba, unsigned long lba_cnt, unsigned char * data_buf)
{
	unsigned long data = 0, res = 0, temp = 0;
	int ret, data_busy = 1, res_busy = 1;
	
	unsigned long res_cnt = 0, data_cnt = 0, num_nac = 0, num_ncr = 0;
	
	unsigned short crc16_array[4] = {0, 0, 0, 0};
	unsigned short crc16 = 0;
	
	unsigned char response[MAX_RESPONSE_BYTES];
	
	unsigned long data_addr,loop_num,blk_cnt;
	
	int i,j;

#ifdef SD_MMC_CRC_CHECK
	unsigned short crc_check = 0, crc_check_array[4]={0,0,0,0};
	int error=0;
	unsigned char *org_buf=data_buf;
#endif
	
	if(lba_cnt == 0)
		return SD_MMC_ERROR_BLOCK_LEN;
		
	if(sd_mmc_info->bus_width == SD_BUS_WIDE)
	{
		sd_set_dat0_3_input();
	}
	else
	{
		sd_set_dat0_input();
	}

	if ((sd_mmc_info->card_type == CARD_TYPE_SDHC) || (sd_mmc_info->card_type == CARD_TYPE_EMMC))
	{
		data_addr = lba;
	}
	else
	{
        data_addr = sd_mmc_info->blk_len;
        data_addr *= lba;
	}
	ret = sd_send_cmd_sw(sd_mmc_info, SD_MMC_READ_MULTIPLE_BLOCK, data_addr, RESPONSE_NONE, 0);
	if(ret)
		return ret;
	
	sd_clear_response(response);
	sd_delay_clocks_z(sd_mmc_info, SD_MMC_Z_CMD_TO_RES);
	
	sd_set_cmd_input();
	//wait until both response and data is valid    
	do
	{
		sd_clk_transfer_low();
		
		res = sd_get_cmd_value();
		data = sd_get_dat0_value();
		
		if (res_busy)
		{
			if (res)
				num_ncr++;
			else
				res_busy = 0;
		}
		else
		{
			if (res_cnt < (RESPONSE_R1_R3_R4_R5_R6_R7_LENGTH*8))
			{
				response[res_cnt>>3] <<= 1;
				response[res_cnt>>3] |= res;
				
				res_cnt++;
			}
		}
		
		if (data_busy)
		{
			if (data)
				num_nac++;
			else
				data_busy = 0;
		}
		else
		{
			if(sd_mmc_info->bus_width == SD_BUS_WIDE)          //4 data lines
			{
				data = sd_get_dat0_3_value();
				temp <<= 4;
				temp |= data;
#ifdef SD_MMC_CRC_CHECK
				SD_CAL_BIT_CRC(crc_check_array[0],data&0x01);
				SD_CAL_BIT_CRC(crc_check_array[1],data&0x02);
				SD_CAL_BIT_CRC(crc_check_array[2],data&0x04);
				SD_CAL_BIT_CRC(crc_check_array[3],data&0x08);
#endif
				if((data_cnt & 0x01) == 1)
				{
#ifdef AMLOGIC_CHIP_SUPPORT
					if((unsigned long)data_buf == 0x3400000)
					{
						WRITE_BYTE_TO_FIFO(temp);
					}
					else
#endif
					{
						*data_buf = temp;
						data_buf++;
					}

					temp = 0;   //one byte received, clear temp varialbe
				}                   
			}
			else                //only data0 lines
			{
				data = sd_get_dat0_value();
				temp <<= 1;
				temp |= data;
				if((data_cnt & 0x07) == 7)
				{
#ifdef AMLOGIC_CHIP_SUPPORT
					if((unsigned)data_buf == 0x3400000)
					{
						WRITE_BYTE_TO_FIFO(temp);
					}
					else
#endif
					{
						*data_buf = temp;
						data_buf++;
					}

#ifdef SD_MMC_CRC_CHECK
					crc_check = (crc_check << 8) ^ sd_crc_table[((crc_check >> 8) ^ temp) & 0xff];
#endif
					temp = 0;   //one byte received, clear temp varialbe
				}
			}
			data_cnt++;
		}
			
		sd_clk_transfer_high();
		
		if(!res_busy && !data_busy)
		{
			if(sd_mmc_info->bus_width == SD_BUS_WIDE)          //4 data lines
			{
				if((res_cnt >= (RESPONSE_R1_R3_R4_R5_R6_R7_LENGTH*8)) && ((data_cnt&0x01) == 0))
				{
					data_cnt >>= 1;
					break;
				}
			}
			else
			{
				if((res_cnt >= (RESPONSE_R1_R3_R4_R5_R6_R7_LENGTH*8)) && ((data_cnt&0x07) == 0))
				{
					data_cnt >>= 3;
					break;
				}
			}
		}

	}while((num_ncr < SD_MMC_TIME_NCR_MAX) && (num_nac < sd_mmc_info->clks_nac));
	
	if((num_ncr >= SD_MMC_TIME_NCR_MAX) || (num_nac >= sd_mmc_info->clks_nac))
		return SD_MMC_ERROR_TIMEOUT;

	//Read all data blocks
	loop_num = sd_mmc_info->blk_len;
	for (blk_cnt = 0; blk_cnt < lba_cnt; blk_cnt++)
	{
		//wait until data is valid
		num_nac = 0;    
		do
		{   
			if(!data_busy)
				break;
				
			sd_clk_transfer_low();
		
			data = sd_get_dat0_value();
		
			if(data)
			{
				num_nac++;
			}
			else
			{
				data_busy = 0;
			}
		
			sd_clk_transfer_high();

		}while(data_busy && (num_nac < sd_mmc_info->clks_nac));
		
		if(num_nac >= sd_mmc_info->clks_nac)
			return SD_MMC_ERROR_TIMEOUT;
		
		//Read data
		if(sd_mmc_info->bus_width == SD_BUS_WIDE)          //4 data lines
		{
#ifdef AMLOGIC_CHIP_SUPPORT
			if((unsigned long)data_buf == 0x3400000)
			{
				for(; data_cnt < loop_num; data_cnt++)
				{
					temp = 0;   //clear temp varialbe
				
					for(i = 0; i < 2; i++)
					{
						sd_clk_transfer_low();
		
						data = sd_get_dat0_3_value();
						temp <<= 4;
						temp |= data;
					
						sd_clk_transfer_high();
						
#ifdef SD_MMC_CRC_CHECK
						SD_CAL_BIT_CRC(crc_check_array[0],data&0x01);
						SD_CAL_BIT_CRC(crc_check_array[1],data&0x02);
						SD_CAL_BIT_CRC(crc_check_array[2],data&0x04);
						SD_CAL_BIT_CRC(crc_check_array[3],data&0x08);
#endif
					}
					
					WRITE_BYTE_TO_FIFO(temp);
				}
			}
			else
#endif
			{
				for(; data_cnt < loop_num; data_cnt++)
				{
					temp = 0;   //clear temp varialbe
				
					for(i = 0; i < 2; i++)
					{
						sd_clk_transfer_low();
						
						data = sd_get_dat0_3_value();
						temp <<= 4;
						temp |= data;
					
						sd_clk_transfer_high();
						
#ifdef SD_MMC_CRC_CHECK
						SD_CAL_BIT_CRC(crc_check_array[0],data&0x01);
						SD_CAL_BIT_CRC(crc_check_array[1],data&0x02);
						SD_CAL_BIT_CRC(crc_check_array[2],data&0x04);
						SD_CAL_BIT_CRC(crc_check_array[3],data&0x08);
#endif
					}
					
					*data_buf = temp;
					data_buf++;
				}
			}
			
			//Read CRC16 data
			for(data_cnt = 0; data_cnt < 16; data_cnt++)    // 16 bits CRC
			{
				sd_clk_transfer_low();
		
				crc16_array[0] <<= 1;
				crc16_array[1] <<= 1;
				crc16_array[2] <<= 1;
				crc16_array[3] <<= 1;
				data = sd_get_dat0_3_value();
				crc16_array[0] |= (data & 0x01);
				crc16_array[1] |= ((data >> 1) & 0x01);
				crc16_array[2] |= ((data >> 2) & 0x01);
				crc16_array[3] |= ((data >> 3) & 0x01);
			
				sd_clk_transfer_high();
			}
			
#ifdef SD_MMC_CRC_CHECK
			for(i=0; i<4; i++)
			{
				//crc_check_array[i] = sd_cal_crc_mode(org_buf, sd_mmc_info->blk_len, char_mode[i]);
				if(crc16_array[i] != crc_check_array[i])
				{
					error = SD_MMC_ERROR_DATA_CRC;
					break;
				}
			}
#endif
		}
		else    //only data0 lines
		{
#ifdef AMLOGIC_CHIP_SUPPORT
			if((unsigned long)data_buf == 0x3400000)
			{
				for(; data_cnt < loop_num; data_cnt++)
				{
					temp = 0;   //clear temp varialbe
				
					for(j = 0; j < 8; j++)
					{
						sd_clk_transfer_low();
					
						data = sd_get_dat0_value();
						temp <<= 1;
						temp |= data;
					
						sd_clk_transfer_high();
					}
					
					WRITE_BYTE_TO_FIFO(temp);
					
#ifdef SD_MMC_CRC_CHECK
					crc_check = (crc_check << 8) ^ sd_crc_table[((crc_check >> 8) ^ temp) & 0xff];
#endif
				}
			}
			else
#endif
			{
				for(; data_cnt < loop_num; data_cnt++)
				{
					temp = 0;   //clear temp varialbe
				
					for(j = 0; j < 8; j++)
					{
						sd_clk_transfer_low();
					
						data = sd_get_dat0_value();
						temp <<= 1;
						temp |= data;
					
						sd_clk_transfer_high();
					}
					
					*data_buf = temp;
					data_buf++;
					
#ifdef SD_MMC_CRC_CHECK
					crc_check = (crc_check << 8) ^ sd_crc_table[((crc_check >> 8) ^ temp) & 0xff];
#endif
				}
			}

			//Read CRC16 data
			for(data_cnt = 0; data_cnt < 16; data_cnt++)    // 16 bits CRC
			{
				sd_clk_transfer_low();
		
				data = sd_get_dat0_value();
				crc16 <<= 1;
				crc16 |= data;      
			
				sd_clk_transfer_high();
			}
			
#ifdef SD_MMC_CRC_CHECK
			if(crc16 != crc_check)
				error = SD_MMC_ERROR_DATA_CRC;
#endif
		}
		
		sd_clk_transfer_low();      //for end bit
		sd_clk_transfer_high();
		
		data_busy = 1;
		data_cnt = 0;
		
#ifdef SD_MMC_CRC_CHECK
		org_buf = data_buf;
		crc_check = 0;
		crc_check_array[0] = crc_check_array[1] = crc_check_array[2] = crc_check_array[3] = 0;
#endif
	}

	sd_delay_clocks_z(sd_mmc_info, SD_MMC_TIME_NRC_NCC);     //Clock delay, Z type
	
	ret = sd_send_cmd_sw(sd_mmc_info, SD_MMC_STOP_TRANSMISSION, 0, RESPONSE_R1B, response);

#ifdef SD_MMC_CRC_CHECK
	if(error == SD_MMC_ERROR_DATA_CRC)
	{
		//#ifdef  SD_MMC_DEBUG
		//Debug_Printf("#%s error occured in sd_read_multi_block()!\n", sd_error_to_string(error));
		//#endif
		return error;
	}
#endif
	
	return ret;
}
#endif 

#ifdef SD_MMC_HW_CONTROL
int sd_write_single_block_hw(SD_MMC_Card_Info_t *sd_mmc_info, unsigned long lba, unsigned char * data_buf)
{
	int ret, write_retry_count;
	unsigned long data_addr;
	unsigned char response[MAX_RESPONSE_BYTES];
	unsigned char *status_data_buf = sd_mmc_info->sd_mmc_buf;

	if ((sd_mmc_info->card_type == CARD_TYPE_SDHC) || (sd_mmc_info->card_type == CARD_TYPE_EMMC))
	{
		data_addr = lba;
	}
	else
	{
        data_addr = sd_mmc_info->blk_len;
        data_addr *= lba;
	}
	
	for(write_retry_count=0; write_retry_count<4; write_retry_count++)
	{
	    ret = sd_send_cmd_hw(sd_mmc_info, SD_MMC_WRITE_BLOCK, data_addr, RESPONSE_R1, response, data_buf, sd_mmc_info->blk_len, 1);
		if(ret == SD_MMC_ERROR_DATA_CRC || ret == SD_MMC_ERROR_COM_CRC)
		{
		    if(sd_mmc_info->spec_version || sd_mmc_info->card_type == CARD_TYPE_SDHC)
		    {  
                memset(status_data_buf, 0, 64);
#ifdef SD_MMC_HW_CONTROL
	            if(SD_WORK_MODE == CARD_HW_MODE)
		            ret = sd_send_cmd_hw(sd_mmc_info, SD_SWITCH_FUNCTION, 0x00FFFF01, RESPONSE_R1, response, status_data_buf, 64, 1);
#endif
            }
            //for some sdhc card write problem on 7216 picframe
			continue;
		}
		else
		{    
			break;
		}
    }

    if(write_retry_count >= 4)
        return SD_MMC_ERROR_DATA_CRC;

    if(ret)
        return ret;

	return SD_MMC_NO_ERROR;
}
#endif

#ifdef SD_MMC_SW_CONTROL
int sd_write_single_block_sw(SD_MMC_Card_Info_t *sd_mmc_info, unsigned long lba, unsigned char * data_buf)
{
	int ret, i, j;
	unsigned long crc_status, data;
	
	unsigned long data_cnt = 0;
	
	unsigned char * org_buf = data_buf;
	unsigned short crc16_array[4] = {0, 0, 0, 0};
	unsigned short crc16;
		
	unsigned char response[MAX_RESPONSE_BYTES];
	
	unsigned long data_addr,loop_num;
	
	//Set data lines busy
	if(sd_mmc_info->bus_width == SD_BUS_WIDE)
	{
		sd_set_dat0_3_output();
		
		sd_clk_transfer_low();

		sd_set_dat0_3_value(0x0F);
		
		sd_clk_transfer_high();
	}
	else
	{
		sd_set_dat0_output();
		
		sd_clk_transfer_low();
		
		sd_set_dat0_value(0x01);
		
		sd_clk_transfer_high();
	}

	if ((sd_mmc_info->card_type == CARD_TYPE_SDHC) || (sd_mmc_info->card_type == CARD_TYPE_EMMC))
	{
		data_addr = lba;
	}
	else
	{
	data_addr = sd_mmc_info->blk_len;
	data_addr *= lba;
	}
	
	ret = sd_send_cmd_sw(sd_mmc_info, SD_MMC_WRITE_BLOCK, data_addr, RESPONSE_R1, response);
	if(ret)
		return ret;
		
	//Nwr cycles delay
	sd_delay_clocks_h(sd_mmc_info, SD_MMC_TIME_NWR);
	
	//Start bit
	sd_clk_transfer_low();
	
	if(sd_mmc_info->bus_width == SD_BUS_WIDE)
	{
		sd_set_dat0_3_value(0x00);
	}
	else
	{
		sd_set_dat0_value(0x00);
	}
	
	sd_clk_transfer_high();
	
	//Write data
	loop_num = sd_mmc_info->blk_len;
	if(sd_mmc_info->bus_width == SD_BUS_WIDE)
	{
		for(data_cnt = 0; data_cnt < loop_num; data_cnt++)
		{
			for(i=1; i>=0; i--)
			{
				sd_clk_transfer_low();
				
				data = (*data_buf >> (i<<2)) & 0x0F;
				sd_set_dat0_3_value(data);
				
				sd_clk_transfer_high();
			}
			
			data_buf++;
		}
		
		//Caculate CRC16 value and write to line
		for(i=0; i<4; i++)
		{
			crc16_array[i] = sd_cal_crc_mode(org_buf, sd_mmc_info->blk_len, char_mode[i]);
		}
	
		//Write CRC16
		for(i=15; i>=0; i--)
		{
			sd_clk_transfer_low();
		
			data = 0;
			for(j=3; j>=0; j--)
			{
				data <<= 1; 
				data |= (crc16_array[j] >> i) & 0x0001;
			}
			sd_set_dat0_3_value(data);
			
			sd_clk_transfer_high();
		}
	}
	else    //only dat0 line
	{
		for(data_cnt = 0; data_cnt < loop_num; data_cnt++)
		{
			for(i=7; i>=0; i--)
			{
				sd_clk_transfer_low();
				
				data = (*data_buf >> i) & 0x01;
				sd_set_dat0_value(data);
				
				sd_clk_transfer_high();
			}
			
			data_buf++;
		}
		
		//Caculate CRC16 value and write to line
		crc16 = sd_cal_crc16(org_buf, sd_mmc_info->blk_len);

		//Write CRC16
		for(i=15; i>=0; i--)
		{
			sd_clk_transfer_low();
		
			data = (crc16 >> i) & 0x0001;
			sd_set_dat0_value(data);
			
			sd_clk_transfer_high();
		}
	}

	//End bit
	if(sd_mmc_info->bus_width == SD_BUS_WIDE)
	{
		sd_clk_transfer_low();
	
		sd_set_dat0_3_value(0x0F);
		
		sd_clk_transfer_high();
		
		sd_set_dat0_3_input();
	}
	else
	{
		sd_clk_transfer_low();
	
		sd_set_dat0_value(0x01);
		
		sd_clk_transfer_high();
		
		sd_set_dat0_input();
	}

	sd_delay_clocks_h(sd_mmc_info, SD_MMC_Z_CMD_TO_RES);
	
	crc_status = 0;
	//Check CRC status
	sd_set_dat0_input();
	for(i = 0; i < 5; i++)
	{
		sd_clk_transfer_low();
		
		data = sd_get_dat0_value();
		crc_status <<= 1;
		crc_status |= data; 
		
		sd_clk_transfer_high();
	}
	if (crc_status == 0x0A)         //1011, CRC error
		return SD_MMC_ERROR_DATA_CRC;
	else if (crc_status == 0x0F)        //1111, Programming error
		return SD_MMC_ERROR_DRIVER_FAILURE;
						//0101, CRC ok
		
	//Check busy
	sd_start_timer(SD_PROGRAMMING_TIMEOUT);
	do
	{
		sd_clk_transfer_low();
		
		data = sd_get_dat0_value();
		
		sd_clk_transfer_high();
		
		if(data)
			break;

	}while(!sd_check_timer());
	
	if(sd_check_timeout())
	{
		return SD_MMC_ERROR_TIMEOUT;
	}

	return SD_MMC_NO_ERROR;
}
#endif

#ifdef SD_MMC_HW_CONTROL
int sd_write_multi_block_hw(SD_MMC_Card_Info_t *sd_mmc_info, unsigned long lba, unsigned long lba_cnt, unsigned char * data_buf)
{
	int ret, write_retry_count;
	unsigned long lba_num, data_addr, data_offset = 0;
	unsigned char *status_data_buf = sd_mmc_info->sd_mmc_buf;
	unsigned char response[MAX_RESPONSE_BYTES];
	
	if(lba_cnt == 0)
		return SD_MMC_ERROR_BLOCK_LEN;

	if ((sd_mmc_info->card_type == CARD_TYPE_SDHC) || (sd_mmc_info->card_type == CARD_TYPE_EMMC))
	{
		data_addr = lba;
	}
	else
	{
		data_addr = sd_mmc_info->blk_len;
		data_addr *= lba;
	}

    if(sd_mmc_info->write_multi_block_failed)
    {
        for(lba_num=lba; lba_num<(lba+lba_cnt); lba_num++)
	    {
		    ret = sd_write_single_block_hw(sd_mmc_info, lba_num, data_buf+data_offset);
		    if(ret)
			    return ret;
		
		    data_offset += sd_mmc_info->blk_len;
	    }
	}
	else
    {
        while(lba_cnt)
        {
            if(lba_cnt > sd_mmc_info->max_blk_count)
                lba_num = sd_mmc_info->max_blk_count;
            else
                lba_num = lba_cnt;
	        if(sd_mmc_info->card_type == CARD_TYPE_SD || sd_mmc_info->card_type == CARD_TYPE_SDHC)
	        {
		        ret = sd_send_cmd_hw(sd_mmc_info, SD_APP_CMD, sd_mmc_info->card_rca<<16, RESPONSE_R1, response, NULL, 0, 0);
		        if (ret) 
		            return ret;
		
		        ret = sd_send_cmd_hw(sd_mmc_info, SD_SET_WR_BLK_ERASE_COUNT, lba_num, RESPONSE_R1, response, NULL, 0, 0);
		        if (ret) 
		            return ret;
	        }

            if((sd_mmc_info->card_type == CARD_TYPE_SDHC) || (sd_mmc_info->card_type == CARD_TYPE_EMMC))
	            data_addr += data_offset/512;
	        else
	            data_addr += data_offset;

            data_buf += data_offset;
	        for(write_retry_count=0; write_retry_count<4; write_retry_count++)
	        {
	            ret = sd_send_cmd_hw(sd_mmc_info, SD_MMC_WRITE_MULTIPLE_BLOCK, data_addr, RESPONSE_R1, response, data_buf, lba_num*512, 1);
	            if(ret == SD_MMC_ERROR_DATA_CRC || ret == SD_MMC_ERROR_COM_CRC)
	            {
	                ret = sd_send_cmd_hw(sd_mmc_info, SD_MMC_STOP_TRANSMISSION, 0, RESPONSE_R1B, response, NULL, 0, 0);
	                if(ret)
	                {
	                    return ret;
	                }
	                else
	                {
	                    if(sd_mmc_info->spec_version || sd_mmc_info->card_type == CARD_TYPE_SDHC)
		                {  
                            memset(status_data_buf, 0, 64);
#ifdef SD_MMC_HW_CONTROL
	                        if(SD_WORK_MODE == CARD_HW_MODE)
		                        ret = sd_send_cmd_hw(sd_mmc_info, SD_SWITCH_FUNCTION, 0x00FFFF01, RESPONSE_R1, response, status_data_buf, 64, 1);
#endif
                        //for some sdhc card write problem on 7216 picframe
                        }
	                    continue;
	                }
	            }
	            else
	                break;
	        }

            if(write_retry_count >= 4)
            {
                sd_mmc_info->write_multi_block_failed = 1;
                return SD_MMC_ERROR_DATA_CRC;
            }

            if(ret)
            { 
                return ret;
            }
            else
            {        
	            ret = sd_send_cmd_hw(sd_mmc_info, SD_MMC_STOP_TRANSMISSION, 0, RESPONSE_R1B, response, NULL, 0, 0);
	            if(ret)
	                return ret;
	        }

            lba_cnt -= lba_num;
            data_offset = lba_num*512;
	    }
	}
	
	return SD_MMC_NO_ERROR;
}
#endif

#ifdef SD_MMC_SW_CONTROL
int sd_write_multi_block_sw(SD_MMC_Card_Info_t *sd_mmc_info, unsigned long lba, unsigned long lba_cnt, unsigned char * data_buf)
{
	int ret,i,j;
	unsigned long crc_status, data;
	
	unsigned long data_cnt = 0;
	
	unsigned char * org_buf = data_buf;
	unsigned short crc16_array[4] = {0, 0, 0, 0};
	unsigned short crc16;
	unsigned char char_mode[4][3] = {{0x10, 0x01, 0},
					{0x20, 0x02, 0},
					{0x40, 0x04, 0},
					{0x80, 0x08, 0}};
	
	unsigned char response[MAX_RESPONSE_BYTES];
	
	unsigned long data_addr,loop_num,blk_cnt;
	
	if(lba_cnt == 0)
		return SD_MMC_ERROR_BLOCK_LEN;
		
	if (sd_mmc_info->card_type == CARD_TYPE_SD || sd_mmc_info->card_type == CARD_TYPE_SDHC)
	{
		ret = sd_send_cmd_sw(sd_mmc_info, SD_APP_CMD, sd_mmc_info->card_rca<<16, RESPONSE_R1, response);
		if (ret) return ret;
		
		ret = sd_send_cmd_sw(sd_mmc_info, SD_SET_WR_BLK_ERASE_COUNT, lba_cnt, RESPONSE_R1, response);
		if (ret) return ret;
	}

	//Set data lines busy
	if(sd_mmc_info->bus_width == SD_BUS_WIDE)
	{
		sd_set_dat0_3_output();
		
		sd_clk_transfer_low();
		
		sd_set_dat0_3_value(0x0F);
		
		sd_clk_transfer_high();
	}
	else
	{
		sd_set_dat0_output();
		
		sd_clk_transfer_low();
		
		sd_set_dat0_value(0x01);
		
		sd_clk_transfer_high();
	}

	if ((sd_mmc_info->card_type == CARD_TYPE_SDHC) || (sd_mmc_info->card_type == CARD_TYPE_EMMC))
	{
		data_addr = lba;
	}
	else
	{
	    data_addr = sd_mmc_info->blk_len;
	    data_addr *= lba;
	}
	
	ret = sd_send_cmd_sw(sd_mmc_info, SD_MMC_WRITE_MULTIPLE_BLOCK, data_addr, RESPONSE_R1, response);
	if(ret)
		return ret;
	
	loop_num = sd_mmc_info->blk_len;
	for(blk_cnt = 0; blk_cnt < lba_cnt; blk_cnt++)
	{
		org_buf = data_buf;
		
		//Nwr cycles delay
		sd_delay_clocks_h(sd_mmc_info, SD_MMC_TIME_NWR);
		
		//Start bit
		if(sd_mmc_info->bus_width == SD_BUS_WIDE)
		{
			sd_set_dat0_3_output();
			
			sd_clk_transfer_low();
			
			sd_set_dat0_3_value(0x00);
			
			sd_clk_transfer_high();
		}
		else
		{
			sd_set_dat0_output();
			
			sd_clk_transfer_low();
			
			sd_set_dat0_value(0x00);
			
			sd_clk_transfer_high();
		}

		//Write data
		if(sd_mmc_info->bus_width == SD_BUS_WIDE)
		{
			for(data_cnt = 0; data_cnt < loop_num; data_cnt++)
			{
				for(i=1; i>=0; i--)
				{
					sd_clk_transfer_low();
				
					data = (*data_buf >> (i<<2)) & 0x0F;
					sd_set_dat0_3_value(data);
					
					sd_clk_transfer_high();
				}
				
				data_buf++;
			}
			
			//Caculate CRC16 value and write to line
			for(i=0; i<4; i++)
			{
				crc16_array[i] = sd_cal_crc_mode(org_buf, sd_mmc_info->blk_len, char_mode[i]);
			}
			
			//Write CRC16
			for(i=15; i>=0; i--)
			{
				sd_clk_transfer_low();
		
				data = 0;
				for(j=3; j>=0; j--)
				{
					data <<= 1; 
					data |= (crc16_array[j] >> i) & 0x0001;
				}
				sd_set_dat0_3_value(data);
			
				sd_clk_transfer_high();
			}
		}
		else    // only dat0 line
		{
			for(data_cnt = 0; data_cnt < loop_num; data_cnt++)
			{
				for(i=7; i>=0; i--)
				{
					sd_clk_transfer_low();
					
					data = (*data_buf >> i) & 0x01;
					sd_set_dat0_value(data);
					
					sd_clk_transfer_high();
				}
				
				data_buf++;
			}
			
			//Caculate CRC16 value and write to line
			crc16 = sd_cal_crc16(org_buf, sd_mmc_info->blk_len);

			//Write CRC16
			for(i=15; i>=0; i--)
			{
				sd_clk_transfer_low();
		
				data = (crc16 >> i) & 0x0001;
				sd_set_dat0_value(data);
			
				sd_clk_transfer_high();
			}
		}

		//End bit
		if(sd_mmc_info->bus_width == SD_BUS_WIDE)
		{
			sd_clk_transfer_low();
			
			sd_set_dat0_3_value(0x0F);
			
			sd_clk_transfer_high();
			
			sd_set_dat0_3_input();
		}
		else
		{
			sd_clk_transfer_low();
			
			sd_set_dat0_value(0x01);
			
			sd_clk_transfer_high();
			
			sd_set_dat0_input();
		}

		sd_delay_clocks_h(sd_mmc_info, SD_MMC_Z_CMD_TO_RES);
	
		crc_status = 0;
	
		//Check CRC status
		sd_set_dat0_input();
		for(i = 0; i < 5; i++)
		{
			sd_clk_transfer_low();
		
			data = sd_get_dat0_value();
			crc_status <<= 1;
			crc_status |= data; 
		
			sd_clk_transfer_high();
		}
		if (crc_status == 0x0A)         //1010, CRC error
			return SD_MMC_ERROR_DATA_CRC;
		else if (crc_status == 0x0F)        //1111, Programming error
			return SD_MMC_ERROR_DRIVER_FAILURE;
							//0101, CRC ok
							
		//Check busy
		sd_start_timer(SD_PROGRAMMING_TIMEOUT);
		do
		{
			sd_clk_transfer_low();
			
			data = sd_get_dat0_value();
			
			sd_clk_transfer_high();
		
			if(data)
				break;

		}while(!sd_check_timer());
	
		if(sd_check_timeout())
		{
			return SD_MMC_ERROR_TIMEOUT;
		}
	}
	
	ret = sd_send_cmd_sw(sd_mmc_info, SD_MMC_STOP_TRANSMISSION, 0 , RESPONSE_R1B, response);
	if(ret)
		return ret;
	
	//Check busy
	sd_start_timer(SD_PROGRAMMING_TIMEOUT);
	do
	{
		sd_clk_transfer_low();
		
		data = sd_get_dat0_value();
		
		sd_clk_transfer_high();
		
		if(data)
			break;

	}while(!sd_check_timer());
	
	if(sd_check_timeout())
	{
		return SD_MMC_ERROR_TIMEOUT;
	}

	return SD_MMC_NO_ERROR;
}
#endif

//Functions for SD INIT
int sd_hw_reset(SD_MMC_Card_Info_t *sd_mmc_info)
{
	int ret=SD_MMC_NO_ERROR;
	sd_mmc_info->operation_mode = CARD_INDENTIFICATION_MODE;

#ifdef SD_MMC_HW_CONTROL
	if(SD_WORK_MODE == CARD_HW_MODE)
		ret = sd_send_cmd_hw(sd_mmc_info, SD_MMC_GO_IDLE_STATE, 0, RESPONSE_NONE, 0, 0, 0, 1);
#endif	
#ifdef SD_MMC_SW_CONTROL
	if(SD_WORK_MODE == CARD_SW_MODE)
	{
		sd_delay_clocks_h(sd_mmc_info, 74);  //74 is enough according to spec
	
		sd_delay_ms(1);
		
		ret = sd_send_cmd_sw(sd_mmc_info, SD_MMC_GO_IDLE_STATE, 0, RESPONSE_NONE, 0);
	}
#endif
	
	return ret;
}

int sd_sw_reset(SD_MMC_Card_Info_t *sd_mmc_info)
{
	int ret=SD_MMC_NO_ERROR;
	
	sd_mmc_info->operation_mode = CARD_INDENTIFICATION_MODE;

#ifdef SD_MMC_HW_CONTROL
	if(SD_WORK_MODE == CARD_HW_MODE)
		ret = sd_send_cmd_hw(sd_mmc_info, SD_MMC_GO_IDLE_STATE, 0, RESPONSE_NONE, 0, 0, 0, 1);
#endif	
#ifdef SD_MMC_SW_CONTROL
	if(SD_WORK_MODE == CARD_SW_MODE)	
		ret = sd_send_cmd_sw(sd_mmc_info, SD_MMC_GO_IDLE_STATE, 0, RESPONSE_NONE, 0);
#endif
	
	return ret;
}

int sd_voltage_validation(SD_MMC_Card_Info_t *sd_mmc_info)
{
	unsigned char response[MAX_RESPONSE_BYTES];
	SD_Response_R3_t * r3;
	SD_Response_R7_t * r7;
	SDIO_Response_R4_t *r4;
	int ret = 0,error = 0,delay_time,delay_cnt;

	//sd_delay_ms(10);
	
	delay_time = 10;
	delay_cnt = 1;
	//Detect if SD card is inserted first

	do
	{
#ifdef SD_MMC_HW_CONTROL
		if(SD_WORK_MODE == CARD_HW_MODE)
			ret = sd_send_cmd_hw(sd_mmc_info, IO_SEND_OP_COND, 0x00200000, RESPONSE_R4, response, 0, 0, 0);   // 0x00200000: 3.3v~3.4v
#endif
#ifdef SD_MMC_SW_CONTROL
		if(SD_WORK_MODE == CARD_SW_MODE)
			ret = sd_send_cmd_sw(sd_mmc_info, IO_SEND_OP_COND, 0x00200000, RESPONSE_R4, response);   // 0x00200000: 3.3v~3.4v
#endif
#ifdef SD_MMC_HW_CONTROL
		if(SD_WORK_MODE == CARD_HW_MODE)
			ret = sd_send_cmd_hw(sd_mmc_info, IO_SEND_OP_COND, 0x00200000, RESPONSE_R4, response, 0, 0, 0);   // 0x00200000: 3.3v~3.4v
#endif
#ifdef SD_MMC_SW_CONTROL
		if(SD_WORK_MODE == CARD_SW_MODE)
			ret = sd_send_cmd_sw(sd_mmc_info, IO_SEND_OP_COND, 0x00200000, RESPONSE_R4, response);   // 0x00200000: 3.3v~3.4v
#endif
	    if(ret == SD_MMC_ERROR_TIMEOUT)
	    {	
		    error = sd_hw_reset(sd_mmc_info);
		    if(error)
		    {
#ifdef  SD_MMC_DEBUG
			    Debug_Printf("#%s error occured in sd_hw_reset()\n", sd_error_to_string(error));
#endif
			    return error;
		    }

		    break;
	    }
	    else
	    {
            r4 = (SDIO_Response_R4_t *)response;
            if(r4->Card_Ready)
            {
                sd_mmc_info->card_type = CARD_TYPE_SDIO;
                sd_mmc_info->sdio_function_nums = r4->IO_Function_No;
                if(r4->Memory_Present)
                   break;
                else {
#ifdef SD_MMC_DEBUG
					Debug_Printf("Actual delay time in sdio_voltage_validation() = %d ms\n", delay_time*delay_cnt);
#endif
                    return SD_MMC_NO_ERROR;
            	}
            }

            sd_delay_ms(delay_time);
		    delay_cnt++;
        }
    } while(delay_cnt < (SD_MMC_IDENTIFY_TIMEOUT/delay_time));

	sd_delay_ms(10);

#ifdef SD_MMC_HW_CONTROL
	if(SD_WORK_MODE == CARD_HW_MODE)
		ret = sd_send_cmd_hw(sd_mmc_info, SD_SEND_IF_COND, 0x000001aa, RESPONSE_R7, response, 0, 0, 0);
#endif		
#ifdef SD_MMC_SW_CONTROL
	if(SD_WORK_MODE == CARD_SW_MODE)
		ret = sd_send_cmd_sw(sd_mmc_info, SD_SEND_IF_COND, 0x000001aa, RESPONSE_R7, response);
#endif


	if(ret)
	{
		if(ret == SD_MMC_ERROR_TIMEOUT) {
			error = sd_hw_reset(sd_mmc_info);
			if(error)
			{
#ifdef  SD_MMC_DEBUG
				Debug_Printf("#%s error occured in sd_hw_reset()\n", sd_error_to_string(error));
#endif
				return error;
			}
		}
		else
			return ret;
	}
	else
	{
		r7 = (SD_Response_R7_t *)response;
		if(r7->cmd_version == 0 && r7->voltage_accept == 1 && r7->check_pattern == 0xAA)
			sd_mmc_info->card_type = CARD_TYPE_SDHC;
		else
			sd_mmc_info->card_type = CARD_TYPE_SD;
	}

	delay_cnt = 2;
    do
    {
#ifdef SD_MMC_HW_CONTROL
		if(SD_WORK_MODE == CARD_HW_MODE)
			ret = sd_send_cmd_hw(sd_mmc_info, SD_APP_CMD, 0, RESPONSE_R1, response, 0, 0, 0);
#endif		
#ifdef SD_MMC_SW_CONTROL
		if(SD_WORK_MODE == CARD_SW_MODE)
			ret = sd_send_cmd_sw(sd_mmc_info, SD_APP_CMD, 0, RESPONSE_R1, response);
#endif
					
		if(ret)
		{
			if(ret == SD_MMC_ERROR_TIMEOUT)
				break;
				
			sd_delay_ms(delay_time);
			delay_cnt++;
				
			continue;
		}

		if(sd_mmc_info->card_type == CARD_TYPE_SDHC)
		{	
#ifdef SD_MMC_HW_CONTROL
			if(SD_WORK_MODE == CARD_HW_MODE)
				ret = sd_send_cmd_hw(sd_mmc_info, SD_APP_OP_COND, 0x40200000, RESPONSE_R3, response, 0, 0, 0);   // 0x00200000: 3.3v~3.4v
#endif
#ifdef SD_MMC_SW_CONTROL
			if(SD_WORK_MODE == CARD_SW_MODE)
				ret = sd_send_cmd_sw(sd_mmc_info, SD_APP_OP_COND, 0x40200000, RESPONSE_R3, response);   // 0x00200000: 3.3v~3.4v
#endif
		}
		else
		{	
#ifdef SD_MMC_HW_CONTROL
		if(SD_WORK_MODE == CARD_HW_MODE)
			ret = sd_send_cmd_hw(sd_mmc_info, SD_APP_OP_COND, 0x00200000, RESPONSE_R3, response, 0, 0, 0);   // 0x00200000: 3.3v~3.4v
#endif
#ifdef SD_MMC_SW_CONTROL
		if(SD_WORK_MODE == CARD_SW_MODE)
			ret = sd_send_cmd_sw(sd_mmc_info, SD_APP_OP_COND, 0x00200000, RESPONSE_R3, response);   // 0x00200000: 3.3v~3.4v
#endif
		}	

		r3 = (SD_Response_R3_t *)response;
		if(ret == SD_MMC_NO_ERROR && r3->ocr.Card_Busy)
		{
#ifdef SD_MMC_DEBUG
			Debug_Printf("Actual delay time in sd_voltage_validation() = %d ms\n", delay_time*delay_cnt);
#endif
			if(!r3->ocr.Card_Capacity_Status) 
				sd_mmc_info->card_type = CARD_TYPE_SD;

			return SD_MMC_NO_ERROR;
		}

		sd_delay_ms(delay_time);
		delay_cnt++;
	} while(delay_cnt < (SD_MMC_IDENTIFY_TIMEOUT/delay_time));

	sd_sw_reset(sd_mmc_info);
	sd_delay_ms(10);

	delay_cnt = 2;
	//No SD card, detect if MMC card is inserted then
	do
	{
#ifdef SD_MMC_HW_CONTROL
		if(SD_WORK_MODE == CARD_HW_MODE)
			ret = sd_send_cmd_hw(sd_mmc_info, MMC_SEND_OP_COND, 0x40FF8000, RESPONSE_R3, response, 0, 0, 0); // 0x00200000: 3.3v~3.4v
#endif 
#ifdef SD_MMC_SW_CONTROL
		if(SD_WORK_MODE == CARD_SW_MODE)
			ret = sd_send_cmd_sw(sd_mmc_info, MMC_SEND_OP_COND, 0x40FF8000, RESPONSE_R3, response); // 0x00200000: 3.3v~3.4v
#endif
			r3 = (SD_Response_R3_t *)response;
		if(ret == SD_MMC_ERROR_TIMEOUT)
		{
			break;
		}
		else if((ret == SD_MMC_NO_ERROR) && r3->ocr.Card_Busy)
		{
#ifdef SD_MMC_DEBUG
			Debug_Printf("Actual delay time in sd_voltage_validation() = %d ms\n", delay_time*delay_cnt);
#endif
			if(!r3->ocr.Card_Capacity_Status)
			sd_mmc_info->card_type = CARD_TYPE_MMC;
			else
				sd_mmc_info->card_type = CARD_TYPE_EMMC;
			
			return SD_MMC_NO_ERROR;
		}
		
		sd_delay_ms(delay_time);
		delay_cnt++;
	} while(delay_cnt < (SD_MMC_IDENTIFY_TIMEOUT/delay_time));

#ifdef SD_MMC_DEBUG
	Debug_Printf("No any SD/MMC card detected!\n");
#endif
	return SD_MMC_ERROR_DRIVER_FAILURE;
}

int sd_identify_process(SD_MMC_Card_Info_t *sd_mmc_info)
{
	int ret = 0, slot_id, times;
	unsigned temp;  ///< for compiler bug
	unsigned delay_time, delay_cnt = 0;
	
	unsigned char response[MAX_RESPONSE_BYTES];
	SD_Response_R2_CSD_t * r2_csd = NULL;
	SDHC_Response_R2_CSD_t * sdhc_r2_csd = NULL;
	SD_Response_R6_t * sd_response_r6;
	
	unsigned short c_size;
	unsigned char c_size_multi;
	unsigned char read_reg_data, write_reg_data;
	unsigned sdio_config;
	SDIO_Config_Reg_t *config_reg = NULL;
	
	unsigned char *mmc_ext_csd_buf = sd_mmc_info->sd_mmc_buf;
	MMC_REG_EXT_CSD_t *mmc_ext_csd_reg;
	
	//Request all devices to send their CIDs
	if(sd_mmc_info->card_type != CARD_TYPE_SDIO)
	{
#ifdef SD_MMC_HW_CONTROL
	if(SD_WORK_MODE == CARD_HW_MODE)
		ret = sd_send_cmd_hw(sd_mmc_info, SD_MMC_ALL_SEND_CID, 0, RESPONSE_R2_CID, response, 0, 0, 1);
#endif	
#ifdef SD_MMC_SW_CONTROL
	if(SD_WORK_MODE == CARD_SW_MODE)
		ret = sd_send_cmd_sw(sd_mmc_info, SD_MMC_ALL_SEND_CID, 0, RESPONSE_R2_CID, response);
#endif
	}

	sd_delay_ms(50);  //for MUSE 64MB CARD sd_identify_process timeout
	//sd_delay_ms(10);  //for samsung card
	/* Assign IDs to all devices found */
	slot_id = 1;
	delay_time = 10;
	while(delay_cnt < SD_IDENTIFICATION_TIMEOUT/TIMER_1MS)
	{
#ifdef SD_MMC_HW_CONTROL
		if(SD_WORK_MODE == CARD_HW_MODE)
		{
			if(sd_mmc_info->card_type == CARD_TYPE_SD || sd_mmc_info->card_type == CARD_TYPE_SDHC || sd_mmc_info->card_type == CARD_TYPE_SDIO)
				ret = sd_send_cmd_hw(sd_mmc_info, SD_MMC_SEND_RELATIVE_ADDR, slot_id<<16, RESPONSE_R6, response, 0, 0, 0);   ///* Send out a byte to read RCA*/
			else if((sd_mmc_info->card_type == CARD_TYPE_MMC) || (sd_mmc_info->card_type == CARD_TYPE_EMMC)) 
				ret = sd_send_cmd_hw(sd_mmc_info, SD_MMC_SEND_RELATIVE_ADDR, slot_id<<16, RESPONSE_R1, response, 0, 0, 0);   ///* Send out a byte to read RCA*/
		}
#endif			
#ifdef SD_MMC_SW_CONTROL
		if(SD_WORK_MODE == CARD_SW_MODE)
		{
			if(sd_mmc_info->card_type == CARD_TYPE_SD || sd_mmc_info->card_type == CARD_TYPE_SDHC || sd_mmc_info->card_type == CARD_TYPE_SDIO)
				ret = sd_send_cmd_sw(sd_mmc_info, SD_MMC_SEND_RELATIVE_ADDR, slot_id<<16, RESPONSE_R6, response);   ///* Send out a byte to read RCA*/
			else if((sd_mmc_info->card_type == CARD_TYPE_MMC) || (sd_mmc_info->card_type == CARD_TYPE_EMMC))
				ret = sd_send_cmd_sw(sd_mmc_info, SD_MMC_SEND_RELATIVE_ADDR, slot_id<<16, RESPONSE_R1, response);   ///* Send out a byte to read RCA*/
		}
#endif

		sd_response_r6 = (SD_Response_R6_t *)response;
		/* Check for SD card */
		if((sd_mmc_info->card_type == CARD_TYPE_SD || sd_mmc_info->card_type == CARD_TYPE_SDHC || sd_mmc_info->card_type == CARD_TYPE_SDIO) && (ret == SD_MMC_NO_ERROR))
			break;

				/* Get device information and assign an RCA to it. */
		if (((sd_mmc_info->card_type == CARD_TYPE_MMC) || (sd_mmc_info->card_type == CARD_TYPE_EMMC)) && ( ret == SD_MMC_NO_ERROR))
		{
			/* There isn't any more device found */
			break;        
		}
		else if((sd_mmc_info->card_type == CARD_TYPE_MMC) || (sd_mmc_info->card_type == CARD_TYPE_EMMC))
		{
			/* The RCA is returned in pc->LastResponse[4] */
			slot_id += 1;
		}
	
		sd_delay_ms(delay_time);
		delay_cnt += delay_time;
	}
	
	if(delay_cnt >= SD_IDENTIFICATION_TIMEOUT/TIMER_1MS)
	{
		return SD_MMC_ERROR_DRIVER_FAILURE;
	}
	
	if(sd_mmc_info->card_type == CARD_TYPE_SD || sd_mmc_info->card_type == CARD_TYPE_SDHC || sd_mmc_info->card_type == CARD_TYPE_SDIO)
		sd_mmc_info->card_rca = ((SD_Response_R6_t *)response)->rca_high << 8 | ((SD_Response_R6_t *)response)->rca_low;
	else if((sd_mmc_info->card_type == CARD_TYPE_MMC)  || (sd_mmc_info->card_type == CARD_TYPE_EMMC))
		sd_mmc_info->card_rca = slot_id;
	
    if(sd_mmc_info->card_type == CARD_TYPE_SDIO)
    {
#ifdef SD_MMC_HW_CONTROL
	    if(SD_WORK_MODE == CARD_HW_MODE)
		    ret = sd_send_cmd_hw(sd_mmc_info, SD_MMC_SELECT_DESELECT_CARD, sd_mmc_info->card_rca<<16, RESPONSE_R1B, response, 0, 0, 1);
#endif	
#ifdef SD_MMC_SW_CONTROL
	    if(SD_WORK_MODE == CARD_SW_MODE)
		    ret = sd_send_cmd_sw(sd_mmc_info, SD_MMC_SELECT_DESELECT_CARD, sd_mmc_info->card_rca<<16, RESPONSE_R1B, response);
#endif

		ret = sdio_read_reg(sd_mmc_info, 0, BUS_Interface_Control_REG, &read_reg_data);
		if(ret)
			return ret;

		sd_mmc_info->operation_mode = DATA_TRANSFER_MODE;
		write_reg_data = ((read_reg_data & 0xfc) | SDIO_Wide_bus_Bit);
		ret = sdio_write_reg(sd_mmc_info, 0, BUS_Interface_Control_REG, &write_reg_data, SDIO_Read_After_Write);
		if(ret)
		{
			write_reg_data = SDIO_Single_bus_Bit;
    		ret = sdio_write_reg(sd_mmc_info, 0, BUS_Interface_Control_REG, &write_reg_data, SDIO_Read_After_Write);
    		if(ret)
				return ret;

			sd_mmc_info->bus_width = SD_BUS_SINGLE;
	        return SD_MMC_NO_ERROR;
	    }  
		else
		{
			sd_mmc_info->bus_width = SD_BUS_WIDE;
#ifdef SD_MMC_HW_CONTROL
			if(SD_WORK_MODE == CARD_HW_MODE)
			{
				sdio_config = 0;
				config_reg = (void *)&sdio_config;
				sdio_config = READ_CBUS_REG(SDIO_CONFIG);
				config_reg->bus_width = 1;
				WRITE_CBUS_REG(SDIO_CONFIG, sdio_config);
			}
#endif
			return SD_MMC_NO_ERROR;
		}    
    }

	ret = sd_read_reg_cid(sd_mmc_info, &sd_mmc_info->raw_cid);
	if(ret)
		return ret;

#ifdef SD_MMC_HW_CONTROL
	if(SD_WORK_MODE == CARD_HW_MODE)
		ret = sd_send_cmd_hw(sd_mmc_info, SD_MMC_SEND_CSD, sd_mmc_info->card_rca<<16, RESPONSE_R2_CSD, response, 0, 0, 1);
#endif	
#ifdef SD_MMC_SW_CONTROL
	if(SD_WORK_MODE == CARD_SW_MODE)
		ret = sd_send_cmd_sw(sd_mmc_info, SD_MMC_SEND_CSD, sd_mmc_info->card_rca<<16, RESPONSE_R2_CSD, response);
#endif
	if(ret)
	{
#ifdef  SD_MMC_DEBUG
		Debug_Printf("#%s error occured at line: %d in file %s\n", sd_error_to_string(ret),__LINE__,__FILE__);
#endif
		return ret;
	}
	
	if(sd_mmc_info->card_type == CARD_TYPE_SDHC)
		sdhc_r2_csd = (SDHC_Response_R2_CSD_t *)response;
	else
	    r2_csd = (SD_Response_R2_CSD_t *)response;

	if((sd_mmc_info->card_type == CARD_TYPE_MMC) || (sd_mmc_info->card_type == CARD_TYPE_EMMC))
    	sd_mmc_info->mmc_spec_version = r2_csd->csd.MMC_SPEC_VERS;

	if(sd_mmc_info->card_type == CARD_TYPE_SDHC)
	{
		sd_mmc_info->clks_nac = 50000;
		c_size = (sdhc_r2_csd->csd.C_SIZE_high << 16) | (sdhc_r2_csd->csd.C_SIZE_mid << 8) | (sdhc_r2_csd->csd.C_SIZE_low);
		sd_mmc_info->blk_nums = (c_size + 1) << 10;
		sd_mmc_info->blk_len = 512;
	}
	else 
	{
	    sd_mmc_info->clks_nac = sd_cal_clks_nac(r2_csd->csd.TAAC, r2_csd->csd.NSAC);

	    c_size = (r2_csd->csd.C_SIZE_high << 10) | (r2_csd->csd.C_SIZE_mid << 2) | r2_csd->csd.C_SIZE_low;
	    c_size_multi = (r2_csd->csd.C_SIZE_MULT_high << 1) | r2_csd->csd.C_SIZE_MULT_low;
	    temp = (c_size+1) * (1 << (c_size_multi+2));
	    sd_mmc_info->blk_nums = temp;
	
	    sd_mmc_info->blk_len = 1 << r2_csd->csd.READ_BL_LEN;
	    if(sd_mmc_info->blk_len != 512)
	    {
		    temp = sd_mmc_info->blk_len;
		    if((temp % 512) != 0)
			    return SD_MMC_ERROR_BLOCK_LEN;
		
		    times = temp / 512;
		    temp = sd_mmc_info->blk_nums;
		    temp *= times;
		    sd_mmc_info->blk_nums = temp;
		    sd_mmc_info->blk_len = 512;
	    }
	}
	if(sd_mmc_info->card_type == CARD_TYPE_EMMC)
	{
		ret = sd_send_cmd_hw(sd_mmc_info, SD_MMC_SELECT_DESELECT_CARD, sd_mmc_info->card_rca<<16, RESPONSE_R1B, response, 0, 0, 1);
		
		ret = sd_send_cmd_hw(sd_mmc_info,MMC_SEND_EXT_CSD, 0, RESPONSE_R1, response, mmc_ext_csd_buf, sizeof(MMC_REG_EXT_CSD_t), 1);		
        if(ret)
            return ret;
            
        mmc_ext_csd_reg = (MMC_REG_EXT_CSD_t *)mmc_ext_csd_buf;
        sd_mmc_info->blk_nums = *((unsigned *)&mmc_ext_csd_reg->SEC_COUNT);	
        sd_mmc_info->blk_len = 512;
        printk("sd_mmc_info->blk_nums : 0x%x \n",sd_mmc_info->blk_nums);
	}

#ifdef SD_MMC_HW_CONTROL
	if(SD_WORK_MODE == CARD_HW_MODE)
		ret = sd_send_cmd_hw(sd_mmc_info, SD_MMC_SELECT_DESELECT_CARD, sd_mmc_info->card_rca<<16, RESPONSE_R1B, response, 0, 0, 1);
#endif	
#ifdef SD_MMC_SW_CONTROL
	if(SD_WORK_MODE == CARD_SW_MODE)
		ret = sd_send_cmd_sw(sd_mmc_info, SD_MMC_SELECT_DESELECT_CARD, sd_mmc_info->card_rca<<16, RESPONSE_R1B, response);
#endif

	if(sd_mmc_info->card_type == CARD_TYPE_SD || sd_mmc_info->card_type == CARD_TYPE_SDHC)
	{
		SD_REG_SCR_t scr;
		ret = sd_read_reg_scr(sd_mmc_info, &scr);
		if(ret)
		{
#ifdef  SD_MMC_DEBUG
			Debug_Printf("#%s error occured at line: %d in file %s\n", sd_error_to_string(ret),__LINE__,__FILE__);
#endif			
			return ret;
		}

		sd_mmc_info->spec_version = scr.SD_SPEC;
		
		if(sd_mmc_info->disable_wide_bus)
			scr.SD_BUS_WIDTHS = SD_BUS_SINGLE;
			
		if(!scr.SD_BUS_WIDTHS)
			scr.SD_BUS_WIDTHS = SD_BUS_WIDE | SD_BUS_SINGLE;
			
		if(scr.SD_BUS_WIDTHS & SD_BUS_WIDE)
		{//then set to 4bits width
#ifdef SD_MMC_HW_CONTROL
			if(SD_WORK_MODE == CARD_HW_MODE)
			{
				ret = sd_send_cmd_hw(sd_mmc_info, SD_APP_CMD, sd_mmc_info->card_rca<<16, RESPONSE_R1, response, 0, 0, 1);
				ret = sd_send_cmd_hw(sd_mmc_info, SD_SET_BUS_WIDTHS, 2, RESPONSE_R1, response, 0, 0, 1); //0 1bit, 10=4bits
			}
#endif			
#ifdef SD_MMC_SW_CONTROL
			if(SD_WORK_MODE == CARD_SW_MODE)
			{
				ret = sd_send_cmd_sw(sd_mmc_info, SD_APP_CMD, sd_mmc_info->card_rca<<16, RESPONSE_R1, response);
				ret = sd_send_cmd_sw(sd_mmc_info, SD_SET_BUS_WIDTHS, 2, RESPONSE_R1, response); //0 1bit, 10=4bits
			}
#endif
			if(ret)
			{
				sd_mmc_info->bus_width = SD_BUS_SINGLE;
			}
			else
			{
				sd_mmc_info->bus_width = SD_BUS_WIDE;
#ifdef SD_MMC_HW_CONTROL
				if(SD_WORK_MODE == CARD_HW_MODE)
				{
					unsigned long sdio_config = 0;
					SDIO_Config_Reg_t *config_reg = (void *)&sdio_config;
					sdio_config = READ_CBUS_REG(SDIO_CONFIG);
					config_reg->bus_width = 1;
					WRITE_CBUS_REG(SDIO_CONFIG, sdio_config);
				}
#endif
			}
		}
		else
		{//then set to 1bits width
#ifdef SD_MMC_HW_CONTROL
			if(SD_WORK_MODE == CARD_HW_MODE)
			{
				ret = sd_send_cmd_hw(sd_mmc_info, SD_APP_CMD, sd_mmc_info->card_rca<<16, RESPONSE_R1, response, 0, 0, 1);
				ret = sd_send_cmd_hw(sd_mmc_info, SD_SET_BUS_WIDTHS, 0, RESPONSE_R1, response, 0, 0, 1); //0 1bit, 10=4bits
			}
#endif

#ifdef SD_MMC_SW_CONTROL
			if(SD_WORK_MODE == CARD_SW_MODE)
			{
				ret = sd_send_cmd_sw(sd_mmc_info, SD_APP_CMD, sd_mmc_info->card_rca<<16, RESPONSE_R1, response);
				ret = sd_send_cmd_sw(sd_mmc_info, SD_SET_BUS_WIDTHS, 0, RESPONSE_R1, response); //0 1bit, 10=4bits
			}
#endif
			
			sd_mmc_info->bus_width = SD_BUS_SINGLE;
		}
	}
	else    //MMC card
	{
	    if(sd_mmc_info->mmc_spec_version == SPEC_VERSION_40_41)
	    {
#ifdef SD_MMC_HW_CONTROL
            if(SD_WORK_MODE == CARD_HW_MODE)
            {
                ret = sd_send_cmd_hw(sd_mmc_info, MMC_SWITCH_FUNTION, 0x03b70100, RESPONSE_R1, response, 0, 0, 1);
            }
#endif			
#ifdef SD_MMC_SW_CONTROL
            if(SD_WORK_MODE == CARD_SW_MODE)
            {
                ret = sd_send_cmd_sw(sd_mmc_info, MMC_SWITCH_FUNTION, 0x03b70100, RESPONSE_R1, response);
            }
#endif
            if(ret)
            {
                sd_mmc_info->bus_width = SD_BUS_SINGLE;
            }
            else
            {
                sd_mmc_info->bus_width = SD_BUS_WIDE;
#ifdef SD_MMC_HW_CONTROL
                if(SD_WORK_MODE == CARD_HW_MODE)
                {
                    unsigned long sdio_config = 0;
                    SDIO_Config_Reg_t *config_reg = (void *)&sdio_config;
                    sdio_config = READ_CBUS_REG(SDIO_CONFIG);
                    config_reg->bus_width = 1;
                    WRITE_CBUS_REG(SDIO_CONFIG, sdio_config);
                }
#endif
            }
        }
	    else
        {
            sd_mmc_info->bus_width = SD_BUS_SINGLE;
        }
	}

#ifdef SD_MMC_HW_CONTROL
	if(SD_WORK_MODE == CARD_HW_MODE)
	{
		unsigned long sdio_config = 0;
		SDIO_Config_Reg_t *config_reg = (void *)&sdio_config;
		sdio_config = READ_CBUS_REG(SDIO_CONFIG);
		if(sd_mmc_info->disable_high_speed == 1)
		{
			config_reg->cmd_clk_divide = 4;
			sd_mmc_info->sdio_clk_unit = 1000/SD_MMC_TRANSFER_SLOWER_CLK;
		}
		else
		{
			//config_reg->cmd_clk_divide = 3;
			config_reg->cmd_clk_divide = 4;
			sd_mmc_info->sdio_clk_unit = 1000/SD_MMC_TRANSFER_CLK;
		}

		WRITE_CBUS_REG(SDIO_CONFIG, sdio_config);
	}
#endif

	sd_mmc_info->operation_mode = DATA_TRANSFER_MODE;
	
	sd_mmc_check_wp(sd_mmc_info);

	return SD_MMC_NO_ERROR;
}

int sd_mmc_init(SD_MMC_Card_Info_t *sd_mmc_info)
{
	int error;
	
	
	/*close IF INT before change to sd to avoid error IF INT*/
	sdio_close_host_interrupt(SDIO_IF_INT);
	WRITE_CBUS_REG(SDIO_CONFIG, 0);
	WRITE_CBUS_REG(SDIO_MULT_CONFIG, 0);

	if(sd_mmc_info->inited_flag && !sd_mmc_info->removed_flag)
	{
#ifdef SD_MMC_HW_CONTROL
	    if(SD_WORK_MODE == CARD_HW_MODE)
		    sd_sdio_enable(sd_mmc_info->io_pad_type);
#endif
#ifdef SD_MMC_SW_CONTROL
	    if(SD_WORK_MODE == CARD_SW_MODE)
	    {	
		    sd_gpio_enable(sd_mmc_info->io_pad_type);
	    }
#endif		
			error = sd_mmc_cmd_test(sd_mmc_info);
			if(!error)
				goto error;
	}
	if(++sd_mmc_info->init_retry > SD_MMC_INIT_RETRY)
		return SD_MMC_ERROR_DRIVER_FAILURE;

#ifdef  SD_MMC_DEBUG
	Debug_Printf("\nSD/MMC initialization started......\n");
#endif
	
	error = sd_mmc_staff_init(sd_mmc_info);
	
	if(error)
	{
#ifdef  SD_MMC_DEBUG
		Debug_Printf("#%s error occured in sd_mmc_staff_init()()\n", sd_error_to_string(error));
#endif
		goto error;
	}

	sd_mmc_set_input(sd_mmc_info);
	error = sd_hw_reset(sd_mmc_info);

	if(error)
	{
#ifdef  SD_MMC_DEBUG
		Debug_Printf("#%s error occured in sd_hw_reset()\n", sd_error_to_string(error));
#endif
		goto error;
	}

	error = sd_voltage_validation(sd_mmc_info);
	
	if(error)
	{
#ifdef  SD_MMC_DEBUG
		Debug_Printf("#%s error occured in sd_voltage_validation()\n", sd_error_to_string(error));
#endif
		goto error;
	}

	error = sd_identify_process(sd_mmc_info);
	
    if(error)
	{
#ifdef  SD_MMC_DEBUG
		Debug_Printf("#%s error occured in sd_identify_process()\n", sd_error_to_string(error));
#endif
		goto error;
	}
	
#ifdef SD_MMC_HW_CONTROL
	if(SD_WORK_MODE == CARD_HW_MODE)
	{
		if(!sd_mmc_info->disable_high_speed)	
		{
	    	error = sd_mmc_switch_function(sd_mmc_info);
	    	if(error)
	    	{
#ifdef  SD_MMC_DEBUG
		    	Debug_Printf("#%s error occured in sd_switch_funtion()\n", sd_error_to_string(error));
#endif
			    //goto error;
	    	}
		}
	}
#endif

	if(sd_mmc_info->card_type == CARD_TYPE_SDIO)
	{
		error = sd_check_sdio_card_type(sd_mmc_info);
		if(error)
	    {
#ifdef  SD_MMC_DEBUG
		    Debug_Printf("#%s error occured in sd_check_sdio_card_type()\n", sd_error_to_string(error));
#endif
			goto error;
	    }
	}

//	error = sd_check_data_consistency(sd_mmc_info);
//	if(error)
//	{
//#ifdef  SD_MMC_DEBUG
//		Debug_Printf("#%s error occured in sd_check_data_consistency()!\n", sd_error_to_string(error));
//#endif
//		goto error;
//	}

#ifdef SD_MMC_DEBUG
	Debug_Printf("sd_mmc_init() is completed successfully!\n");
	if(sd_mmc_info->bus_width == SD_BUS_WIDE)
	{
	    if(sd_mmc_info->speed_class == HIGH_SPEED)
	    {
			if(sd_mmc_info->card_type == CARD_TYPE_SD)
			    Debug_Printf("This SD card is working in Wide Bus and high speed mode!\n\n");
		    else if(sd_mmc_info->card_type == CARD_TYPE_SDHC)
			    Debug_Printf("This SDHC card is working in Wide Bus and high speed mode!\n\n");
		    else if(sd_mmc_info->card_type == CARD_TYPE_SDIO)
		    	Debug_Printf("This SDIO card is working in Wide Bus and high speed mode!\n\n");
			else
			    Debug_Printf("This MMC card is working in Wide Bus and high speed mode!\n\n");
		}
		else
		{
			if(sd_mmc_info->card_type == CARD_TYPE_SD)
				Debug_Printf("This SD card is working in Wide Bus and normal speed mode!\n\n");
			else if(sd_mmc_info->card_type == CARD_TYPE_SDHC)
			 	Debug_Printf("This SDHC card is working in Wide Bus and normal speed mode!\n\n");
			else if(sd_mmc_info->card_type == CARD_TYPE_SDIO)
		    	Debug_Printf("This SDIO card is working in Wide Bus and normal speed mode!\n\n");
			else
			Debug_Printf("This MMC card is working in Wide Bus and normal speed mode!\n\n");
		}	
	}
	else
	{
	    if(sd_mmc_info->speed_class == HIGH_SPEED)
	    {
		    if(sd_mmc_info->card_type == CARD_TYPE_SD)
			    Debug_Printf("This SD card is working in Single Bus and high speed mode!\n\n");
		    else if(sd_mmc_info->card_type == CARD_TYPE_SDHC)
			    Debug_Printf("This SDHC card is working in Single Bus and high speed mode!\n\n");
			else if(sd_mmc_info->card_type == CARD_TYPE_SDIO)
		    	Debug_Printf("This SDIO card is working in Single Bus and high speed mode!\n\n");
		    else
			    Debug_Printf("This MMC card is working in Single Bus and high speed mode!\n\n");
		}
		else
		{
		    if(sd_mmc_info->card_type == CARD_TYPE_SD)
			    Debug_Printf("This SD card is working in Single Bus and normal speed mode!\n\n");
		    else if(sd_mmc_info->card_type == CARD_TYPE_SDHC)
			    Debug_Printf("This SDHC card is working in Single Bus and normal speed mode!\n\n");
			else if(sd_mmc_info->card_type == CARD_TYPE_SDIO)
		    	Debug_Printf("This SDIO card is working in Wide Bus and normal speed mode!\n\n");
			else
			    Debug_Printf("This MMC card is working in Single Bus and normal speed mode!\n\n");
		}
	}
#endif

/*	if (sd_mmc_info->card_type == CARD_TYPE_EMMC)
		emmc_test_upgrade(); */
	
	sd_mmc_info->inited_flag = 1;
	sd_mmc_info->init_retry = 0;

	sd_mmc_info->sd_save_hw_io_flag = 1;
	sd_mmc_info->sd_save_hw_io_config = READ_CBUS_REG(SDIO_CONFIG);
	sd_mmc_info->sd_save_hw_io_mult_config = READ_CBUS_REG(SDIO_MULT_CONFIG);
	sd_gpio_enable(sd_mmc_info->io_pad_type);

	return SD_MMC_NO_ERROR;

error:
	sd_gpio_enable(sd_mmc_info->io_pad_type);
	return error;
}

void sd_mmc_io_config(SD_MMC_Card_Info_t *sd_mmc_info)
{
	sd_gpio_enable(sd_mmc_info->io_pad_type);
	
	sd_set_cmd_output();
	sd_set_cmd_value(1);
	sd_set_clk_output();
	sd_set_clk_high();
	sd_set_dat0_3_input();
}

int sd_mmc_staff_init(SD_MMC_Card_Info_t *sd_mmc_info)
{
	unsigned int sdio_config, sdio_multi_config;
	SDIO_Config_Reg_t *config_reg;

    sd_mmc_prepare_power(sd_mmc_info);
   
	sd_mmc_power_on(sd_mmc_info);
#ifdef SD_MMC_HW_CONTROL
	if(SD_WORK_MODE == CARD_HW_MODE)
	{
		sdio_multi_config = (READ_CBUS_REG(SDIO_MULT_CONFIG) & 0x00000003);
		WRITE_CBUS_REG(SDIO_MULT_CONFIG, sdio_multi_config);

		sdio_config = 0;
		config_reg = (void *)&sdio_config;
		config_reg->cmd_clk_divide = 375;
		config_reg->cmd_argument_bits = 39;
		config_reg->m_endian = 3;
		config_reg->write_Nwr = 2;
		config_reg->write_crc_ok_status = 2;
		WRITE_CBUS_REG(SDIO_CONFIG, sdio_config);
		sd_mmc_info->sdio_clk_unit = (1000/SD_MMC_IDENTIFY_CLK)*1000;

		sd_sdio_enable(sd_mmc_info->io_pad_type);
	}
#endif
#ifdef SD_MMC_SW_CONTROL
	if(SD_WORK_MODE == CARD_SW_MODE)
		sd_mmc_io_config(sd_mmc_info);
#endif

	sd_mmc_info->card_type = CARD_TYPE_NONE;
	sd_mmc_info->operation_mode = CARD_INDENTIFICATION_MODE;
	sd_mmc_info->bus_width = SD_BUS_SINGLE;
    sd_mmc_info->spec_version = SPEC_VERSION_10_101;
    sd_mmc_info->speed_class = NORMAL_SPEED;
	
	sd_mmc_info->card_rca = 0;
	
	sd_mmc_info->blk_len = 0;
	sd_mmc_info->blk_nums = 0;
	
	sd_mmc_info->clks_nac = SD_MMC_TIME_NAC_DEFAULT;
	
	sd_mmc_info->inited_flag = 0;
	sd_mmc_info->removed_flag = 0;
	
	sd_mmc_info->write_protected_flag = 0;
	sd_mmc_info->single_blk_failed = 0;
	
	return SD_MMC_NO_ERROR;
}

//Read Operation Conditions Register
int sd_read_reg_ocr(SD_MMC_Card_Info_t *sd_mmc_info, SD_REG_OCR_t * ocr);
//Read Card_Identification Register
int sd_read_reg_cid(SD_MMC_Card_Info_t *sd_mmc_info, SD_REG_CID_t * cid)
{
	int ret;
	unsigned char response[MAX_RESPONSE_BYTES];

	sd_clear_response(response);

#ifdef SD_MMC_HW_CONTROL
	if(SD_WORK_MODE == CARD_HW_MODE)
	{
		ret = sd_send_cmd_hw(sd_mmc_info, SD_MMC_SEND_CID, sd_mmc_info->card_rca<<16, RESPONSE_R2_CID, response, NULL, 0, 1);
		if(ret)
			return ret;
		memcpy(cid, response, sizeof(SD_REG_CID_t));
	}
#endif

#ifdef SD_MMC_SW_CONTROL
	if(SD_WORK_MODE == CARD_SW_MODE)
	{
		ret = sd_send_cmd_sw(sd_mmc_info, SD_MMC_SEND_CID,sd_mmc_info->card_rca<<16, RESPONSE_R2_CID, response);
		if(ret)
			return ret;
		memcpy(cid, response, sizeof(SD_REG_CID_t));
	}
#endif

	return SD_MMC_NO_ERROR;
}

//Read Card-Specific Data Register
int sd_read_reg_csd(SD_MMC_Card_Info_t *sd_mmc_info, SD_REG_CSD_t * csd);
//Read Relative Card Address Register
int sd_read_reg_rca(SD_MMC_Card_Info_t *sd_mmc_info, unsigned short * rca);
//Read Driver Stage Register
int sd_read_reg_dsr(SD_MMC_Card_Info_t *sd_mmc_info, SD_REG_DSR_t * dsr);
//Read SD CARD Configuration Register
int sd_read_reg_scr(SD_MMC_Card_Info_t *sd_mmc_info, SD_REG_SCR_t * scr)
{
	int ret;
	unsigned char response[MAX_RESPONSE_BYTES];
#ifdef SD_MMC_SW_CONTROL
	unsigned short crc16;
#endif
	
	sd_clear_response(response);

#ifdef SD_MMC_HW_CONTROL
	if(SD_WORK_MODE == CARD_HW_MODE)
	{
		ret = sd_send_cmd_hw(sd_mmc_info, SD_APP_CMD, sd_mmc_info->card_rca<<16, RESPONSE_R1, response, 0, 0, 1);   
		 if(ret)
			return ret;
		
		ret = sd_send_cmd_hw(sd_mmc_info, SD_SEND_SCR, 0, RESPONSE_R1, response, sd_mmc_info->sd_mmc_buf, sizeof(SD_REG_SCR_t), 1);
		if(ret)
			return ret;
		memcpy(scr, sd_mmc_info->sd_mmc_buf, sizeof(SD_REG_SCR_t));
	}
#endif		
#ifdef SD_MMC_SW_CONTROL
	if(SD_WORK_MODE == CARD_SW_MODE)
	{
		ret = sd_send_cmd_sw(sd_mmc_info, SD_APP_CMD, sd_mmc_info->card_rca<<16, RESPONSE_R1, response);   
		if(ret)
			return ret;
	
		ret = sd_send_cmd_sw(sd_mmc_info, SD_SEND_SCR, 0, RESPONSE_NONE, response);
		if(ret)
			return ret;

		ret = sd_get_dat0_data(sd_mmc_info, sizeof(SD_REG_SCR_t), (unsigned char *)scr, &crc16);
		   if(ret)
			return ret;
	}
#endif
	
	return SD_MMC_NO_ERROR;
}

//Check if any card is connected to adapter
#ifdef SD_MMC_SW_CONTROL
SD_Card_Type_t sd_mmc_check_present(SD_MMC_Card_Info_t *sd_mmc_info)
{
	int retry, ret;
	
	unsigned char response[MAX_RESPONSE_BYTES];
	SD_Response_R3_t * r3;
	
	//Detect if SD card is inserted first
	for(retry = 0; retry < MAX_CHECK_INSERT_RETRY; retry++)
	{
		ret = sd_send_cmd_sw(sd_mmc_info, SD_APP_CMD, 0, RESPONSE_R1, response);
		if(ret)
			continue;

		ret = sd_send_cmd_sw(sd_mmc_info, SD_APP_OP_COND, 0x00200000, RESPONSE_R3, response);   // 0x00200000: 3.3v~3.4v
		r3 = (SD_Response_R3_t *)response;
		
		if((ret == SD_MMC_NO_ERROR) && r3->ocr.Card_Busy)
			return CARD_TYPE_SD;
	} 
	
	sd_sw_reset(sd_mmc_info);
	
	//No SD card, detect if MMC card is inserted then
	for(retry = 0; retry < MAX_CHECK_INSERT_RETRY; retry++)
	{
		ret = sd_send_cmd_sw(sd_mmc_info, MMC_SEND_OP_COND, 0x00200000, RESPONSE_R3, response); // 0x00200000: 3.3v~3.4v
		r3 = (SD_Response_R3_t *)response;
		
		if((ret == SD_MMC_NO_ERROR) && r3->ocr.Card_Busy)
			return CARD_TYPE_MMC;
	}
	
	return CARD_TYPE_NONE;
}
#endif
//Check if any card is inserted according to pull up resistor
int sd_mmc_check_insert(SD_MMC_Card_Info_t *sd_mmc_info)
{
	int level;
	
	if(sd_mmc_info->sd_mmc_get_ins)
	{
		level = sd_mmc_info->sd_mmc_get_ins();
	}
	else
	{	
		sd_set_ins_input();
		level = sd_get_ins_value();
	}

	if (level)
	{
		if(sd_mmc_info->init_retry)
		{
			sd_mmc_power_off(sd_mmc_info);
			sd_mmc_info->init_retry = 0;
		}
		if(sd_mmc_info->inited_flag)
		{
			sd_mmc_power_off(sd_mmc_info);
			sd_mmc_info->removed_flag = 1;
			sd_mmc_info->inited_flag = 0;
		}

		return 0;       //No card is inserted
	}
	else
	{
		return 1;       //A card is inserted
	}
}

/*
int sd_mmc_access_boot_area(unsigned partition_num)
{
	int ret = 0;
	unsigned cmd_argument;
	unsigned char response[MAX_RESPONSE_BYTES];
	unsigned char mmc_ext_csd_buf[512];
	MMC_REG_EXT_CSD_t *mmc_ext_csd_reg;

	if (!sd_mmc_info->emmc_boot_support)
		return SD_ERROR_SWITCH_FUNCTION_COMUNICATION;

	if (partition_num == 2)
		cmd_argument = 0x03b31200;
	else if (partition_num == 1)
		cmd_argument = 0x03b30900;
	else
		cmd_argument = 0x03b30000;
    if(sd_mmc_info->mmc_spec_version == SPEC_VERSION_40_41)
    {
    	sd_delay_ms(2);
        ret = sd_send_cmd_hw(MMC_SWITCH_FUNTION, cmd_argument, RESPONSE_R1, response, 0, 0, 1);
        if(ret)
            return ret;

        sd_delay_ms(2);
        ret = sd_send_cmd_hw(MMC_SEND_EXT_CSD, 0, RESPONSE_R1, response, mmc_ext_csd_buf, sizeof(MMC_REG_EXT_CSD_t), 1);
        if(ret)
            return ret;
        mmc_ext_csd_reg = (MMC_REG_EXT_CSD_t *)mmc_ext_csd_buf;
        if (!mmc_ext_csd_reg->PARTITION_CONFIG)
        	return SD_ERROR_SWITCH_FUNCTION_COMUNICATION;
    }

	return SD_MMC_NO_ERROR;
}*/

//Read data from SD/MMC card
int sd_mmc_read_data(SD_MMC_Card_Info_t *sd_mmc_info, unsigned long lba, unsigned long byte_cnt, unsigned char * data_buf)
{
	int error = 0;
	unsigned long lba_nums;

#ifdef SD_MMC_HW_CONTROL
	if(SD_WORK_MODE == CARD_HW_MODE)
	{
		//sd_sdio_enable(sd_mmc_info->io_pad_type);
		if (sd_mmc_info->sd_save_hw_io_flag) {
	    		WRITE_CBUS_REG(SDIO_CONFIG, sd_mmc_info->sd_save_hw_io_config);
	      		WRITE_CBUS_REG(SDIO_MULT_CONFIG, sd_mmc_info->sd_save_hw_io_mult_config);
	    	}		
	}
#endif       
#ifdef SD_MMC_SW_CONTROL
	if(SD_WORK_MODE == CARD_SW_MODE)
	   	sd_mmc_io_config(sd_mmc_info);
#endif

	lba_nums = sd_mmc_info->blk_len;
	lba_nums = (byte_cnt + sd_mmc_info->blk_len - 1) / lba_nums;
	if(lba_nums == 0)
	{
		error = SD_MMC_ERROR_BLOCK_LEN;
#ifdef  SD_MMC_DEBUG
		Debug_Printf("#%s error occured in sd_mmc_read_data()\n", sd_error_to_string(error));
#endif
		return error;
	}
	else if ((lba_nums == 1) && !sd_mmc_info->single_blk_failed)
	{
#ifdef SD_MMC_HW_CONTROL
		if(SD_WORK_MODE == CARD_HW_MODE)
			error = sd_read_single_block_hw(sd_mmc_info, lba, data_buf);
#endif 
#ifdef SD_MMC_SW_CONTROL
		if(SD_WORK_MODE == CARD_SW_MODE)
					error = sd_read_single_block_sw(sd_mmc_info, lba, data_buf);
#endif 		
		if(error)
		{			
#ifdef  SD_MMC_DEBUG
			Debug_Printf("#%s error occured in sd_read_single_block()\n", sd_error_to_string(error));
#endif
			return error;
		}
	}
	else
	{		
#ifdef SD_MMC_HW_CONTROL
		if(SD_WORK_MODE == CARD_HW_MODE)
		{
			error = sd_read_multi_block_hw(sd_mmc_info, lba, lba_nums, data_buf);
			if(error)
				error = sd_read_multi_block_hw(sd_mmc_info, lba, lba_nums, data_buf);
		}
#endif 
#ifdef SD_MMC_SW_CONTROL
		if(SD_WORK_MODE == CARD_SW_MODE)
			error = sd_read_multi_block_sw(sd_mmc_info, lba, lba_nums, data_buf);
#endif		

		if(error)
		{			
#ifdef  SD_MMC_DEBUG
			Debug_Printf("#%s error occured in sd_read_multi_block()\n", sd_error_to_string(error));
#endif
			return error;
		}
	}	
	
	return SD_MMC_NO_ERROR;
}

//Write data to SD/MMC card
int sd_mmc_write_data(SD_MMC_Card_Info_t *sd_mmc_info, unsigned long lba, unsigned long byte_cnt, unsigned char * data_buf)
{
	int error = 0;
	unsigned long lba_nums;

	if(sd_mmc_info->write_protected_flag)
	{
		error = SD_MMC_ERROR_WRITE_PROTECTED;
#ifdef  SD_MMC_DEBUG
		Debug_Printf("#%s error occured in sd_mmc_write_data()\n", sd_error_to_string(error));
#endif
		return error;
	}

#ifdef SD_MMC_HW_CONTROL
	if(SD_WORK_MODE == CARD_HW_MODE)
	{
		//sd_sdio_enable(sd_mmc_info->io_pad_type);
		if (sd_mmc_info->sd_save_hw_io_flag) {
    		WRITE_CBUS_REG(SDIO_CONFIG, sd_mmc_info->sd_save_hw_io_config);
      		WRITE_CBUS_REG(SDIO_MULT_CONFIG, sd_mmc_info->sd_save_hw_io_mult_config);
    	}		
	}
#endif        	
#ifdef SD_MMC_SW_CONTROL
	if(SD_WORK_MODE == CARD_SW_MODE)    	    
       	sd_mmc_io_config(sd_mmc_info);	  	
#endif

	//memcpy(sd_write_buf, data_buf, byte_cnt);
	lba_nums = sd_mmc_info->blk_len;
	lba_nums = (byte_cnt + sd_mmc_info->blk_len - 1) / lba_nums;
	if(lba_nums == 0)
	{
		error = SD_MMC_ERROR_BLOCK_LEN;
#ifdef  SD_MMC_DEBUG
		Debug_Printf("#%s error occured in sd_mmc_write_data()\n", sd_error_to_string(error));
#endif
		return error;
	}
	else if ((lba_nums == 1) && !sd_mmc_info->single_blk_failed)
	{
#ifdef SD_MMC_HW_CONTROL
		if(SD_WORK_MODE == CARD_HW_MODE)
			error = sd_write_single_block_hw(sd_mmc_info, lba, data_buf);
#endif 
#ifdef SD_MMC_SW_CONTROL
		if(SD_WORK_MODE == CARD_SW_MODE)
			error = sd_write_single_block_sw(sd_mmc_info, lba, data_buf);
#endif		
		if(error)
		{
#ifdef  SD_MMC_DEBUG
			Debug_Printf("#%s error occured in sd_write_single_block()\n", sd_error_to_string(error));
#endif
			return error;
		}
	}
	else
	{
#ifdef SD_MMC_HW_CONTROL
		if(SD_WORK_MODE == CARD_HW_MODE)
			error = sd_write_multi_block_hw(sd_mmc_info, lba, lba_nums, data_buf);
#endif 
#ifdef SD_MMC_SW_CONTROL
		if(SD_WORK_MODE == CARD_SW_MODE)
			error = sd_write_multi_block_sw(sd_mmc_info, lba, lba_nums, data_buf);
#endif		
		if(error)
		{
#ifdef  SD_MMC_DEBUG
			Debug_Printf("#%s error occured in sd_write_multi_block()\n", sd_error_to_string(error));
#endif
			return error;
		}
	}
	
	return SD_MMC_NO_ERROR;
}

#ifdef SD_MMC_SW_CONTROL
int sd_mmc_cmd_test(SD_MMC_Card_Info_t *sd_mmc_info)
{
	int ret;
	unsigned char response[MAX_RESPONSE_BYTES];

	sd_clear_response(response);
	
	sd_mmc_io_config(sd_mmc_info);
	
	ret = sd_send_cmd_sw(sd_mmc_info, SD_MMC_SEND_STATUS, sd_mmc_info->card_rca<<16, RESPONSE_R1, response);   
	
	return ret;
}
#endif 

void sd_mmc_power_on(SD_MMC_Card_Info_t *sd_mmc_info)
{
	sd_delay_ms(sd_mmc_info->sd_mmc_power_delay+1);
	
#ifdef SD_MMC_POWER_CONTROL

	if(sd_mmc_info->sd_mmc_power)
	{
		sd_mmc_info->sd_mmc_power(0);
	}
	else
	{
		sd_set_disable();
	}
	sd_delay_ms(500);

	if(sd_mmc_info->sd_mmc_power)
	{
		if(sd_mmc_check_insert(sd_mmc_info)) //ensure card wasn't removed at this time 
		{
			sd_mmc_info->sd_mmc_power(1);
		}
	}
	else
	{
		if(sd_mmc_check_insert(sd_mmc_info)) //ensure card wasn't removed at this time 
		{
			sd_set_enable();
		}
	}
	sd_delay_ms(200);
#else
	sd_delay_ms(10);
#endif
}

void sd_mmc_power_off(SD_MMC_Card_Info_t *sd_mmc_info)
{
#ifdef SD_MMC_POWER_CONTROL
	if(sd_mmc_info->sd_mmc_power)
	{
		sd_mmc_info->sd_mmc_power(0);
	}
	else
	{
		sd_set_disable();
	}
#endif
}

//Check if Write-Protected switch is on
int sd_mmc_check_wp(SD_MMC_Card_Info_t *sd_mmc_info)
{
#ifdef SD_MMC_WP_CHECK
	int ret = 0;

	if(sd_mmc_info->sd_get_wp)
	{
		ret = sd_mmc_info->sd_get_wp();
	}
	else
	{	
		sd_set_wp_input();
		ret = sd_get_wp_value();
	}
		
	if (ret)
	{
		sd_mmc_info->write_protected_flag = 1;
		
		return 1;       //switch is on
	}
	else
	{
		return 0;       //switch is off
	}
#else
	return 0;
#endif
}

/*
#define SPI_DEV "/dev/mtd0"
int emmc_test_upgrade(void)
{
	int i = 0, fd, error, read_count;
	unsigned char *emmc_wite_buf;
	unsigned char *emmc_read_buf;
	emmc_wite_buf = (unsigned char *)sd_mmc_malloc(8192, GFP_KERNEL);
	emmc_read_buf = (unsigned char *)sd_mmc_malloc(8192, GFP_KERNEL);
	if ((!emmc_wite_buf) || (!emmc_read_buf))
		return -1;

    error = sd_mmc_access_boot_area(0);
    if (error)
    	printk("emmc access_boot_area error\n");
    error = sd_mmc_read_data(0, sd_mmc_info->blk_len*8, emmc_wite_buf);
	if (error)
		return error;
	fd = sys_open(SPI_DEV, O_RDWR, 0);
	if (fd < 0) {
		printk("spi open error\n");
		return -1;
	}
	for (i=0; i<0x50000; i+=8192)
	{
		error = sys_lseek(fd, i, 0);
		read_count = sys_read(fd, emmc_wite_buf, 8192);
		if (read_count != 8192)
			return -1;

		error = sd_mmc_write_data(i/512, 8192, emmc_wite_buf);
		if (error)
			return error;
	    error = sd_mmc_read_data(i/512, 8192, emmc_read_buf);
	    if (error)
		    return error;
		if(memcmp(emmc_wite_buf, emmc_read_buf, 8192))
			printk("emmc upgrade error at addr %x \n", i);
	}
	sd_mmc_access_boot_area(0);
	sd_mmc_free(emmc_wite_buf);
	sd_mmc_free(emmc_read_buf);
	printk("emmc upgrade completely\n");

	return SD_MMC_NO_ERROR;
} */

//check data lines consistency
int sd_check_data_consistency(SD_MMC_Card_Info_t *sd_mmc_info)
{
	int error, func_num=0;
	unsigned char read_reg_data;
	unsigned read_addr, sdio_cis_addr;
#ifdef SD_MMC_SW_CONTROL
	unsigned char response[MAX_RESPONSE_BYTES];
#endif
	
	unsigned char *mbr_buf = sd_mmc_info->sd_mmc_buf;

	//This card is working in wide bus mode!
	memset(mbr_buf, 0, sd_mmc_info->blk_len);
	if(sd_mmc_info->bus_width == SD_BUS_WIDE)
	{
		if(sd_mmc_info->card_type == CARD_TYPE_SDIO)
		{
			for (func_num=0; func_num<sd_mmc_info->sdio_function_nums; func_num++) {

				read_addr = ((func_num << 8) | Common_CIS_Pointer3_REG);
				error = sdio_read_reg(sd_mmc_info, 0, read_addr, &read_reg_data);
			if(error)
				return error;
				sdio_cis_addr = read_reg_data;

				read_addr = ((func_num << 8) | Common_CIS_Pointer2_REG);
				error = sdio_read_reg(sd_mmc_info, 0, read_addr, &read_reg_data);
			if(error)
				return error;
				sdio_cis_addr = ((sdio_cis_addr << 8) | read_reg_data);

				read_addr = ((func_num << 8) | Common_CIS_Pointer1_REG);
				error = sdio_read_reg(sd_mmc_info, 0, read_addr, &read_reg_data);
				if (error)
					return error;
				sdio_cis_addr = ((sdio_cis_addr << 8) | read_reg_data);
				sd_mmc_info->sdio_cis_addr[func_num] = sdio_cis_addr;
				//printk("sdio cis addr is %x \n", sdio_cis_addr);

			}

			return SD_MMC_NO_ERROR;
		}
		//read MBR information
		error = sd_mmc_read_data(sd_mmc_info, 0, sd_mmc_info->blk_len, mbr_buf);
		if(error)
		{
			//error! retry again!
			error = sd_mmc_read_data(sd_mmc_info, 0, sd_mmc_info->blk_len, mbr_buf);
			if(error)
				return error;
		}
		
		//check MBR data consistency
		if((mbr_buf[510] != 0x55) || (mbr_buf[511] != 0xAA))
		{
			//data consistency error! retry again!
			error = sd_mmc_read_data(sd_mmc_info, 0, sd_mmc_info->blk_len*2, mbr_buf);
			if(error)
				return error;

			//check MBR data consistency
			if((mbr_buf[510] != 0x55) || (mbr_buf[511] != 0xAA))
			{
#ifdef  SD_MMC_DEBUG
				Debug_Printf("SD/MMC data consistency error in Wide Bus mode! Try Single Bus mode...\n");
#endif

				//error again! retry single bus mode!
#ifdef SD_MMC_SW_CONTROL
				if(SD_WORK_MODE == CARD_SW_MODE)	
				{				
					error = sd_send_cmd_sw(sd_mmc_info, SD_APP_CMD, sd_mmc_info->card_rca<<16, RESPONSE_R1, response);
					if(error)
						return error;
					error = sd_send_cmd_sw(sd_mmc_info, SD_SET_BUS_WIDTHS, 0, RESPONSE_R1, response); //0 1bit, 10=4bits
					if(error)
						return error;
				}
#endif					

				sd_mmc_info->bus_width = SD_BUS_SINGLE;
#ifdef SD_MMC_HW_CONTROL
				if(SD_WORK_MODE == CARD_HW_MODE)
				{
					unsigned long sdio_config = 0;
					SDIO_Config_Reg_t *config_reg = (void *)&sdio_config;
					sdio_config = READ_CBUS_REG(SDIO_CONFIG);
					config_reg->bus_width = 0;
					WRITE_CBUS_REG(SDIO_CONFIG, sdio_config);
				}
#endif
			}
			else
			{
				sd_mmc_info->single_blk_failed = 1;
				return SD_MMC_NO_ERROR;
			}
		}
		else
		{
			return SD_MMC_NO_ERROR;
		}
	}

	//This card is working in single bus mode!
	memset(mbr_buf, 0, sd_mmc_info->blk_len);
	//read MBR information
	error = sd_mmc_read_data(sd_mmc_info, 0, sd_mmc_info->blk_len, mbr_buf);
	//For Kingston MMC mobile card
	error = sd_mmc_read_data(sd_mmc_info, 0, sd_mmc_info->blk_len, mbr_buf);
	if(error)
	{
		//error! retry again!
		error = sd_mmc_read_data(sd_mmc_info, 0, sd_mmc_info->blk_len, mbr_buf);
		if(error)
			return error;
	}
		
	//check MBR data consistency
	if((mbr_buf[510] != 0x55) || (mbr_buf[511] != 0xAA))
	{
		//data consistency error! retry again!
		error = sd_mmc_read_data(sd_mmc_info, 0, sd_mmc_info->blk_len*2, mbr_buf);
		if(error)
			return error;

		//check MBR data consistency
		if((mbr_buf[510] != 0x55) || (mbr_buf[511] != 0xAA))
		{
			return SD_MMC_ERROR_DATA_CRC;
		}
		else
		{
			sd_mmc_info->single_blk_failed = 1;
		}
	}

	return SD_MMC_NO_ERROR;
}

void sd_mmc_exit(SD_MMC_Card_Info_t *sd_mmc_info)
{
	if(sd_mmc_info->sd_mmc_io_release != NULL)
		sd_mmc_info->sd_mmc_io_release();

	if(sd_mmc_info->card_type == CARD_TYPE_SD)
		Debug_Printf("SD card unpluged!\n\n");
	else if(sd_mmc_info->card_type == CARD_TYPE_SDHC)
		Debug_Printf("SDHC card unpluged!\n\n");
	else if(sd_mmc_info->card_type == CARD_TYPE_SDIO)
		Debug_Printf("SDIO card unpluged!\n\n");
	else
		Debug_Printf("MMC card unpluged!\n\n");

	return;
}

void sd_mmc_prepare_init(SD_MMC_Card_Info_t *sd_mmc_info)
{
	if(sd_mmc_power_register != NULL)
		sd_mmc_info->sd_mmc_power = sd_mmc_power_register;
	if(sd_mmc_ins_register != NULL)	
		sd_mmc_info->sd_mmc_get_ins = sd_mmc_ins_register;
	if(sd_mmc_wp_register != NULL)	
		sd_mmc_info->sd_get_wp = sd_mmc_wp_register;
	if(sd_mmc_io_release_register != NULL)
		sd_mmc_info->sd_mmc_io_release = sd_mmc_io_release_register;
}

void sd_mmc_prepare_power(SD_MMC_Card_Info_t *sd_mmc_info)
{
    sd_gpio_enable(sd_mmc_info->io_pad_type);

	sd_set_cmd_output();
	sd_set_cmd_value(0);
	sd_set_clk_output();
	sd_set_clk_low();
	sd_set_dat0_3_output();
	sd_set_dat0_3_value(0);
}

void sd_mmc_set_input(SD_MMC_Card_Info_t *sd_mmc_info)
{
	sd_set_cmd_input();
	sd_set_clk_input();
	sd_set_dat0_3_input();
}

int sd_mmc_switch_function(SD_MMC_Card_Info_t *sd_mmc_info)
{
	int ret;
	unsigned char response[MAX_RESPONSE_BYTES];
	unsigned char *status_data_buf = sd_mmc_info->sd_mmc_buf;
	unsigned char *mmc_ext_csd_buf = sd_mmc_info->sd_mmc_buf;
	SD_Switch_Function_Status_t *switch_funtion_status;
	MMC_REG_EXT_CSD_t *mmc_ext_csd_reg;
//	int i;
	unsigned char read_reg_data;
	unsigned char write_reg_data;
	unsigned long sdio_config = 0;
	SDIO_Config_Reg_t *config_reg = NULL;

	if(sd_mmc_info->spec_version || sd_mmc_info->card_type == CARD_TYPE_SDHC)
	{    

	    memset(status_data_buf, 0, 64);
		ret = sd_send_cmd_hw(sd_mmc_info, SD_SWITCH_FUNCTION, 0x00FFFF01, RESPONSE_R1, response, status_data_buf, 64, 1);
	    if(ret)
		    return ret;

	    switch_funtion_status = (SD_Switch_Function_Status_t *)status_data_buf;
	    if(switch_funtion_status->Max_Current_Consumption == 0)
#if 0        
		    return SD_ERROR_SWITCH_FUNCTION_COMUNICATION;
#else
        //
        // ddd
        {
            msleep( 100 );
            memset(status_data_buf, 0, 64);
            ret = sd_send_cmd_hw(sd_mmc_info, SD_SWITCH_FUNCTION, 0x00FFFF01, RESPONSE_R1, response, status_data_buf, 64, 1);
            if(ret)
                return ret;

            switch_funtion_status = (SD_Switch_Function_Status_t *)status_data_buf;
            if(switch_funtion_status->Max_Current_Consumption == 0) {
                printk( "switch err 1\n" );
                return SD_ERROR_SWITCH_FUNCTION_COMUNICATION;
            }

        }

#endif // ddd        

	    if(!((switch_funtion_status->Function_Group[5]>>8) & 0x02))
	    {	
		    return SD_ERROR_NO_FUNCTION_SWITCH;
	    }
	    else	
	    {
		    memset(status_data_buf, 0, 64);
			ret = sd_send_cmd_hw(sd_mmc_info, SD_SWITCH_FUNCTION, 0x80FFFF01, RESPONSE_R1, response, status_data_buf, 64, 1);	
		    if(ret)
			    return ret;

		    switch_funtion_status = (SD_Switch_Function_Status_t *)status_data_buf;
		    if(switch_funtion_status->Max_Current_Consumption == 0 || switch_funtion_status->Function_Group_Status1 != 0x01) 
#if 0
			    return SD_ERROR_SWITCH_FUNCTION_COMUNICATION;
#else
            //
            // ddd
            {
                msleep( 100 );
                memset(status_data_buf, 0, 64);
                ret = sd_send_cmd_hw(sd_mmc_info, SD_SWITCH_FUNCTION, 0x80FFFF01, RESPONSE_R1, response, status_data_buf, 64, 1);	
                if(ret)
                    return ret;

                switch_funtion_status = (SD_Switch_Function_Status_t *)status_data_buf;
                if(switch_funtion_status->Max_Current_Consumption == 0 || switch_funtion_status->Function_Group_Status1 != 0x01) {
                    printk( "switch err 2" );
                    return SD_ERROR_SWITCH_FUNCTION_COMUNICATION;
                }
            }
#endif // ddd

			sdio_config = 0;
			config_reg = (void *)&sdio_config;
			sdio_config = READ_CBUS_REG(SDIO_CONFIG);
			config_reg->cmd_clk_divide = 1;//aml_system_clk / (2*SD_MMC_TRANSFER_HIGHSPEED_CLK) -1;
			WRITE_CBUS_REG(SDIO_CONFIG, sdio_config);

			sd_mmc_info->sdio_clk_unit = 1000/SD_MMC_TRANSFER_HIGHSPEED_CLK;
            sd_mmc_info->speed_class = HIGH_SPEED;		
	    }
	}
    else if(sd_mmc_info->mmc_spec_version == SPEC_VERSION_40_41)
    {
    	sd_delay_ms(2);	  
    	 
        ret = sd_send_cmd_hw(sd_mmc_info,MMC_SEND_EXT_CSD, 0, RESPONSE_R1, response, mmc_ext_csd_buf, sizeof(MMC_REG_EXT_CSD_t), 1);
        if(ret)
            return ret;
            
        mmc_ext_csd_reg = (MMC_REG_EXT_CSD_t *)mmc_ext_csd_buf;
        if (mmc_ext_csd_reg->PARTITIONING_SUPPORT)
    	{
        	sd_mmc_info->emmc_boot_support = 1;
        	sd_mmc_info->emmc_boot_partition_size[0] = sd_mmc_info->emmc_boot_partition_size[1] = mmc_ext_csd_reg->BOOT_SIZE_MULTI * EMMC_BOOT_SIZE_UNIT;
        }
        ret = sd_send_cmd_hw(sd_mmc_info, MMC_SWITCH_FUNTION, 0x03b90100, RESPONSE_R1, response, 0, 0, 1);
        if(ret)
            return ret;

        sdio_config = 0;
        config_reg = (void *)&sdio_config;
        sdio_config = READ_CBUS_REG(SDIO_CONFIG);
        config_reg->cmd_clk_divide = 3;
        WRITE_CBUS_REG(SDIO_CONFIG, sdio_config);

		sd_mmc_info->sdio_clk_unit = 1000/SD_MMC_TRANSFER_HIGHSPEED_CLK;
        sd_mmc_info->speed_class = HIGH_SPEED;	
    }
    else if(sd_mmc_info->card_type == CARD_TYPE_SDIO)
    {
		ret = sdio_read_reg(sd_mmc_info, 0, High_Speed_REG, &read_reg_data);
    	if(read_reg_data & SDIO_Support_High_Speed & (!ret))
    	{
			write_reg_data = ((read_reg_data & 0xfd) | SDIO_Enable_High_Speed);
			ret = sdio_write_reg(sd_mmc_info, 0, High_Speed_REG, &write_reg_data, SDIO_Read_After_Write);
			if(!ret) {

        	sdio_config = 0;
        	config_reg = (void *)&sdio_config;
        	sdio_config = READ_CBUS_REG(SDIO_CONFIG);
        	config_reg->cmd_clk_divide =2;
        	WRITE_CBUS_REG(SDIO_CONFIG, sdio_config);

			sd_mmc_info->sdio_clk_unit = 1000/SD_MMC_TRANSFER_HIGHSPEED_CLK;
        	sd_mmc_info->speed_class = HIGH_SPEED;
        }
        }

		sdio_config = 0;
        config_reg = (void *)&sdio_config;
        sdio_config = READ_CBUS_REG(SDIO_CONFIG);
        config_reg->cmd_clk_divide = 5;
        WRITE_CBUS_REG(SDIO_CONFIG, sdio_config);

		sd_mmc_info->sdio_clk_unit = 1000/SD_MMC_TRANSFER_CLK;
    }

	if(!config_reg){
		sdio_config = READ_CBUS_REG(SDIO_CONFIG);
		config_reg = (void *)&sdio_config;
	}
	printk("set sd_mmc config_reg->cmd_clk_divide %d, CLK %ldM\n", 
		config_reg->cmd_clk_divide, 
		clk_get_rate(clk_get_sys("clk81", NULL))/2000000/(config_reg->cmd_clk_divide + 1));
	
	return SD_MMC_NO_ERROR;
}

int sd_check_sdio_card_type(SD_MMC_Card_Info_t *sd_mmc_info)
{
	int error, function_no, timeout_count = SDIO_FUNCTION_TIMEOUT;
	unsigned char read_reg_data;
	unsigned char write_reg_data;
	unsigned write_addr;

	for(function_no=1; function_no<sd_mmc_info->sdio_function_nums; function_no++)
	{
		sd_mmc_info->sdio_blk_len[function_no] = 1;
		read_reg_data = 0;
		error = sdio_read_reg(sd_mmc_info, 0, function_no<<8, &read_reg_data);
		if(error)
			return error;

		switch(read_reg_data) {

			case 0:
				sd_mmc_info->sdio_card_type[function_no] = CARD_TYPE_NONE_SDIO;
				break;
				case 1:
				sd_mmc_info->sdio_card_type[function_no] = CARD_TYPE_SDIO_STD_UART;
					break;
				case 2:
				sd_mmc_info->sdio_card_type[function_no] = CARD_TYPE_SDIO_BT_TYPEA;
					break;
				case 3:
				sd_mmc_info->sdio_card_type[function_no] = CARD_TYPE_SDIO_BT_TYPEB;
					break;
				case 4:
				sd_mmc_info->sdio_card_type[function_no] = CARD_TYPE_SDIO_GPS;
					break;
				case 5:
				sd_mmc_info->sdio_card_type[function_no] = CARD_TYPE_SDIO_CAMERA;
					break;
				case 6:
				sd_mmc_info->sdio_card_type[function_no] = CARD_TYPE_SDIO_PHS;
					break;	
				case 7:
				sd_mmc_info->sdio_card_type[function_no] = CARD_TYPE_SDIO_WLAN;
					break;
				case 8:
				case 9:
				case 10:
				case 11:
				case 12:
				case 13:
				case 14:
				sd_mmc_info->sdio_card_type[function_no] = CARD_TYPE_NONE;
					break;
				case 15:
				sd_mmc_info->sdio_card_type[function_no] = CARD_TYPE_SDIO_OTHER_IF;
					break;
				default:
					break;
			}
	}

	error = sdio_read_reg(sd_mmc_info, 0, IO_READY_REG, &write_reg_data);
	if(error)
		return error;

	//function enable would do at up layer
	/*write_reg_data |= 2;//((1<<sd_mmc_info->sdio_function_nums)-2);
    error = sdio_write_reg(sd_mmc_info, 0, IO_ENABLE_REG, &write_reg_data, SDIO_Read_After_Write);
    if(error)
    	return error;	

	while(timeout_count--)
	{
		error = sdio_read_reg(sd_mmc_info, 0, IO_READY_REG, &read_reg_data);
		if(error)
			return error;

		if(read_reg_data == write_reg_data)
			break;
		else
			sd_delay_ms(1);
	}

#ifdef  SD_MMC_DEBUG
	Debug_Printf("#read_reg_data %x timeout_count %d \n", read_reg_data, timeout_count);
#endif*/

	if(timeout_count > 0)
	{
		error = sdio_read_reg(sd_mmc_info, 0, Card_Capability_REG, &read_reg_data);
		if(error)
			return error;

		if((read_reg_data) & SDIO_Support_Multi_Block)
		{
			if(SDIO_BLOCK_SIZE&0xFF)
			{
				write_reg_data = (SDIO_BLOCK_SIZE & 0xFF);
				error = sdio_write_reg(sd_mmc_info, 0, FN0_Block_Size_Low_REG, &write_reg_data, SDIO_Read_After_Write);
				if(error)
				{
					sd_mmc_info->sdio_blk_len[0] = 1;
					return SD_MMC_NO_ERROR;
				}

				write_addr = ((2 << 8) | 0x10);
				error = sdio_write_reg(sd_mmc_info, 0, write_addr, &write_reg_data, SDIO_Read_After_Write);
				if(error)
				{
					sd_mmc_info->sdio_blk_len[2] = 1;
					return SD_MMC_NO_ERROR;
				}
			}

			write_reg_data = (SDIO_BLOCK_SIZE>>8);
			error = sdio_write_reg(sd_mmc_info, 0, FN0_Block_Size_High_REG, &write_reg_data, SDIO_Read_After_Write);
			if(error)
			{
				sd_mmc_info->sdio_blk_len[0] = 1;
			}
			else
			{
				sd_mmc_info->sdio_blk_len[0] = SDIO_BLOCK_SIZE;
			}

			write_addr = ((2 << 8) | 0x11);
			error = sdio_write_reg(sd_mmc_info, 0, write_addr, &write_reg_data, SDIO_Read_After_Write);
			if(error)
			{
				sd_mmc_info->sdio_blk_len[1] = 1;
			}
			else
			{
				sd_mmc_info->sdio_blk_len[2] = SDIO_BLOCK_SIZE;
			}
		}
		else
		{
			return SD_SDIO_ERROR_NO_FUNCTION;
		}
	}
	else
	{
		return SD_SDIO_ERROR_NO_FUNCTION;
	}

	if(error)
	{
#ifdef  SD_MMC_DEBUG
		Debug_Printf("#%s error occured in sdio_open_interrupt()()\n", sd_error_to_string(error));
#endif
		//return error;
	}

	return SD_MMC_NO_ERROR;
}

int sdio_read_reg(SD_MMC_Card_Info_t *sd_mmc_info, int function_no, unsigned long sdio_register, unsigned char *reg_data)
{
	int ret = 0;
	unsigned long sdio_direct_rw = 0;
	unsigned char response[MAX_RESPONSE_BYTES];
	SDIO_IO_RW_CMD_ARG_t *sdio_io_direct_rw;
	SDIO_RW_CMD_Response_R5_t * sdio_rw_response = (SDIO_RW_CMD_Response_R5_t *)response;

#ifdef SD_MMC_HW_CONTROL
	if(SD_WORK_MODE == CARD_HW_MODE)
	{
		//sd_sdio_enable(sd_mmc_info->io_pad_type);
		if (sd_mmc_info->sd_save_hw_io_flag) {
    		WRITE_CBUS_REG(SDIO_CONFIG, sd_mmc_info->sd_save_hw_io_config);
      		WRITE_CBUS_REG(SDIO_MULT_CONFIG, sd_mmc_info->sd_save_hw_io_mult_config);
      	}	
	}
#endif       
#ifdef SD_MMC_SW_CONTROL
	if(SD_WORK_MODE == CARD_SW_MODE)
		sd_mmc_io_config(sd_mmc_info);
#endif

	sdio_io_direct_rw = (void *)&sdio_direct_rw;

	sdio_io_direct_rw->Function_No = function_no;
	sdio_io_direct_rw->Register_Address = (sdio_register & 0x1FFFF);

#ifdef SD_MMC_HW_CONTROL
	if(SD_WORK_MODE == CARD_HW_MODE)
		ret = sd_send_cmd_hw(sd_mmc_info, IO_RW_DIRECT, sdio_direct_rw, RESPONSE_R5, response, 0, 0, 1);
#endif	
#ifdef SD_MMC_SW_CONTROL
	if(SD_WORK_MODE == CARD_SW_MODE)
		ret = sd_send_cmd_sw(sd_mmc_info, IO_RW_DIRECT, sdio_direct_rw, RESPONSE_R5, response);
#endif
	if(ret)
		return ret;

	*reg_data = sdio_rw_response->read_or_write_data;

	return SD_MMC_NO_ERROR;
}

int sdio_write_reg(SD_MMC_Card_Info_t *sd_mmc_info, int function_no, unsigned int sdio_register, unsigned char *reg_data, unsigned read_after_write_flag)
{
	int ret = 0;
	unsigned long sdio_direct_rw = 0;
	SDIO_IO_RW_CMD_ARG_t *sdio_io_direct_rw;
	unsigned char response[MAX_RESPONSE_BYTES];
	SDIO_RW_CMD_Response_R5_t * sdio_rw_response = (SDIO_RW_CMD_Response_R5_t *)response;

#ifdef SD_MMC_HW_CONTROL
	if(SD_WORK_MODE == CARD_HW_MODE)
	{
		//sd_sdio_enable(sd_mmc_info->io_pad_type);
		if (sd_mmc_info->sd_save_hw_io_flag) {
    		WRITE_CBUS_REG(SDIO_CONFIG, sd_mmc_info->sd_save_hw_io_config);
      		WRITE_CBUS_REG(SDIO_MULT_CONFIG, sd_mmc_info->sd_save_hw_io_mult_config);
      	}		
	}
#endif       
#ifdef SD_MMC_SW_CONTROL
	if(SD_WORK_MODE == CARD_SW_MODE)
		sd_mmc_io_config(sd_mmc_info);
#endif

	sdio_io_direct_rw = (void *)&sdio_direct_rw;

    sdio_io_direct_rw->R_W_Flag = SDIO_Write_Data;
    sdio_io_direct_rw->RAW_Flag = read_after_write_flag;
	sdio_io_direct_rw->Function_No = function_no;
	sdio_io_direct_rw->write_data_bytes = (*reg_data);
	sdio_io_direct_rw->Register_Address = (sdio_register & 0x1FFFF);

#ifdef SD_MMC_HW_CONTROL
	if(SD_WORK_MODE == CARD_HW_MODE)
		ret = sd_send_cmd_hw(sd_mmc_info, IO_RW_DIRECT, sdio_direct_rw, RESPONSE_R5, response, 0, 0, 0);
#endif	
#ifdef SD_MMC_SW_CONTROL
	if(SD_WORK_MODE == CARD_SW_MODE)
		ret = sd_send_cmd_sw(sd_mmc_info, IO_RW_DIRECT, sdio_direct_rw, RESPONSE_R5, response);
#endif

#ifdef  SD_MMC_DEBUG
	//Debug_Printf("#sdio_write_reg write: %x addr: %x read: %x function_no %d\n", (*reg_data), sdio_register, sdio_rw_response->read_or_write_data, function_no);
#endif

	if(ret)
		return ret;

	if(read_after_write_flag && (sdio_rw_response->read_or_write_data != (*reg_data))) {
#ifdef  SD_MMC_DEBUG
		Debug_Printf("#this sdio card could not support read after write\n");
#endif
		return SD_MMC_NO_ERROR;
	}

	return SD_MMC_NO_ERROR;
}

int sdio_card_reset(SD_MMC_Card_Info_t *sd_mmc_info)
{
	int ret;
	unsigned char write_reg_data;

	write_reg_data = SDIO_RES_bit;
    ret = sdio_write_reg(sd_mmc_info, 0, IO_ABORT_REG, &write_reg_data, SDIO_DONT_Read_After_Write);
    if(ret)
    	return ret;

	return SD_MMC_NO_ERROR;
}

int sdio_data_transfer_abort(SD_MMC_Card_Info_t *sd_mmc_info, int function_no)
{
	int ret;
	unsigned char write_reg_data;

	write_reg_data = function_no;
    ret = sdio_write_reg(sd_mmc_info, 0, IO_ABORT_REG, &write_reg_data, SDIO_DONT_Read_After_Write);
    if(ret)
    	return ret;

	return SD_MMC_NO_ERROR;
}

int sdio_read_data_block_hw(SD_MMC_Card_Info_t *sd_mmc_info, int function_no, int buf_or_fifo, unsigned long sdio_addr, unsigned long block_count, unsigned char *data_buf)
{
	int ret;
	unsigned char response[MAX_RESPONSE_BYTES];
	unsigned long read_block_count, data_offset = 0;

	unsigned long sdio_extend_rw = 0;
	SDIO_IO_RW_EXTENDED_ARG *sdio_io_extend_rw = (void *)&sdio_extend_rw;

	while(block_count)
	{
		if(block_count > sd_mmc_info->max_blk_count)
			read_block_count = sd_mmc_info->max_blk_count;
		else
			read_block_count = block_count;

		sdio_io_extend_rw->R_W_Flag = SDIO_Read_Data;
		sdio_io_extend_rw->Block_Mode = SDIO_Block_MODE;
		sdio_io_extend_rw->OP_Code = buf_or_fifo;
		sdio_io_extend_rw->Function_No = function_no;
		sdio_io_extend_rw->Byte_Block_Count = read_block_count;
		sdio_io_extend_rw->Register_Address = ((sdio_addr+data_offset) & 0x1FFFF);

		ret = sd_send_cmd_hw(sd_mmc_info, IO_RW_EXTENDED, sdio_extend_rw, RESPONSE_R5, response, data_buf+data_offset, read_block_count*sd_mmc_info->blk_len, 0);
		if(ret)
			return ret;

		data_offset += read_block_count*sd_mmc_info->blk_len;
		block_count -= read_block_count;
	}

	return SD_MMC_NO_ERROR;
}

int sdio_read_data_block_sw(SD_MMC_Card_Info_t *sd_mmc_info, int function_no, int buf_or_fifo, unsigned long sdio_addr, unsigned long block_count, unsigned char *data_buf)
{
	unsigned long data = 0, res = 0, temp = 0;
	int ret, data_busy = 1, res_busy = 1;
	
	unsigned long res_cnt = 0, data_cnt = 0, num_nac = 0, num_ncr = 0;
	
	unsigned short crc16_array[4] = {0, 0, 0, 0};
	unsigned short crc16 = 0;
	
	unsigned long loop_num,blk_cnt;
	
	int i,j;

#ifdef SD_MMC_CRC_CHECK
	unsigned short crc_check = 0, crc_check_array[4]={0,0,0,0};
	int error=0;
	unsigned char *org_buf=data_buf;
#endif

	unsigned char response[MAX_RESPONSE_BYTES];
	unsigned long read_block_count, data_offset = 0;

	unsigned long sdio_extend_rw = 0;
	SDIO_IO_RW_EXTENDED_ARG *sdio_io_extend_rw = (void *)&sdio_extend_rw;

	while(block_count)
	{
		if(block_count > sd_mmc_info->max_blk_count)
			read_block_count = sd_mmc_info->max_blk_count;
		else
			read_block_count = block_count;

		sdio_io_extend_rw->R_W_Flag = SDIO_Read_Data;
		sdio_io_extend_rw->Block_Mode = SDIO_Block_MODE;
		sdio_io_extend_rw->OP_Code = buf_or_fifo;
		sdio_io_extend_rw->Function_No = function_no;
		sdio_io_extend_rw->Byte_Block_Count = read_block_count;
		sdio_io_extend_rw->Register_Address = ((sdio_addr+data_offset) & 0x1FFFF);

		ret = sd_send_cmd_sw(sd_mmc_info, IO_RW_EXTENDED, sdio_extend_rw, RESPONSE_NONE, NULL);
		if(ret)
			return ret;

		if(sd_mmc_info->bus_width == SD_BUS_WIDE)
		{
			sd_set_dat0_3_input();
		}
		else
		{
			sd_set_dat0_input();
		}

		sd_clear_response(response);
		sd_delay_clocks_z(sd_mmc_info, SD_MMC_Z_CMD_TO_RES);
	
		sd_set_cmd_input();
		//wait until both response and data is valid    
		do
		{
			sd_clk_transfer_low();
		
			res = sd_get_cmd_value();
			data = sd_get_dat0_value();
		
			if (res_busy)
			{
				if (res)
					num_ncr++;
				else
					res_busy = 0;
			}
			else
			{
				if (res_cnt < (RESPONSE_R1_R3_R4_R5_R6_R7_LENGTH*8))
				{
					response[res_cnt>>3] <<= 1;
					response[res_cnt>>3] |= res;
				
					res_cnt++;
				}
			}
		
			if (data_busy)
			{
				if (data)
					num_nac++;
				else
					data_busy = 0;
			}
			else
			{
				if(sd_mmc_info->bus_width == SD_BUS_WIDE)          //4 data lines
				{
					data = sd_get_dat0_3_value();
					temp <<= 4;
					temp |= data;
#ifdef SD_MMC_CRC_CHECK
					SD_CAL_BIT_CRC(crc_check_array[0],data&0x01);
					SD_CAL_BIT_CRC(crc_check_array[1],data&0x02);
					SD_CAL_BIT_CRC(crc_check_array[2],data&0x04);
					SD_CAL_BIT_CRC(crc_check_array[3],data&0x08);
#endif
					if((data_cnt & 0x01) == 1)
					{
#ifdef AMLOGIC_CHIP_SUPPORT
						if((unsigned long)data_buf == 0x3400000)
						{
							WRITE_BYTE_TO_FIFO(temp);
						}
						else
#endif
						{
							*data_buf = temp;
							data_buf++;
						}

						temp = 0;   //one byte received, clear temp varialbe
					}                   
				}
				else                //only data0 lines
				{
					data = sd_get_dat0_value();
					temp <<= 1;
					temp |= data;
					if((data_cnt & 0x07) == 7)
					{
#ifdef AMLOGIC_CHIP_SUPPORT
						if((unsigned)data_buf == 0x3400000)
						{
							WRITE_BYTE_TO_FIFO(temp);
						}
						else
#endif
						{
							*data_buf = temp;
							data_buf++;
						}

#ifdef SD_MMC_CRC_CHECK
						crc_check = (crc_check << 8) ^ sd_crc_table[((crc_check >> 8) ^ temp) & 0xff];
#endif
						temp = 0;   //one byte received, clear temp varialbe
					}
				}
				data_cnt++;
			}
			
			sd_clk_transfer_high();
		
			if(!res_busy && !data_busy)
			{
				if(sd_mmc_info->bus_width == SD_BUS_WIDE)          //4 data lines
				{
					if((res_cnt >= (RESPONSE_R1_R3_R4_R5_R6_R7_LENGTH*8)) && ((data_cnt&0x01) == 0))
					{
						data_cnt >>= 1;
						break;
					}
				}
				else
				{
					if((res_cnt >= (RESPONSE_R1_R3_R4_R5_R6_R7_LENGTH*8)) && ((data_cnt&0x07) == 0))
					{
						data_cnt >>= 3;
						break;
					}
				}
			}

		}while((num_ncr < SD_MMC_TIME_NCR_MAX) && (num_nac < sd_mmc_info->clks_nac));
	
		if((num_ncr >= SD_MMC_TIME_NCR_MAX) || (num_nac >= sd_mmc_info->clks_nac))
			return SD_MMC_ERROR_TIMEOUT;

		//Read all data blocks
		loop_num = sd_mmc_info->blk_len;
		for (blk_cnt = 0; blk_cnt < block_count; blk_cnt++)
		{
		//wait until data is valid
			num_nac = 0;    
			do
			{   
				if(!data_busy)
					break;
				
				sd_clk_transfer_low();
		
				data = sd_get_dat0_value();
		
				if(data)
				{
					num_nac++;
				}
				else
				{
					data_busy = 0;
				}
		
				sd_clk_transfer_high();

			}while(data_busy && (num_nac < sd_mmc_info->clks_nac));
		
			if(num_nac >= sd_mmc_info->clks_nac)
				return SD_MMC_ERROR_TIMEOUT;
		
		//Read data
			if(sd_mmc_info->bus_width == SD_BUS_WIDE)          //4 data lines
			{
#ifdef AMLOGIC_CHIP_SUPPORT
				if((unsigned long)data_buf == 0x3400000)
				{
					for(; data_cnt < loop_num; data_cnt++)
					{
						temp = 0;   //clear temp varialbe
				
						for(i = 0; i < 2; i++)
						{
							sd_clk_transfer_low();
		
							data = sd_get_dat0_3_value();
							temp <<= 4;
							temp |= data;
					
							sd_clk_transfer_high();
						
#ifdef SD_MMC_CRC_CHECK
							SD_CAL_BIT_CRC(crc_check_array[0],data&0x01);
							SD_CAL_BIT_CRC(crc_check_array[1],data&0x02);
							SD_CAL_BIT_CRC(crc_check_array[2],data&0x04);
							SD_CAL_BIT_CRC(crc_check_array[3],data&0x08);
#endif
						}
					
						WRITE_BYTE_TO_FIFO(temp);
					}
				}
				else
#endif
				{
					for(; data_cnt < loop_num; data_cnt++)
					{
						temp = 0;   //clear temp varialbe
				
						for(i = 0; i < 2; i++)
						{
							sd_clk_transfer_low();
						
							data = sd_get_dat0_3_value();
							temp <<= 4;
							temp |= data;
					
							sd_clk_transfer_high();
						
#ifdef SD_MMC_CRC_CHECK
							SD_CAL_BIT_CRC(crc_check_array[0],data&0x01);
							SD_CAL_BIT_CRC(crc_check_array[1],data&0x02);
							SD_CAL_BIT_CRC(crc_check_array[2],data&0x04);
							SD_CAL_BIT_CRC(crc_check_array[3],data&0x08);
#endif
						}
					
						*data_buf = temp;
						data_buf++;
					}
				}
			
				//Read CRC16 data
				for(data_cnt = 0; data_cnt < 16; data_cnt++)    // 16 bits CRC
				{
					sd_clk_transfer_low();
		
					crc16_array[0] <<= 1;
					crc16_array[1] <<= 1;
					crc16_array[2] <<= 1;
					crc16_array[3] <<= 1;
					data = sd_get_dat0_3_value();
					crc16_array[0] |= (data & 0x01);
					crc16_array[1] |= ((data >> 1) & 0x01);
					crc16_array[2] |= ((data >> 2) & 0x01);
					crc16_array[3] |= ((data >> 3) & 0x01);
			
					sd_clk_transfer_high();
				}
			
#ifdef SD_MMC_CRC_CHECK
				for(i=0; i<4; i++)
				{
					//crc_check_array[i] = sd_cal_crc_mode(org_buf, sd_mmc_info->blk_len, char_mode[i]);
					if(crc16_array[i] != crc_check_array[i])
					{
						error = SD_MMC_ERROR_DATA_CRC;
						break;
					}
				}
#endif
			}
			else    //only data0 lines
			{
#ifdef AMLOGIC_CHIP_SUPPORT
				if((unsigned long)data_buf == 0x3400000)
				{
					for(; data_cnt < loop_num; data_cnt++)
					{
						temp = 0;   //clear temp varialbe
				
						for(j = 0; j < 8; j++)
						{
							sd_clk_transfer_low();
					
							data = sd_get_dat0_value();
							temp <<= 1;
							temp |= data;
					
							sd_clk_transfer_high();
						}
					
						WRITE_BYTE_TO_FIFO(temp);
					
#ifdef SD_MMC_CRC_CHECK
						crc_check = (crc_check << 8) ^ sd_crc_table[((crc_check >> 8) ^ temp) & 0xff];
#endif
					}
				}
				else
#endif
				{
					for(; data_cnt < loop_num; data_cnt++)
					{
						temp = 0;   //clear temp varialbe
				
						for(j = 0; j < 8; j++)
						{
							sd_clk_transfer_low();
					
							data = sd_get_dat0_value();
							temp <<= 1;
							temp |= data;
					
							sd_clk_transfer_high();
						}
					
						*data_buf = temp;
						data_buf++;
					
#ifdef SD_MMC_CRC_CHECK
						crc_check = (crc_check << 8) ^ sd_crc_table[((crc_check >> 8) ^ temp) & 0xff];
#endif
					}
				}

				//Read CRC16 data
				for(data_cnt = 0; data_cnt < 16; data_cnt++)    // 16 bits CRC
				{
					sd_clk_transfer_low();
		
					data = sd_get_dat0_value();
					crc16 <<= 1;
					crc16 |= data;      
			
					sd_clk_transfer_high();
				}
			
#ifdef SD_MMC_CRC_CHECK
				if(crc16 != crc_check)
					error = SD_MMC_ERROR_DATA_CRC;
#endif
			}
		
			sd_clk_transfer_low();      //for end bit
			sd_clk_transfer_high();
		
			data_busy = 1;
			data_cnt = 0;
		
#ifdef SD_MMC_CRC_CHECK
			org_buf = data_buf;
			crc_check = 0;
			crc_check_array[0] = crc_check_array[1] = crc_check_array[2] = crc_check_array[3] = 0;
#endif
		}

		sd_delay_clocks_z(sd_mmc_info, SD_MMC_TIME_NRC_NCC);     //Clock delay, Z type
	
		data_offset += read_block_count*sd_mmc_info->blk_len;
		block_count -= read_block_count;
	}

	return SD_MMC_NO_ERROR;
}

unsigned char sdio_4bytes_buf[PAGE_SIZE];
int sdio_read_data_byte_hw(SD_MMC_Card_Info_t *sd_mmc_info, int function_no, int buf_or_fifo, unsigned long sdio_addr, unsigned long byte_count, unsigned char *data_buf)
{
	int ret;
	unsigned char response[MAX_RESPONSE_BYTES];
	unsigned long read_byte_count, four_byte_count, data_offset = 0;

	unsigned long sdio_extend_rw = 0;
	SDIO_IO_RW_EXTENDED_ARG *sdio_io_extend_rw = (void *)&sdio_extend_rw;

	while(byte_count)
	{
		if(byte_count > 512)
			read_byte_count = 512;
		else
			read_byte_count = byte_count;

		sdio_io_extend_rw->R_W_Flag = SDIO_Read_Data;
		sdio_io_extend_rw->Block_Mode = SDIO_Byte_MODE;
		sdio_io_extend_rw->OP_Code = buf_or_fifo;
		sdio_io_extend_rw->Function_No = function_no;
		sdio_io_extend_rw->Byte_Block_Count = read_byte_count;
		sdio_io_extend_rw->Register_Address = ((sdio_addr+data_offset) & 0x1FFFF);

		if(read_byte_count%4)
			sd_mmc_info->sdio_read_crc_close = 1;
		else
			sd_mmc_info->sdio_read_crc_close = 0;

		four_byte_count = (read_byte_count + 3) / 4;
		four_byte_count *= 4;
		ret = sd_send_cmd_hw(sd_mmc_info, IO_RW_EXTENDED, sdio_extend_rw, RESPONSE_R5, response, sdio_4bytes_buf, four_byte_count, 0);
		if(ret)
			return ret;
		memcpy(data_buf + data_offset, sdio_4bytes_buf, read_byte_count);

		data_offset += read_byte_count;
		byte_count -= read_byte_count;
	}

	sd_mmc_info->sdio_read_crc_close = 0;
	return SD_MMC_NO_ERROR;
}

int sdio_read_data_byte_sw(SD_MMC_Card_Info_t *sd_mmc_info, int function_no, int buf_or_fifo, unsigned long sdio_addr, unsigned long byte_count, unsigned char *data_buf)
{
	unsigned long data = 0, res = 0, temp = 0;
	int ret, data_busy = 1, res_busy = 1;
	
	unsigned long res_cnt = 0, data_cnt = 0, num_nac = 0, num_ncr = 0, crc_data_cnt = 0;
	
	unsigned short crc16_array[4] = {0, 0, 0, 0};
	unsigned short crc16 = 0;
	
	unsigned long loop_num;
	
	int i,j;

#ifdef SD_MMC_CRC_CHECK
	unsigned short crc_check = 0, crc_check_array[4]={0,0,0,0};
	int error=0;
	//unsigned char *org_buf=data_buf;
#endif

	unsigned char response[MAX_RESPONSE_BYTES];
	unsigned long read_byte_count, data_offset = 0;

	unsigned long sdio_extend_rw = 0;
	SDIO_IO_RW_EXTENDED_ARG *sdio_io_extend_rw = (void *)&sdio_extend_rw;

	while(byte_count)
	{
		if(byte_count > 512)
			read_byte_count = 512;
		else
			read_byte_count = byte_count;

		sdio_io_extend_rw->R_W_Flag = SDIO_Read_Data;
		sdio_io_extend_rw->Block_Mode = SDIO_Byte_MODE;
		sdio_io_extend_rw->OP_Code = buf_or_fifo;
		sdio_io_extend_rw->Function_No = function_no;
		sdio_io_extend_rw->Byte_Block_Count = read_byte_count;
		sdio_io_extend_rw->Register_Address = ((sdio_addr+data_offset) & 0x1FFFF);

		ret = sd_send_cmd_sw(sd_mmc_info, IO_RW_EXTENDED, sdio_extend_rw, RESPONSE_NONE, NULL);
		if(ret)
			return ret;

		if(sd_mmc_info->bus_width == SD_BUS_WIDE)
		{
			sd_set_dat0_3_input();
		}
		else
		{
			sd_set_dat0_input();
		}

		sd_clear_response(response);
		sd_delay_clocks_z(sd_mmc_info, SD_MMC_Z_CMD_TO_RES);
	
		sd_set_cmd_input();
		//wait until both response and data is valid    
		do
		{
			sd_clk_transfer_low();
		
			res = sd_get_cmd_value();
			data = sd_get_dat0_value();
		
			if(res_busy)
			{
				if (res)
					num_ncr++;
				else
					res_busy = 0;
			}
			else
			{
				if (res_cnt < (RESPONSE_R1_R3_R4_R5_R6_R7_LENGTH*8))
				{
					response[res_cnt>>3] <<= 1;
					response[res_cnt>>3] |= res;
				
					res_cnt++;
				}
			}
		
			if (data_busy)
			{
				if (data)
					num_nac++;
				else
					data_busy = 0;
			}
			else
			{
				if(sd_mmc_info->bus_width == SD_BUS_WIDE)          //4 data lines
				{
					if(data_cnt < read_byte_count*2)
					{
					data = sd_get_dat0_3_value();
					temp <<= 4;
					temp |= data;
#ifdef SD_MMC_CRC_CHECK
					SD_CAL_BIT_CRC(crc_check_array[0],data&0x01);
					SD_CAL_BIT_CRC(crc_check_array[1],data&0x02);
					SD_CAL_BIT_CRC(crc_check_array[2],data&0x04);
					SD_CAL_BIT_CRC(crc_check_array[3],data&0x08);
#endif
					if((data_cnt & 0x01) == 1)
					{
#ifdef AMLOGIC_CHIP_SUPPORT
						if((unsigned long)data_buf == 0x3400000)
						{
							WRITE_BYTE_TO_FIFO(temp);
						}
						else
#endif
						{
							*data_buf = temp;
							data_buf++;
						}

						temp = 0;   //one byte received, clear temp varialbe
					}                   
						data_cnt++;
					}
					else if(crc_data_cnt < 16)
					{
						//Read CRC16 data
						crc16_array[0] <<= 1;
						crc16_array[1] <<= 1;
						crc16_array[2] <<= 1;
						crc16_array[3] <<= 1;
						data = sd_get_dat0_3_value();
						crc16_array[0] |= (data & 0x01);
						crc16_array[1] |= ((data >> 1) & 0x01);
						crc16_array[2] |= ((data >> 2) & 0x01);
						crc16_array[3] |= ((data >> 3) & 0x01);

						crc_data_cnt++;
					}                
				}
				else                //only data0 lines
				{
					if(data_cnt < read_byte_count*8)
					{
					data = sd_get_dat0_value();
					temp <<= 1;
					temp |= data;
					if((data_cnt & 0x07) == 7)
					{
#ifdef AMLOGIC_CHIP_SUPPORT
						if((unsigned)data_buf == 0x3400000)
						{
							WRITE_BYTE_TO_FIFO(temp);
						}
						else
#endif
						{
							*data_buf = temp;
							data_buf++;
						}

#ifdef SD_MMC_CRC_CHECK
						crc_check = (crc_check << 8) ^ sd_crc_table[((crc_check >> 8) ^ temp) & 0xff];
#endif
						temp = 0;   //one byte received, clear temp varialbe
					}
				}
				data_cnt++;
			}
		
			}
		
			sd_clk_transfer_high();
		
			if(!res_busy && !data_busy)
			{
				if(sd_mmc_info->bus_width == SD_BUS_WIDE)          //4 data lines
				{
					if((res_cnt >= (RESPONSE_R1_R3_R4_R5_R6_R7_LENGTH*8)) && ((data_cnt&0x01) == 0))
					{
						data_cnt >>= 1;
						break;
					}
				}
				else
				{
					if((res_cnt >= (RESPONSE_R1_R3_R4_R5_R6_R7_LENGTH*8)) && ((data_cnt&0x07) == 0))
					{
						data_cnt >>= 3;
						break;
					}
				}
			}

		}while((num_ncr < SD_MMC_TIME_NCR_MAX) && (num_nac < sd_mmc_info->clks_nac));
	
		if((num_ncr >= SD_MMC_TIME_NCR_MAX) || (num_nac >= sd_mmc_info->clks_nac))
			return SD_MMC_ERROR_TIMEOUT;

		//Read data and response
		loop_num = read_byte_count;
		if(sd_mmc_info->bus_width == SD_BUS_WIDE)          //4 data lines
		{
#ifdef AMLOGIC_CHIP_SUPPORT
			if((unsigned long)data_buf == 0x3400000)
			{
				for(; data_cnt < loop_num; data_cnt++)
				{
					temp = 0;   //clear temp varialbe

					for(i = 0; i < 2; i++)
					{
						sd_clk_transfer_low();

						data = sd_get_dat0_3_value();
						temp <<= 4;
						temp |= data;

						sd_clk_transfer_high();

#ifdef SD_MMC_CRC_CHECK
						SD_CAL_BIT_CRC(crc_check_array[0],data&0x01);
						SD_CAL_BIT_CRC(crc_check_array[1],data&0x02);
						SD_CAL_BIT_CRC(crc_check_array[2],data&0x04);
						SD_CAL_BIT_CRC(crc_check_array[3],data&0x08);
#endif
					}

					WRITE_BYTE_TO_FIFO(temp);
				}
			}
			else
#endif
			{
				for(; data_cnt < loop_num; data_cnt++)
				{
					temp = 0;   //clear temp varialbe

					for(i = 0; i < 2; i++)
					{
						sd_clk_transfer_low();

						data = sd_get_dat0_3_value();
						temp <<= 4;
						temp |= data;

						sd_clk_transfer_high();
					
#ifdef SD_MMC_CRC_CHECK
						SD_CAL_BIT_CRC(crc_check_array[0],data&0x01);
						SD_CAL_BIT_CRC(crc_check_array[1],data&0x02);
						SD_CAL_BIT_CRC(crc_check_array[2],data&0x04);
						SD_CAL_BIT_CRC(crc_check_array[3],data&0x08);
#endif
					}
				
					*data_buf = temp;
					data_buf++;
				}
			}
		
			//Read CRC16 data
			for(; crc_data_cnt < 16; crc_data_cnt++)    // 16 bits CRC
			{
				sd_clk_transfer_low();

				crc16_array[0] <<= 1;
				crc16_array[1] <<= 1;
				crc16_array[2] <<= 1;
				crc16_array[3] <<= 1;
				data = sd_get_dat0_3_value();
				crc16_array[0] |= (data & 0x01);
				crc16_array[1] |= ((data >> 1) & 0x01);
				crc16_array[2] |= ((data >> 2) & 0x01);
				crc16_array[3] |= ((data >> 3) & 0x01);

				sd_clk_transfer_high();
			}
		
#ifdef SD_MMC_CRC_CHECK
			for(i=0; i<4; i++)
			{
				//crc_check_array[i] = sd_cal_crc_mode(org_buf, sd_mmc_info->blk_len, char_mode[i]);
				if(crc16_array[i] != crc_check_array[i])
				{
					error = SD_MMC_ERROR_DATA_CRC;
					break;
				}
			}
#endif
		}
		else    //only data0 lines
		{
#ifdef AMLOGIC_CHIP_SUPPORT
			if((unsigned)data_buf == 0x3400000)
			{
				for(; data_cnt < loop_num; data_cnt++)
				{
					temp = 0;   //clear temp varialbe

					for(j = 0; j < 8; j++)
					{
						sd_clk_transfer_low();
				
						data = sd_get_dat0_value();
						temp <<= 1;
						temp |= data;

						sd_clk_transfer_high();
					}

					WRITE_BYTE_TO_FIFO(temp);
				
#ifdef SD_MMC_CRC_CHECK
					crc_check = (crc_check << 8) ^ sd_crc_table[((crc_check >> 8) ^ temp) & 0xff];
#endif
				}
			}
			else
#endif
			{
				for(; data_cnt < loop_num; data_cnt++)
				{
					temp = 0;   //clear temp varialbe

					for(j = 0; j < 8; j++)
					{
						sd_clk_transfer_low();
				
						data = sd_get_dat0_value();
						temp <<= 1;
						temp |= data;

						sd_clk_transfer_high();
					}

					*data_buf = temp;
					data_buf++;
				
#ifdef SD_MMC_CRC_CHECK
					crc_check = (crc_check << 8) ^ sd_crc_table[((crc_check >> 8) ^ temp) & 0xff];
#endif
				}
			}

			//Read CRC16 data
			for(data_cnt = 0; data_cnt < 16; data_cnt++)    // 16 bits CRC
			{
				sd_clk_transfer_low();

				data = sd_get_dat0_value();
				crc16 <<= 1;
				crc16 |= data;

				sd_clk_transfer_high();
			}
		
#ifdef SD_MMC_CRC_CHECK
			if(crc16 != crc_check)
				error = SD_MMC_ERROR_DATA_CRC;
#endif
		}

		sd_clk_transfer_low();      //for end bit
		sd_clk_transfer_high();
	
		sd_delay_clocks_z(sd_mmc_info, SD_MMC_TIME_NRC_NCC);     //Clock delay, Z type

		data_offset += read_byte_count;
		byte_count -= read_byte_count;
	}

	return SD_MMC_NO_ERROR;
}

int sdio_read_data(SD_MMC_Card_Info_t *sd_mmc_info, int function_no, int buf_or_fifo, unsigned long sdio_addr, unsigned long byte_count, unsigned char *data_buf)
{
	int error = 0, ret;
	unsigned long block_nums, byte_nums;
	BUG_ON(sd_mmc_info->sdio_blk_len[function_no] == 0);

#ifdef SD_MMC_HW_CONTROL
	if(SD_WORK_MODE == CARD_HW_MODE)
	{
		//sd_sdio_enable(sd_mmc_info->io_pad_type);
		if (sd_mmc_info->sd_save_hw_io_flag) {
    		WRITE_CBUS_REG(SDIO_CONFIG, sd_mmc_info->sd_save_hw_io_config);
      		WRITE_CBUS_REG(SDIO_MULT_CONFIG, sd_mmc_info->sd_save_hw_io_mult_config);
    	}		
	}
#endif       
#ifdef SD_MMC_SW_CONTROL
	if(SD_WORK_MODE == CARD_SW_MODE)
		sd_mmc_io_config(sd_mmc_info);
#endif
	
	sd_mmc_info->blk_len = sd_mmc_info->sdio_blk_len[function_no];
	byte_nums = byte_count % sd_mmc_info->blk_len;
	block_nums = byte_count / sd_mmc_info->blk_len;

	if(block_nums == 0)
	{
		if(byte_nums)
		{
#ifdef SD_MMC_HW_CONTROL
			if(SD_WORK_MODE == CARD_HW_MODE)
				error = sdio_read_data_byte_hw(sd_mmc_info, function_no, buf_or_fifo, sdio_addr, byte_nums, data_buf);
#endif
#ifdef SD_MMC_SW_CONTROL
			if(SD_WORK_MODE == CARD_SW_MODE)
				error = sdio_read_data_byte_sw(sd_mmc_info, function_no, buf_or_fifo, sdio_addr, byte_nums, data_buf);
#endif
			if(error == SD_MMC_ERROR_TIMEOUT)
			{
				ret = sdio_data_transfer_abort(sd_mmc_info, function_no);
				if(ret)
					return ret;

#ifdef SD_MMC_HW_CONTROL
				if(SD_WORK_MODE == CARD_HW_MODE)
					error = sdio_read_data_byte_hw(sd_mmc_info, function_no, buf_or_fifo, sdio_addr, byte_nums, data_buf);
#endif
#ifdef SD_MMC_SW_CONTROL
				if(SD_WORK_MODE == CARD_SW_MODE)
					error = sdio_read_data_byte_sw(sd_mmc_info, function_no, buf_or_fifo, sdio_addr, byte_nums, data_buf);
#endif
				if(error == SD_MMC_ERROR_TIMEOUT)
					return error;
			}
		}
		else
		{
			error = SD_MMC_ERROR_BLOCK_LEN;
#ifdef  SD_MMC_DEBUG
			Debug_Printf("#%s error occured in sdio_read_data()\n", sd_error_to_string(error));
#endif
			return error;
		}

		if(error)
		{			
#ifdef  SD_MMC_DEBUG
			Debug_Printf("#%s error occured in sdio_read_byte()\n", sd_error_to_string(error));
#endif
			ret = sdio_data_transfer_abort(sd_mmc_info, function_no);
			if(ret)
				return ret;

			return error;
		}
	}
	else
	{
		if(sd_mmc_info->blk_len == sd_mmc_info->sdio_blk_len[function_no])
		{
#ifdef SD_MMC_HW_CONTROL
			if(SD_WORK_MODE == CARD_HW_MODE)
			{
				error = sdio_read_data_block_hw(sd_mmc_info, function_no, buf_or_fifo, sdio_addr, block_nums, data_buf);
				if(error == SD_MMC_ERROR_TIMEOUT)
				{
					ret = sdio_data_transfer_abort(sd_mmc_info, function_no);
					if(ret)
						return ret;

					error = sdio_read_data_byte_hw(sd_mmc_info, function_no, buf_or_fifo, sdio_addr, block_nums*sd_mmc_info->blk_len, data_buf);
				}

				if(byte_nums)
				{
					error = sdio_read_data_byte_hw(sd_mmc_info, function_no, buf_or_fifo, sdio_addr + block_nums*sd_mmc_info->blk_len, byte_nums, data_buf + block_nums*sd_mmc_info->blk_len);
				}
			}
#endif
#ifdef SD_MMC_SW_CONTROL
			if(SD_WORK_MODE == CARD_SW_MODE)
			{
				error = sdio_read_data_block_sw(sd_mmc_info, function_no, buf_or_fifo, sdio_addr, block_nums, data_buf);
				if(error == SD_MMC_ERROR_TIMEOUT)
				{
					ret = sdio_data_transfer_abort(sd_mmc_info, function_no);
					if(ret)
						return ret;

					error = sdio_read_data_byte_sw(sd_mmc_info, function_no, buf_or_fifo, sdio_addr, block_nums*sd_mmc_info->blk_len, data_buf);
				}

				if(byte_nums)
				{
					error = sdio_read_data_byte_sw(sd_mmc_info, function_no, buf_or_fifo, sdio_addr + block_nums*sd_mmc_info->blk_len, byte_nums, data_buf + block_nums*sd_mmc_info->blk_len);
				}
			}
#endif
		}
		else
		{
#ifdef SD_MMC_HW_CONTROL
			if(SD_WORK_MODE == CARD_HW_MODE)
				error = sdio_read_data_byte_hw(sd_mmc_info, function_no, buf_or_fifo, sdio_addr, byte_count, data_buf);
#endif
#ifdef SD_MMC_SW_CONTROL
			if(SD_WORK_MODE == CARD_SW_MODE)
				error = sdio_read_data_byte_sw(sd_mmc_info, function_no, buf_or_fifo, sdio_addr, byte_count, data_buf);
#endif
		}

		if(error)
		{			
#ifdef  SD_MMC_DEBUG
			Debug_Printf("#%s error occured in sdio_read_block()\n", sd_error_to_string(error));
#endif
			ret = sdio_data_transfer_abort(sd_mmc_info, function_no);
			if(ret)
				return ret;

			return error;
		}
	}

	return SD_MMC_NO_ERROR;	
}

int sdio_write_data_block_hw(SD_MMC_Card_Info_t *sd_mmc_info, int function_no, int buf_or_fifo, unsigned long sdio_addr, unsigned long block_count, unsigned char *data_buf)
{
	int ret;
	unsigned char response[MAX_RESPONSE_BYTES];
	unsigned long write_block_count, data_offset = 0;

	unsigned long sdio_extend_rw = 0;
	SDIO_IO_RW_EXTENDED_ARG *sdio_io_extend_rw = (void *)&sdio_extend_rw;

	while(block_count)
	{
		if(block_count > sd_mmc_info->max_blk_count)
			write_block_count = sd_mmc_info->max_blk_count;
		else
			write_block_count = block_count;

		sdio_io_extend_rw->R_W_Flag = SDIO_Write_Data;
		sdio_io_extend_rw->Block_Mode = SDIO_Block_MODE;
		sdio_io_extend_rw->OP_Code = buf_or_fifo;
		sdio_io_extend_rw->Function_No = function_no;
		sdio_io_extend_rw->Byte_Block_Count = write_block_count;
		sdio_io_extend_rw->Register_Address = ((sdio_addr+data_offset) & 0x1FFFF);

		ret = sd_send_cmd_hw(sd_mmc_info, IO_RW_EXTENDED, sdio_extend_rw, RESPONSE_R5, response, data_buf+data_offset, write_block_count*sd_mmc_info->blk_len, 0);
		if(ret)
			return ret;

		data_offset += write_block_count*sd_mmc_info->blk_len;
		block_count -= write_block_count;
	}

	return SD_MMC_NO_ERROR;
}

int sdio_write_data_block_sw(SD_MMC_Card_Info_t *sd_mmc_info, int function_no, int buf_or_fifo, unsigned long sdio_addr, unsigned long block_count, unsigned char *data_buf)
{
	unsigned long write_block_count, data_offset = 0;

	int ret,i,j;
	unsigned long crc_status, data;
	
	unsigned long data_cnt = 0;
	unsigned timeout;
	
	unsigned char * org_buf = data_buf;
	unsigned short crc16_array[4] = {0, 0, 0, 0};
	unsigned short crc16;
	unsigned char char_mode[4][3] = {{0x10, 0x01, 0},
					{0x20, 0x02, 0},
					{0x40, 0x04, 0},
					{0x80, 0x08, 0}};
	
	unsigned char response[MAX_RESPONSE_BYTES];
	unsigned long loop_num,blk_cnt;

	unsigned long sdio_extend_rw = 0;
	SDIO_IO_RW_EXTENDED_ARG *sdio_io_extend_rw = (void *)&sdio_extend_rw;

	while(block_count)
	{
		if(block_count > sd_mmc_info->max_blk_count)
			write_block_count = sd_mmc_info->max_blk_count;
		else
			write_block_count = block_count;

		sdio_io_extend_rw->R_W_Flag = SDIO_Write_Data;
		sdio_io_extend_rw->Block_Mode = SDIO_Block_MODE;
		sdio_io_extend_rw->OP_Code = buf_or_fifo;
		sdio_io_extend_rw->Function_No = function_no;
		sdio_io_extend_rw->Byte_Block_Count = write_block_count;
		sdio_io_extend_rw->Register_Address = ((sdio_addr+data_offset) & 0x1FFFF);

		ret = sd_send_cmd_sw(sd_mmc_info, IO_RW_EXTENDED, sdio_extend_rw, RESPONSE_R5, response);
		if(ret)
			return ret;

		loop_num = sd_mmc_info->blk_len;
		for(blk_cnt = 0; blk_cnt < write_block_count; blk_cnt++)
		{
			org_buf = data_buf;

			//Nwr cycles delay
			sd_delay_clocks_h(sd_mmc_info, SD_MMC_TIME_NWR);

			//Start bit
			if(sd_mmc_info->bus_width == SD_BUS_WIDE)
			{
				sd_set_dat0_3_output();

				sd_clk_transfer_low();

				sd_set_dat0_3_value(0x00);

				sd_clk_transfer_high();
			}
			else
			{
				sd_set_dat0_output();

				sd_clk_transfer_low();

				sd_set_dat0_value(0x00);

				sd_clk_transfer_high();
			}

			//Write data
			if(sd_mmc_info->bus_width == SD_BUS_WIDE)
			{
				for(data_cnt = 0; data_cnt < loop_num; data_cnt++)
				{
					for(i=1; i>=0; i--)
					{
						sd_clk_transfer_low();

						data = (*data_buf >> (i<<2)) & 0x0F;
						sd_set_dat0_3_value(data);

						sd_clk_transfer_high();
					}
				
					data_buf++;
				}

				//Caculate CRC16 value and write to line
				for(i=0; i<4; i++)
				{
					crc16_array[i] = sd_cal_crc_mode(org_buf, sd_mmc_info->blk_len, char_mode[i]);
				}

				//Write CRC16
				for(i=15; i>=0; i--)
				{
					sd_clk_transfer_low();

					data = 0;
					for(j=3; j>=0; j--)
					{
						data <<= 1; 
						data |= (crc16_array[j] >> i) & 0x0001;
					}
					sd_set_dat0_3_value(data);

					sd_clk_transfer_high();
				}
			}
			else    // only dat0 line
			{
				for(data_cnt = 0; data_cnt < loop_num; data_cnt++)
				{
					for(i=7; i>=0; i--)
					{
						sd_clk_transfer_low();

						data = (*data_buf >> i) & 0x01;
						sd_set_dat0_value(data);

						sd_clk_transfer_high();
					}

					data_buf++;
				}
			
				//Caculate CRC16 value and write to line
				crc16 = sd_cal_crc16(org_buf, sd_mmc_info->blk_len);

				//Write CRC16
				for(i=15; i>=0; i--)
				{
					sd_clk_transfer_low();

					data = (crc16 >> i) & 0x0001;
					sd_set_dat0_value(data);

					sd_clk_transfer_high();
				}
			}

			//End bit
			if(sd_mmc_info->bus_width == SD_BUS_WIDE)
			{
				sd_clk_transfer_low();

				sd_set_dat0_3_value(0x0F);

				sd_clk_transfer_high();

				sd_set_dat0_3_input();
			}
			else
			{
				sd_clk_transfer_low();

				sd_set_dat0_value(0x01);

				sd_clk_transfer_high();

				sd_set_dat0_input();
			}

			sd_delay_clocks_h(sd_mmc_info, SD_MMC_Z_CMD_TO_RES);

			crc_status = 0;
	
			//Check CRC status
			sd_set_dat0_input();
			for(i = 0; i < 5; i++)
			{
				sd_clk_transfer_low();

				data = sd_get_dat0_value();
				crc_status <<= 1;
				crc_status |= data; 

				sd_clk_transfer_high();
			}
			if (crc_status == 0x0A)         //1010, CRC error
				return SD_MMC_ERROR_DATA_CRC;
			else if (crc_status == 0x0F)        //1111, Programming error
				return SD_MMC_ERROR_DRIVER_FAILURE;
							//0101, CRC ok
		
			//Check busy
			timeout = 0;
			do
			{
				sd_clk_transfer_low();

				data = sd_get_dat0_value();

				sd_clk_transfer_high();

				if(data)
					break;

				sd_delay_ms(1);
			}while(timeout < SD_PROGRAMMING_TIMEOUT/TIMER_1MS);

			if(timeout >= SD_PROGRAMMING_TIMEOUT/TIMER_1MS)
				return SD_MMC_ERROR_TIMEOUT;
		}

		data_offset += write_block_count*sd_mmc_info->blk_len;
		block_count -= write_block_count;
	}

	return SD_MMC_NO_ERROR;
}

int sdio_write_data_byte_hw(SD_MMC_Card_Info_t *sd_mmc_info, int function_no, int buf_or_fifo, unsigned long sdio_addr, unsigned long byte_count, unsigned char *data_buf)
{
	int ret;
	unsigned char response[MAX_RESPONSE_BYTES];
	unsigned long write_byte_count, data_offset = 0;

	unsigned long sdio_extend_rw = 0;
	SDIO_IO_RW_EXTENDED_ARG *sdio_io_extend_rw = (void *)&sdio_extend_rw;

	while(byte_count)
	{
		if(byte_count > 512)
			write_byte_count = 512;
		else
			write_byte_count = byte_count;

		sdio_io_extend_rw->R_W_Flag = SDIO_Write_Data;
		sdio_io_extend_rw->Block_Mode = SDIO_Byte_MODE;
		sdio_io_extend_rw->OP_Code = buf_or_fifo;
		sdio_io_extend_rw->Function_No = function_no;
		sdio_io_extend_rw->Byte_Block_Count = write_byte_count;
		sdio_io_extend_rw->Register_Address = ((sdio_addr+data_offset) & 0x1FFFF);

		ret = sd_send_cmd_hw(sd_mmc_info, IO_RW_EXTENDED, sdio_extend_rw, RESPONSE_R5, response, data_buf+data_offset, write_byte_count, 0);
		if(ret)
			return ret;

		data_offset += write_byte_count;
		byte_count -= write_byte_count;
	}

	return SD_MMC_NO_ERROR;
}

int sdio_write_data_byte_sw(SD_MMC_Card_Info_t *sd_mmc_info, int function_no, int buf_or_fifo, unsigned long sdio_addr, unsigned long byte_count, unsigned char *data_buf)
{
	int ret,i,j;
	unsigned long crc_status, data;
	
	unsigned long data_cnt = 0;
	unsigned timeout;
	
	unsigned char * org_buf = data_buf;
	unsigned short crc16_array[4] = {0, 0, 0, 0};
	unsigned short crc16;
	unsigned char char_mode[4][3] = {{0x10, 0x01, 0},
					{0x20, 0x02, 0},
					{0x40, 0x04, 0},
					{0x80, 0x08, 0}};

	unsigned long loop_num;

	unsigned char response[MAX_RESPONSE_BYTES];
	unsigned long write_byte_count, data_offset = 0;

	unsigned long sdio_extend_rw = 0;
	SDIO_IO_RW_EXTENDED_ARG *sdio_io_extend_rw = (void *)&sdio_extend_rw;

	while(byte_count)
	{
		if(byte_count > 512)
			write_byte_count = 512;
		else
			write_byte_count = byte_count;

		sdio_io_extend_rw->R_W_Flag = SDIO_Write_Data;
		sdio_io_extend_rw->Block_Mode = SDIO_Byte_MODE;
		sdio_io_extend_rw->OP_Code = buf_or_fifo;
		sdio_io_extend_rw->Function_No = function_no;
		sdio_io_extend_rw->Byte_Block_Count = write_byte_count;
		sdio_io_extend_rw->Register_Address = ((sdio_addr+data_offset) & 0x1FFFF);

		ret = sd_send_cmd_sw(sd_mmc_info, IO_RW_EXTENDED, sdio_extend_rw, RESPONSE_R5, response);
		if(ret)
			return ret;

		loop_num = write_byte_count;
		org_buf = data_buf;

		//Nwr cycles delay
		sd_delay_clocks_h(sd_mmc_info, SD_MMC_TIME_NWR);

		//Start bit
		if(sd_mmc_info->bus_width == SD_BUS_WIDE)
		{
			sd_set_dat0_3_output();

			sd_clk_transfer_low();

			sd_set_dat0_3_value(0x00);

			sd_clk_transfer_high();
		}
		else
		{
			sd_set_dat0_output();

			sd_clk_transfer_low();

			sd_set_dat0_value(0x00);

			sd_clk_transfer_high();
		}

		//Write data
		if(sd_mmc_info->bus_width == SD_BUS_WIDE)
		{
			for(data_cnt = 0; data_cnt < loop_num; data_cnt++)
			{
				for(i=1; i>=0; i--)
				{
					sd_clk_transfer_low();

					data = (*data_buf >> (i<<2)) & 0x0F;
					sd_set_dat0_3_value(data);

					sd_clk_transfer_high();
				}
				
				data_buf++;
			}

			//Caculate CRC16 value and write to line
			for(i=0; i<4; i++)
			{
				crc16_array[i] = sd_cal_crc_mode(org_buf, write_byte_count, char_mode[i]);
			}

			//Write CRC16
			for(i=15; i>=0; i--)
			{
				sd_clk_transfer_low();

				data = 0;
				for(j=3; j>=0; j--)
				{
					data <<= 1; 
					data |= (crc16_array[j] >> i) & 0x0001;
				}
				sd_set_dat0_3_value(data);

				sd_clk_transfer_high();
			}
		}
		else    // only dat0 line
		{
			for(data_cnt = 0; data_cnt < loop_num; data_cnt++)
			{
				for(i=7; i>=0; i--)
				{
					sd_clk_transfer_low();

					data = (*data_buf >> i) & 0x01;
					sd_set_dat0_value(data);

					sd_clk_transfer_high();
				}

				data_buf++;
			}
			
			//Caculate CRC16 value and write to line
			crc16 = sd_cal_crc16(org_buf, write_byte_count);

			//Write CRC16
			for(i=15; i>=0; i--)
			{
				sd_clk_transfer_low();

				data = (crc16 >> i) & 0x0001;
				sd_set_dat0_value(data);

				sd_clk_transfer_high();
			}
		}

		//End bit
		if(sd_mmc_info->bus_width == SD_BUS_WIDE)
		{
			sd_clk_transfer_low();

			sd_set_dat0_3_value(0x0F);

			sd_clk_transfer_high();

			sd_set_dat0_3_input();
		}
		else
		{
			sd_clk_transfer_low();

			sd_set_dat0_value(0x01);

			sd_clk_transfer_high();

			sd_set_dat0_input();
		}

		sd_delay_clocks_h(sd_mmc_info, SD_MMC_Z_CMD_TO_RES);

		crc_status = 0;
	
		//Check CRC status
		sd_set_dat0_input();
		for(i = 0; i < 5; i++)
		{
			sd_clk_transfer_low();

			data = sd_get_dat0_value();
			crc_status <<= 1;
			crc_status |= data; 

			sd_clk_transfer_high();
		}
		if (crc_status == 0x0A)         //1010, CRC error
			return SD_MMC_ERROR_DATA_CRC;
		else if (crc_status == 0x0F)        //1111, Programming error
			return SD_MMC_ERROR_DRIVER_FAILURE;
							//0101, CRC ok
		
		//Check busy
		timeout = 0;
		do
		{
			sd_clk_transfer_low();
			
			data = sd_get_dat0_value();
			
			sd_clk_transfer_high();
		
			if(data)
				break;

			sd_delay_ms(1);
		}while(timeout < SD_PROGRAMMING_TIMEOUT/TIMER_1MS);

		if(timeout >= SD_PROGRAMMING_TIMEOUT/TIMER_1MS)
			return SD_MMC_ERROR_TIMEOUT;

		data_offset += write_byte_count;
		byte_count -= write_byte_count;
	}

	return SD_MMC_NO_ERROR;
}

int sdio_write_data(SD_MMC_Card_Info_t *sd_mmc_info, int function_no, int buf_or_fifo, unsigned long sdio_addr, unsigned long byte_count, unsigned char *data_buf)
{
	int error = 0,ret;
	unsigned long block_nums, byte_nums;
	BUG_ON(sd_mmc_info->sdio_blk_len[function_no] == 0);

#ifdef SD_MMC_HW_CONTROL
	if(SD_WORK_MODE == CARD_HW_MODE)
	{
		//sd_sdio_enable(sd_mmc_info->io_pad_type);
		if (sd_mmc_info->sd_save_hw_io_flag) {
    		WRITE_CBUS_REG(SDIO_CONFIG, sd_mmc_info->sd_save_hw_io_config);
      		WRITE_CBUS_REG(SDIO_MULT_CONFIG, sd_mmc_info->sd_save_hw_io_mult_config);
    	}		
	}
#endif
#ifdef SD_MMC_SW_CONTROL
	if(SD_WORK_MODE == CARD_SW_MODE)
		sd_mmc_io_config(sd_mmc_info);
#endif

	sd_mmc_info->blk_len = sd_mmc_info->sdio_blk_len[function_no];
	block_nums = sd_mmc_info->blk_len;
	block_nums = byte_count / block_nums;
	byte_nums = byte_count % sd_mmc_info->blk_len;
	//printk("sdio write data addr %x at fun %d cnt: %d blk len %d\n", sdio_addr, function_no, byte_count, sd_mmc_info->blk_len);

	if(block_nums == 0)
	{
		if(byte_nums)
		{
#ifdef SD_MMC_HW_CONTROL
			if(SD_WORK_MODE == CARD_HW_MODE)
				error = sdio_write_data_byte_hw(sd_mmc_info, function_no, buf_or_fifo, sdio_addr, byte_nums, data_buf);
#endif
#ifdef SD_MMC_SW_CONTROL
			if(SD_WORK_MODE == CARD_SW_MODE)
				error = sdio_write_data_byte_sw(sd_mmc_info, function_no, buf_or_fifo, sdio_addr, byte_nums, data_buf);
#endif
			if(error)
			{
				ret = sdio_data_transfer_abort(sd_mmc_info, function_no);
				if(ret)
					return ret;

#ifdef SD_MMC_HW_CONTROL
				if(SD_WORK_MODE == CARD_HW_MODE)
					error = sdio_write_data_byte_hw(sd_mmc_info, function_no, buf_or_fifo, sdio_addr, byte_nums, data_buf);
#endif
#ifdef SD_MMC_SW_CONTROL
				if(SD_WORK_MODE == CARD_SW_MODE)
					error = sdio_write_data_byte_sw(sd_mmc_info, function_no, buf_or_fifo, sdio_addr, byte_nums, data_buf);
#endif
				if(error)
					return error;
			}
		}
		else
		{
			error = SD_MMC_ERROR_BLOCK_LEN;
#ifdef  SD_MMC_DEBUG
			Debug_Printf("#%s error occured in sdio_write_data() blklen %d fun no %d\n", sd_error_to_string(error), sd_mmc_info->blk_len, function_no);
#endif
			return error;
		}

		if(error)
		{			
#ifdef  SD_MMC_DEBUG
			Debug_Printf("#%s error occured in sdio_write_byte()\n", sd_error_to_string(error));
#endif
			ret = sdio_data_transfer_abort(sd_mmc_info, function_no);
			if(ret)
				return ret;

			return error;
		}
	}
	else
	{
		if(sd_mmc_info->blk_len == sd_mmc_info->sdio_blk_len[function_no])
		{
#ifdef SD_MMC_HW_CONTROL
			if(SD_WORK_MODE == CARD_HW_MODE)
			{
				error = sdio_write_data_block_hw(sd_mmc_info, function_no, buf_or_fifo, sdio_addr, block_nums, data_buf);
				if(error)
				{
					ret = sdio_data_transfer_abort(sd_mmc_info, function_no);
					if(ret)
						return ret;

					error = sdio_write_data_block_hw(sd_mmc_info, function_no, buf_or_fifo, sdio_addr, block_nums, data_buf);
					if(error)
					{
						ret = sdio_data_transfer_abort(sd_mmc_info, function_no);
						if(ret)
							return ret;

						error = sdio_write_data_byte_hw(sd_mmc_info, function_no, buf_or_fifo, sdio_addr, block_nums*sd_mmc_info->blk_len, data_buf);
					}
				}

				if(byte_nums)
				{
					error = sdio_write_data_byte_hw(sd_mmc_info, function_no, buf_or_fifo, sdio_addr+block_nums*sd_mmc_info->blk_len, byte_nums, data_buf+block_nums*sd_mmc_info->blk_len);
				}
			}
#endif
#ifdef SD_MMC_SW_CONTROL
			if(SD_WORK_MODE == CARD_SW_MODE)
			{
				error = sdio_write_data_block_sw(sd_mmc_info, function_no, buf_or_fifo, sdio_addr, block_nums, data_buf);
				if(error)
				{
					ret = sdio_data_transfer_abort(sd_mmc_info, function_no);
					if(ret)
						return ret;

					error = sdio_write_data_block_sw(sd_mmc_info, function_no, buf_or_fifo, sdio_addr, block_nums, data_buf);
					if(error)
					{
						ret = sdio_data_transfer_abort(sd_mmc_info, function_no);
						if(ret)
							return ret;

						error = sdio_write_data_byte_sw(sd_mmc_info, function_no, buf_or_fifo, sdio_addr, block_nums*sd_mmc_info->blk_len, data_buf);
					}
				}

				if(byte_nums)
				{
					error = sdio_write_data_byte_sw(sd_mmc_info, function_no, buf_or_fifo, sdio_addr+block_nums*sd_mmc_info->blk_len, byte_nums, data_buf+block_nums*sd_mmc_info->blk_len);
				}
			}
#endif
		}
		else
		{
#ifdef SD_MMC_HW_CONTROL
			if(SD_WORK_MODE == CARD_HW_MODE)
				error = sdio_write_data_byte_hw(sd_mmc_info, function_no, buf_or_fifo, sdio_addr, byte_count, data_buf);
#endif
#ifdef SD_MMC_SW_CONTROL
			if(SD_WORK_MODE == CARD_SW_MODE)
				error = sdio_write_data_byte_sw(sd_mmc_info, function_no, buf_or_fifo, sdio_addr, byte_count, data_buf);
#endif
		}

		if(error)
		{			
#ifdef  SD_MMC_DEBUG
			Debug_Printf("#%s error occured in sdio_write_block() blklen %d fun no %d\n", sd_error_to_string(error), sd_mmc_info->blk_len, function_no);
#endif
			ret = sdio_data_transfer_abort(sd_mmc_info, function_no);
			if(ret)
				return ret;

			return error;
		}
	}

	return SD_MMC_NO_ERROR;
}

int sdio_open_target_interrupt(SD_MMC_Card_Info_t *sd_mmc_info, int function_no)
{
	int error = 0;
	unsigned char read_reg_data, write_reg_data;

	error = sdio_read_reg(sd_mmc_info, 0, Card_Capability_REG, &read_reg_data);
	if(error)
		return error;

	write_reg_data = ((1 << function_no) | SDIO_INT_EN_MASK);
	error = sdio_write_reg(sd_mmc_info, 0, INT_ENABLE_REG, &write_reg_data, SDIO_Read_After_Write);
	if(error)
    	return error;

	return SD_MMC_NO_ERROR;
}

int sdio_close_target_interrupt(SD_MMC_Card_Info_t *sd_mmc_info, int function_no)
{
	int error = 0;
	unsigned char read_reg_data, write_reg_data;

	error = sdio_read_reg(sd_mmc_info, 0, Card_Capability_REG, &read_reg_data);
	if(error)
		return error;

	write_reg_data = 0;
	error = sdio_write_reg(sd_mmc_info, 0, INT_ENABLE_REG, &write_reg_data, SDIO_Read_After_Write);
	if(error)
    	return error;

	return SD_MMC_NO_ERROR;
}

/*void sd_mmc_get_info(blkdev_stat_t *info)
{
	if(info->magic != BLKDEV_STAT_MAGIC)
		return;
	info->valid = 1;
	info->blk_size = sd_mmc_info->blk_len;
	info->blk_num = sd_mmc_info->blk_nums;
	info->serial_no = sd_mmc_info->card_psn;
	info->st_write_protect = sd_mmc_info->write_protected_flag;
	switch(sd_mmc_info->card_type)
	{
		case CARD_TYPE_SD:
			info->blkdev_name = "SD card";
			break;
		case CARD_TYPE_SDHC:
			info->blkdev_name = "SDHC card";
			break;
		case CARD_TYPE_MMC:
			info->blkdev_name = "MMC card";
			break;
		default:
			break;
	}
}*/

