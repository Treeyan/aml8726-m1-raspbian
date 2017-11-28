/* 
* linux/arch/arm/mach-meson/include/mach/i2c.h
*/
#ifndef AML_MACH_I2C
#define AML_MACH_I2C

#include <mach/am_regs.h>

#define MESON_I2C_MASTER_A_START		CBUS_REG_ADDR(I2C_M_0_CONTROL_REG)
#define MESON_I2C_MASTER_A_END		(CBUS_REG_ADDR(I2C_M_0_RDATA_REG1+1)-1)

#define MESON_I2C_MASTER_B_START		CBUS_REG_ADDR(I2C_M_1_CONTROL_REG)
#define MESON_I2C_MASTER_B_END		(CBUS_REG_ADDR(I2C_M_1_RDATA_REG1+1)-1)

#define MESON_I2C_SLAVE_START			CBUS_REG_ADDR(I2C_S_CONTROL_REG)
#define MESON_I2C_SLAVE_END			(CBUS_REG_ADDR(I2C_S_CNTL1_REG+1)-1)

#define AML_I2C_MASTER_A			0
#define AML_I2C_MASTER_B 			1

#define AML_I2C_SLAVE_ADDR			0x6c

/*M1 i2c pinmux
 *       I/O			I2C_MASTER_A		I2C_MASTER_B		I2C_SLAVE	
 * GPIO_JTAG_TMS	SCK_A REG1[12]							SCK_A REG1[13]
 * GPIO_JTAG_TDI		SDA_A REG1[12]							SDA_A REG1[13]
 * GPIO_JTAG_TCK						SCK_B REG1[16]		SCK_A REG1[17]
 * GPIO_JTAG_TDO						SDA_B REG1[20]		SDA_A REG1[21]
 * GPIOB_0								SCK_B REG2[5]		SCK_A REG2[6]
 * GPIOB_1								SDA_B REG2[2]		SDA_A REG2[3]
 * GPIOB_2			SCK_A REGS[13]							SCK_A REG2[14]
 * GPIOB_3			SDA_A REG2[9]							SDA_A REG2[10]
 * GPIOC_13								SCK_B REG3[28]		SCK_A REG3[29]
 * GPIOC_14								SDA_B REG3[25]		SDA_A REG3[26]
 * GPIOC_21			SCK_A REG7[9]							SCK_A REG7[10]
 * GPIOC_22			SDA_A REG7[6]							SDA_A REG7[7]
 * GPIOE_16								SCK_B REG5[27]		SCK_A REG5[28]
 * GPIOE_17								SDA_B REG5[25]		SDA_A REG5[26]
*/

/*i2c master a*/
#define MESON_I2C_MASTER_A_JTAG_TMS_REG		CBUS_REG_ADDR(PERIPHS_PIN_MUX_1)
#define MESON_I2C_MASTER_A_JTAG_TMS_BIT		(1<<12)
#define MESON_I2C_MASTER_A_JTAG_TDI_REG		CBUS_REG_ADDR(PERIPHS_PIN_MUX_1)
#define MESON_I2C_MASTER_A_JTAG_TDI_BIT		(1<<12)

#define MESON_I2C_MASTER_A_GPIOB_2_REG  		CBUS_REG_ADDR(PERIPHS_PIN_MUX_2)
#define MESON_I2C_MASTER_A_GPIOB_2_BIT  		(1<<13)
#define MESON_I2C_MASTER_A_GPIOB_3_REG  		CBUS_REG_ADDR(PERIPHS_PIN_MUX_2)
#define MESON_I2C_MASTER_A_GPIOB_3_BIT  		(1<<9)

#define MESON_I2C_MASTER_A_GPIOC_21_REG		CBUS_REG_ADDR(PERIPHS_PIN_MUX_7)
#define MESON_I2C_MASTER_A_GPIOC_21_BIT		(1<<9)
#define MESON_I2C_MASTER_A_GPIOC_22_REG		CBUS_REG_ADDR(PERIPHS_PIN_MUX_7)
#define MESON_I2C_MASTER_A_GPIOC_22_BIT		(1<<6)

/*i2c master b*/
#define MESON_I2C_MASTER_B_JTAG_TCK_REG		CBUS_REG_ADDR(PERIPHS_PIN_MUX_1)
#define MESON_I2C_MASTER_B_JTAG_TCK_BIT		(1<<16)
#define MESON_I2C_MASTER_B_JTAG_TDO_REG		CBUS_REG_ADDR(PERIPHS_PIN_MUX_1)
#define MESON_I2C_MASTER_B_JTAG_TDO_BIT		(1<<20)

#define MESON_I2C_MASTER_B_GPIOB_0_REG  		CBUS_REG_ADDR(PERIPHS_PIN_MUX_2)
#define MESON_I2C_MASTER_B_GPIOB_0_BIT  		(1<<5)
#define MESON_I2C_MASTER_B_GPIOB_1_REG  		CBUS_REG_ADDR(PERIPHS_PIN_MUX_2)
#define MESON_I2C_MASTER_B_GPIOB_1_BIT  		(1<<2)

#define MESON_I2C_MASTER_B_GPIOC_13_REG		CBUS_REG_ADDR(PERIPHS_PIN_MUX_3)
#define MESON_I2C_MASTER_B_GPIOC_13_BIT		(1<<28)
#define MESON_I2C_MASTER_B_GPIOC_14_REG		CBUS_REG_ADDR(PERIPHS_PIN_MUX_3)
#define MESON_I2C_MASTER_B_GPIOC_14_BIT		(1<<25)

#define MESON_I2C_MASTER_B_GPIOC_16_REG		CBUS_REG_ADDR(PERIPHS_PIN_MUX_5)
#define MESON_I2C_MASTER_B_GPIOC_16_BIT		(1<<27)
#define MESON_I2C_MASTER_B_GPIOC_17_REG		CBUS_REG_ADDR(PERIPHS_PIN_MUX_5)
#define MESON_I2C_MASTER_B_GPIOC_17_BIT		(1<<25)

/*i2c slave*/
#define MESON_I2C_SLAVE_JTAG_TMS_REG			CBUS_REG_ADDR(PERIPHS_PIN_MUX_1)
#define MESON_I2C_SLAVE_JTAG_TMS_BIT			(1<<13)
#define MESON_I2C_SLAVE_JTAG_TDI_REG			CBUS_REG_ADDR(PERIPHS_PIN_MUX_1)
#define MESON_I2C_SLAVE_JTAG_TDI_BIT			(1<<13)

#define MESON_I2C_SLAVE_GPIOB_2_REG  			CBUS_REG_ADDR(PERIPHS_PIN_MUX_2)
#define MESON_I2C_SLAVE_GPIOB_2_BIT  			(1<<14)
#define MESON_I2C_SLAVE_GPIOB_3_REG  			CBUS_REG_ADDR(PERIPHS_PIN_MUX_2)
#define MESON_I2C_SLAVE_GPIOB_3_BIT  			(1<<10)

#define MESON_I2C_SLAVE_GPIOC_21_REG			CBUS_REG_ADDR(PERIPHS_PIN_MUX_7)
#define MESON_I2C_SLAVE_GPIOC_21_BIT			(1<<10)
#define MESON_I2C_SLAVE_GPIOC_22_REG			CBUS_REG_ADDR(PERIPHS_PIN_MUX_7)
#define MESON_I2C_SLAVE_GPIOC_22_BIT			(1<<7)

#define MESON_I2C_SLAVE_JTAG_TCK_REG			CBUS_REG_ADDR(PERIPHS_PIN_MUX_1)
#define MESON_I2C_SLAVE_JTAG_TCK_BIT			(1<<17)
#define MESON_I2C_SLAVE_JTAG_TDO_REG			CBUS_REG_ADDR(PERIPHS_PIN_MUX_1)
#define MESON_I2C_SLAVE_JTAG_TDO_BIT			(1<<21)

#define MESON_I2C_SLAVE_GPIOB_0_REG  			CBUS_REG_ADDR(PERIPHS_PIN_MUX_2)
#define MESON_I2C_SLAVE_GPIOB_0_BIT  			(1<<6)
#define MESON_I2C_SLAVE_GPIOB_1_REG  			CBUS_REG_ADDR(PERIPHS_PIN_MUX_2)
#define MESON_I2C_SLAVE_GPIOB_1_BIT  			(1<<3)

#define MESON_I2C_SLAVE_GPIOC_13_REG			CBUS_REG_ADDR(PERIPHS_PIN_MUX_3)
#define MESON_I2C_SLAVE_GPIOC_13_BIT			(1<<29)
#define MESON_I2C_SLAVE_GPIOC_14_REG			CBUS_REG_ADDR(PERIPHS_PIN_MUX_3)
#define MESON_I2C_SLAVE_GPIOC_14_BIT			(1<<26)

#define MESON_I2C_SLAVE_GPIOC_16_REG			CBUS_REG_ADDR(PERIPHS_PIN_MUX_5)
#define MESON_I2C_SLAVE_GPIOC_16_BIT			(1<<28)
#define MESON_I2C_SLAVE_GPIOC_17_REG			CBUS_REG_ADDR(PERIPHS_PIN_MUX_5)
#define MESON_I2C_SLAVE_GPIOC_17_BIT			(1<<26)


#define AML_I2C_SPPED_50K			50000
#define AML_I2C_SPPED_100K			100000
#define AML_I2C_SPPED_200K			200000
#define AML_I2C_SPPED_300K			300000
#define AML_I2C_SPPED_400K			400000

struct aml_pinmux_reg_bit {
	unsigned int	scl_reg;
	unsigned int  			scl_bit;
	unsigned int	sda_reg;
	unsigned int  			sda_bit;
};

struct aml_i2c_platform{
	unsigned int		slave_addr;/*7bit addr*/
	unsigned int 		wait_count;/*i2c wait ack timeout = 
											wait_count * wait_ack_interval */
	unsigned int 		wait_ack_interval;
	unsigned int 		wait_read_interval;
	unsigned int 		wait_xfer_interval;
	unsigned int 		master_no;
	unsigned int		use_pio;/*0: hardware i2c, 1: manual pio i2c*/
	unsigned int		master_i2c_speed;

	/* only need 1 i2c master to comunicate with several devices, 
	  * should I prepare 2 master interface to use simultaneously?*/
	struct aml_pinmux_reg_bit master_a_pinmux;
	struct aml_pinmux_reg_bit master_b_pinmux;

	struct aml_pinmux_reg_bit slave_reg_bit;
};

/**************i2c software gpio***************/

#define MESON_I2C_PREG_GPIOC_OE			CBUS_REG_ADDR(PREG_FGPIO_EN_N)
#define MESON_I2C_PREG_GPIOC_OUTLVL		CBUS_REG_ADDR(PREG_FGPIO_O)
#define MESON_I2C_PREG_GPIOC_INLVL		CBUS_REG_ADDR(PREG_FGPIO_I)

#define MESON_I2C_PREG_GPIOE_OE			CBUS_REG_ADDR(PREG_HGPIO_EN_N)
#define MESON_I2C_PREG_GPIOE_OUTLVL		CBUS_REG_ADDR(PREG_HGPIO_O)
#define MESON_I2C_PREG_GPIOE_INLVL		CBUS_REG_ADDR(PREG_HGPIO_I)

#define MESON_I2C_PREG_GPIOA_OE			CBUS_REG_ADDR(PREG_EGPIO_EN_N)
#define MESON_I2C_PREG_GPIOA_OUTLVL		CBUS_REG_ADDR(PREG_EGPIO_O)
#define MESON_I2C_PREG_GPIOA_INLVL		CBUS_REG_ADDR(PREG_EGPIO_I)

struct aml_sw_i2c_pins
{
	unsigned int scl_reg_out;
	unsigned int scl_reg_in;
	unsigned int scl_bit;
	unsigned int scl_oe;
	unsigned int sda_reg_out;
	unsigned int sda_reg_in;
	unsigned int sda_bit;
	unsigned int sda_oe;
};


struct aml_sw_i2c_platform {
	struct aml_sw_i2c_pins sw_pins;
	
	/* local settings */
	int udelay;		/* half clock cycle time in us,
				   minimum 2 us for fast-mode I2C,
				   minimum 5 us for standard-mode I2C and SMBus,
				   maximum 50 us for SMBus */
	int timeout;		/* in jiffies */	
};

#endif

