
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include "am_regs.h"

#define MESON_SPI_MASTER_A_START		CBUS_REG_ADDR(SPI_FLASH_CMD)
#define MESON_SPI_MASTER_A_END		(CBUS_REG_ADDR(SPI_FLASH_B15+1)-1)

#define MESON_SPI_MASTER_B_START		CBUS_REG_ADDR(SPI2_FLASH_CMD)
#define MESON_SPI_MASTER_B_END		(CBUS_REG_ADDR(SPI2_FLASH_B15+1)-1)

#define MESON_SPI_SLAVE_A_START			CBUS_REG_ADDR(SPI_FLASH_SLAVE)
#define MESON_SPI_SLAVE_A_END			(CBUS_REG_ADDR(SPI_FLASH_B15+1)-1)

#define MESON_SPI_SLAVE_B_START			CBUS_REG_ADDR(SPI2_FLASH_SLAVE)
#define MESON_SPI_SLAVE_B_END			(CBUS_REG_ADDR(SPI2_FLASH_B15+1)-1)

#define AML_SPI_MASTER_A			0
#define AML_SPI_MASTER_B 			1

#define AML_SPI_SLAVE_A			    0
#define AML_SPI_SLAVE_B 			1

#define SPI_0		0
#define SPI_1		1
#define SPI_2		2

//spi clk low than 3M 
#define SPI_CLK_1M      1000000
#define SPI_CLK_1M5     1500000
#define SPI_CLK_2M      2000000
#define SPI_CLK_2M5     2500000

typedef enum _SPI_Pad_Type
{
    SPI_A_GPIOE_0_7,
    SPI_B_GPIOA_9_14,
    SPI_B_GPIOB_0_7
} SPI_Pad_Type_t;

#define SPI_DEV_NAME				"aml_spi"
/* Status Register bits. */
#define	SR_WIP			1	/* Write in progress */
#define	SR_WEL			2	/* Write enable latch */
/* meaning of other SR_* bits may differ between vendors */
#define	SR_BP0			4	/* Block protect 0 */
#define	SR_BP1			8	/* Block protect 1 */
#define	SR_BP2			0x10	/* Block protect 2 */
#define	SR_SRWD			0x80	/* SR write protect */

#define	CMD_SIZE		4

#ifdef CONFIG_SPI_USE_FAST_READ
#define OPCODE_READ 	OPCODE_FAST_READ
#define FAST_READ_DUMMY_BYTE 1
#else
#define OPCODE_READ 	OPCODE_NORM_READ
#define FAST_READ_DUMMY_BYTE 0
#endif

struct aml_spi_platform{
	unsigned int 		master_no;
	unsigned int        io_pad_type;
	unsigned int		master_spi_speed;	
	unsigned char       num_cs;
	unsigned char       wp_pin_enable;
	unsigned char       hold_pin_enable;
};

