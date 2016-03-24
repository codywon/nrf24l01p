#define DRIVER_NAME "nrf24l01p"
#define __dbg(fmt, args...) 			\
	pr_debug("[DEBUG] " DRIVER_NAME 	\
		"::%s(%d): " fmt "\n", 		\
		__func__, __LINE__, ##args)

/* bitops */
#define isset(bit, mask) (bit & mask)
#define setbit(v, mask)  (v |= (mask))
#define clearbit(v, mask) (v &= ~(mask))

/* Commands */
#define R_REGISTER(addr) (addr)
#define W_REGISTER(addr) ((addr) + 0x20)
#define W_ACK_PAYLOAD(p) (0xa8 | (0x07 & p))  /* write ack payload */

#define R_RX_PAYLOAD 0x61
#define W_TX_PAYLOAD 0xa0
#define FLUSH_TX     0xe1
#define FLUSH_RX     0xe2
#define REUSE_TX_PL  0xe3
#define ACTIVATE     0x50
#define R_RX_PL_WID  0x60
#define W_TX_PAYLOAD_NO_ACK 0xb0
#define NOP_CMD      0xff

/* Registers Address */
#define CONFIG_REG      0x00
#define EN_AA_REG       0x01
#define EN_RXADDR_REG   0x02
#define SETUP_AW_REG    0x03
#define SETUP_RETR_REG  0x04
#define RF_CH_REG       0x05
#define RF_SETUP_REG    0x06
#define NRF24L01_RF_SETUP_PLL_LOCK     BIT(4)
#define NRF24L01_RF_SETUP_RF_DR        BIT(3)
#define NRF24L01_RF_SETUP_RF_PWR_HIGH  BIT(2)
#define NRF24L01_RF_SETUP_RF_PWR_LOW   BIT(1)
#define NRF24L01_RF_SETUP_LNA_HCURR    BIT(0)
#define NRF24L01P_RF_SETUP_CONT_WAVE   BIT(7)
#define NRF24L01P_RF_SETUP_RF_DR_LOW   BIT(6)
#define NRF24L01P_RF_SETUP_PLL_LOCK    BIT(4)
#define NRF24L01P_RF_SETUP_RF_DR_HIGH  BIT(3)
#define NRF24L01P_RF_SETUP_RF_PWR_HIGH BIT(2)
#define NRF24L01P_RF_SETUP_RF_PWR_LOW  BIT(1)
#define NRF24L01P_RF_SETUP
#define STATUS_REG      0x07
#define _RX_DR BIT(6)
#define _TX_DS BIT(5)
#define OBSERVE_TX_REG  0x08
#define CD_REG          0x09
#define RX_ADDR_P0_REG  0x0a
#define RX_ADDR_P1_REG  0x0b
#define RX_ADDR_P2_REG  0x0c
#define RX_ADDR_P3_REG  0x0d
#define RX_ADDR_P4_REG  0x0e
#define RX_ADDR_P5_REG  0x0f
#define TX_ADDR_REG     0x10
#define RX_PW_P0_REG    0x11
#define RX_PW_P1_REG    0x12
#define RX_PW_P2_REG    0x13
#define RX_PW_P3_REG    0x14
#define RX_PW_P4_REG    0x15
#define RX_PW_P5_REG    0x16
#define FIFO_STATUS_REG 0x17
#define DYNPD_REG       0x1c
#define FEATURE_REG     0x1d


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/workqueue.h>

typedef enum {
	__CMD_R_REGISTER,
	__CMD_W_REGISTER,
	__CMD_R_RX_PAYLOAD,
	__CMD_W_TX_PAYLOAD,
	__CMD_FLUSH_TX,
	__CMD_FLUSH_RX,
	__CMD_REUSE_TX_PL,
	__CMD_R_RX_PL_WID,
	__CMD_W_ACK_PAYLOAD,
	__CMD_W_TX_PAYLOAD_NO_ACK,
	__CMD_NOP,
} nrf24l01p_cmd_t;

struct nrf24l01p {
	struct spi_device *spi;
	struct spi_message m;
#define NRF24L01P_SPI_TRANSFER_MAX 10
	struct spi_transfer t[NRF24L01P_SPI_TRANSFER_MAX];
	unsigned int spi_transfer_index;
	u8 status;
	struct sk_buff *tx_skb;
	struct work_struct irq_worker;
	int ce;
};

static int _spi_push_cmd(struct nrf24l01p *rf,
		nrf24l01p_cmd_t cmd,
		u8 *op,
		u8 *val,
		size_t valsiz)
{
	int i = rf->spi_transfer_index;
	if (i >= NRF24L01P_SPI_TRANSFER_MAX) {
		dev_err(&rf->spi->dev, "To many spi transfer attached, no more space");
		return -1;
	}

	switch (cmd) {
	case __CMD_FLUSH_TX:
	case __CMD_FLUSH_RX:
	case __CMD_REUSE_TX_PL:
	case __CMD_NOP:
		rf->t[i] = (struct spi_transfer) {
			.tx_buf = op,
			.len = 1,
		};
		rf->spi_transfer_index++;
		break;
	case __CMD_R_RX_PL_WID:
	case __CMD_R_RX_PAYLOAD:
	case __CMD_R_REGISTER:
		rf->t[i] = (struct spi_transfer) {
			.tx_buf = op,
			.len = 1,
			.cs_change = 1,
		};
		rf->t[i+1] = (struct spi_transfer) {
			.rx_buf = val,
			.len = valsiz,
		};
		spi_message_add_tail(&rf->t[i], &rf->m);
		spi_message_add_tail(&rf->t[i+1], &rf->m);
		rf->spi_transfer_index += 2;
		break;
	case __CMD_W_REGISTER:
	case __CMD_W_TX_PAYLOAD:
	case __CMD_W_ACK_PAYLOAD:
	case __CMD_W_TX_PAYLOAD_NO_ACK:
		rf->t[i] = (struct spi_transfer) {
			.tx_buf = op,
			.len = 1,
			.cs_change = 1,
		};
		rf->t[i+1] = (struct spi_transfer) {
			.tx_buf = val,
			.len = valsiz,
		};
		spi_message_add_tail(&rf->t[i], &rf->m);
		spi_message_add_tail(&rf->t[i+1], &rf->m);
		rf->spi_transfer_index += 2;
		break;
	default:
		BUG_ON(1);
	}
	return 0;
}

static inline u8 *_spi_pop_cmd(struct nrf24l01p *rf)
{
	BUG_ON(rf->spi_transfer_index <= 0);
	return rf->t[--rf->spi_transfer_index].rx_buf;
}

static int _spi_sync(struct nrf24l01p *rf)
{
	return spi_sync(rf->spi, &rf->m);
}

static int _spi_async(struct nrf24l01p *rf,
		void (*_complete)(void *),
		void *arg)
{
	rf->m.complete = _complete;
	rf->m.context  = arg;
	return spi_async(rf->spi, &rf->m);
}

static void __wrk_complete(void *arg)
{
	struct work_struct *w = arg;
	schedule_work(w);
}

static int _spi_worker(struct nrf24l01p *rf,
		struct work_struct *w)
{
	return _spi_async(rf, __wrk_complete, w);
}

static void _spi_reinit(struct nrf24l01p *rf)
{
	memset(&rf->m, '\0', sizeof(rf->m));
	memset(rf->t, '\0', sizeof(rf->t));
	spi_message_init(&rf->m);
	rf->spi_transfer_index = 0;
}

static size_t __rx_pld_siz(struct nrf24l01p *rf, u8 pipe_no)
{
	u8 feature_val, feature_cmd = R_REGISTER(FEATURE_REG);
	u8 dynpd_val, dynpd_cmd = R_REGISTER(DYNPD_REG);
	u8 rx_pw_p_val, rx_pw_p_cmd = R_REGISTER(RX_PW_P0_REG + pipe_no);
	u8 r_rx_pld_wid_val, r_rx_pld_wid_cmd = R_RX_PL_WID;

	_spi_reinit(rf);
	_spi_push_cmd(rf,
		__CMD_R_REGISTER,
		&feature_cmd,
		&feature_val, 1);
	_spi_push_cmd(rf,
		__CMD_R_REGISTER,
		&dynpd_cmd,
		&dynpd_val, 1);
	_spi_push_cmd(rf,
		__CMD_R_REGISTER,
		&rx_pw_p_cmd,
		&rx_pw_p_val, 1);
	_spi_push_cmd(rf,
		__CMD_R_RX_PL_WID,
		&r_rx_pld_wid_cmd,
		&r_rx_pld_wid_val, 1);
	_spi_sync(rf);


	if (isset(feature_val, BIT(2) &&
	    isset(dynpd_val,   BIT(pipe_no))))
		return r_rx_pld_wid_val;

	return rx_pw_p_val;
}

static void nrf24l01p_irq_worker(struct work_struct *w)
{
	struct nrf24l01p *rf = container_of(w, struct nrf24l01p, irq_worker);
	u8 *status_val;
	u8 status_cmd = W_REGISTER(STATUS_REG);

	BUG_ON(!rf);
	status_val = _spi_pop_cmd(rf);
	if (isset(*status_val, _TX_DS)) {

	}
	if (isset(*status_val, _RX_DR)) {

	}
	_spi_reinit(rf);
	_spi_push_cmd(rf,
		__CMD_W_REGISTER,
		&status_cmd,
		status_val, 1);
	_spi_sync(rf);
}

static irqreturn_t nrf24l01p_irq(int irq, void *arg)
{
	static u8 status_val, status_cmd = R_REGISTER(STATUS_REG);
	struct nrf24l01p *rf = arg;

	INIT_WORK(&rf->irq_worker, nrf24l01p_irq_worker);
	_spi_reinit(rf);
	_spi_push_cmd(rf,
		__CMD_R_REGISTER,
		&status_cmd,
		&status_val, 1);
	BUG_ON(_spi_worker(rf, &rf->irq_worker));
	return IRQ_HANDLED;
}

static int nrf24l01p_probe(struct spi_device *spi)
{
	int status = -ENODEV;
	int gpio_ce, gpio_irq;
	struct nrf24l01p*rf = NULL;
	struct device_node *np= spi->dev.of_node;
	__dbg("");

	if (!np) {
		dev_err(&spi->dev, "no device-tree node");
		return -ENODEV;
	}

	rf = kzalloc(sizeof(*rf), GFP_KERNEL);
	if (!rf) {
		dev_err(&spi->dev, "no memory");
		return -ENOMEM;
	}

	gpio_irq = of_get_named_gpio(np, "gpio-irq", 0);
	gpio_ce  = of_get_named_gpio(np, "gpio-ce", 0);
	if (gpio_irq < 0 ||
	    gpio_ce  < 0) {
		dev_err(&spi->dev, "no gpio-ce or gpio-irq in device-tree node");
		status = -ENODEV;
		goto free_rf;
	}

	spi->irq = gpio_to_irq(gpio_irq);
	if (spi->irq < 0) {
		dev_err(&spi->dev, "invalid irq");
		status = -ENODEV;
		goto free_rf;
	}
	spi_set_drvdata(spi, rf);

	status = request_irq(spi->irq, nrf24l01p_irq, IRQF_TRIGGER_FALLING, "nRF24L01+", rf);
	if (status) {
		dev_err(&spi->dev, "request_irq failed");
		goto free_rf;
	}
	rf->ce = gpio_ce;

	{ 
		u8 tx_addr_cmd = R_REGISTER(TX_ADDR_REG);
		u8 tx_addr[5];
		_spi_reinit(rf);
		_spi_push_cmd(rf, 
			__CMD_R_REGISTER,
			&tx_addr_cmd,
			tx_addr, 5);	
		_spi_sync(rf);
		__dbg("TX addr: %02x:%02x:%02x:%02x:%02x",
			tx_addr[0], tx_addr[1], tx_addr[2], 
			tx_addr[3], tx_addr[4]); 

	}

	return 0;
free_rf:
	kfree(rf);
	return status;
}

static int nrf24l01p_remove(struct spi_device *spi)
{
	struct nrf24l01p*rf = spi_get_drvdata(spi);
	__dbg("");
	free_irq(spi->irq, rf);
	kfree(rf);
	return 0;
}

static const struct of_device_id nrf24l01p_of_ids[] = {
        { .compatible = "nrf24l01+" },
        {},
};

static struct spi_driver nrf24l01p_driver = {
	.driver =  {
		.name = "nrf24l01+",
		.owner = THIS_MODULE,
		.of_match_table = nrf24l01p_of_ids,
	},
	.probe = nrf24l01p_probe,
	.remove = nrf24l01p_remove,
};

int nrf24l01p_init(void)
{
	__dbg("");
	return spi_register_driver(&nrf24l01p_driver);
}

void nrf24l01p_exit(void)
{
	__dbg("");
	spi_unregister_driver(&nrf24l01p_driver);
}
module_init(nrf24l01p_init);
module_exit(nrf24l01p_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Daniel Hilst Selli <danielhilst@gmail.com>");
