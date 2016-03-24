#define DRIVER_NAME "nrf24l01p"
#define _dbg(fmt, args...) 			\
	pr_debug("[DEBUG] " DRIVER_NAME 	\
		":%s(%d): " fmt "\n", 		\
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
#define RPD_REG         0x09
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
	_CMD_R_REGISTER,
	_CMD_W_REGISTER,
	_CMD_R_RX_PAYLOAD,
	_CMD_W_TX_PAYLOAD,
	_CMD_FLUSH_TX,
	_CMD_FLUSH_RX,
	_CMD_REUSE_TX_PL,
	_CMD_R_RX_PL_WID,
	_CMD_W_ACK_PAYLOAD,
	_CMD_W_TX_PAYLOAD_NO_ACK,
	_CMD_NOP,
} nrf24l01p_cmd_t;

struct nrf24l01p {
	struct spi_device *spi;
	struct spi_message m;
#define NRF24L01P_SPI_TRANSFER_MAX 128
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

	if (i > 0)
		rf->t[i-1].cs_change = 1;

	switch (cmd) {
	case _CMD_FLUSH_TX:
	case _CMD_FLUSH_RX:
	case _CMD_REUSE_TX_PL:
	case _CMD_NOP:
		rf->t[i] = (struct spi_transfer) {
			.tx_buf = op,
			.len = 1,
		};
		rf->spi_transfer_index++;
		break;
	case _CMD_R_RX_PL_WID:
	case _CMD_R_RX_PAYLOAD:
	case _CMD_R_REGISTER:
		rf->t[i] = (struct spi_transfer) {
			.tx_buf = op,
			.len = 1,
		};
		rf->t[i+1] = (struct spi_transfer) {
			.rx_buf = val,
			.len = valsiz,
		};
		spi_message_add_tail(&rf->t[i], &rf->m);
		spi_message_add_tail(&rf->t[i+1], &rf->m);
		rf->spi_transfer_index += 2;
		break;
	case _CMD_W_REGISTER:
	case _CMD_W_TX_PAYLOAD:
	case _CMD_W_ACK_PAYLOAD:
	case _CMD_W_TX_PAYLOAD_NO_ACK:
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

static void _wrk_complete(void *arg)
{
	struct work_struct *w = arg;
	schedule_work(w);
}

static int _spi_worker(struct nrf24l01p *rf,
		struct work_struct *w)
{
	return _spi_async(rf, _wrk_complete, w);
}

static void _spi_reinit(struct nrf24l01p *rf)
{
	memset(rf->t, '\0', sizeof(rf->t));
	spi_message_init(&rf->m);
	rf->spi_transfer_index = 0;
}

static size_t _rx_pld_siz(struct nrf24l01p *rf, u8 pipe_no)
{
	u8 feature_val, feature_cmd = R_REGISTER(FEATURE_REG);
	u8 dynpd_val, dynpd_cmd = R_REGISTER(DYNPD_REG);
	u8 rx_pw_p_val, rx_pw_p_cmd = R_REGISTER(RX_PW_P0_REG + pipe_no);
	u8 r_rx_pld_wid_val, r_rx_pld_wid_cmd = R_RX_PL_WID;

	_spi_reinit(rf);
	_spi_push_cmd(rf,
		_CMD_R_REGISTER,
		&feature_cmd,
		&feature_val, 1);
	_spi_push_cmd(rf,
		_CMD_R_REGISTER,
		&dynpd_cmd,
		&dynpd_val, 1);
	_spi_push_cmd(rf,
		_CMD_R_REGISTER,
		&rx_pw_p_cmd,
		&rx_pw_p_val, 1);
	_spi_push_cmd(rf,
		_CMD_R_RX_PL_WID,
		&r_rx_pld_wid_cmd,
		&r_rx_pld_wid_val, 1);
	_spi_sync(rf);


	if (isset(feature_val, BIT(2) &&
	    isset(dynpd_val,   BIT(pipe_no))))
		return r_rx_pld_wid_val;

	return rx_pw_p_val;
}

ssize_t show_reg(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = to_spi_device(dev);
	struct nrf24l01p *rf   = spi_get_drvdata(spi);
	int r; /* register index */
	unsigned int off = 0;
	u8 regs_cmd[FEATURE_REG + 1];
	u8 regs_val[FEATURE_REG + 1][5];

	_spi_reinit(rf);
	for (r = CONFIG_REG; r <= FEATURE_REG; r++) {
		regs_cmd[r] = R_REGISTER(r);
		switch (r) {
		case CONFIG_REG ... RPD_REG:
		case RX_PW_P0_REG ... FIFO_STATUS_REG:
		case DYNPD_REG:
		case FEATURE_REG:
		case RX_ADDR_P2_REG ... RX_ADDR_P5_REG:
			_spi_push_cmd(rf,
				_CMD_R_REGISTER,
				&regs_cmd[r],
				regs_val[r], 1);
			break;
		case RX_ADDR_P0_REG:
		case RX_ADDR_P1_REG:
		case TX_ADDR_REG:
			_spi_push_cmd(rf,
				_CMD_R_REGISTER,
				&regs_cmd[r],
				regs_val[r], 5);
			break;
		default:
			break;
		}	

	}
	_spi_sync(rf);
	for (r = CONFIG_REG; r <= FEATURE_REG; r++) {
		switch (r) {
		case CONFIG_REG ... RPD_REG:
		case RX_PW_P0_REG ... FIFO_STATUS_REG:
		case DYNPD_REG:
		case FEATURE_REG:
		case RX_ADDR_P2_REG ... RX_ADDR_P5_REG:
			off += scnprintf(buf + off,
					PAGE_SIZE - off,
					"%02x:%02x\n",
					regs_cmd[r], regs_val[r][0]);
			break;
		case RX_ADDR_P0_REG:
		case RX_ADDR_P1_REG:
		case TX_ADDR_REG:
			off += scnprintf(buf + off,
					PAGE_SIZE - off,
					"%02x:%02x%02x%02x%02x%02x\n",
					regs_cmd[r], 
					regs_val[r][0],
					regs_val[r][1],
					regs_val[r][2],
					regs_val[r][3],
					regs_val[r][4]);
			break;
		default:
			break;
		}	

	}
	return off;
}

static DEVICE_ATTR(registers, 0400, show_reg, NULL);

static struct attribute *_attrs[] = {
	&dev_attr_registers.attr,
	NULL,
};
static struct attribute_group _attr_group = {
	.attrs = _attrs,
};

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
		_CMD_W_REGISTER,
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
		_CMD_R_REGISTER,
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
	_dbg("");

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
	rf->spi = spi;

	status = sysfs_create_group(&spi->dev.kobj, &_attr_group);	
	if (status) {
		dev_err(&spi->dev, "sysfs_create_group failed");
		goto free_rf;
	}

	return 0;
/* sysfs_remove:
	sysfs_remove_group(&spi->dev.kobj, &_attr_group); */
free_rf:
	kfree(rf);
	return status;
}

static int nrf24l01p_remove(struct spi_device *spi)
{
	struct nrf24l01p*rf = spi_get_drvdata(spi);
	_dbg("");
	sysfs_remove_group(&spi->dev.kobj, &_attr_group);
	free_irq(spi->irq, rf);
	kfree(rf);
	return 0;
}

static const struct of_device_id nrf24l01p_of_ids[] = {
        { .compatible = "nrf24l01p" },
        {},
};

static struct spi_driver nrf24l01p_driver = {
	.driver =  {
		.name = "nrf24l01p",
		.owner = THIS_MODULE,
		.of_match_table = nrf24l01p_of_ids,
	},
	.probe = nrf24l01p_probe,
	.remove = nrf24l01p_remove,
};

int nrf24l01p_init(void)
{
	_dbg("");
	return spi_register_driver(&nrf24l01p_driver);
}

void nrf24l01p_exit(void)
{
	_dbg("");
	spi_unregister_driver(&nrf24l01p_driver);
}
module_init(nrf24l01p_init);
module_exit(nrf24l01p_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Daniel Hilst Selli <danielhilst@gmail.com>");
