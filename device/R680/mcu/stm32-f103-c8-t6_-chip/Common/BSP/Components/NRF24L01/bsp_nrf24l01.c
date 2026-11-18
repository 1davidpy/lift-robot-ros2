#include "bsp_nrf24l01.h"

#include "bsp_pin.h"

#ifndef BSP_NRF24_SPI
#error "BSP_NRF24L01 requires project NRF24L01 SPI macros in bsp_pin.h"
#endif

#define NRF24_CMD_R_REGISTER          0x00U
#define NRF24_CMD_W_REGISTER          0x20U
#define NRF24_CMD_ACTIVATE            0x50U
#define NRF24_CMD_R_RX_PL_WID         0x60U
#define NRF24_CMD_R_RX_PAYLOAD        0x61U
#define NRF24_CMD_W_TX_PAYLOAD        0xA0U
#define NRF24_CMD_W_ACK_PAYLOAD       0xA8U
#define NRF24_CMD_FLUSH_TX            0xE1U
#define NRF24_CMD_FLUSH_RX            0xE2U
#define NRF24_CMD_NOP                 0xFFU

#define NRF24_REG_CONFIG              0x00U
#define NRF24_REG_EN_AA               0x01U
#define NRF24_REG_EN_RXADDR           0x02U
#define NRF24_REG_SETUP_AW            0x03U
#define NRF24_REG_SETUP_RETR          0x04U
#define NRF24_REG_RF_CH               0x05U
#define NRF24_REG_RF_SETUP            0x06U
#define NRF24_REG_STATUS              0x07U
#define NRF24_REG_RX_ADDR_P0          0x0AU
#define NRF24_REG_TX_ADDR             0x10U
#define NRF24_REG_RX_PW_P0            0x11U
#define NRF24_REG_DYNPD               0x1CU
#define NRF24_REG_FEATURE             0x1DU

#define NRF24_CONFIG_EN_CRC           0x08U
#define NRF24_CONFIG_CRCO             0x04U
#define NRF24_CONFIG_PWR_UP           0x02U
#define NRF24_CONFIG_PRIM_RX          0x01U

#define NRF24_RF_DR_LOW               0x20U
#define NRF24_RF_DR_HIGH              0x08U
#define NRF24_RF_PWR_MASK             0x06U

#define NRF24_SETUP_AW_3_BYTES        0x01U
#define NRF24_SETUP_AW_4_BYTES        0x02U
#define NRF24_SETUP_AW_5_BYTES        0x03U

#define NRF24_FEATURE_EN_DYN_ACK      0x01U
#define NRF24_FEATURE_EN_ACK_PAY      0x02U
#define NRF24_FEATURE_EN_DPL          0x04U
#define NRF24_DYNPD_DPL_P0            0x01U
#define NRF24_STATUS_TX_FULL          0x01U

#define NRF24_SPI_WAIT_LIMIT          100000U

static BspNrf24l01DelayMsCallback nrf24_delay_ms_callback;
static BspNrf24l01DelayUsCallback nrf24_delay_us_callback;
static uint8_t nrf24_payload_length = BSP_NRF24L01_DEFAULT_PAYLOAD_LEN;

static void nrf24_default_delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++)
    {
        for (volatile uint32_t j = 0; j < 8000U; j++)
        {
        }
    }
}

static void nrf24_default_delay_us(uint32_t us)
{
    for (uint32_t i = 0; i < us; i++)
    {
        for (volatile uint32_t j = 0; j < 8U; j++)
        {
        }
    }
}

static void nrf24_delay_ms(uint32_t ms)
{
    if (nrf24_delay_ms_callback != 0)
    {
        nrf24_delay_ms_callback(ms);
    }
    else
    {
        nrf24_default_delay_ms(ms);
    }
}

static void nrf24_delay_us(uint32_t us)
{
    if (nrf24_delay_us_callback != 0)
    {
        nrf24_delay_us_callback(us);
    }
    else
    {
        nrf24_default_delay_us(us);
    }
}

static void nrf24_csn_low(void)
{
    GPIO_ResetBits(BSP_NRF24_CSN_PORT, BSP_NRF24_CSN_PIN);
}

static void nrf24_csn_high(void)
{
    GPIO_SetBits(BSP_NRF24_CSN_PORT, BSP_NRF24_CSN_PIN);
}

static void nrf24_ce_low(void)
{
    GPIO_ResetBits(BSP_NRF24_CE_PORT, BSP_NRF24_CE_PIN);
}

static void nrf24_ce_high(void)
{
    GPIO_SetBits(BSP_NRF24_CE_PORT, BSP_NRF24_CE_PIN);
}

static uint8_t nrf24_spi_transfer(uint8_t data)
{
    uint32_t wait_count = NRF24_SPI_WAIT_LIMIT;

    while (SPI_I2S_GetFlagStatus(BSP_NRF24_SPI, SPI_I2S_FLAG_TXE) == RESET)
    {
        if (wait_count-- == 0U)
        {
            return 0xFFU;
        }
    }

    SPI_I2S_SendData(BSP_NRF24_SPI, data);
    wait_count = NRF24_SPI_WAIT_LIMIT;

    while (SPI_I2S_GetFlagStatus(BSP_NRF24_SPI, SPI_I2S_FLAG_RXNE) == RESET)
    {
        if (wait_count-- == 0U)
        {
            return 0xFFU;
        }
    }

    return (uint8_t)SPI_I2S_ReceiveData(BSP_NRF24_SPI);
}

static uint8_t nrf24_read_register(uint8_t reg)
{
    uint8_t value;

    nrf24_csn_low();
    (void)nrf24_spi_transfer(NRF24_CMD_R_REGISTER | (reg & 0x1FU));
    value = nrf24_spi_transfer(NRF24_CMD_NOP);
    nrf24_csn_high();

    return value;
}

static uint8_t nrf24_write_register(uint8_t reg, uint8_t value)
{
    uint8_t status;

    nrf24_csn_low();
    status = nrf24_spi_transfer(NRF24_CMD_W_REGISTER | (reg & 0x1FU));
    (void)nrf24_spi_transfer(value);
    nrf24_csn_high();

    return status;
}

static uint8_t nrf24_read_buffer(uint8_t reg, uint8_t *buffer, uint8_t length)
{
    uint8_t status;

    nrf24_csn_low();
    status = nrf24_spi_transfer(reg);
    for (uint8_t i = 0; i < length; i++)
    {
        buffer[i] = nrf24_spi_transfer(NRF24_CMD_NOP);
    }
    nrf24_csn_high();

    return status;
}

static uint8_t nrf24_write_buffer(uint8_t reg, const uint8_t *buffer, uint8_t length)
{
    uint8_t status;

    nrf24_csn_low();
    status = nrf24_spi_transfer(reg);
    for (uint8_t i = 0; i < length; i++)
    {
        (void)nrf24_spi_transfer(buffer[i]);
    }
    nrf24_csn_high();

    return status;
}

static void nrf24_send_command(uint8_t command)
{
    nrf24_csn_low();
    (void)nrf24_spi_transfer(command);
    nrf24_csn_high();
}

static void nrf24_activate_features(void)
{
    nrf24_csn_low();
    (void)nrf24_spi_transfer(NRF24_CMD_ACTIVATE);
    (void)nrf24_spi_transfer(0x73U);
    nrf24_csn_high();
}

static uint8_t nrf24_read_dynamic_payload_width(void)
{
    uint8_t width;

    nrf24_csn_low();
    (void)nrf24_spi_transfer(NRF24_CMD_R_RX_PL_WID);
    width = nrf24_spi_transfer(NRF24_CMD_NOP);
    nrf24_csn_high();

    return width;
}

static uint8_t nrf24_address_width_register_value(uint8_t length)
{
    switch (length)
    {
    case 3U:
        return NRF24_SETUP_AW_3_BYTES;
    case 4U:
        return NRF24_SETUP_AW_4_BYTES;
    case 5U:
    default:
        return NRF24_SETUP_AW_5_BYTES;
    }
}

static void nrf24_gpio_init(void)
{
    GPIO_InitTypeDef gpio_init;

    RCC_APB2PeriphClockCmd(BSP_NRF24_CSN_CLOCK |
                               BSP_NRF24_CE_CLOCK |
                               BSP_NRF24_IRQ_CLOCK |
                               BSP_NRF24_SCK_CLOCK |
                               BSP_NRF24_MISO_CLOCK |
                               BSP_NRF24_MOSI_CLOCK |
                               RCC_APB2Periph_AFIO,
                           ENABLE);

    gpio_init.GPIO_Pin = BSP_NRF24_CSN_PIN;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(BSP_NRF24_CSN_PORT, &gpio_init);

    gpio_init.GPIO_Pin = BSP_NRF24_CE_PIN;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(BSP_NRF24_CE_PORT, &gpio_init);

    gpio_init.GPIO_Pin = BSP_NRF24_IRQ_PIN;
    gpio_init.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(BSP_NRF24_IRQ_PORT, &gpio_init);

    gpio_init.GPIO_Pin = BSP_NRF24_SCK_PIN;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(BSP_NRF24_SCK_PORT, &gpio_init);

    gpio_init.GPIO_Pin = BSP_NRF24_MOSI_PIN;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(BSP_NRF24_MOSI_PORT, &gpio_init);

    gpio_init.GPIO_Pin = BSP_NRF24_MISO_PIN;
    gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(BSP_NRF24_MISO_PORT, &gpio_init);

    nrf24_csn_high();
    nrf24_ce_low();
}

static void nrf24_spi_init(void)
{
    SPI_InitTypeDef spi_init;

    RCC_APB2PeriphClockCmd(BSP_NRF24_SPI_CLOCK, ENABLE);
    SPI_I2S_DeInit(BSP_NRF24_SPI);

    spi_init.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spi_init.SPI_Mode = SPI_Mode_Master;
    spi_init.SPI_DataSize = SPI_DataSize_8b;
    spi_init.SPI_CPOL = SPI_CPOL_Low;
    spi_init.SPI_CPHA = SPI_CPHA_1Edge;
    spi_init.SPI_NSS = SPI_NSS_Soft;
    spi_init.SPI_BaudRatePrescaler = BSP_NRF24_SPI_BAUD_PRESCALER;
    spi_init.SPI_FirstBit = SPI_FirstBit_MSB;
    spi_init.SPI_CRCPolynomial = 7U;

    SPI_Init(BSP_NRF24_SPI, &spi_init);
    SPI_Cmd(BSP_NRF24_SPI, ENABLE);
}

void BSP_NRF24L01_SetDelayCallbacks(BspNrf24l01DelayMsCallback delay_ms,
                                    BspNrf24l01DelayUsCallback delay_us)
{
    nrf24_delay_ms_callback = delay_ms;
    nrf24_delay_us_callback = delay_us;
}

BspNrf24l01Result BSP_NRF24L01_Init(void)
{
    static const uint8_t default_address[BSP_NRF24L01_ADDRESS_WIDTH] = {
        0x34U, 0x43U, 0x10U, 0x10U, 0x01U,
    };

    nrf24_gpio_init();
    nrf24_spi_init();
    nrf24_delay_ms(5U);

    nrf24_ce_low();
    nrf24_csn_high();
    BSP_NRF24L01_ClearIrq(BSP_NRF24L01_IRQ_ALL);
    BSP_NRF24L01_FlushRx();
    BSP_NRF24L01_FlushTx();

    (void)nrf24_write_register(NRF24_REG_FEATURE, 0x00U);
    (void)nrf24_write_register(NRF24_REG_EN_AA, 0x01U);
    (void)nrf24_write_register(NRF24_REG_EN_RXADDR, 0x01U);
    (void)nrf24_write_register(NRF24_REG_SETUP_AW, NRF24_SETUP_AW_5_BYTES);
    (void)nrf24_write_register(NRF24_REG_SETUP_RETR, 0x4FU);
    (void)nrf24_write_register(NRF24_REG_RF_CH, 0U);
    (void)BSP_NRF24L01_SetRfConfig(BSP_NRF24L01_DATA_RATE_1MBPS, BSP_NRF24L01_POWER_0DBM);
    (void)BSP_NRF24L01_SetPayloadLength(BSP_NRF24L01_DEFAULT_PAYLOAD_LEN);
    (void)BSP_NRF24L01_SetAddress(default_address, BSP_NRF24L01_ADDRESS_WIDTH);
    (void)nrf24_write_register(NRF24_REG_CONFIG,
                               NRF24_CONFIG_EN_CRC |
                                   NRF24_CONFIG_CRCO |
                                   NRF24_CONFIG_PWR_UP);
    nrf24_delay_ms(2U);

    return BSP_NRF24L01_IsPresent() ? BSP_NRF24L01_OK : BSP_NRF24L01_ERROR;
}

bool BSP_NRF24L01_IsPresent(void)
{
    const uint8_t original = nrf24_read_register(NRF24_REG_SETUP_AW);

    (void)nrf24_write_register(NRF24_REG_SETUP_AW, NRF24_SETUP_AW_5_BYTES);
    if (nrf24_read_register(NRF24_REG_SETUP_AW) != NRF24_SETUP_AW_5_BYTES)
    {
        (void)nrf24_write_register(NRF24_REG_SETUP_AW, original);
        return false;
    }

    (void)nrf24_write_register(NRF24_REG_SETUP_AW, original);
    return true;
}

BspNrf24l01Result BSP_NRF24L01_SetChannel(uint8_t channel)
{
    if (channel > 125U)
    {
        return BSP_NRF24L01_INVALID_ARGUMENT;
    }

    (void)nrf24_write_register(NRF24_REG_RF_CH, channel);
    return BSP_NRF24L01_OK;
}

BspNrf24l01Result BSP_NRF24L01_SetRfConfig(BspNrf24l01DataRate data_rate,
                                           BspNrf24l01Power power)
{
    uint8_t rf_setup = ((uint8_t)power << 1U) & NRF24_RF_PWR_MASK;

    switch (data_rate)
    {
    case BSP_NRF24L01_DATA_RATE_2MBPS:
        rf_setup |= NRF24_RF_DR_HIGH;
        break;
    case BSP_NRF24L01_DATA_RATE_250KBPS:
        rf_setup |= NRF24_RF_DR_LOW;
        break;
    case BSP_NRF24L01_DATA_RATE_1MBPS:
        break;
    default:
        return BSP_NRF24L01_INVALID_ARGUMENT;
    }

    (void)nrf24_write_register(NRF24_REG_RF_SETUP, rf_setup);
    return BSP_NRF24L01_OK;
}

BspNrf24l01Result BSP_NRF24L01_SetAddress(const uint8_t *address, uint8_t length)
{
    if (address == 0 || length < 3U || length > BSP_NRF24L01_ADDRESS_WIDTH)
    {
        return BSP_NRF24L01_INVALID_ARGUMENT;
    }

    (void)nrf24_write_register(NRF24_REG_SETUP_AW, nrf24_address_width_register_value(length));
    (void)nrf24_write_buffer(NRF24_CMD_W_REGISTER | NRF24_REG_TX_ADDR, address, length);
    (void)nrf24_write_buffer(NRF24_CMD_W_REGISTER | NRF24_REG_RX_ADDR_P0, address, length);

    return BSP_NRF24L01_OK;
}

BspNrf24l01Result BSP_NRF24L01_SetPayloadLength(uint8_t length)
{
    if (length == 0U || length > BSP_NRF24L01_MAX_PAYLOAD_LENGTH)
    {
        return BSP_NRF24L01_INVALID_ARGUMENT;
    }

    nrf24_payload_length = length;
    (void)nrf24_write_register(NRF24_REG_RX_PW_P0, length);

    return BSP_NRF24L01_OK;
}

BspNrf24l01Result BSP_NRF24L01_EnableAckPayload(void)
{
    uint8_t feature;

    (void)nrf24_write_register(NRF24_REG_FEATURE, NRF24_FEATURE_EN_DPL | NRF24_FEATURE_EN_ACK_PAY);
    feature = nrf24_read_register(NRF24_REG_FEATURE);
    if ((feature & (NRF24_FEATURE_EN_DPL | NRF24_FEATURE_EN_ACK_PAY)) !=
        (NRF24_FEATURE_EN_DPL | NRF24_FEATURE_EN_ACK_PAY))
    {
        nrf24_activate_features();
        (void)nrf24_write_register(NRF24_REG_FEATURE,
                                   NRF24_FEATURE_EN_DPL | NRF24_FEATURE_EN_ACK_PAY);
        feature = nrf24_read_register(NRF24_REG_FEATURE);
    }

    if ((feature & (NRF24_FEATURE_EN_DPL | NRF24_FEATURE_EN_ACK_PAY)) !=
        (NRF24_FEATURE_EN_DPL | NRF24_FEATURE_EN_ACK_PAY))
    {
        return BSP_NRF24L01_ERROR;
    }

    (void)nrf24_write_register(NRF24_REG_DYNPD, NRF24_DYNPD_DPL_P0);

    return BSP_NRF24L01_OK;
}

BspNrf24l01Result BSP_NRF24L01_WriteAckPayload(uint8_t pipe,
                                               const uint8_t *data,
                                               uint8_t length)
{
    uint8_t status;

    if (pipe > 5U || data == 0 || length == 0U ||
        length > BSP_NRF24L01_MAX_PAYLOAD_LENGTH)
    {
        return BSP_NRF24L01_INVALID_ARGUMENT;
    }

    status = nrf24_write_buffer((uint8_t)(NRF24_CMD_W_ACK_PAYLOAD | pipe), data, length);
    return ((status & NRF24_STATUS_TX_FULL) == 0U) ? BSP_NRF24L01_OK
                                                   : BSP_NRF24L01_ERROR;
}

BspNrf24l01Result BSP_NRF24L01_StartRx(void)
{
    uint8_t config;

    nrf24_ce_low();
    BSP_NRF24L01_ClearIrq(BSP_NRF24L01_IRQ_ALL);
    BSP_NRF24L01_FlushRx();

    config = nrf24_read_register(NRF24_REG_CONFIG);
    config |= NRF24_CONFIG_PWR_UP | NRF24_CONFIG_PRIM_RX;
    config |= NRF24_CONFIG_EN_CRC | NRF24_CONFIG_CRCO;
    (void)nrf24_write_register(NRF24_REG_CONFIG, config);
    nrf24_delay_ms(2U);
    nrf24_ce_high();
    nrf24_delay_us(130U);

    return BSP_NRF24L01_OK;
}

void BSP_NRF24L01_Standby(void)
{
    nrf24_ce_low();
}

BspNrf24l01Result BSP_NRF24L01_Send(const uint8_t *data, uint8_t length, uint32_t timeout_ms)
{
    uint8_t status;
    uint8_t config;
    uint8_t payload[BSP_NRF24L01_MAX_PAYLOAD_LENGTH] = {0U};

    if (data == 0 || length == 0U || length > nrf24_payload_length)
    {
        return BSP_NRF24L01_INVALID_ARGUMENT;
    }

    nrf24_ce_low();
    BSP_NRF24L01_ClearIrq(BSP_NRF24L01_IRQ_ALL);
    BSP_NRF24L01_FlushTx();

    config = nrf24_read_register(NRF24_REG_CONFIG);
    config &= (uint8_t)~NRF24_CONFIG_PRIM_RX;
    config |= NRF24_CONFIG_PWR_UP | NRF24_CONFIG_EN_CRC | NRF24_CONFIG_CRCO;
    (void)nrf24_write_register(NRF24_REG_CONFIG, config);
    nrf24_delay_ms(2U);

    for (uint8_t i = 0; i < length; i++)
    {
        payload[i] = data[i];
    }

    (void)nrf24_write_buffer(NRF24_CMD_W_TX_PAYLOAD, payload, nrf24_payload_length);
    nrf24_ce_high();
    nrf24_delay_us(15U);
    nrf24_ce_low();

    do
    {
        status = BSP_NRF24L01_ReadStatus();
        if ((status & BSP_NRF24L01_IRQ_TX_DS) != 0U)
        {
            BSP_NRF24L01_ClearIrq(BSP_NRF24L01_IRQ_TX_DS);
            return BSP_NRF24L01_OK;
        }

        if ((status & BSP_NRF24L01_IRQ_MAX_RT) != 0U)
        {
            BSP_NRF24L01_ClearIrq(BSP_NRF24L01_IRQ_MAX_RT);
            BSP_NRF24L01_FlushTx();
            return BSP_NRF24L01_TX_FAILED;
        }

        nrf24_delay_ms(1U);
    } while (timeout_ms-- > 0U);

    BSP_NRF24L01_FlushTx();
    return BSP_NRF24L01_TIMEOUT;
}

BspNrf24l01Result BSP_NRF24L01_SendWithAckPayload(const uint8_t *data,
                                                  uint8_t length,
                                                  uint8_t *ack_buffer,
                                                  uint8_t ack_buffer_size,
                                                  uint8_t *ack_length,
                                                  uint32_t timeout_ms)
{
    uint8_t status;
    uint8_t config;

    if (data == 0 || length == 0U || length > BSP_NRF24L01_MAX_PAYLOAD_LENGTH ||
        ack_buffer == 0 || ack_length == 0U || ack_buffer_size == 0U)
    {
        return BSP_NRF24L01_INVALID_ARGUMENT;
    }

    *ack_length = 0U;
    nrf24_ce_low();
    BSP_NRF24L01_ClearIrq(BSP_NRF24L01_IRQ_ALL);
    BSP_NRF24L01_FlushTx();
    BSP_NRF24L01_FlushRx();

    config = nrf24_read_register(NRF24_REG_CONFIG);
    config &= (uint8_t)~NRF24_CONFIG_PRIM_RX;
    config |= NRF24_CONFIG_PWR_UP | NRF24_CONFIG_EN_CRC | NRF24_CONFIG_CRCO;
    (void)nrf24_write_register(NRF24_REG_CONFIG, config);
    nrf24_delay_ms(2U);

    (void)nrf24_write_buffer(NRF24_CMD_W_TX_PAYLOAD, data, length);
    nrf24_ce_high();
    nrf24_delay_us(15U);
    nrf24_ce_low();

    do
    {
        status = BSP_NRF24L01_ReadStatus();
        if ((status & BSP_NRF24L01_IRQ_MAX_RT) != 0U)
        {
            BSP_NRF24L01_ClearIrq(BSP_NRF24L01_IRQ_MAX_RT);
            BSP_NRF24L01_FlushTx();
            return BSP_NRF24L01_TX_FAILED;
        }

        if ((status & BSP_NRF24L01_IRQ_TX_DS) != 0U)
        {
            if ((status & BSP_NRF24L01_IRQ_RX_DR) != 0U)
            {
                uint8_t width = nrf24_read_dynamic_payload_width();
                if (width == 0U || width > BSP_NRF24L01_MAX_PAYLOAD_LENGTH)
                {
                    BSP_NRF24L01_FlushRx();
                    BSP_NRF24L01_ClearIrq(BSP_NRF24L01_IRQ_TX_DS | BSP_NRF24L01_IRQ_RX_DR);
                    return BSP_NRF24L01_ACK_MISSING;
                }

                if (width > ack_buffer_size)
                {
                    BSP_NRF24L01_FlushRx();
                    BSP_NRF24L01_ClearIrq(BSP_NRF24L01_IRQ_TX_DS | BSP_NRF24L01_IRQ_RX_DR);
                    return BSP_NRF24L01_INVALID_ARGUMENT;
                }

                (void)nrf24_read_buffer(NRF24_CMD_R_RX_PAYLOAD, ack_buffer, width);
                *ack_length = width;
                BSP_NRF24L01_ClearIrq(BSP_NRF24L01_IRQ_TX_DS | BSP_NRF24L01_IRQ_RX_DR);
                return BSP_NRF24L01_OK;
            }

            BSP_NRF24L01_ClearIrq(BSP_NRF24L01_IRQ_TX_DS);
            return BSP_NRF24L01_ACK_MISSING;
        }

        nrf24_delay_ms(1U);
    } while (timeout_ms-- > 0U);

    BSP_NRF24L01_FlushTx();
    return BSP_NRF24L01_TIMEOUT;
}

BspNrf24l01Result BSP_NRF24L01_ReadPayload(uint8_t *data, uint8_t max_length, uint8_t *out_length)
{
    uint8_t status;
    uint8_t width;

    if (data == 0 || out_length == 0 || max_length == 0U)
    {
        return BSP_NRF24L01_INVALID_ARGUMENT;
    }

    status = BSP_NRF24L01_ReadStatus();
    if ((status & BSP_NRF24L01_IRQ_RX_DR) == 0U)
    {
        *out_length = 0U;
        return BSP_NRF24L01_TIMEOUT;
    }

    nrf24_ce_low();
    width = nrf24_read_dynamic_payload_width();
    if (width == 0U || width > BSP_NRF24L01_MAX_PAYLOAD_LENGTH)
    {
        BSP_NRF24L01_FlushRx();
        BSP_NRF24L01_ClearIrq(BSP_NRF24L01_IRQ_RX_DR);
        *out_length = 0U;
        nrf24_ce_high();
        return BSP_NRF24L01_ERROR;
    }

    if (width > max_length)
    {
        BSP_NRF24L01_FlushRx();
        BSP_NRF24L01_ClearIrq(BSP_NRF24L01_IRQ_RX_DR);
        *out_length = 0U;
        nrf24_ce_high();
        return BSP_NRF24L01_INVALID_ARGUMENT;
    }

    (void)nrf24_read_buffer(NRF24_CMD_R_RX_PAYLOAD, data, width);
    *out_length = width;
    BSP_NRF24L01_ClearIrq(BSP_NRF24L01_IRQ_RX_DR);
    nrf24_ce_high();

    return BSP_NRF24L01_OK;
}

uint8_t BSP_NRF24L01_ReadStatus(void)
{
    uint8_t status;

    nrf24_csn_low();
    status = nrf24_spi_transfer(NRF24_CMD_NOP);
    nrf24_csn_high();

    return status;
}

void BSP_NRF24L01_ClearIrq(uint8_t irq_flags)
{
    (void)nrf24_write_register(NRF24_REG_STATUS, irq_flags & BSP_NRF24L01_IRQ_ALL);
}

void BSP_NRF24L01_FlushTx(void)
{
    nrf24_send_command(NRF24_CMD_FLUSH_TX);
}

void BSP_NRF24L01_FlushRx(void)
{
    nrf24_send_command(NRF24_CMD_FLUSH_RX);
}

bool BSP_NRF24L01_IsIrqActive(void)
{
    return GPIO_ReadInputDataBit(BSP_NRF24_IRQ_PORT, BSP_NRF24_IRQ_PIN) == Bit_RESET;
}
