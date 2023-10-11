#include "I2CHost.h"

I2CHost::I2CHost(OpFlowBoard *board)
    : _i2c_initialized(false),
      _i2c_busy(false),
      _wait_timeout(65535),
      _number_of_devices(0),
      _error_happend(false)
{
    this->_board = board;
}

I2CHost::~I2CHost()
{
}

uint8_t I2CHost::init()
{
    uint8_t sda_pin_AF_source, sda_pin_AF, scl_pin_AF_source, scl_pin_AF;
    uint16_t sda_pin, scl_pin;
    GPIO_TypeDef *sda_pin_group, *scl_pin_group;

    // TODO

    this->_i2c_typedef = I2C2;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin   = sda_pin;
    GPIO_Init(sda_pin_group, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = scl_pin;
    GPIO_PinAFConfig(sda_pin_group, sda_pin_AF_source, sda_pin_AF);
    GPIO_PinAFConfig(scl_pin_group, scl_pin_AF_source, scl_pin_AF);
    GPIO_Init(scl_pin_group, &GPIO_InitStructure);

    GPIO_SetBits(sda_pin_group, sda_pin);
    GPIO_SetBits(scl_pin_group, scl_pin);

    if (this->_i2c_typedef == I2C2) {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
    }

    I2C_DeInit(this->_i2c_typedef);
    I2C_InitTypeDef I2C_def;
    I2C_def.I2C_Mode                = I2C_Mode_I2C;
    I2C_def.I2C_DutyCycle           = I2C_DutyCycle_2;
    I2C_def.I2C_OwnAddress1         = 0x30;
    I2C_def.I2C_Ack                 = I2C_Ack_Enable;
    I2C_def.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_def.I2C_ClockSpeed          = 100 * 1000;
    I2C_Init(this->_i2c_typedef, &I2C_def);

    I2C_ITConfig(this->_i2c_typedef, I2C_IT_ERR, ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure;
    if (this->_i2c_typedef == I2C2) {
        NVIC_InitStructure.NVIC_IRQChannel = I2C2_ER_IRQn;
    }
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    I2C_Cmd(this->_i2c_typedef, ENABLE);
    I2C_AcknowledgeConfig(this->_i2c_typedef, ENABLE);

    this->wait_line_idle();
    this->close();

    return SUCCESS;
}

uint8_t I2CHost::deinit()
{
    uint8_t sda_pin_AF_source, sda_pin_AF, scl_pin_AF_source, scl_pin_AF;
    uint16_t sda_pin, scl_pin;
    GPIO_TypeDef *sda_pin_group, *scl_pin_group;

    // TODO

    I2C_DeInit(this->_i2c_typedef);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin   = scl_pin;
    GPIO_Init(scl_pin_group, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = sda_pin;
    GPIO_Init(sda_pin_group, &GPIO_InitStructure);

    return SUCCESS;
}

uint8_t I2CHost::retreive_ready_device(uint8_t *&device_addresses, uint8_t *number_of_address)
{
    if (this->_device_addresses) {
        vPortFree(this->_device_addresses);
    }

    this->_device_addresses = (uint8_t *)pvPortMalloc(128);

    // start to detetc all potential devices
    this->open();
    for (uint8_t i = 0x00; i <= 0x7F; i++) {
        this->wait_line_idle();
        this->generate_start();
        if (SUCCEED(this->send_address(i, 0))) {
            // device detected
            *(this->_device_addresses + this->_number_of_devices) = i;
            this->_number_of_devices++;
        }
        this->generate_stop();
    }
    this->close();

    uint8_t *temp_device_address = (uint8_t *)pvPortMalloc(this->_number_of_devices);
    memcpy(temp_device_address, this->_device_addresses, this->_number_of_devices);
    vPortFree(this->_device_addresses);
    this->_device_addresses = temp_device_address;

    *number_of_address = this->_number_of_devices;
    device_addresses   = this->_device_addresses;
    return SUCCESS;
}

uint8_t I2CHost::write(uint8_t address, uint8_t reg_address, uint8_t *data, uint8_t data_length)
{
    this->open();
    uint8_t res = SUCCEED(this->wait_line_idle());
    res         = res && SUCCEED(this->generate_start());
    res         = res && SUCCEED(this->send_address(address, 0));
    res         = res && SUCCEED(this->send_reg_address(reg_address));

    if (res) {
        for (int i = 0; i < data_length; i++) {
            if (!SUCCEED(this->write_byte(*(data + i)))) {
                res = 0;
                break;
            }
        }
    }
    this->generate_stop();
    this->close();
    return res;
}

uint8_t I2CHost::read(uint8_t address, uint8_t reg_address, uint8_t *data, uint8_t data_length)
{
    this->open();
    uint8_t res = SUCCEED(this->wait_line_idle());
    res         = res && SUCCEED(this->generate_start());
    res         = res && SUCCEED(this->send_address(address, 0));
    res         = res && SUCCEED(this->send_reg_address(reg_address));
    res         = res && SUCCEED(this->generate_start());

    if (res) {
        // here due to stm32 hardware register design, we have to set NACK ahead before reading the last byte
        if (data_length == 1) {
            // send address with read request
            this->_i2c_typedef->DR = address << 1 | 1;
            // I2C_StretchClockCmd(this->_i2c_typedef, ENABLE);
            this->disable_ack_response();
            res = SUCCEED(wait_SR1_flag(I2C_SR1_ADDR, 1));
            (void)this->_i2c_typedef->SR2;
            this->generate_stop();
            res = res && SUCCEED(wait_SR1_flag(I2C_SR1_RXNE, 1));

            if (res) {
                this->read_byte(data);
            }
        } else if (data_length == 2) {
            // similar with reading 1 byte
            // send address with read request
            this->_i2c_typedef->DR = address << 1 | 1;
            res                    = wait_SR1_flag(I2C_SR1_ADDR, 1);

            if (SUCCEED(res)) {
                I2C_StretchClockCmd(this->_i2c_typedef, ENABLE);
                this->disable_ack_response();
                this->_i2c_typedef->CR1 |= I2C_CR1_POS;
                (void)this->_i2c_typedef->SR2;
                I2C_StretchClockCmd(this->_i2c_typedef, DISABLE);

                // instead of waiting for RXNE, we have to wait for BTF
                res = wait_SR1_flag(I2C_SR1_BTF, 1);

                if (SUCCEED(res)) {
                    I2C_StretchClockCmd(this->_i2c_typedef, ENABLE);
                    res = this->read_byte(data);
                    I2C_StretchClockCmd(this->_i2c_typedef, DISABLE);
                    res = SUCCEED(res) && SUCCEED(this->read_byte(data + 1));
                }
                this->generate_stop();
            }
        } else {
            // read more than 2 byte
            int i = 0;
            while (data_length > 3 & res) {
                res = wait_SR1_flag(I2C_SR1_BTF, 1) & this->read_byte(data + i);
                data_length--;
                i++;
            }

            if (SUCCEED(res)) {
                res = wait_SR1_flag(I2C_SR1_BTF, 1);
                if (SUCCEED(res)) {
                    // start to read the N-2 byte which is right now in DR and N-1 byte in DSR(Data Shift Register)
                    I2C_StretchClockCmd(this->_i2c_typedef, ENABLE);
                    this->disable_ack_response();
                    this->read_byte(data + i);
                    I2C_StretchClockCmd(this->_i2c_typedef, DISABLE);

                    res = wait_SR1_flag(I2C_SR1_BTF, 1);
                    // now the last two byte should be in the DR and DSR
                    I2C_StretchClockCmd(this->_i2c_typedef, ENABLE);
                    this->generate_stop();
                    this->read_byte(data + i + 1);
                    I2C_StretchClockCmd(this->_i2c_typedef, DISABLE);
                    this->read_byte(data + i + 2);
                }
            }
        }
    }

    this->enable_ack_response();
    this->close();
    return res;
}

uint8_t I2CHost::eeprom_write(uint8_t address, uint16_t reg_address, uint8_t data)
{
    return this->write((address & 0xFC) | (uint8_t)((reg_address & 0x0300) >> 8),
                       (uint8_t)(reg_address & 0x00FF),
                       &data,
                       1);
}

uint8_t I2CHost::eeprom_read(uint8_t address, uint16_t reg_address, uint8_t *data, uint8_t data_length)
{
    return this->read((address & 0xFC) | (uint8_t)((reg_address & 0x0300) >> 8),
                      (uint8_t)(reg_address & 0x00FF),
                      data,
                      data_length);
}

uint8_t I2CHost::wait_SR1_flag(uint16_t flag, uint8_t set)
{
    uint32_t timeout = this->_wait_timeout;
    while (1) {
        if (set) {
            if (this->_i2c_typedef->SR1 & flag) {
                break;
            }
        } else {
            if (!(this->_i2c_typedef->SR1 & flag)) {
                break;
            }
        }
        timeout -= 1;
        if (this->_error_happend || (timeout == 0)) {
            this->_error_happend = false;
            return FAILURE;
        }
    }
    this->_error_happend = false;
    return SUCCESS;
}

uint8_t I2CHost::wait_SR2_flag(uint16_t flag, uint8_t set)
{
    uint32_t timeout = this->_wait_timeout;
    while (1) {
        if (set) {
            if (this->_i2c_typedef->SR2 & flag) {
                break;
            }
        } else {
            if (!(this->_i2c_typedef->SR2 & flag)) {
                break;
            }
        }
        timeout -= 1;
        if (this->_error_happend || (timeout == 0)) {
            this->_error_happend = false;
            return FAILURE;
        }
    }
    this->_error_happend = false;
    return SUCCESS;
}

uint8_t I2CHost::generate_start()
{
    this->_i2c_typedef->CR1 |= I2C_CR1_START;
    // generating start won't result in uncntrolled situation
    return this->wait_SR1_flag(I2C_SR1_SB, 1);
}

uint8_t I2CHost::software_reset()
{
    this->_i2c_typedef->CR1 |= I2C_CR1_SWRST;

    /**
     * @brief To release I2C bus, 9 scl clock must be generated and floowed by a stop signal
     */
    this->deinit();

    uint8_t sda_pin_AF_source, sda_pin_AF, scl_pin_AF_source, scl_pin_AF;
    uint16_t sda_pin, scl_pin;
    GPIO_TypeDef *sda_pin_group, *scl_pin_group;

    // TODO

    // generate 9 scl clocks
    for (int i = 0; i < 9; i++) {
        GPIO_SetBits(scl_pin_group, scl_pin);
        us_sleep(5);
        GPIO_ResetBits(scl_pin_group, scl_pin);
        us_sleep(5);
    }

    // generate stop signal
    GPIO_SetBits(scl_pin_group, scl_pin);
    us_sleep(5);
    GPIO_SetBits(sda_pin_group, sda_pin);
    us_sleep(5);
    GPIO_ResetBits(scl_pin_group, scl_pin);
    us_sleep(5);
    GPIO_ResetBits(sda_pin_group, sda_pin);
    us_sleep(5);

    this->_i2c_typedef->CR1 &= ~(I2C_CR1_SWRST);

    this->init();

    this->close();

    return SUCCESS;
}

uint8_t I2CHost::wait_line_idle()
{
    uint8_t wait_res = this->wait_SR2_flag(I2C_SR2_BUSY, 0);
    if (wait_res) {
        // wait start timeout, reset I2C
        this->_i2c_line_busy_times++;
        this->software_reset();
        return FAILURE;
    } else {
        return SUCCESS;
    }
}

uint8_t I2CHost::generate_stop()
{
    if (this->_i2c_typedef->SR2 & I2C_SR2_BUSY) {
        this->_i2c_typedef->CR1 |= I2C_CR1_STOP;
    }
    return SUCCESS;
}

uint8_t I2CHost::send_address(uint8_t address, uint8_t is_read_request)
{
    this->_i2c_typedef->DR = address << 1 | is_read_request;
    if (!this->wait_SR1_flag(I2C_SR1_ADDR, 1)) {
        (void)this->_i2c_typedef->SR2;
        return SUCCESS;
    } else {
        // didn't receive address answer, generate stop signal
        this->generate_stop();
        this->wait_line_idle();
        return FAILURE;
    }
}

uint8_t I2CHost::send_reg_address(uint8_t reg_address)
{
    this->_i2c_typedef->DR = reg_address;
    if (SUCCEED(this->wait_SR1_flag(I2C_SR1_TXE, 1))) {
        (void)this->_i2c_typedef->SR2;
        return SUCCESS;
    } else {
        // didn't receive address answer, generate stop signal
        this->generate_stop();
        this->wait_line_idle();
        return FAILURE;
    }
}

uint8_t I2CHost::write_byte(uint8_t byte)
{
    this->_i2c_typedef->DR = byte;
    if (!this->wait_SR1_flag(I2C_SR1_TXE, 1)) {
        (void)this->_i2c_typedef->SR2;
        return SUCCESS;
    } else {
        // fail to send byte
        this->generate_stop();
        this->wait_line_idle();
        return FAILURE;
    }
}

uint8_t I2CHost::read_byte(uint8_t *byte)
{
    *byte = this->_i2c_typedef->DR;
    return SUCCESS;
}

uint8_t I2CHost::open()
{
    this->_i2c_typedef->CR1 |= I2C_CR1_PE;
    return SUCCESS;
}

uint8_t I2CHost::close()
{
    this->_i2c_typedef->CR1 &= ~(I2C_CR1_PE);
    // without return 1, stm32 will crash during runtime
    return SUCCESS;
}

uint8_t I2CHost::disable_ack_response()
{
    this->_i2c_typedef->CR1 &= ~(I2C_CR1_ACK);
    return SUCCESS;
}

uint8_t I2CHost::enable_ack_response()
{
    this->_i2c_typedef->CR1 |= I2C_CR1_ACK;
    return SUCCESS;
}

uint8_t I2CHost::error_check(void)
{
    if (I2C_GetFlagStatus(this->_i2c_typedef, I2C_FLAG_BERR)) {
        // Bus error
        // external SDA change detected while SCL is in HIGH
        // discard this communication session
        I2C_ClearITPendingBit(this->_i2c_typedef, I2C_FLAG_BERR);
    } else if (I2C_GetFlagStatus(this->_i2c_typedef, I2C_FLAG_AF)) {
        I2C_ClearITPendingBit(this->_i2c_typedef, I2C_FLAG_AF);
    } else if (I2C_GetFlagStatus(this->_i2c_typedef, I2C_FLAG_TIMEOUT)) {
        // do nothing, relavant operation will be done by hardware
        I2C_ClearITPendingBit(this->_i2c_typedef, I2C_FLAG_TIMEOUT);
    } else if (I2C_GetFlagStatus(this->_i2c_typedef, I2C_FLAG_OVR)) {
        // do nothing, may receive incomplete data
        // discard current session
        I2C_ClearITPendingBit(this->_i2c_typedef, I2C_FLAG_OVR);
    } else if (I2C_GetFlagStatus(this->_i2c_typedef, I2C_FLAG_ARLO)) {
        // discard current session
        // hardware will go back to slave mode
        I2C_ClearITPendingBit(this->_i2c_typedef, I2C_FLAG_ARLO);
    }
    this->_error_happend = true;
    return SUCCESS;
}

void I2CHost_error_handler_callback(I2CHost *i2c_host)
{
    i2c_host->error_check();
}