#include <stm32f10x.h>

#define SUCCESS    1
#define FAILURE    0
#define SUCCEED(x) (x == SUCCESS)

class I2CHost
{
public:
    I2CHost(OpFlowBoard *board);
    ~I2CHost();

    uint8_t init();
    uint8_t deinit();
    uint8_t retreive_ready_device(uint8_t *&device_addresses, uint8_t *number_of_address);

    /**
     * @brief Write only one byte to the slave device.
     *
     * @param address device I2C address
     * @param reg_address device register address
     * @param data data to be written
     * @return uint8_t return 1 as success, 0 as failure
     */
    uint8_t write(uint8_t address, uint8_t reg_address, uint8_t *data, uint8_t data_length);
    uint8_t read(uint8_t address, uint8_t reg_address, uint8_t *data, uint8_t data_length);
    uint8_t error_check(void);

    uint8_t eeprom_write(uint8_t address, uint16_t reg_address, uint8_t data);
    uint8_t eeprom_read(uint8_t address, uint16_t reg_address, uint8_t *data, uint8_t data_length);

    I2C_TypeDef *_i2c_typedef;

private:
    uint8_t _i2c_initialized;
    uint8_t _i2c_busy;
    uint8_t _i2c_address;
    uint32_t _wait_timeout;
    uint32_t _i2c_line_busy_times;

    uint8_t _number_of_devices;
    uint8_t *_device_addresses;

    volatile bool _error_happend;

private:
    uint8_t detect();
    uint8_t wait_SR1_flag(uint16_t flag, uint8_t set);
    uint8_t wait_SR2_flag(uint16_t flag, uint8_t set);
    uint8_t software_reset();
    uint8_t wait_line_idle();
    uint8_t generate_start();
    uint8_t generate_stop();
    uint8_t send_address(uint8_t address, uint8_t is_read_request);
    uint8_t send_reg_address(uint8_t reg_address);
    uint8_t disable_ack_response();
    uint8_t enable_ack_response();

    /**
     * @brief Write byte to DATA REGISTER and wait for data to be sent
     *
     * @param byte byte to be written
     * @return uint8_t return 1 as success, 0 as failure
     */
    uint8_t write_byte(uint8_t byte);
    uint8_t read_byte(uint8_t *byte);
    uint8_t open();
    uint8_t close();
};

extern "C" {
extern void I2CHost_error_handler_callback(I2CHost *i2c_host);
}