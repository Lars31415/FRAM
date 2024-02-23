/**
 * @file fm24w256.cpp
 * @brief Implementation of a class to access a Fm24w256 I2C F-RAM.
 *
 * This file provides the implementation for reading and writing data to an I2C F-RAM of the Fm24w256 type.
 */

#include "fm24w256.hpp"
#include "i2c_scanner.hpp"
#include <cstring>
#include <iostream>

const size_t BUF_SIZE = 0x1000; //!< Buffer size used for reading and writing.

/**
 * @brief Constructor for the Fm24w256 class.
 *
 * @param i2c Pointer to the I2C instance.
 * @param addr I2C address of the Fm24w256 device.
 */
Fm24w256::Fm24w256(i2c_inst_t *i2c, const uint8_t &addr)
    : PromBase(i2c, addr)
{
}

/**
 * @brief Destructor for the Fm24w256 class.
 *
 * @throw Nothing.
 */
Fm24w256::~Fm24w256() throw() {}

/**
 * @brief Read data from the Fm24w256 device.
 *
 * @param addr Start address of the data to be read.
 * @param bytes Pointer to the buffer to store the read data.
 * @param len Number of bytes to read.
 * @return The number of bytes read, or an error code.
 */
int Fm24w256::read(const uint16_t &addr, const void *bytes, size_t len) const
{
    // std::cout << "Fm24w256::read(0x" << std::hex << addr << ", 0x" << bytes << ", " << std::dec << len << ")" << std::endl;

    uint16_t offset = 0;
    int ret = 0;
    int count = 0;

    // Read data in chunks of BUF_SIZE
    while ((len > 0) && (ret >= 0))
    {
        size_t l = std::min(BUF_SIZE, len);

        // Set the address for the read operation
        ret = do_write(addr + offset, nullptr, 0);
        if (ret < 0)
        {
            std::cout << "Fm24w256::read(0x" << std::hex << addr << ", 0x" << bytes << ", " << std::dec << len << ")" << std::endl;
            std::cout << "Set address timeout" << std::endl;
            return ret;
        }

        // Read data from the device
        ret = i2c_read_timeout_us(i2c_, addr_, (uint8_t *)bytes + offset, l, false, 26 * l + 100000);
        while (ret == PICO_ERROR_GENERIC)
            ret = i2c_read_timeout_us(i2c_, addr_, (uint8_t *)bytes + offset, l, false, 26 * l + 100000);

        len -= l;
        offset += l;
        count += l;
    }

    if (ret >= 0)
        ret = count;

    return ret;
}

/**
 * @brief Write data to the Fm24w256 device.
 *
 * @param addr Start address for the data to be written.
 * @param bytes Pointer to the data to be written.
 * @param len Number of bytes to write.
 * @return The number of bytes written, or an error code.
 */
int Fm24w256::write(uint16_t addr, const void *bytes, size_t len) const
{
    uint16_t offset = 0;
    int ret = 0;

    // Write data in chunks of BUF_SIZE
    while ((len > 0) && (ret >= 0))
    {
        size_t l = std::min(BUF_SIZE, len);

        // Perform the write operation
        ret = do_write(addr + offset, (uint8_t *)bytes + offset, l);

        len -= l;
        offset += l;
    }

    return ret;
}

/**
 * @brief Internal function to perform write operations on the Fm24w256 device.
 *
 * @param addr Start address for the data to be written.
 * @param bytes Pointer to the data to be written.
 * @param len Number of bytes to write.
 * @return The result of the write operation, or an error code.
 */
int Fm24w256::do_write(uint16_t addr, const void *bytes, size_t len) const
{
    // std::cout << "Fm24w256::do_write(0x" << std::hex << addr << ", 0x" << bytes << ", " << std::dec << len << ")" << std::endl;

    uint8_t buf[len + 2];
    buf[1] = addr & 0x00FF;
    buf[0] = (addr >> 8) & 0x00FF;
    std::memcpy(buf + 2, bytes, len);

    // Perform the write operation with a timeout
    int ret(i2c_write_timeout_us(i2c_, addr_, buf, len + 2, false, 25 * len + 100000));
    while (ret == PICO_ERROR_GENERIC)
        ret = i2c_write_timeout_us(i2c_, addr_, buf, len + 2, false, 25 * len + 100000);

    return ret;
}

/**
 * @brief Get the size of the Fm24w256 device in bytes.
 *
 * @return Size of the device in bytes.
 */
size_t Fm24w256::size() const // bytes
{
    return 32768LU;
}
