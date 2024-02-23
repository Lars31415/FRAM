/**
 * @file fm24w256.hpp
 * @brief This file contains the declaration of the Fm24w256 class.
 */

#ifndef FM24W256_HPP
#define FM24W256_HPP

#include "prom_base.hpp"

/**
 * @brief The Fm24w256 class provides an interface to the FM24W256 FRAM chip.
 */
class Fm24w256 : public PromBase
{
public:
    /**
     * @brief Construct a new Fm24w256 object.
     *
     * @param i2c The I2C instance to use.
     * @param addr The I2C address of the chip.
     */
    Fm24w256(i2c_inst_t *i2c, const uint8_t &addr);

    /**
     * @brief Destroy the Fm24w256 object.
     */
    virtual ~Fm24w256() throw();

    /**
     * @brief Write data to the chip.
     *
     * @param addr The address to write to.
     * @param bytes The data to write.
     * @param len The length of the data to write.
     * @return int The number of bytes written, or a negative error code.
     */
    int write(uint16_t addr, const void *bytes, size_t len) const;

    /**
     * @brief Read data from the chip.
     *
     * @param addr The address to read from.
     * @param bytes The buffer to read into.
     * @param len The length of the data to read.
     * @return int The number of bytes read, or a negative error code.
     */
    int read(const uint16_t &addr, const void *bytes, size_t len) const;

    /**
     * @brief Get the size of the EEPROM in bytes
     *
     * @return size_t Size of the EEPROM in bytes
     */
    size_t size() const; // bytes

private:
    int do_write(uint16_t addr, const void *bytes, size_t len) const;

    // Disallow copy and assignment
    Fm24w256(const Fm24w256 &);
    Fm24w256 &operator = (const Fm24w256 &);
};

#endif // FM24W256_HPP