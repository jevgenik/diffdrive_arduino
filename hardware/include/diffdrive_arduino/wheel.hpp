#ifndef DIFFDRIVE_ARDUINO_WHEEL_HPP
#define DIFFDRIVE_ARDUINO_WHEEL_HPP

#include <string>
#include <cmath>

/**
 * @brief The Wheel class represents a wheel with its properties and methods.
 */
class Wheel
{
    public:

    std::string name = ""; // name of the wheel (std::string is a class/object that holds a string)
    int enc = 0;
    double cmd = 0;
    double pos = 0;
    double vel = 0;
    double rads_per_count = 0;

    Wheel() = default;

    /**
     * @brief Constructor for Wheel class.
     * @param wheel_name The name of the wheel (e.g. "left" or "right")
     * @param counts_per_rev The number of counts per revolution of the wheel.
     */
    Wheel(const std::string &wheel_name, int counts_per_rev)
    {
      setup(wheel_name, counts_per_rev);
    }

    
    /**
     * @brief Sets up the wheel with its name and counts per revolution.
     * @param wheel_name The name of the wheel.
     * @param counts_per_rev The number of counts per revolution of the wheel.
     */
    void setup(const std::string &wheel_name, int counts_per_rev)
    {
      name = wheel_name;
      rads_per_count = (2*M_PI)/counts_per_rev;
    }

    /**
     * @brief Calculates the angle of the wheel based on its encoder count.
     * @return The angle of the wheel in radians.
     */
    double calc_enc_angle()
    {
      return enc * rads_per_count;
    }



};


#endif // DIFFDRIVE_ARDUINO_WHEEL_HPP
