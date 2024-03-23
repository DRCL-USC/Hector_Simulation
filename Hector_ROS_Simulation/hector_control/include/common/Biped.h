#ifndef PROJECT_BIPED_H
#define PROJECT_BIPED_H

#include <vector>
#include "cppTypes.h"

class Biped {
    public:
        Biped() :
            mass(13.856),
            leg_yaw_offset_x(-0.005),
            leg_yaw_offset_y(-0.057),
            leg_yaw_offset_z(-0.126),
            leg_roll_offset_x(0.0465),
            leg_roll_offset_y(0.015),
            leg_roll_offset_z(-0.0705),
            hipLinkLength(0.038),
            thighLinkLength(0.22),
            calfLinkLength(0.22) {}

        Vec3<double> getHipYawLocation(int leg) const {
            checkLegIndex(leg);
            return Vec3<double>(leg_yaw_offset_x, leg == 0 ? leg_yaw_offset_y : -leg_yaw_offset_y, leg_yaw_offset_z);
        }

        Vec3<double> getHipRollLocation(int leg) const {
            checkLegIndex(leg);
            return Vec3<double>(leg_roll_offset_x, leg == 0 ? leg_roll_offset_y : -leg_roll_offset_y, leg_roll_offset_z);
        }

    private:
        void checkLegIndex(int leg) const {
            if (leg < 0 || leg >= 2) {
                throw std::invalid_argument("Invalid leg index");
            }
        }

        const double hipLinkLength, thighLinkLength, calfLinkLength;
        const double leg_yaw_offset_x, leg_yaw_offset_y, leg_yaw_offset_z;
        const double leg_roll_offset_x, leg_roll_offset_y, leg_roll_offset_z;
        const double mass;
    };

#endif
