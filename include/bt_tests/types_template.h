#include "header.h"

#ifndef TYPES_TAMPLATES_H
#define TYPES_TAMPLATES_H

    struct Position2D 
    { 
    double x;
    double y; 
    };

    struct Position3D 
    { 
    double x;
    double y; 
    double z; 
    };

    struct Pose3D 
    { 
    double x;
    double y; 
    double z; 
    double ox;
    double oy; 
    double oz;
    double ow; 
    };

    // Template specialization to converts a string to Position2D, Position3D


    namespace BT
    {
        template <> inline Position2D convertFromString(StringView str)
        {
            // We expect real numbers separated by semicolons
            auto parts = splitString(str, ';');
            if (parts.size() != 2)
            {
                throw RuntimeError("invalid input)");
            }
            else{
                Position2D output;
                output.x     = convertFromString<double>(parts[0]);
                output.y     = convertFromString<double>(parts[1]);
                return output;
            }
        }

        template <> inline Position3D convertFromString(StringView str)
        {
            // We expect real numbers separated by semicolons
            auto parts = splitString(str, ';');
            if (parts.size() != 3)
            {
                throw RuntimeError("invalid input)");
            }
            else{
                Position3D output;
                output.x     = convertFromString<double>(parts[0]);
                output.y     = convertFromString<double>(parts[1]);
                output.z     = convertFromString<double>(parts[2]);
                return output;
            }
        }

            template <> inline Pose3D convertFromString(StringView str)
        {
            // We expect real numbers separated by semicolons
            auto parts = splitString(str, ';');
            if (parts.size() != 7)
            {
                throw RuntimeError("invalid input)");
            }
            else{
                Pose3D output;
                output.x     = convertFromString<double>(parts[0]);
                output.y     = convertFromString<double>(parts[1]);
                output.z     = convertFromString<double>(parts[2]);
                output.ox     = convertFromString<double>(parts[3]);
                output.oy     = convertFromString<double>(parts[4]);
                output.oz     = convertFromString<double>(parts[5]);
                output.ow     = convertFromString<double>(parts[6]);

                return output;
            }
        }

    } // end namespace BT

#endif 