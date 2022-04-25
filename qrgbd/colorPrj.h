#pragma once

#include <iostream>
#include <tuple>
#include <cmath>

namespace ns_clp
{
    using rgb = std::tuple<int, int, int>;
    using hsv = std::tuple<float, float, float>;
    /**
     * \brief translate the hsv color space to rgb color space
     * 
     */
    class HSV2RGB
    {
    private:
        HSV2RGB() = delete;

    public:
        static rgb trans(const hsv &val);
    };

    /** 
     * \brief different color types for the projection
     */
    class Color
    {
    public:
        enum class Elem
        {
            Hue,
            Saturation,
            Value
        };
        /**
         * \brief define your own color style by writing this structure
         *        the [_min, _max] means the change range, it's for 
         *        h[0.0, 360.0],
         *        s[0.0, 1.0],
         *        v[0.0, 1.0]
         *        and the [_hsv] points the change elem.
         * \attention for example, ColorTypw{0.0, 180.0, 1.0, 0.5, Elem::Hue}
         *        it means:
         *        h[0.0, 180.0],
         *        s = 1.0,
         *        v = 0.5
         */
        struct ColorType
        {
            float _min;
            float _max;
            float _v1;
            float _v2;
            Elem _hsv;
        };

    public:
        /**
     * 
     * \brief color types
     */
        constexpr static ColorType red_yellow{0.0, 60.0, 1.0, 1.0, Elem::Hue};
        constexpr static ColorType yellow_green{45.0, 130.0, 1.0, 1.0, Elem::Hue};
        constexpr static ColorType green_cyan{100.0, 190.0, 1.0, 1.0, Elem::Hue};
        constexpr static ColorType cyan_blue{180.0, 240.0, 1.0, 1.0, Elem::Hue};
        constexpr static ColorType blue_purple{220.0, 300.0, 1.0, 1.0, Elem::Hue};
        constexpr static ColorType purple_red{290.0, 360.0, 1.0, 1.0, Elem::Hue};

        constexpr static ColorType panchromatic{0.0, 360.0, 1.0, 1.0, Elem::Hue};

        constexpr static ColorType red{0.0, 1.0, 360.0, 1.0, Elem::Saturation};
        constexpr static ColorType pink{0.0, 1.0, 340.0, 1.0, Elem::Saturation};
        constexpr static ColorType purple{0.0, 1.0, 310.0, 1.0, Elem::Saturation};
        constexpr static ColorType blue{0.0, 1.0, 240.0, 1.0, Elem::Saturation};
        constexpr static ColorType cyan{0.0, 1.0, 190.0, 1.0, Elem::Saturation};
        constexpr static ColorType green{0.0, 1.0, 120.0, 1.0, Elem::Saturation};
        constexpr static ColorType yellow{0.0, 1.0, 60.0, 1.0, Elem::Saturation};
        constexpr static ColorType orange{0.0, 1.0, 20.0, 1.0, Elem::Saturation};

        constexpr static ColorType gray{0.0, 1.0, 0.0, 0.0, Elem::Value};
        constexpr static ColorType red_yellow_green{0.0, 150.0, 1.0, 1.0, Elem::Hue};
        constexpr static ColorType yellow_green_cyan{50.0, 180.0, 1.0, 1.0, Elem::Hue};
        constexpr static ColorType green_cyan_blue{50.0, 250.0, 1.0, 1.0, Elem::Hue};
        constexpr static ColorType cyan_blue_purple{180.0, 300.0, 1.0, 1.0, Elem::Hue};
        constexpr static ColorType blue_purple_red{240.0, 360.0, 1.0, 1.0, Elem::Hue};
        constexpr static ColorType cold{180.0, 360.0, 1.0, 1.0, Elem::Hue};
        constexpr static ColorType worm{0.0, 180.0, 1.0, 1.0, Elem::Hue};
    };

    class ColorPrj
    {
    private:
        ColorPrj() = delete;

    public:
        /**
     * \brief project the val to the [color] space
     * \param val the value
     * \param min the min value
     * \param max the max value
     * \param isReversal to reverse the color grade
     * \param classify to decide the color Graded or successive
     *        if it's zero or one or negative, means successive
     * \param color the color type, default: gray
     */
        static rgb project(float val, float min, float max,
                           bool isReversal = false, int classify = 0, const Color::ColorType &color = Color::gray);
    };
} // namespace ns_clp
