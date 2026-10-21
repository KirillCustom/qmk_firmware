/* The stock firmware's lighting modes, in stock order (stock_fx.c holds the
 * engine and the stock-number table). They come FIRST and stay contiguous and
 * in sfx_id_t order: stock_fx.c maps the enum by offset and asserts it. */
RGB_MATRIX_EFFECT(STOCK_STATIC)
RGB_MATRIX_EFFECT(STOCK_SINGLE_ON)
RGB_MATRIX_EFFECT(STOCK_SINGLE_OFF)
RGB_MATRIX_EFFECT(STOCK_GLITTERING)
RGB_MATRIX_EFFECT(STOCK_FALLING)
RGB_MATRIX_EFFECT(STOCK_COLOURFUL)
RGB_MATRIX_EFFECT(STOCK_BREATH)
RGB_MATRIX_EFFECT(STOCK_SPECTRUM)
RGB_MATRIX_EFFECT(STOCK_OUTWARD)
RGB_MATRIX_EFFECT(STOCK_SCROLL_DOWN)
RGB_MATRIX_EFFECT(STOCK_SCROLL_UP)
RGB_MATRIX_EFFECT(STOCK_ROLL_RIGHT)
RGB_MATRIX_EFFECT(STOCK_ROLL_LEFT)
RGB_MATRIX_EFFECT(STOCK_ROTATING)
RGB_MATRIX_EFFECT(STOCK_EXPLODE)
RGB_MATRIX_EFFECT(STOCK_LAUNCH)
RGB_MATRIX_EFFECT(STOCK_RIPPLES)
RGB_MATRIX_EFFECT(STOCK_FLOW_RIGHT)
RGB_MATRIX_EFFECT(STOCK_FLOW_LEFT)
RGB_MATRIX_EFFECT(STOCK_PULSATING)
RGB_MATRIX_EFFECT(STOCK_TILT_RIGHT)
RGB_MATRIX_EFFECT(STOCK_TILT_LEFT)
RGB_MATRIX_EFFECT(STOCK_SHUTTLE)

#ifdef RGB_MATRIX_CUSTOM_EFFECT_IMPLS
#    include "stock_fx.h"
static bool STOCK_STATIC(effect_params_t* params) { return stock_fx_run(SFX_STATIC, params); }
static bool STOCK_SINGLE_ON(effect_params_t* params) { return stock_fx_run(SFX_SINGLE_ON, params); }
static bool STOCK_SINGLE_OFF(effect_params_t* params) { return stock_fx_run(SFX_SINGLE_OFF, params); }
static bool STOCK_GLITTERING(effect_params_t* params) { return stock_fx_run(SFX_GLITTER, params); }
static bool STOCK_FALLING(effect_params_t* params) { return stock_fx_run(SFX_FALLING, params); }
static bool STOCK_COLOURFUL(effect_params_t* params) { return stock_fx_run(SFX_COLOURFUL, params); }
static bool STOCK_BREATH(effect_params_t* params) { return stock_fx_run(SFX_BREATH, params); }
static bool STOCK_SPECTRUM(effect_params_t* params) { return stock_fx_run(SFX_SPECTRUM, params); }
static bool STOCK_OUTWARD(effect_params_t* params) { return stock_fx_run(SFX_OUTWARD, params); }
static bool STOCK_SCROLL_DOWN(effect_params_t* params) { return stock_fx_run(SFX_SCROLL_DOWN, params); }
static bool STOCK_SCROLL_UP(effect_params_t* params) { return stock_fx_run(SFX_SCROLL_UP, params); }
static bool STOCK_ROLL_RIGHT(effect_params_t* params) { return stock_fx_run(SFX_ROLL_RIGHT, params); }
static bool STOCK_ROLL_LEFT(effect_params_t* params) { return stock_fx_run(SFX_ROLL_LEFT, params); }
static bool STOCK_ROTATING(effect_params_t* params) { return stock_fx_run(SFX_ROTATING, params); }
static bool STOCK_EXPLODE(effect_params_t* params) { return stock_fx_run(SFX_EXPLODE, params); }
static bool STOCK_LAUNCH(effect_params_t* params) { return stock_fx_run(SFX_LAUNCH, params); }
static bool STOCK_RIPPLES(effect_params_t* params) { return stock_fx_run(SFX_RIPPLES, params); }
static bool STOCK_FLOW_RIGHT(effect_params_t* params) { return stock_fx_run(SFX_FLOW_FWD, params); }
static bool STOCK_FLOW_LEFT(effect_params_t* params) { return stock_fx_run(SFX_FLOW_BACK, params); }
static bool STOCK_PULSATING(effect_params_t* params) { return stock_fx_run(SFX_PULSATING, params); }
static bool STOCK_TILT_RIGHT(effect_params_t* params) { return stock_fx_run(SFX_TILT_RIGHT, params); }
static bool STOCK_TILT_LEFT(effect_params_t* params) { return stock_fx_run(SFX_TILT_LEFT, params); }
static bool STOCK_SHUTTLE(effect_params_t* params) { return stock_fx_run(SFX_SHUTTLE, params); }
#endif // RGB_MATRIX_CUSTOM_EFFECT_IMPLS


/* RAINFALL -- a decaying twinkle.
 *
 * QMK's stock PIXEL_RAIN does NOT fade: it sets one LED per interval (half the
 * time to black) and leaves every other LED lit at full until it happens to be
 * picked again. The result is a mostly-static scatter, not rain. STARLIGHT and
 * STARLIGHT_SMOOTH are no closer -- both keep the entire board lit, pulsing on
 * a shared or per-LED sine.
 *
 * This one keeps a brightness and a hue per LED. Every frame each brightness
 * decays; on a timer a random LED is seeded to full with a random hue. So a key
 * flashes and falls away over about a second, and only the few most recent
 * drops are anywhere near full.
 *
 * SPEED sets the DROP RATE, i.e. the density. Decay is deliberately fixed, so
 * turning speed up adds raindrops rather than making each one shorter -- which
 * is the knob you actually want when the effect looks too sparse or too busy.
 *
 * Cost: 2 bytes per LED (162 B here) and one qsub8 per LED per frame.
 */
RGB_MATRIX_EFFECT(RAINFALL)

#ifdef RGB_MATRIX_CUSTOM_EFFECT_IMPLS

/* ~1.4 s from full to dark at the ~62 Hz effect rate (255/4 frames). Long
 * enough to read as a fall rather than a blink. */
#    define RAINFALL_DECAY 4

static uint8_t rainfall_v[RGB_MATRIX_LED_COUNT];
static uint8_t rainfall_h[RGB_MATRIX_LED_COUNT];

static bool RAINFALL(effect_params_t* params) {
    static uint8_t countdown = 0;

    /* init stays true for every chunk of the first frame: reset once, on the
     * first, or a drop seeded there is wiped by the next (codex). */
    if (params->init && params->iter == 0) {
        memset(rainfall_v, 0, sizeof(rainfall_v));
        countdown = 0;
    }

    RGB_MATRIX_USE_LIMITS(led_min, led_max);

    /* The effect is called once per LED_PROCESS_LIMIT chunk (5 chunks per frame
     * here), so the simulation must advance on the FIRST chunk only -- stepping
     * it per chunk would decay five times a frame and quintuple the drop rate. */
    if (params->iter == 0) {
        for (uint8_t i = 0; i < RGB_MATRIX_LED_COUNT; i++)
            rainfall_v[i] = qsub8(rainfall_v[i], RAINFALL_DECAY);

        if (countdown) countdown--;
        if (!countdown) {
            uint8_t i     = random8_max(RGB_MATRIX_LED_COUNT);
            rainfall_v[i] = 255;
            rainfall_h[i] = random8();
            /* Frames between drops: 25 at speed 0 down to 1 at full. At the
             * default speed this leaves roughly half a dozen lit at once. */
            countdown = 1 + ((255 - rgb_matrix_config.speed) * 24) / 255;
        }
    }

    for (uint8_t i = led_min; i < led_max; i++) {
        RGB_MATRIX_TEST_LED_FLAGS();
        hsv_t hsv = {rainfall_h[i], rgb_matrix_config.hsv.s,
                     scale8(rainfall_v[i], rgb_matrix_config.hsv.v)};
        rgb_t rgb = rgb_matrix_hsv_to_rgb(hsv);
        rgb_matrix_set_color(i, rgb.r, rgb.g, rgb.b);
    }
    return rgb_matrix_check_finished_leds(led_max);
}

#endif // RGB_MATRIX_CUSTOM_EFFECT_IMPLS


/* DRIFT -- a scrolling 2D value-noise field on a hand-picked colour ramp.
 *
 * The noise controls POSITION ALONG A RAMP, never hue directly. Mapping a
 * scalar onto hue is what makes every stock effect read as "RGB keyboard"; this
 * one only ever wears colours that were chosen deliberately.
 *
 * Value noise rather than Perlin: Perlin needs gradient vectors and dot
 * products, value noise needs a table and three lerps, and at 81 sparse samples
 * they are visually indistinguishable.
 *
 * WARNING: Cortex-M0 has NO hardware divide and NO FPU. Every divide or modulo
 * on a non-power-of-two is a libgcc call, times 81 LEDs, times ~62 fps.
 * Everything below is shifts, masks, adds and multiplies -- keep it that way.
 * The objective check is `arm-none-eabi-nm` for __aeabi_idiv on the object.
 */
RGB_MATRIX_EFFECT(DRIFT)

#ifdef RGB_MATRIX_CUSTOM_EFFECT_IMPLS

#    include "power.h"   /* power_val_cap(): the ladder's ceiling on the value knob */

/* 16x16 so indexing is masks only. Authored here, never computed at runtime. */
static const uint8_t drift_noise[256] = {
     12, 194,  70, 215, 173,  19, 198, 146, 113, 186, 146,  29,  58, 149, 221,  27,
    235,   5, 115, 126, 187, 105, 221, 126,  75, 142, 202, 116, 142,  60, 109, 248,
    175, 221,  92, 149, 158,  88, 175,  15, 165, 115,  43,  58, 163,  43, 114,  29,
     53, 200,  11, 243, 156,  82,  17, 132, 131,  66, 191,  47,  22, 222,  66,  33,
    158, 231, 145, 248, 182, 152,  88,  42, 114, 152, 124,  88, 189,  85, 222,  54,
    207, 103,  62,  60, 190, 141,  35, 138, 254, 147, 250,   2, 150, 211, 153, 128,
     42,  29,  59, 188,   4, 185, 219,  26, 132,  20, 169, 185, 110, 103, 132,  98,
    146, 115, 128,  85,  16,  53,  74,  65, 192, 176, 188,  41,  90, 163,   7,  58,
    213, 168,  14,  61,  18, 166, 158,  23, 225,  80, 251, 148, 252,  45, 169,  31,
     71, 191,  92, 229, 174,  18,  97,  30, 208, 244,   8, 250, 213,  95,  68,  44,
     80, 190,  30,  51, 157,  61,  26, 236, 121,  32,  70,  59,  96, 205,  56, 129,
    197, 235, 138, 116, 170, 188,   8, 145, 139,  69, 142, 144, 140, 174,  24, 205,
    129,  55,  38, 252, 173,  95, 175, 211,  38,  14, 162, 239, 237,  68,  83, 206,
    218,  11,  19, 210,  52,  62, 211, 180,  50, 201, 115,  40, 135, 220,   2,  88,
    140, 137, 159,  30, 237,  11, 245, 199, 244,  46,  43,   6, 180, 126, 190, 212,
    228, 148,  59, 186, 213, 195, 219, 211, 152, 199, 167,  14,  30, 220, 227,   4,
};

/* The ramp is authored in HSV, not RGB, SPECIFICALLY so hue and saturation can
 * act on it: rotating an RGB ramp would need an RGB->HSV conversion per stop
 * (which QMK does not provide), while in HSV a rotation is an add.
 *
 * Dark indigo -> violet -> slate -> warm grey, weighted dark on purpose:
 * understated, not demo-pop. Each stop's v carries that dark weighting;
 * rgb_matrix_config.hsv.v scales the whole thing on top, per LED.
 *
 * NINE entries with the last duplicated so the top ramp cell needs no clamp
 * branch. */
static const hsv_t drift_ramp_hsv[9] = {
    {168, 255,   0}, {170, 255,   7}, {173, 250,  18}, {177, 245,  36},
    {181, 235,  60}, {186, 215,  88}, {192, 185, 118}, {200, 150, 150},
    {200, 150, 150},
};

/* Derived palette, rebuilt only when hue or saturation actually changes -- the
 * demoscene trick: keep the field, swap the palette. The per-LED path never
 * does an HSV conversion. Built UNCAPPED (hsv_to_rgb, not the power ladder's
 * capped rgb_matrix_hsv_to_rgb): the user's value scales the RGB after the
 * conversion, so a cap on the stops would clip a Drift that is already under
 * it. The ladder caps the value knob instead, below. */
static rgb_t  drift_ramp[9];
static uint8_t drift_pal_h = 0, drift_pal_s = 0;
static bool    drift_pal_valid = false;

static void drift_build_palette(uint8_t h, uint8_t s) {
    for (uint8_t i = 0; i < 9; i++) {
        hsv_t c = drift_ramp_hsv[i];
        /* Hue ADDS, so the palette keeps its internal hue relationships and
         * rotates as a whole -- the stops stay a family instead of collapsing
         * to one colour. Saturation SCALES, so turning it down washes the
         * whole ramp toward grey rather than shifting it. */
        c.h = (uint8_t)(c.h + h);
        c.s = scale8(c.s, s);
        drift_ramp[i] = hsv_to_rgb(c);
    }
    drift_pal_h = h; drift_pal_s = s; drift_pal_valid = true;
}

/* lerp with an EIGHT-bit fraction. int16_t so (b - a) may go negative.
 *
 * This was 4 bits, which is where the jerkiness came from: 4 bits is 16
 * interpolation steps per 16-unit cell, i.e. the field could only be sampled on
 * whole-unit boundaries. At the slow end the position advances one unit every
 * ~4 s, so the field STEPPED once every four seconds instead of drifting.
 * Slowing the effect down did not cause that -- it just spread the existing
 * steps far enough apart to see. */
static inline uint8_t drift_lerp8(uint8_t a, uint8_t b, uint8_t f) {
    return (uint8_t)(a + (((int16_t)(b - a) * f) >> 8));
}

/* Smoothstep as a 256-entry table: one load instead of two multiplies, on a CPU
 * where this runs 162 times a frame. Removes the diamond-shaped grid artifact
 * that makes linearly interpolated value noise look cheap. */
static const uint8_t drift_ease[256] = {
      0,   0,   0,   0,   0,   0,   0,   1,   1,   1,   1,   1,   2,   2,   2,   3,
      3,   3,   4,   4,   4,   5,   5,   6,   6,   7,   7,   8,   9,   9,  10,  10,
     11,  12,  12,  13,  14,  15,  15,  16,  17,  18,  18,  19,  20,  21,  22,  23,
     24,  25,  26,  27,  27,  28,  29,  30,  31,  33,  34,  35,  36,  37,  38,  39,
     40,  41,  42,  44,  45,  46,  47,  48,  50,  51,  52,  53,  54,  56,  57,  58,
     60,  61,  62,  63,  65,  66,  67,  69,  70,  72,  73,  74,  76,  77,  78,  80,
     81,  83,  84,  85,  87,  88,  90,  91,  93,  94,  96,  97,  98, 100, 101, 103,
    104, 106, 107, 109, 110, 112, 113, 115, 116, 118, 119, 121, 122, 124, 125, 127,
    128, 130, 131, 133, 134, 136, 137, 139, 140, 142, 143, 145, 146, 148, 149, 151,
    152, 154, 155, 157, 158, 159, 161, 162, 164, 165, 167, 168, 170, 171, 172, 174,
    175, 177, 178, 179, 181, 182, 183, 185, 186, 188, 189, 190, 192, 193, 194, 195,
    197, 198, 199, 201, 202, 203, 204, 205, 207, 208, 209, 210, 211, 213, 214, 215,
    216, 217, 218, 219, 220, 221, 222, 224, 225, 226, 227, 228, 228, 229, 230, 231,
    232, 233, 234, 235, 236, 237, 237, 238, 239, 240, 240, 241, 242, 243, 243, 244,
    245, 245, 246, 246, 247, 248, 248, 249, 249, 250, 250, 251, 251, 251, 252, 252,
    252, 253, 253, 253, 254, 254, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255,
};

static bool DRIFT(effect_params_t* params) {
    /* Sub-unit precision, LOW TEN BITS FRACTIONAL.
     *
     * This was 4 bits, and that made the whole speed control useless at the
     * interesting end: the smallest possible step was 1/16 unit per frame, i.e.
     * 3.9 units/s, so 0% and 1% were the SAME speed and the field still crossed
     * the board in 14 s. At 100% it crossed in 0.4 s. The range was not too
     * wide, it was too COARSE to reach slow at all.
     *
     * At 10 bits the slow end is ~65 s per noise cell -- about 3.7 minutes to
     * cross the board, which is the "slow lava" end this effect wants -- and
     * 100% is ~7 s across, lively but not frantic. */
    static uint32_t acc_x = 0, acc_y = 0;

    if (params->init && params->iter == 0) { acc_x = 0; acc_y = 0; drift_pal_valid = false; }   /* once a frame */

    /* Rebuild the palette only on an actual change: 9 HSV conversions when the
     * knob moves, zero on every other frame. */
    if (!drift_pal_valid || drift_pal_h != rgb_matrix_config.hsv.h
                         || drift_pal_s != rgb_matrix_config.hsv.s) {
        drift_build_palette(rgb_matrix_config.hsv.h, rgb_matrix_config.hsv.s);
    }

    /* Advance ONCE per frame. LED_PROCESS_LIMIT splits a frame into chunks, and
     * stepping per chunk would multiply the drift rate by the chunk count. */
    if (params->iter == 0) {
        uint16_t inc = 8 + (rgb_matrix_config.speed >> 1);
        acc_x += inc;
        /* y at 5/8 of x. A CONSTANT ratio matters more than a different one:
         * two independently-derived rates land on 1:1 at the bottom of the
         * range (both clamp to their minimum), which is exactly the 45-degree
         * slide this is meant to avoid. 1.6:1 is never axis-aligned and never
         * diagonal, at any speed. */
        acc_y += (inc * 5) >> 3;
    }

    /* ⚠️ THE ACCUMULATOR MUST BE 32-BIT, and the reason is not size.
     *
     * A wrap is only seamless if it lands on a whole number of TABLE PERIODS
     * (16 cells = 256 units), not merely on a whole number of cells. As a
     * uint16 with 10 fractional bits this wrapped at 64 units -- integral in
     * cells, but a quarter of the way through the table -- so the field jumped
     * to a different part of the noise on every wrap and read as a short video
     * looping. At 32 bits the wrap is 2^22 units, an exact multiple of 256, and
     * at the slowest drift it is ~100 days away regardless. */

    /* The power ladder's ceiling (power.c) on the value knob itself: a Drift
     * at or under it is untouched, and over it looks as the knob at the
     * ceiling would. Linear in drive, so it saves less than on the other
     * effects; the gauge counts what is driven. */
    uint8_t v = rgb_matrix_config.hsv.v;
    if (v > power_val_cap()) v = power_val_cap();

    RGB_MATRIX_USE_LIMITS(led_min, led_max);
    for (uint8_t i = led_min; i < led_max; i++) {
        RGB_MATRIX_TEST_LED_FLAGS();

        /* The board is 217 x 59, about 3.7:1. Sampling raw would stretch every
         * feature horizontally, which reads as a bug rather than a style.
         * Compress x rather than stretching y, so both stay in range. */
        /* Sample in the SAME fixed point the accumulator uses (10 fractional
         * bits), so sub-unit drift actually reaches the interpolation instead
         * of being rounded away. A cell is 16 units = 1<<14 in this scale. */
        uint32_t sx = ((uint32_t)(g_led_config.point[i].x >> 2) << 10) + acc_x;
        uint32_t sy = ((uint32_t) g_led_config.point[i].y        << 10) + acc_y;

        uint8_t gx = (sx >> 14) & 15, fx = drift_ease[(sx >> 6) & 0xFF];
        uint8_t gy = (sy >> 14) & 15, fy = drift_ease[(sy >> 6) & 0xFF];
        uint8_t gx1 = (gx + 1) & 15, gy1 = (gy + 1) & 15;

        uint8_t top = drift_lerp8(drift_noise[(gy  << 4) | gx],
                                  drift_noise[(gy  << 4) | gx1], fx);
        uint8_t bot = drift_lerp8(drift_noise[(gy1 << 4) | gx],
                                  drift_noise[(gy1 << 4) | gx1], fx);
        uint8_t n   = drift_lerp8(top, bot, fy);

        /* Bias the field DARK. Bilinearly interpolated uniform noise clusters
         * hard around the middle -- measured, 62% of samples land in the top
         * four ramp stops and only 8% in the darkest two -- so a linear mapping
         * lights the whole board and reads as "lit, with blobs" rather than
         * "dark, with a few bright spots". Squaring flips that to 56% in the
         * darkest three. This is a gamma on the FIELD, not on brightness;
         * rgb_matrix_config.hsv.v still scales the result. */
        n = scale8(n, n);

        /* Ramp position, 5-bit fraction between adjacent stops. */
        uint8_t  idx  = n >> 5;
        uint8_t  frac = n & 31;
        const rgb_t *a = &drift_ramp[idx], *b = &drift_ramp[idx + 1];

        uint8_t r  = (uint8_t)(a->r + (((int16_t)(b->r - a->r) * frac) >> 5));
        uint8_t g  = (uint8_t)(a->g + (((int16_t)(b->g - a->g) * frac) >> 5));
        uint8_t bl = (uint8_t)(a->b + (((int16_t)(b->b - a->b) * frac) >> 5));

        /* Scale by the configured value, or the brightness keys stop working on
         * this mode. Kept as the last step so a decay trail could modulate it
         * later without touching the colour path. */
        rgb_matrix_set_color(i, scale8(r, v), scale8(g, v), scale8(bl, v));
    }
    return rgb_matrix_check_finished_leds(led_max);
}

#endif // RGB_MATRIX_CUSTOM_EFFECT_IMPLS
