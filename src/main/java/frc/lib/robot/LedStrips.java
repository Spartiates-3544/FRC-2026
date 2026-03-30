package frc.lib.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public final class LedStrips {
    private static AddressableLED led;
    private static AddressableLEDBuffer buffer;

    public enum LedColor {
        RED, GREEN, BLUE, YELLOW, PURPLE, ERROR,
        RED_FLASH, GREEN_FLASH, BLUE_FLASH,
    }

    // Pre-created patterns to avoid per-frame allocations
    private static final LEDPattern PATTERN_RED       = LEDPattern.solid(Color.kRed);
    private static final LEDPattern PATTERN_GREEN     = LEDPattern.solid(Color.kGreen);
    private static final LEDPattern PATTERN_BLUE      = LEDPattern.solid(Color.kBlue);
    private static final LEDPattern PATTERN_YELLOW    = LEDPattern.solid(Color.kYellow);
    private static final LEDPattern PATTERN_PURPLE    = LEDPattern.solid(Color.kPurple);
    private static final LEDPattern PATTERN_OFF       = LEDPattern.solid(Color.kBlack);
    private static final LEDPattern PATTERN_RED_FLASH   = PATTERN_RED.blink(Seconds.of(0.1), Seconds.of(0.1));
    private static final LEDPattern PATTERN_GREEN_FLASH = PATTERN_GREEN.blink(Seconds.of(0.15), Seconds.of(0.15));
    private static final LEDPattern PATTERN_BLUE_FLASH  = PATTERN_BLUE.blink(Seconds.of(0.15), Seconds.of(0.15));

    private LedStrips() {
    }

    public static void init(int port, int length) {
        led = new AddressableLED(port);
        buffer = new AddressableLEDBuffer(length);

        led.setLength(length);
        led.setData(buffer);
        led.start();
    }

    public static void setLED(LedColor color) {
        if (led == null || buffer == null) {
            return;
        }

        LEDPattern pattern;

        switch (color) {
            case RED:         pattern = PATTERN_RED;         break;
            case GREEN:       pattern = PATTERN_GREEN;       break;
            case BLUE:        pattern = PATTERN_BLUE;        break;
            case YELLOW:      pattern = PATTERN_YELLOW;      break;
            case PURPLE:      pattern = PATTERN_PURPLE;      break;
            case RED_FLASH:   pattern = PATTERN_RED_FLASH;   break;
            case GREEN_FLASH: pattern = PATTERN_GREEN_FLASH; break;
            case BLUE_FLASH:  pattern = PATTERN_BLUE_FLASH;  break;
            case ERROR:
            default:          pattern = PATTERN_OFF;         break;
        }

        pattern.applyTo(buffer);
        led.setData(buffer);
    }
}