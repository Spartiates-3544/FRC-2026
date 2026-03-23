package frc.lib.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public final class LedStrips {
    private static AddressableLED led;
    private static AddressableLEDBuffer buffer;

    public enum LedColor {
        RED, GREEN, BLUE, ERROR,
    }

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
            case RED:
                pattern = LEDPattern.solid(Color.kRed);
                break;

            case BLUE:
                pattern = LEDPattern.solid(Color.kBlue);
                break;

            case GREEN:
                pattern = LEDPattern.solid(Color.kGreen);
                break;

            case ERROR:
            default:
                pattern = LEDPattern.solid(Color.kBlack);
                break;
        }

        pattern.applyTo(buffer);
        led.setData(buffer);
    }
}