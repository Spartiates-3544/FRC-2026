package frc.lib.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class LedStrips {
    private static AddressableLED led;

    private static int length2 = 0;

    public static enum Couleurs {
        RED,
        GREEN,
        BLUE,
        ERREUR_MOTEUR_BRISE
    }

    public static void init(int port, int length) {
        led = new AddressableLED(port);
        led.setLength(length);
        led.start();
        length2 = length;
    }

    public static void definirCouleurs(Couleurs couleur) {
        LEDPattern pattern;
    
        switch (couleur) {
            case RED:
                pattern = LEDPattern.solid(Color.kRed);
                break;
            
            case BLUE:
                pattern = LEDPattern.solid(Color.kBlue);
                break;
        
            case GREEN:
                pattern = LEDPattern.solid(Color.kGreen);
                break;

            default: 
                pattern = LEDPattern.solid(Color.kBlack);
                break;
        }

        AddressableLEDBuffer liste = new AddressableLEDBuffer(length2);
        pattern.applyTo(liste);
        led.setData(liste);
    }


}
