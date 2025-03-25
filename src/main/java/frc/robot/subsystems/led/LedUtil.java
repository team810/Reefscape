package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LedUtil {
    private static LedUtil INSTANCE;
    private final AddressableLEDBuffer buffer;
    private final AddressableLED led;

    public LedUtil() {
        led = new AddressableLED(9);
        buffer = new AddressableLEDBuffer(190);
        led.setLength(190);
        buffer.setRGB(0,0,0,0);
        led.setColorOrder(AddressableLED.ColorOrder.kRBG);
        led.setData(buffer);
        led.start();
        setState(LedState.DRIVING);
    }

    public void setState(LedState state) {
        int r,g,b;
        r = 0;
        g = 0;
        b = 0;

        switch(state) {
            case AUTO -> {
                b = 255;
            }
            case GOOD -> {
                r = 255;
            }
            case DRIVING -> {
                g = 255;
            }
        }
        for (int i = 0; i < 190; i++) {
            buffer.setRGB(i,r,g,b);
        }
        led.setData(buffer);
        led.setSyncTime(1000);
    }

    public static LedUtil getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new LedUtil();
        }
        return INSTANCE;
    }

}
