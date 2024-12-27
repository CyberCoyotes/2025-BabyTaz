// frc/robot/io/led/Animation.java
package frc.robot.io.led;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.FireAnimation;

public class LEDAnimation {
    public static Animation rainbow(int brightness) {
        return new RainbowAnimation(brightness, 0.7, 8);
    }

    public static Animation strobe(int r, int g, int b) {
        return new StrobeAnimation(r, g, b, 0, 0.3, 8);
    }

    public static Animation kitt(int r, int g, int b) {
        return new LarsonAnimation(r, g, b, 0, 0.7, 8, 
            LarsonAnimation.BounceMode.Front, 3);
    }

    public static Animation fire(int brightness, double speed) {
        return new FireAnimation(brightness, speed, 8, 0.7, 0.5);
    }
}