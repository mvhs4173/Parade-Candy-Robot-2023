package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class LEDStrip {
    private AddressableLED ledStrip; // Object for the PWM-based LED strip
    private AddressableLEDBuffer ledBuffer; // Essentially an array that organizes data for each
                                            // individual LED. Argue this object into the .setData() method
                                            // inside the AddressableLED object to update the data.
                                            // (once .start() is called, LEDs update whenever data is changed)

    private int chaseIndex = 0;

    // CONSTRUCTOR
    public LEDStrip(int pwmPort, int length) {
        this.ledStrip = new AddressableLED(pwmPort);
        this.ledBuffer = new AddressableLEDBuffer(length);
        ledStrip.setLength(ledBuffer.getLength());
    }

    public void start() {
        ledStrip.start(); // Start automatically updating the led strip to whatever data was last given
    }

    /**
     * Set all lights in the strip to black (off)
     */
    public void turnAllOff() {
        // Loop through all individual lights...
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 0, 0, 0); // Turn off by setting all color values to 0 (black)
        }
        ledStrip.setData(ledBuffer); // Give the led strip the data from the buffer
    }

    public void chaseTest(int r, int g, int b) {
        // Move chase index up by 1 (from what it was last time this method was called)
        chaseIndex = loopLEDIndex(chaseIndex + 1);
        // Turn on the new light
        ledBuffer.setRGB(chaseIndex, r, g, b);
        // Turn off the old light, but it actually works this time
        if (chaseIndex == 0) { 
            ledBuffer.setRGB(ledBuffer.getLength() - 1, 0, 0, 0); // If the index is at the beginning, turn off the last light on the strip
        }
        else {
            ledBuffer.setRGB(chaseIndex - 1, 0, 0, 0); // Otherwise, turn off the previous light like normal
        }

        ledStrip.setData(ledBuffer); // Give the led strip the data from the buffer
    }

    public void setAllToColor(int r, int g, int b) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, r, g, b);
        }

        ledStrip.setData(ledBuffer);
    }

    
    public void setToChristmasColorPattern() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            int indexInPattern = (i + 1) % 10;
            
            if (indexInPattern >= 0 && indexInPattern <= 3) {
                ledBuffer.setRGB(i, 255, 0, 0); // Red
            } else if (indexInPattern == 4){
                ledBuffer.setRGB(i, 0, 0, 0); // nothing
            } else if (indexInPattern >= 5 && indexInPattern <= 8 ){
                ledBuffer.setRGB(i, 0, 255, 0); // Green
            } else if (indexInPattern == 9){
                ledBuffer.setRGB(i, 0, 0, 0); // nothing
            }
        }

        ledStrip.setData(ledBuffer); // Give the led strip the data from the buffer
    }

    public void chaseTrail(int trailLength, int r, int g, int b) {
        // Move chase index up by 1 (from what it was last time this method was called)
        chaseIndex = loopLEDIndex(chaseIndex + 1);

        for (int n = 1; n <= trailLength; n++) {
            int i = loopLEDIndex(chaseIndex - (n - 1));
            double ratioToRemove = ((n-1) / (double) trailLength);
            ledBuffer.setRGB(i, (int) (r - r*ratioToRemove), (int) (g - g*ratioToRemove), (int) (b - b*ratioToRemove));
        }
        ledBuffer.setRGB(loopLEDIndex(chaseIndex - trailLength), 0, 0, 0);

        ledStrip.setData(ledBuffer);
    }

    public void incrementPattern() {
        Color previousValue = ledBuffer.getLED(ledBuffer.getLength() - 1);
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            Color swap = ledBuffer.getLED(i);
            ledBuffer.setLED(i, previousValue);
            previousValue = swap;
        }
        
        ledStrip.setData(ledBuffer); // Give the led strip the data from the buffer
    }

    
    private int loopLEDIndex(int i) {
        if (i >= ledBuffer.getLength()) {
            return loopLEDIndex(i - ledBuffer.getLength());
        } else if (i < 0) {
            return loopLEDIndex(i + ledBuffer.getLength());
        }
        return i;
    }
}
