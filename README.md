# Varifocal Acoustic Focusing System

Built during the Formlabs IAP 2026 hardware hackathon, this project is a device that automatically focuses sound directly at a specific target. 

## Hardware & Operation

*   **Acoustic Lenses:** We designed the sound-focusing lenses using audio simulations and 3D printed them at Formlabs. 
*   **Auto-Focusing:** The system uses an ultrasonic distance sensor to figure out exactly how far away the target is. A Teensy microcontroller then tells a stepper motor to slide the front lens back and forth on a rail to bring the sound into focus.
*   **Power & Controls:** The device is powered by a drone battery (6S LiPo) and uses a repurposed fire alarm as the main power switch. Pressing a mechanical keyboard switch triggers a speaker to play a 1kHz sound through the lenses.