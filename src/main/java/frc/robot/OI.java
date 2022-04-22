package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * button mapping constants
 */
public class OI {
    // public static class DriverMappings{
    //     public static final int RUN_INTAKE = 1,
    //                             AUTO_SHOOT = 2;
    // }

    public static class Mappings {
        public static final int LAYUP_SHOT = 1,
                                PROTECTED_SHOT = 2,
                                INSIDE_LINE_SHOT = 3,
                                OUTSIDE_LINE_SHOT = 4,
                                REVERSE_TRIGGER = 10,
                                EJECT_BALL = 11,
                                RUN_TRAVERSAL = 5,
                                PREPARE_SHOOT = 6,
                                AUTO_SHOOT = 8,
                                RUN_INTAKE = 7,
                                RUN_TRAVERSAL_AND_TRIGGER = 9,
                                CLIMBER_ARM_TOGGLE = 10;
    }


    private Joystick joystick;

    public OI(Joystick joystick){
        this.joystick = joystick;
    }

    public OI bind(int button, Command command){
        new JoystickButton(joystick, button).whileHeld(command);
        return this;
    }
}
