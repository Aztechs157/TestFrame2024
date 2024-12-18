// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.assabet.aztechs157.input.layouts.DynamicLayout;
import org.assabet.aztechs157.input.layouts.Layout;
import org.assabet.aztechs157.input.layouts.MapLayout;
import java.util.function.DoubleSupplier;
import org.assabet.aztechs157.input.models.XboxOne;
import org.assabet.aztechs157.input.values.Axis;
import org.assabet.aztechs157.input.values.Button;
import org.assabet.aztechs157.numbers.Deadzone;
import org.assabet.aztechs157.numbers.Range;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.ControllerConstants;

/** Add your docs here. */
public class Inputs extends DynamicLayout {

    public static final Axis.Key precisionDrive = new Axis.Key();

    //////////////////////////////////////////////////////////
    // HERE'S AN EXAMPLE OF USING ADDING NEW BUTTONS/AXIS
    /////////////////////////////////////////////////////////

    // public static final Button.Key resetGyro = new Button.Key();

    // public static final Button.Key driveToSpeaker = new Button.Key();
    // public static final Button.Key driveToAmp = new Button.Key();
    // public static final Button.Key autoIntake = new Button.Key();

    // public static final Button.Key intake = new Button.Key();
    // public static final Button.Key loadNote = new Button.Key();
    // public static final Button.Key eject = new Button.Key();

    // public static final Button.Key highShotSpinUp = new Button.Key();
    // public static final Button.Key lowShotSpinUp = new Button.Key();

    // public static final Button.Key highShot = new Button.Key();
    // public static final Button.Key lowShot = new Button.Key();
    // public static final Button.Key pass = new Button.Key();

    // public static final Button.Key liftHanger = new Button.Key();
    // public static final Button.Key retractHanger = new Button.Key();
    // public static final Button.Key retractHangerPin = new Button.Key();
    // public static final Button.Key extendHangerPin = new Button.Key();

    public static Inputs createFromChooser() {
        final SendableChooser<Layout> chooser = new SendableChooser<>();
        chooser.setDefaultOption("xbox", doubleXBOXLayout());
        Shuffleboard.getTab("Driver").add("Layout Choose", chooser);

        return new Inputs(chooser);
    }

    private Inputs(final SendableChooser<Layout> chooser) {
        super(chooser::getSelected);
    }

    private static Layout doubleXBOXLayout() {

        final var layout = new MapLayout();
        final var driver = new XboxOne(ControllerConstants.DRIVER_CONTROLLER_PORT);
        final var operator = new XboxOne(ControllerConstants.OPERATOR_CONTROLLER_PORT);

        layout.assign(precisionDrive, driver.leftTriggerHeld.map(Deadzone.forAxis(new Range(0.0, 0.05))::apply).scaledBy(0.7));

        //////////////////////////////////////////////////////////
        // HERE'S AN EXAMPLE OF USING CONTROLER LAYOUT OPTIONS
        /////////////////////////////////////////////////////////

        // layout.assign(driveToSpeaker, operator.a);
        // layout.assign(driveToAmp, operator.b);
        // layout.assign(resetGyro, driver.start);
        // layout.assign(autoIntake, operator.leftBumper);

        // layout.assign(intake, driver.leftBumper);
        // layout.assign(loadNote, operator.x);

        // layout.assign(highShotSpinUp, operator.rightBumper);
        // layout.assign(lowShotSpinUp, operator.leftBumper);

        // layout.assign(highShot, new Button(() -> driver.rightTriggerHeld.get() > 0.2));
        // layout.assign(lowShot, driver.rightBumper);
        // layout.assign(pass, operator.a);
        // layout.assign(eject, operator.b);

        // layout.assign(liftHanger, operator.pov.up);
        // layout.assign(retractHanger, operator.pov.down);

        return layout;
    }

}
