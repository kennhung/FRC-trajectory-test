package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class DriveBase {

    SpeedController left, right;
    DifferentialDrive drive;
    Encoder lEnc, rEnc;
    PIDController lPID, rPID;

    public DriveBase(SpeedController left, SpeedController right, Encoder lEnc, Encoder rEnc) {
        lPID = new PIDController(1, 0, 0);
        rPID = new PIDController(1, 0, 0);

        this.left = left;
        this.right = right;
        this.lEnc = lEnc;
        this.rEnc = rEnc;
        drive = new DifferentialDrive(left, right);
    }

    void feedSpeed(double leftSpeed, double rightSpeed) {
        double lOut = lPID.calculate(lEnc.getRate(), leftSpeed);
        double rOut = lPID.calculate(rEnc.getRate(), rightSpeed);

        left.setVoltage(lOut);
        right.setVoltage(rOut);
    }

}
