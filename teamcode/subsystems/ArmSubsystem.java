package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class ArmSubsystem extends SubsystemBase {

    private DcMotorEx slideMotor, armLeft, armRight;
    private Telemetry telemetry;

    public ArmSubsystem(DcMotorEx armMotor,
                        DcMotorEx armLeft,
                        DcMotorEx armRight,
                        Telemetry telemetry) {
        this.slideMotor = armMotor;
        this.armLeft = armLeft;
        this.armRight = armRight;
        this.telemetry = telemetry;

        armRight.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        armLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetArmEncoders();
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //slideMotor.setVelocityPIDFCoefficients(Constants.SLIDE_P, Constants.SLIDE_I, Constants.SLIDE_D, Constants.SLIDE_F);
        slideMotor.setPositionPIDFCoefficients(Constants.SLIDE_POS);
        //DcMotorControllerEx slideMotorController = (DcMotorControllerEx)slideMotor.getController();
        //PIDFCoefficients slidePIDF = new PIDFCoefficients(Constants.SLIDE_P, Constants.SLIDE_I, Constants.SLIDE_D, Constants.SLIDE_F);
        //slideMotorController.setPIDFCoefficients(slideMotor.getPortNumber(), DcMotor.RunMode.RUN_TO_POSITION, slidePIDF);
    }

    @Override
    public void periodic() {
        telemetry.addData("target pos", slideMotor.getTargetPosition());
        telemetry.addData("current pos", slideMotor.getCurrentPosition());
        telemetry.addData("error", slideMotor.getTargetPosition() - slideMotor.getCurrentPosition());
        telemetry.addData("P", slideMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).p);
        telemetry.addData("I", slideMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).i);
        telemetry.addData("D", slideMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).d);
        telemetry.addData("F", slideMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).f);
        telemetry.addData("armTick", armLeft.getCurrentPosition());
        telemetry.update();
    }

    public void setArmAngle(double a){
        a = (a / (2 * Math.PI)) * 5281 / Constants.ARM_GEAR_RATIO;

        armLeft.setTargetPosition((int) a);
        armRight.setTargetPosition((int) a);

        armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armLeft.setPower(Constants.ARM_MAX_ANGULAR_SPEED);
        armRight.setPower(Constants.ARM_MAX_ANGULAR_SPEED);
    }

    public void setArmAngleByTick(double a){
        armLeft.setTargetPosition((int) a);
        armRight.setTargetPosition((int) a);

        armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armLeft.setPower(Constants.ARM_MAX_ANGULAR_SPEED);
        armRight.setPower(Constants.ARM_MAX_ANGULAR_SPEED);
    }

    public void setSlideLength(double l){
        //double armTick = (Constants.SLIDE_MAX_TICK / Constants.SLIDE_MAX_LENGTH - Constants.SLIDE_MIN_LENGTH) * (l - Constants.SLIDE_MAX_LENGTH) + Constants.SLIDE_MAX_TICK;

        double armTick = ((l - Constants.SLIDE_MIN_LENGTH) / (Constants.SLIDE_MAX_LENGTH - Constants.SLIDE_MIN_LENGTH))
                * Constants.SLIDE_MAX_TICK;

        if (armTick > 2200){
            armTick = 2200;
        }
        if (armTick < 0){
            armTick = 0;
        }

        slideMotor.setTargetPosition((int) armTick);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(1);
    }

    public void setSlideLengthByTick(double armTick){
        slideMotor.setTargetPosition((int) armTick);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(1);
    }

    public void setSlideLengthByAngle(double armAngle){
        //i'm literally sobbing this took so long to figure out
        double armLength = Math.sqrt(Math.pow(((-Math.sqrt(3) * Constants.ARM_PIVOT_DIST_FROM_FRONT)
                - Constants.ARM_PIVOT_HEIGHT) / (Math.tan(armAngle) - Math.sqrt(3)), 2 )
                + Math.pow(((Math.sqrt(3) * Constants.ARM_PIVOT_HEIGHT * Math.tan(armAngle)
                + Constants.ARM_PIVOT_HEIGHT * Math.pow(Math.tan(armAngle), 2)
                + 3 * Constants.ARM_PIVOT_DIST_FROM_FRONT * Math.tan(armAngle)
                + Math.sqrt(3) * Constants.ARM_PIVOT_DIST_FROM_FRONT * Math.pow(Math.tan(armAngle), 2))
                / (3 - Math.pow(Math.tan(armAngle), 2))), 2));

        setSlideLength(armLength);
    }

    public double getArmAngle(){
        double a = (double) armLeft.getCurrentPosition();
        a = (a * Constants.ARM_GEAR_RATIO / 5281) * 2 * Math.PI;
        return a;
    }

    public double getArmTick(){
        return armLeft.getCurrentPosition();
    }

    public double getSlideTick() {return slideMotor.getCurrentPosition();}
    public double getSlideTargetTick() {return slideMotor.getTargetPosition();}

    public void moveArm(double armSpeed) {
        armLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armLeft.setPower(armSpeed);
        armRight.setPower(armSpeed);
    }

    public void runSlide(double armSpeed) {
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setPower(armSpeed);
    }

    public void extendSlide(){
        slideMotor.setTargetPosition((int) Constants.SLIDE_MAX_TICK);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(1);
    }

    public void retractSlide() {
        slideMotor.setTargetPosition((int) Constants.SLIDE_MIN_TICK);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(1);
    }
    public void stopArm(){
        armLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armLeft.setPower(0);
        armRight.setPower(0);
        slideMotor.setPower(0);
    }

    public void resetArmEncoders(){
        armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}

