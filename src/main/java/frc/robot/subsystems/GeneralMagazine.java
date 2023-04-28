package frc.robot.subsystems;

import com.ctre.phoenix.sensors.MagnetFieldStrength;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpiutil.math.MathUtil;
// import frc.team2485.WarlordsLib.Tunable;
// import frc.team2485.WarlordsLib.VelocityPIDSubsystem;
// import frc.team2485.WarlordsLib.motorcontrol.PIDSparkMax;
// import frc.team2485.WarlordsLib.robotConfigs.RobotConfigs;
import frc.robot.Constants;
import frc.robot.Constants.Magazine;

public class GeneralMagazine extends SubsystemBase {

    private final CANSparkMax m_spark;
   private final PIDController m_controller;


    /**
     * Low magazine subystem, controlling the intake rollers and low belt.
     */
    public GeneralMagazine(int port,double gearRatio, int maxCur, boolean invert) {
        m_spark = new CANSparkMax(port, MotorType.kBrushless);
        m_spark.getEncoder().setPositionConversionFactor(gearRatio * Magazine.ROLLER_RADIUS * 2 * Math.PI);
        m_spark.getEncoder().setVelocityConversionFactor(gearRatio * Magazine.ROLLER_RADIUS * 2 * Math.PI / 60);
        m_spark.getEncoder().setPosition(0);
        m_spark.setInverted(invert);
        m_spark.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);
        m_spark.getEncoder().setPosition(0);
        m_spark.setSmartCurrentLimit(maxCur);

        m_spark.setIdleMode(CANSparkMax.IdleMode.kBrake);

        m_controller = new PIDController(1, 0, 0);

        //RobotConfigs.getInstance().addConfigurable(Constants.Magazine.LOW_MAGAZINE_VELOCITY_CONTROLLER_CONFIGURABLE_LABEL, m_spark);

        //this.addToShuffleboard();
    }

    public void addToShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab(Constants.Magazine.TAB_NAME);
        tab.add("Low Spark", m_spark);
        tab.addNumber("Low Position", this::getEncoderPosition);
        tab.addNumber("Low Velocity", this::getEncoderVelocity);
        tab.addNumber("Low Current", m_spark::getOutputCurrent);
    }

    /**
     * Sets talon to a specific PWM
     * @param pwm PWM to set the talon to
     */
    public void setPWM(double pwm) {
        m_spark.set(pwm);
    }

    public void resetPIDs() {
        m_controller.reset();
    }

    /**
     * @return belt encoder position
     */
    public double getEncoderPosition() {
        return m_spark.getEncoder().getPosition();
    }

    public boolean atVelocitySetpoint() {
        return m_controller.atSetpoint();
    }

    /**
     *
     * @return belt encoder velocity in inches per second
     */
    public double getEncoderVelocity() {
        return m_spark.getEncoder().getVelocity();
    }

    public void runVelocityPID(double velocity) {
        m_spark.set(m_controller.calculate(velocity));
    }




    /**
     * Should run periodically and run the motor to tune when enabled
     */
    // public void tunePeriodic(int layer) {
    //     m_spark.runPID();

    // }
}