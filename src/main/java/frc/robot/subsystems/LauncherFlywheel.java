package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.LauncherConstants;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

public class LauncherFlywheel extends SubsystemBase {
    TalonFX shooter;
   // TalonFX turret;
    TalonFX hood;
   
    DoubleSupplier setpointDelta;

    VelocityVoltage pidControlRequest;
    VoltageOut sysIdControlRequest;

    StatusSignal<Angle> positionSignal;
    StatusSignal<AngularVelocity> velocitySignal;
    StatusSignal<Voltage> appliedOutputSignal;

    SysIdRoutine sysIdRoutine;

     private static final InterpolatingTreeMap<Double, Rotation2d> launchHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);

    public LauncherFlywheel(int id) {
        this.shooter = new TalonFX(id, LauncherConstants.canBus);
        this.positionSignal = shooter.getPosition();
        this.velocitySignal = shooter.getVelocity();
        this.appliedOutputSignal = shooter.getMotorVoltage();
        
        this.pidControlRequest = new VelocityVoltage(0);
        this.sysIdControlRequest= new VoltageOut(0);

        var slotConfig = new Slot0Configs()
                .withKV(LauncherConstants.kV)
                .withKA(LauncherConstants.kA)
                .withKS(LauncherConstants.kS)
                .withKP(LauncherConstants.kP)
                .withKI(LauncherConstants.kI)
                .withKD(LauncherConstants.kD);

        var talonFxConfig = new TalonFXConfiguration()
                .withSlot0(slotConfig);
        talonFxConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooter.getConfigurator().apply(talonFxConfig);

        pidControlRequest.withEnableFOC(true);
        sysIdControlRequest.withEnableFOC(true);

        var sysIdMechanism = new SysIdRoutine.Mechanism(
                this::setVoltage,
                null,
                this
        );

        var sysIdConfig = new SysIdRoutine.Config(
                Volts.per(Second).of(LauncherConstants.sysIdRampRate),
                Volts.of(LauncherConstants.sysIdDynamicStepVoltage),
                Second.of(LauncherConstants.sysIdTimeout),
                (state) -> SignalLogger.writeString("state", state.toString())
        );

        this.sysIdRoutine = new SysIdRoutine(sysIdConfig, sysIdMechanism);

        hood = new TalonFX(22, "canivore");
        var talonFXConfigs = new TalonFXConfiguration();
        talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic Expo settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 0; // Unlimited cruise velocity
        motionMagicConfigs.MotionMagicExpo_kV = 0.12; // kV is around 0.12 V/rps
        motionMagicConfigs.MotionMagicExpo_kA = 0.1; // Use a slower kA of 0.1 V/(rps/s)

        hood.getConfigurator().apply(talonFXConfigs);

       
       
    }
    static {
        launchHoodAngleMap.put(1.34, Rotation2d.fromDegrees(19.0));
        launchHoodAngleMap.put(1.78, Rotation2d.fromDegrees(19.0));
        launchHoodAngleMap.put(2.17, Rotation2d.fromDegrees(24.0));
        launchHoodAngleMap.put(2.81, Rotation2d.fromDegrees(27.0));
        launchHoodAngleMap.put(3.82, Rotation2d.fromDegrees(29.0));
        launchHoodAngleMap.put(4.09, Rotation2d.fromDegrees(30.0));
        launchHoodAngleMap.put(4.40, Rotation2d.fromDegrees(31.0));
        launchHoodAngleMap.put(4.77, Rotation2d.fromDegrees(32.0));
        launchHoodAngleMap.put(5.57, Rotation2d.fromDegrees(32.0));
        launchHoodAngleMap.put(5.60, Rotation2d.fromDegrees(35.0));
    }

    public void setVelocity(double velocity) {
        
        shooter.setControl(this.pidControlRequest.withVelocity(velocity));
    }

    public void setVoltage(Voltage voltage) {
        this.sysIdControlRequest.withOutput(voltage);
        shooter.setControl(sysIdControlRequest);
    }

    public void setVoltage(double voltage) {
        this.sysIdControlRequest.withOutput(voltage);
        shooter.setControl(sysIdControlRequest);
    }

    public Angle getPosition() {
        return positionSignal.refresh().getValue();
    }

    public AngularVelocity getVelocity() {
        return velocitySignal.refresh().getValue();
    }
    

    public Voltage getAppliedOutput() {
        return appliedOutputSignal.refresh().getValue();
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    public Command setHoodAtTop() {
        return Commands.runOnce(()-> {
            hood.setControl(new MotionMagicExpoVoltage(1.67).withEnableFOC(true));
        },this);
    }
    public Command setHoodAtBottom() {
        return Commands.runOnce(()->hood.setControl(new MotionMagicExpoVoltage(0.067).withEnableFOC(true)), this);
    }
    public Command setVelocityCommand(double rpm) {
        return Commands.run(()->setVelocity(rpm/60.0), this);
    }

    

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Launcher Velocity", shooter.getVelocity().getValueAsDouble());
        
    }

}
