package org.firstinspires.ftc.teamcode.Utility.Math.kinematics;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.Utility.Math.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.Utility.Math.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.Utility.Math.geometry.Twist2d;

import java.util.Arrays;

public class SwerveDriveKinematics
        implements Kinematics<SwerveDriveKinematics.SwerveDriveWheelStates, SwerveDriveWheelPositions> {
    public static class SwerveDriveWheelStates {
        public SwerveModuleState[] states;

        /**
         * Creates a new SwerveDriveWheelStates instance.
         *
         * @param states The swerve module states. This will be deeply copied.
         */
        public SwerveDriveWheelStates(SwerveModuleState[] states) {
            this.states = new SwerveModuleState[states.length];
            for (int i = 0; i < states.length; i++) {
                this.states[i] = new SwerveModuleState(states[i].speedMetersPerSecond, states[i].angle);
            }
        }
    }

    private final SimpleMatrix m_inverseKinematics;
    private final SimpleMatrix m_forwardKinematics;

    private final int m_numModules;
    private final Translation2d[] m_modules;
    private Rotation2d[] m_moduleHeadings;
    private Translation2d m_prevCoR = new Translation2d();

    /**
     * Constructs a swerve drive kinematics object. This takes in a variable number of module
     * locations as Translation2d objects. The order in which you pass in the module locations is the
     * same order that you will receive the module states when performing inverse kinematics. It is
     * also expected that you pass in the module states in the same order when calling the forward
     * kinematics methods.
     *
     * @param moduleTranslationsMeters The locations of the modules relative to the physical center of
     *     the robot.
     */
    public SwerveDriveKinematics(Translation2d... moduleTranslationsMeters) {
        if (moduleTranslationsMeters.length < 2) {
            throw new IllegalArgumentException("A swerve drive requires at least two modules");
        }
        m_numModules = moduleTranslationsMeters.length;
        m_modules = Arrays.copyOf(moduleTranslationsMeters, m_numModules);
        m_moduleHeadings = new Rotation2d[m_numModules];
        Arrays.fill(m_moduleHeadings, new Rotation2d());
        m_inverseKinematics = new SimpleMatrix(m_numModules * 2, 3);

        for (int i = 0; i < m_numModules; i++) {
            m_inverseKinematics.setRow(i * 2 + 0, 0, /* Start Data */ 1, 0, -m_modules[i].getY());
            m_inverseKinematics.setRow(i * 2 + 1, 0, /* Start Data */ 0, 1, +m_modules[i].getX());
        }
        m_forwardKinematics = m_inverseKinematics.pseudoInverse();
    }

    /**
     * Reset the internal swerve module headings.
     *
     * @param moduleHeadings The swerve module headings. The order of the module headings should be
     *     same as passed into the constructor of this class.
     */
    public void resetHeadings(Rotation2d... moduleHeadings) {
        if (moduleHeadings.length != m_numModules) {
            throw new IllegalArgumentException(
                    "Number of headings is not consistent with number of module locations provided in "
                            + "constructor");
        }
        m_moduleHeadings = Arrays.copyOf(moduleHeadings, 4);
    }

    /**
     * Performs inverse kinematics to return the module states from a desired chassis velocity. This
     * method is often used to convert joystick values into module speeds and angles.
     *
     * <p>This function also supports variable centers of rotation. During normal operations, the
     * center of rotation is usually the same as the physical center of the robot; therefore, the
     * argument is defaulted to that use case. However, if you wish to change the center of rotation
     * for evasive maneuvers, vision alignment, or for any other use case, you can do so.
     *
     * <p>In the case that the desired chassis speeds are zero (i.e. the robot will be stationary),
     * the previously calculated module angle will be maintained.
     *
     * @param chassisSpeeds The desired chassis speed.
     * @param centerOfRotationMeters The center of rotation. For example, if you set the center of
     *     rotation at one corner of the robot and provide a chassis speed that only has a dtheta
     *     component, the robot will rotate around that corner.
     * @return An array containing the module states. Use caution because these module states are not
     *     normalized. Sometimes, a user input may cause one of the module speeds to go above the
     *     attainable max velocity. Use the {@link #desaturateWheelSpeeds(SwerveModuleState[], double)
     *     DesaturateWheelSpeeds} function to rectify this issue.
     */
    public SwerveModuleState[] toSwerveModuleStates(
            ChassisSpeeds chassisSpeeds, Translation2d centerOfRotationMeters) {
        SwerveModuleState[] moduleStates = new SwerveModuleState[m_numModules];

        if (chassisSpeeds.vxMetersPerSecond == 0.0
                && chassisSpeeds.vyMetersPerSecond == 0.0
                && chassisSpeeds.omegaRadiansPerSecond == 0.0) {
            for (int i = 0; i < m_numModules; i++) {
                moduleStates[i] = new SwerveModuleState(0.0, m_moduleHeadings[i]);
            }

            return moduleStates;
        }

        if (!centerOfRotationMeters.equals(m_prevCoR)) {
            for (int i = 0; i < m_numModules; i++) {
                m_inverseKinematics.setRow(
                        i * 2 + 0,
                        0, /* Start Data */
                        1,
                        0,
                        -m_modules[i].getY() + centerOfRotationMeters.getY());
                m_inverseKinematics.setRow(
                        i * 2 + 1,
                        0, /* Start Data */
                        0,
                        1,
                        +m_modules[i].getX() - centerOfRotationMeters.getX());
            }
            m_prevCoR = centerOfRotationMeters;
        }

        SimpleMatrix chassisSpeedsVector = new SimpleMatrix(3, 1);
        chassisSpeedsVector.setColumn(
                0,
                0,
                chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond,
                chassisSpeeds.omegaRadiansPerSecond);

        SimpleMatrix moduleStatesMatrix = m_inverseKinematics.mult(chassisSpeedsVector);

        for (int i = 0; i < m_numModules; i++) {
            double x = moduleStatesMatrix.get(i * 2, 0);
            double y = moduleStatesMatrix.get(i * 2 + 1, 0);

            double speed = Math.hypot(x, y);
            Rotation2d angle = new Rotation2d(x, y);

            moduleStates[i] = new SwerveModuleState(speed, angle);
            m_moduleHeadings[i] = angle;
        }

        return moduleStates;
    }

    /**
     * Performs inverse kinematics. See {@link #toSwerveModuleStates(ChassisSpeeds, Translation2d)}
     * toSwerveModuleStates for more information.
     *
     * @param chassisSpeeds The desired chassis speed.
     * @return An array containing the module states.
     */
    public SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds chassisSpeeds) {
        return toSwerveModuleStates(chassisSpeeds, new Translation2d());
    }

    @Override
    public SwerveDriveWheelStates toWheelSpeeds(ChassisSpeeds chassisSpeeds) {
        return new SwerveDriveWheelStates(toSwerveModuleStates(chassisSpeeds));
    }

    /**
     * Performs forward kinematics to return the resulting chassis state from the given module states.
     * This method is often used for odometry -- determining the robot's position on the field using
     * data from the real-world speed and angle of each module on the robot.
     *
     * @param moduleStates The state of the modules (as a SwerveModuleState type) as measured from
     *     respective encoders and gyros. The order of the swerve module states should be same as
     *     passed into the constructor of this class.
     * @return The resulting chassis speed.
     */
    public ChassisSpeeds toChassisSpeeds(SwerveModuleState... moduleStates) {
        if (moduleStates.length != m_numModules) {
            throw new IllegalArgumentException(
                    "Number of modules is not consistent with number of module locations provided in "
                            + "constructor");
        }
        SimpleMatrix moduleStatesMatrix = new SimpleMatrix(m_numModules * 2, 1);

        for (int i = 0; i < m_numModules; i++) {
            SwerveModuleState module = moduleStates[i];
            moduleStatesMatrix.set(i * 2, 0, module.speedMetersPerSecond * module.angle.getCos());
            moduleStatesMatrix.set(i * 2 + 1, module.speedMetersPerSecond * module.angle.getSin());
        }

        SimpleMatrix chassisSpeedsVector = m_forwardKinematics.mult(moduleStatesMatrix);
        return new ChassisSpeeds(
                chassisSpeedsVector.get(0, 0),
                chassisSpeedsVector.get(1, 0),
                chassisSpeedsVector.get(2, 0));
    }

    @Override
    public ChassisSpeeds toChassisSpeeds(SwerveDriveWheelStates wheelStates) {
        return toChassisSpeeds(wheelStates.states);
    }

    /**
     * Performs forward kinematics to return the resulting chassis state from the given module states.
     * This method is often used for odometry -- determining the robot's position on the field using
     * data from the real-world speed and angle of each module on the robot.
     *
     * @param moduleDeltas The latest change in position of the modules (as a SwerveModulePosition
     *     type) as measured from respective encoders and gyros. The order of the swerve module states
     *     should be same as passed into the constructor of this class.
     * @return The resulting Twist2d.
     */
    public Twist2d toTwist2d(SwerveModulePosition... moduleDeltas) {
        if (moduleDeltas.length != m_numModules) {
            throw new IllegalArgumentException(
                    "Number of modules is not consistent with number of module locations provided in "
                            + "constructor");
        }
        SimpleMatrix moduleDeltaMatrix = new SimpleMatrix(m_numModules * 2, 1);

        for (int i = 0; i < m_numModules; i++) {
            SwerveModulePosition module = moduleDeltas[i];
            moduleDeltaMatrix.set(i * 2, 0, module.distanceMeters * module.angle.getCos());
            moduleDeltaMatrix.set(i * 2 + 1, module.distanceMeters * module.angle.getSin());
        }

        SimpleMatrix chassisDeltaVector = m_forwardKinematics.mult(moduleDeltaMatrix);
        return new Twist2d(
                chassisDeltaVector.get(0, 0), chassisDeltaVector.get(1, 0), chassisDeltaVector.get(2, 0));
    }

    @Override
    public Twist2d toTwist2d(SwerveDriveWheelPositions start, SwerveDriveWheelPositions end) {
        if (start.positions.length != end.positions.length) {
            throw new IllegalArgumentException("Inconsistent number of modules!");
        }
        SwerveModulePosition[] newPositions = new SwerveModulePosition[start.positions.length];
        for (int i = 0; i < start.positions.length; i++) {
            SwerveModulePosition startModule = start.positions[i];
            SwerveModulePosition endModule = end.positions[i];
            newPositions[i] =
                    new SwerveModulePosition(
                            endModule.distanceMeters - startModule.distanceMeters, endModule.angle);
        }
        return toTwist2d(newPositions);
    }

    /**
     * Renormalizes the wheel speeds if any individual speed is above the specified maximum.
     *
     * <p>Sometimes, after inverse kinematics, the requested speed from one or more modules may be
     * above the max attainable speed for the driving motor on that module. To fix this issue, one can
     * reduce all the wheel speeds to make sure that all requested module speeds are at-or-below the
     * absolute threshold, while maintaining the ratio of speeds between modules.
     *
     * @param moduleStates Reference to array of module states. The array will be mutated with the
     *     normalized speeds!
     * @param attainableMaxSpeedMetersPerSecond The absolute max speed that a module can reach.
     */
    public static void desaturateWheelSpeeds(
            SwerveModuleState[] moduleStates, double attainableMaxSpeedMetersPerSecond) {
        double realMaxSpeed = 0;
        for (SwerveModuleState moduleState : moduleStates) {
            realMaxSpeed = Math.max(realMaxSpeed, Math.abs(moduleState.speedMetersPerSecond));
        }
        if (realMaxSpeed > attainableMaxSpeedMetersPerSecond) {
            for (SwerveModuleState moduleState : moduleStates) {
                moduleState.speedMetersPerSecond =
                        moduleState.speedMetersPerSecond / realMaxSpeed * attainableMaxSpeedMetersPerSecond;
            }
        }
    }

    /**
     * Renormalizes the wheel speeds if any individual speed is above the specified maximum, as well
     * as getting rid of joystick saturation at edges of joystick.
     *
     * <p>Sometimes, after inverse kinematics, the requested speed from one or more modules may be
     * above the max attainable speed for the driving motor on that module. To fix this issue, one can
     * reduce all the wheel speeds to make sure that all requested module speeds are at-or-below the
     * absolute threshold, while maintaining the ratio of speeds between modules.
     *
     * @param moduleStates Reference to array of module states. The array will be mutated with the
     *     normalized speeds!
     * @param desiredChassisSpeed The desired speed of the robot
     * @param attainableMaxModuleSpeedMetersPerSecond The absolute max speed that a module can reach
     * @param attainableMaxTranslationalSpeedMetersPerSecond The absolute max speed that your robot
     *     can reach while translating
     * @param attainableMaxRotationalVelocityRadiansPerSecond The absolute max speed the robot can
     *     reach while rotating
     */
    public static void desaturateWheelSpeeds(
            SwerveModuleState[] moduleStates,
            ChassisSpeeds desiredChassisSpeed,
            double attainableMaxModuleSpeedMetersPerSecond,
            double attainableMaxTranslationalSpeedMetersPerSecond,
            double attainableMaxRotationalVelocityRadiansPerSecond) {
        double realMaxSpeed = 0;
        for (SwerveModuleState moduleState : moduleStates) {
            realMaxSpeed = Math.max(realMaxSpeed, Math.abs(moduleState.speedMetersPerSecond));
        }

        if (attainableMaxTranslationalSpeedMetersPerSecond == 0
                || attainableMaxRotationalVelocityRadiansPerSecond == 0
                || realMaxSpeed == 0) {
            return;
        }
        double translationalK =
                Math.hypot(desiredChassisSpeed.vxMetersPerSecond, desiredChassisSpeed.vyMetersPerSecond)
                        / attainableMaxTranslationalSpeedMetersPerSecond;
        double rotationalK =
                Math.abs(desiredChassisSpeed.omegaRadiansPerSecond)
                        / attainableMaxRotationalVelocityRadiansPerSecond;
        double k = Math.max(translationalK, rotationalK);
        double scale = Math.min(k * attainableMaxModuleSpeedMetersPerSecond / realMaxSpeed, 1);
        for (SwerveModuleState moduleState : moduleStates) {
            moduleState.speedMetersPerSecond *= scale;
        }
    }
}