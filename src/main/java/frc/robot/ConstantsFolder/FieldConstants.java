package frc.robot.ConstantsFolder;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.AStar.Obstacle;

public final class FieldConstants {

  public static final double FIELD_LENGTH = Units.inchesToMeters(651.25);
  public static final double FIELD_WIDTH = Units.inchesToMeters(315.5);
  public static final double TAPE_WIDTH = Units.inchesToMeters(2.0);

  // Dimensions for community and charging station, including the tape.
  public static final class Community {
      // Region dimensions
      public static final double COMMUNITY_INNER_X = 0.0;
      public static final double COMMUNITY_MID_X = Units.inchesToMeters(132.375); // Tape to the left of charging station
      public static final double COMMUNITY_OUTER_X = Units.inchesToMeters(193.25); // Tape to the right of charging station
      public static final double COMMUNITY_LEFT_Y = Units.feetToMeters(18.0);
      public static final double COMMUNITY_MID_Y = COMMUNITY_LEFT_Y - Units.inchesToMeters(59.39) + TAPE_WIDTH;
      public static final double COMMUNITY_RIGHT_Y = 0.0;
      public static final Translation2d[] COMMUNITY_REGION_CORNERS = new Translation2d[] {
              new Translation2d(COMMUNITY_INNER_X, COMMUNITY_RIGHT_Y),
              new Translation2d(COMMUNITY_INNER_X, COMMUNITY_LEFT_Y),
              new Translation2d(COMMUNITY_MID_X, COMMUNITY_LEFT_Y),
              new Translation2d(COMMUNITY_MID_X, COMMUNITY_MID_Y),
              new Translation2d(COMMUNITY_OUTER_X, COMMUNITY_MID_Y),
              new Translation2d(COMMUNITY_OUTER_X, COMMUNITY_RIGHT_Y),
      };

      // Charging station dimensions
      public static final double CHARGE_STATION_LENGTH = Units.inchesToMeters(76.125);
      public static final double CHARGE_STATION_WIDTH = Units.inchesToMeters(97.25);
      public static final double CHARGE_STATION_OUTER_X = COMMUNITY_OUTER_X - TAPE_WIDTH;
      public static final double CHARGE_STATION_INNER_X = CHARGE_STATION_OUTER_X - CHARGE_STATION_LENGTH;
      public static final double CHARGE_STATION_LEFT_Y = COMMUNITY_MID_Y - TAPE_WIDTH;
      public static final double CHARGE_STATION_RIGHT_Y = CHARGE_STATION_LEFT_Y - CHARGE_STATION_WIDTH;
      public static final Translation2d[] CHARGE_STATION_CORNERS = new Translation2d[] {
              new Translation2d(CHARGE_STATION_INNER_X, CHARGE_STATION_RIGHT_Y),
              new Translation2d(CHARGE_STATION_INNER_X, CHARGE_STATION_LEFT_Y),
              new Translation2d(CHARGE_STATION_OUTER_X, CHARGE_STATION_RIGHT_Y),
              new Translation2d(CHARGE_STATION_OUTER_X, CHARGE_STATION_LEFT_Y)
      };

      // Cable bump
      public static final double CABLE_BUMP_INNER_X = COMMUNITY_INNER_X + Grids.GRID_OUTER_X + Units.inchesToMeters(95.25);
      public static final double CABLE_BUMP_OUTER_X = CABLE_BUMP_INNER_X + Units.inchesToMeters(7);
      public static final Translation2d[] CABLE_BUMP_CORNERS = new Translation2d[] {
              new Translation2d(CABLE_BUMP_INNER_X, 0.0),
              new Translation2d(CABLE_BUMP_INNER_X, CHARGE_STATION_RIGHT_Y),
              new Translation2d(CABLE_BUMP_OUTER_X, 0.0),
              new Translation2d(CABLE_BUMP_OUTER_X, CHARGE_STATION_RIGHT_Y)
      };
  }

  // Dimensions for grids and nodes
  public static final class Grids {
      // X layout
      public static final double GRID_OUTER_X = Units.inchesToMeters(54.25);
      public static final double GRID_LOW_X = GRID_OUTER_X - (Units.inchesToMeters(14.25) / 2.0); // Centered when under cube
                                                                                      // nodes
      public static final double GRID_MID_X = GRID_OUTER_X - Units.inchesToMeters(22.75);
      public static final double GRID_HIGH_X = GRID_OUTER_X - Units.inchesToMeters(39.75);

      // Y layout
      public static final int NODE_ROW_COUNT = 9;
      public static final double NODE_FIRST_Y = Units.inchesToMeters(20.19);
      public static final double NODE_SEPARATION_Y = Units.inchesToMeters(22.0);

      // Z layout
      public static final double CUBE_EDGE_HIGH = Units.inchesToMeters(3.0);
      public static final double HIGH_CUBE_Z = Units.inchesToMeters(35.5) - CUBE_EDGE_HIGH;
      public static final double MID_CUBE_Z = Units.inchesToMeters(23.5) - CUBE_EDGE_HIGH;
      public static final double HIGH_CONE_Z = Units.inchesToMeters(46.0);
      public static final double MID_CONE_Z = Units.inchesToMeters(34.0);

      // Translations (all nodes in the same column/row have the same X/Y coordinate)
      public static final Translation2d[] LOW_TRANSLATIONS = new Translation2d[NODE_ROW_COUNT];
      public static final Translation2d[] MID_TRANSLATIONS = new Translation2d[NODE_ROW_COUNT];
      public static final Translation3d[] MIDE_3D_TRANSLATIONS = new Translation3d[NODE_ROW_COUNT];
      public static final Translation2d[] HIGH_TRANSLATIONS = new Translation2d[NODE_ROW_COUNT];
      public static final Translation3d[] HIGH_3D_TRANSLATIONS = new Translation3d[NODE_ROW_COUNT];

      static {
          for (int i = 0; i < NODE_ROW_COUNT; i++) {
              boolean isCube = i == 1 || i == 4 || i == 7;
              LOW_TRANSLATIONS[i] = new Translation2d(GRID_LOW_X, NODE_FIRST_Y + NODE_SEPARATION_Y * i);
              MID_TRANSLATIONS[i] = new Translation2d(GRID_MID_X, NODE_FIRST_Y + NODE_SEPARATION_Y * i);
              MIDE_3D_TRANSLATIONS[i] = new Translation3d(GRID_MID_X, NODE_FIRST_Y + NODE_SEPARATION_Y * i,
                      isCube ? MID_CUBE_Z : MID_CONE_Z);
              HIGH_3D_TRANSLATIONS[i] = new Translation3d(
                      GRID_HIGH_X, NODE_FIRST_Y + NODE_SEPARATION_Y * i, isCube ? HIGH_CUBE_Z : HIGH_CONE_Z);
              HIGH_TRANSLATIONS[i] = new Translation2d(GRID_HIGH_X, NODE_FIRST_Y + NODE_SEPARATION_Y * i);
          }
      }

      // Complex low layout (shifted to account for cube vs cone rows and wide edge
      // nodes)
      public static final double COMPLEX_LOW_X_CONES = GRID_OUTER_X - Units.inchesToMeters(16.0) / 2.0; // Centered X under
                                                                                               // cone nodes
      public static final double COMPLEX_LOW_X_CUBES = GRID_LOW_X; // Centered X under cube nodes
      public static final double COMPLEX_LOW_OUTER_Y_OFFSET = NODE_FIRST_Y - Units.inchesToMeters(3.0)
              - (Units.inchesToMeters(25.75) / 2.0);

      public static final Translation2d[] complexLowTranslations = new Translation2d[] {
              new Translation2d(COMPLEX_LOW_X_CONES, NODE_FIRST_Y - COMPLEX_LOW_OUTER_Y_OFFSET),
              new Translation2d(COMPLEX_LOW_X_CUBES, NODE_FIRST_Y + NODE_SEPARATION_Y * 1),
              new Translation2d(COMPLEX_LOW_X_CONES, NODE_FIRST_Y + NODE_SEPARATION_Y * 2),
              new Translation2d(COMPLEX_LOW_X_CONES, NODE_FIRST_Y + NODE_SEPARATION_Y * 3),
              new Translation2d(COMPLEX_LOW_X_CUBES, NODE_FIRST_Y + NODE_SEPARATION_Y * 4),
              new Translation2d(COMPLEX_LOW_X_CONES, NODE_FIRST_Y + NODE_SEPARATION_Y * 5),
              new Translation2d(COMPLEX_LOW_X_CONES, NODE_FIRST_Y + NODE_SEPARATION_Y * 6),
              new Translation2d(COMPLEX_LOW_X_CUBES, NODE_FIRST_Y + NODE_SEPARATION_Y * 7),
              new Translation2d(
                      COMPLEX_LOW_X_CONES, NODE_FIRST_Y + NODE_SEPARATION_Y * 8 + COMPLEX_LOW_OUTER_Y_OFFSET),
      };
  }

  // Dimensions for loading zone and substations, including the tape
  public static final class LoadingZone {
      // Region dimensions
      public static final double LOADING_WIDTH = Units.inchesToMeters(99.0);
      public static final double LOADING_INNER_X = FieldConstants.FIELD_LENGTH;
      public static final double LOADING_MID_X = FIELD_LENGTH - Units.inchesToMeters(132.25);
      public static final double OUTER_X = FIELD_LENGTH - Units.inchesToMeters(264.25);
      public static final double LEFT_Y = FieldConstants.FIELD_WIDTH;
      public static final double MID_Y = LEFT_Y - Units.inchesToMeters(50.5);
      public static final double RIGHT_Y = LEFT_Y - LOADING_WIDTH;
      public static final Translation2d[] regionCorners = new Translation2d[] {
              new Translation2d(
                      LOADING_MID_X, RIGHT_Y), // Start at lower left next to border with opponent community
              new Translation2d(LOADING_MID_X, MID_Y),
              new Translation2d(OUTER_X, MID_Y),
              new Translation2d(OUTER_X, LEFT_Y),
              new Translation2d(LOADING_INNER_X, LEFT_Y),
              new Translation2d(LOADING_INNER_X, RIGHT_Y),
      };

      // Double substation dimensions
      public static final double DOUBLE_SUBSTATION_LENGTH = Units.inchesToMeters(14.0);
      public static final double DOUBLE_SUBSTATION_X = LOADING_INNER_X - DOUBLE_SUBSTATION_LENGTH;
      public static final double DOUBLE_SUBSTATION_SHELF_Z = Units.inchesToMeters(37.375);

      // Single substation dimensions
      public static final double SINGLE_SUBSTATION_WIDTH = Units.inchesToMeters(22.75);
      public static final double SINGLE_SUBSTATION_LEFT_X = FieldConstants.FIELD_LENGTH - DOUBLE_SUBSTATION_LENGTH
              - Units.inchesToMeters(88.77);
      public static final double SINGLE_SUBSTATION_CENTER_X = SINGLE_SUBSTATION_LEFT_X + (SINGLE_SUBSTATION_WIDTH / 2.0);
      public static final double SINGLE_SUBSTATION_RIGHT_X = SINGLE_SUBSTATION_LEFT_X + SINGLE_SUBSTATION_WIDTH;
      public static final Translation2d SINGLE_SUBSTATION_TRANSLATION = new Translation2d(SINGLE_SUBSTATION_CENTER_X,
              LEFT_Y);

      public static final double SINGLE_SUBSTATION_HEIGHT = Units.inchesToMeters(18.0);
      public static final double SINGLE_SUBSTATION_LOW_Z = Units.inchesToMeters(27.125);
      public static final double SINGLE_SUBSTATION_CENTER_Z = SINGLE_SUBSTATION_LOW_Z + (SINGLE_SUBSTATION_HEIGHT / 2.0);
      public static final double SINGLE_SUBSTATION_HIGH_Z = SINGLE_SUBSTATION_LOW_Z + SINGLE_SUBSTATION_HEIGHT;
  }

  // Locations of staged game pieces
  public static final class StagingLocations {
      public static final double STAGING_CENTER_OFF_SET_X = Units.inchesToMeters(47.36);
      public static final double STAGING_POSITION_X = FIELD_LENGTH / 2.0 - Units.inchesToMeters(47.36);
      public static final double STAGING_FIRST_Y = Units.inchesToMeters(36.19);
      public static final double STAGING_SEPERATION_Y = Units.inchesToMeters(48.0);
      public static final Translation2d[] STAGING_TRANSLATIONS = new Translation2d[4];

      static {
          for (int i = 0; i < STAGING_TRANSLATIONS.length; i++) {
              STAGING_TRANSLATIONS[i] = new Translation2d(STAGING_POSITION_X, STAGING_FIRST_Y + (i * STAGING_SEPERATION_Y));
          }
      }
  }

  

    /**
     * Flips a translation to the correct side of the field based on the current
     * alliance color. By
     * default, all translations and poses in {@link FieldConstants} are stored with
     * the origin at the
     * rightmost point on the BLUE ALLIANCE wall.
     */
    public static Translation2d allianceFlip(Translation2d translation) {
      if (DriverStation.getAlliance() == Alliance.Red) {
          return new Translation2d(FieldConstants.FIELD_LENGTH - translation.getX(), translation.getY());
      } else {
          return translation;
      }
  }

  /**
   * Flips a pose to the correct side of the field based on the current alliance
   * color. By default,
   * all translations and poses in {@link FieldConstants} are stored with the
   * origin at the
   * rightmost point on the BLUE ALLIANCE wall.
   */
  public static Pose2d allianceFlip(Pose2d pose) {
      if (DriverStation.getAlliance() == Alliance.Red) {
          return new Pose2d(
                  FieldConstants.FIELD_LENGTH - pose.getX(),
                  pose.getY(),
                  new Rotation2d(-pose.getRotation().getCos(), pose.getRotation().getSin()));
      } else {
          return pose;
      }
  }

  public static List<Obstacle> obstacles = List.of(
    // Blue Charging Station
    new Obstacle(new double[] {
            FieldConstants.Community.CHARGE_STATION_CORNERS[0].getX(),
            FieldConstants.Community.CHARGE_STATION_CORNERS[1].getX(),
            FieldConstants.Community.CHARGE_STATION_CORNERS[3].getX(),
            FieldConstants.Community.CHARGE_STATION_CORNERS[2].getX(),
    }, new double[] {
            FieldConstants.Community.CHARGE_STATION_CORNERS[0].getY(),
            FieldConstants.Community.CHARGE_STATION_CORNERS[1].getY(),
            FieldConstants.Community.CHARGE_STATION_CORNERS[3].getY(),
            FieldConstants.Community.CHARGE_STATION_CORNERS[2].getY()
    })
    ,
    // Red Charging Station
    new Obstacle(new double[] {
            FieldConstants.allianceFlip(FieldConstants.Community.CHARGE_STATION_CORNERS[2]).getX(),
            FieldConstants.allianceFlip(FieldConstants.Community.CHARGE_STATION_CORNERS[3]).getX(),
            FieldConstants.allianceFlip(FieldConstants.Community.CHARGE_STATION_CORNERS[1]).getX(),
            FieldConstants.allianceFlip(FieldConstants.Community.CHARGE_STATION_CORNERS[0]).getX(),
    }, new double[] {
            FieldConstants.allianceFlip(FieldConstants.Community.CHARGE_STATION_CORNERS[2]).getY(),
            FieldConstants.allianceFlip(FieldConstants.Community.CHARGE_STATION_CORNERS[3]).getY(),
            FieldConstants.allianceFlip(FieldConstants.Community.CHARGE_STATION_CORNERS[1]).getY(),
            FieldConstants.allianceFlip(FieldConstants.Community.CHARGE_STATION_CORNERS[0]).getY()
    })
    );

}
