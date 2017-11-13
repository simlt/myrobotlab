package org.myrobotlab.service;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.stream.Collectors;

import org.myrobotlab.framework.Service;
import org.myrobotlab.framework.ServiceType;
import org.myrobotlab.framework.interfaces.NameProvider;
import org.myrobotlab.logging.LoggerFactory;
import org.slf4j.Logger;

public class MapService extends Service {

  private static final long serialVersionUID = 1L;
  public final static Logger log = LoggerFactory.getLogger(MapService.class.getCanonicalName());
  transient private InMoov inMoov;
  transient public InverseKinematics3D ik3D;

  private double[] currentTcpPoint = { -1, -1, -1 };
  private double[] currentJointAngles = { -1, -1, -1, -1 };

  private Map<String, Location> locationMap = new HashMap<>();

  // calibration points in world coordinates NO, NE, SO, SE
  private double[][] mapCalibrationPoints;
  private static final String mapNO = "mapNO";
  private static final String mapNE = "mapNE";
  private static final String mapSO = "mapSO";
  private static final String mapSE = "mapSE";

  class Location {
    public String name;
    public double[] point;
    public double[] joints;

    public Location(String name, double[] point, double[] joints) {
      this.name = name;
      this.point = point;
      this.joints = joints;
    }

    public Location(String name, double[] joints) {
      this.name = name;
      this.joints = joints;
    }

    public Location(String name, List<Double> jointValues) {
      this.name = name;
      double[] joints = new double[4];
      for (int i = 0; i < jointValues.size(); i++) {
        double value = jointValues.get(i).doubleValue();
        joints[i] = value;
      }
      this.joints = joints;
    }

    public void moveTo() {
      if (joints != null) {
        send("i01.rightArm.omoplate", "moveTo", joints[0]);
        send("i01.rightArm.shoulder", "moveTo", joints[1]);
        send("i01.rightArm.rotate", "moveTo", joints[2]);
        send("i01.rightArm.bicep", "moveTo", joints[3]);
        log.info(String.format("Moving joints to %s (%.2f %.2f %.2f %.2f)", name, joints[0], joints[1], joints[2],
            joints[3]));
      } else if (point != null) {
        send("ik3D", "moveTo", point[0], point[1], point[2]);
      } else {
        log.warn("MapService: no arm position stored for " + name);
        return;
      }
    }
  }

  public MapService(String reservedKey) {
    super(reservedKey);

    inMoov = (InMoov) Runtime.getService("i01");
    try {
      inMoov.startVinMoov();
    } catch (InterruptedException e) {
    }

    ik3D = (InverseKinematics3D) Runtime.start("ik3D", "InverseKinematics3D");

    subscribe("ik3D", "publishTcpPosition");
    subscribe("ik3D", "publishJointAngles");

    // Initial map calibration
    double[] orig = { 0, 300, -50 };
    double[] dim = { 400, 200 };
    mapCalibrationPoints = new double[][] { { orig[0], orig[1], orig[2] }, // NO
        { orig[0] - dim[0], orig[1], orig[2] }, // NE
        { orig[0], orig[1] + dim[1], orig[2] }, // SO
        { orig[0] - dim[0], orig[1] + dim[1], orig[2] } // SE
    };
    invoke("publishMapCalibrationEvent", (Object) mapCalibrationPoints);
  }

  public void onTcpPosition(double[][] tcpPoint) {
    currentTcpPoint = tcpPoint[1];
  }

  public void onJointAngles(Map<String, Double> angleMap) {
    // omoplate, shoulder, rotate, bicep
    currentJointAngles[0] = angleMap.get("omoplate");
    currentJointAngles[1] = angleMap.get("shoulder");
    currentJointAngles[2] = angleMap.get("rotate");
    currentJointAngles[3] = angleMap.get("bicep");
  }

  public void loadMapCalibration(double[][] mapCalibrationPoints) {
    this.mapCalibrationPoints = mapCalibrationPoints;
    invoke("publishMapCalibrationEvent", (Object) mapCalibrationPoints);
    log.info("Calibration points published.");
  }

  public void addLocation(String name) {
    if (currentTcpPoint == null) {
      log.error("Cannot add location. TCP point is not available");
    } else {
      boolean updateCal = false;
      double[] point = currentTcpPoint.clone();
      if (name.equals(mapNO)) {
        mapCalibrationPoints[0] = point;
        updateCal = true;
      } else if (name.equals(mapNE)) {
        mapCalibrationPoints[1] = point;
        updateCal = true;
      } else if (name.equals(mapSO)) {
        mapCalibrationPoints[2] = point;
        updateCal = true;
      } else if (name.equals(mapSE)) {
        mapCalibrationPoints[3] = point;
        updateCal = true;
      }
      locationMap.put(name, new Location(name, point, currentJointAngles.clone()));
      log.info(String.format("Add location %s (%.2f %.2f %.2f)", name, point[0], point[1], point[2]));
      log.info(String.format("Joints at %.2f %.2f %.2f %.2f", currentJointAngles[0], currentJointAngles[1],
          currentJointAngles[2], currentJointAngles[3]));

      if (updateCal) {
        invoke("publishMapCalibrationEvent", (Object) mapCalibrationPoints);
        log.info("Calibration point " + name + " published.");
      }
    }
  }

  public void addJointLocation(String name, double[] pose) {
    locationMap.put(name, new Location(name, null, pose));
    log.info(String.format("Add joint location %s at %.2f %.2f %.2f %.2f", name, pose[0], pose[1], pose[2], pose[3]));
  }

  /*
   * public void moveJointToLocation(String name) { Location loc =
   * locationMap.get(name); if (loc == null) {
   * log.warn("MapService: no location named " + name + " exists."); return; }
   * if (loc.joints == null) {
   * log.warn("MapService: no arm position stored for " + name); return; }
   * 
   * send("i01.rightArm.omoplate", "moveTo", loc.joints[0]);
   * send("i01.rightArm.shoulder", "moveTo", loc.joints[1]);
   * send("i01.rightArm.rotate", "moveTo", loc.joints[2]);
   * send("i01.rightArm.bicep", "moveTo", loc.joints[3]); log.info(String.
   * format("MapService: moving joints to %s (%.2f %.2f %.2f %.2f)", name,
   * currentJointAngles[0], currentJointAngles[1], currentJointAngles[2],
   * currentJointAngles[3])); }
   */

  public void moveToLocation(String name) {
    Location loc = locationMap.get(name);
    if (loc == null) {
      log.warn("MapService: no location named " + name + " exists.");
    }
    loc.moveTo();
  }

  public Object[] getLocations() {
    return locationMap.values().toArray();
  }

  public String getPrintableLocations() {
    return locationMap.values().stream().map(l -> String.format("'%s' : [%.2f, %.2f, %.2f, %.2f]", l.name, l.joints[0],
        l.joints[1], l.joints[2], l.joints[3])).collect(Collectors.joining(",\r\n"));
  }

  public void loadLocations(Map<String, List<Double>> data) {
    log.info("Loading locations...");
    for (Entry<String, List<Double>> entry : data.entrySet()) {
      List<Double> value = entry.getValue();
      Location location = new Location(entry.getKey(), value);
      locationMap.put(entry.getKey(), location);
    }
  }

  public double[][] publishMapCalibrationEvent(double[][] data) {
    return data;
  }

  public void addMapCalibrationHandler(NameProvider service) {
    addListener("publishMapCalibrationEvent", service.getName(), "onMapCalibration");
  }

  /**
   * This static method returns all the details of the class without it having
   * to be constructed. It has description, categories, dependencies, and peer
   * definitions.
   * 
   * @return ServiceType - returns all the data
   * 
   */
  static public ServiceType getMetaData() {
    ServiceType meta = new ServiceType(MapService.class.getCanonicalName());
    meta.addDescription("the InMoov Map Service");
    meta.addCategory("robot", "control");
    // meta.addPeer("ik3D", "InverseKinematics3D", "Inverse kinematics
    // service");

    return meta;
  }
}
